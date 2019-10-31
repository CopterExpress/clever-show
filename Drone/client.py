import os
import time
import errno
import random
import socket
import struct
import logging
import collections
import ConfigParser
import selectors2 as selectors
import threading

from contextlib import closing

import os,sys,inspect  # Add parent dir to PATH to import messaging_lib
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

import messaging_lib as messaging

ConfigOption = collections.namedtuple("ConfigOption", ["section", "option", "value"])

active_client = None  # maybe needs to be refactored

class Client(object):
    def __init__(self, config_path="client_config.ini"):
        self.selector = selectors.DefaultSelector()
        self.client_socket = None

        self.server_connection = messaging.ConnectionManager()

        self.server_host = None
        self.server_port = None
        self.broadcast_port = None

        self.connected = False
        self.client_id = None

        # Init configs
        self.config_path = config_path
        self.config = ConfigParser.ConfigParser()
        self.load_config()

        global active_client
        active_client = self

        # self._last_ping_time = 0

    def load_config(self):
        self.config.read(self.config_path)

        self.broadcast_port = self.config.getint('SERVER', 'broadcast_port')
        self.server_port = self.config.getint('SERVER', 'port')
        self.server_host = self.config.get('SERVER', 'host')
        self.BUFFER_SIZE = self.config.getint('SERVER', 'buffer_size')
        self.USE_NTP = self.config.getboolean('NTP', 'use_ntp')
        self.NTP_HOST = self.config.get('NTP', 'host')
        self.NTP_PORT = self.config.getint('NTP', 'port')

        self.files_directory = self.config.get('FILETRANSFER', 'files_directory') # not used?!

        self.client_id = self.config.get('PRIVATE', 'id')
        if self.client_id == '/default':
            self.client_id = 'copter' + str(random.randrange(9999)).zfill(4)
            self.write_config(False, ConfigOption('PRIVATE', 'id', self.client_id))
        elif self.client_id == '/hostname':
            self.client_id = socket.gethostname()
        elif self.client_id == '/ip':
            self.client_id = messaging.get_ip_address()

    def rewrite_config(self):
        with open(self.config_path, 'w') as file:
            self.config.write(file)
        os.system("chown -R pi:pi /home/pi/clever-show")

    def write_config(self, reload_config=True, *config_options):
        for config_option in config_options:
            self.config.set(config_option.section, config_option.option, config_option.value)
        self.rewrite_config()

        if reload_config:
            self.load_config()

    @staticmethod
    def get_ntp_time(ntp_host, ntp_port):
        NTP_PACKET_FORMAT = "!12I"
        NTP_DELTA = 2208988800L  # 1970-01-01 00:00:00
        NTP_QUERY = '\x1b' + 47 * '\0'

        with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as s:
            s.sendto(bytes(NTP_QUERY), (ntp_host, ntp_port))
            msg, address = s.recvfrom(1024)
        unpacked = struct.unpack(NTP_PACKET_FORMAT, msg[0:struct.calcsize(NTP_PACKET_FORMAT)])
        return unpacked[10] + float(unpacked[11]) / 2 ** 32 - NTP_DELTA

    def time_now(self):
        if self.USE_NTP:
            timenow = self.get_ntp_time(self.NTP_HOST, self.NTP_PORT)
        else:
            timenow = time.time()
        return timenow

    def start(self):
        logger.info("Starting client")
        try:
            while True:
                self._reconnect()
                self._process_connections()

        except (KeyboardInterrupt, ):
            logger.critical("Caught interrupt, exiting!")
            self.selector.close()

    def _reconnect(self, timeout=3.0, attempt_limit=5):
        logger.info("Trying to connect to {}:{} ...".format(self.server_host, self.server_port))
        attempt_count = 0
        while not self.connected:
            logger.info("Waiting for connection, attempt {}".format(attempt_count))
            try:
                self.client_socket = socket.socket()
                self.client_socket.settimeout(timeout)
                self.client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.client_socket.connect((self.server_host, self.server_port))
            except socket.error as error:
                if isinstance(error, OSError):
                    if error.errno == errno.EINTR:
                        logger.critical("Shutting down on keyboard interrupt")
                        raise KeyboardInterrupt

                logger.warning("Can not connect due error: {}".format(error))
                attempt_count += 1
                time.sleep(timeout)

            else:
                logger.info("Connection to server successful!")
                self._connect()
                break

            if attempt_count >= attempt_limit:
                logger.info("Too many attempts. Trying to get new server IP")
                self.broadcast_bind(timeout*2, attempt_limit)
                attempt_count = 0

    def _connect(self):
        self.connected = True
        self.client_socket.setblocking(False)
        events = selectors.EVENT_READ # | selectors.EVENT_WRITE
        self.selector.register(self.client_socket, events, data=self.server_connection)
        self.server_connection.connect(self.selector, self.client_socket, (self.server_host, self.server_port))

    def broadcast_bind(self, timeout=3.0, attempt_limit=5):
        broadcast_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        broadcast_client.bind(("", self.broadcast_port))
        broadcast_client.settimeout(timeout)

        attempt_count = 0
        try:
            while attempt_count <= attempt_limit:
                try:
                    data, addr = broadcast_client.recvfrom(self.BUFFER_SIZE)
                except socket.error as error:
                    logger.warning("Could not receive broadcast due error: {}".format(error))
                    attempt_count += 1
                else:
                    message = messaging.MessageManager()
                    message.income_raw = data
                    message.process_message()
                    if message.content:
                        logger.info("Received broadcast message {} from {}".format(message.content, addr))
                        if message.content["command"] == "server_ip":
                            args = message.content["args"]
                            self.server_port = int(args["port"])
                            self.server_host = args["host"]
                            self.write_config(False,
                                              ConfigOption("SERVER", "port", self.server_port),
                                              ConfigOption("SERVER", "host", self.server_host))
                            logger.info("Binding to new IP: {}:{}".format(self.server_host, self.server_port))
                            self.on_broadcast_bind()
                            break
        finally:
            broadcast_client.close()

    def on_broadcast_bind(self):
        pass

    def _process_connections(self):
        while True:
            events = self.selector.select(timeout=1)
            # if time.time() - self._last_ping_time > 5:
            #    self.server_connection.send_message("ping")
            #    self._last_ping_time = time.time()
            # logging.debug("tick")
            for key, mask in events:  # TODO add notifier to client!
                connection = key.data
                if connection is None:
                    pass
                else:
                    try:
                        connection.process_events(mask)

                    except Exception as error:
                        logger.error(
                            "Exception {} occurred for {}! Resetting connection!".format(error, connection.addr)
                        )
                        self.server_connection._close()
                        self.connected = False

                        if isinstance(error, OSError):
                            if error.errno == errno.EINTR:
                                raise KeyboardInterrupt


            if not self.selector.get_map():
                logger.warning("No active connections left!")
                return


@messaging.message_callback("config_write")
def _command_config_write(*args, **kwargs):
    options = [ConfigOption(**raw_option) for raw_option in kwargs["options"]]
    logger.info("Writing config options: {}".format(options))
    active_client.write_config(kwargs["reload"], *options)


@messaging.request_callback("id")
def _response_id(*args, **kwargs):
    new_id = kwargs.get("new_id", None)
    if new_id is not None:
        cfg = ConfigOption("PRIVATE", "id", new_id)
        active_client.write_config(True, cfg)

    return active_client.client_id


@messaging.request_callback("time")
def _response_time(*args, **kwargs):
    return active_client.time_now()


if __name__ == "__main__":
    client = Client()
    client.start()
