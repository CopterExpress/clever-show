"""
Is a client-side module (meant to be run on Python 2.7) containing base Client class, utility functions and basic callbacks declarations. Main focus of the module is client-specific communication without reliance on `clover` Raspberry Pi environment.
"""

import os
import sys
import time
import errno
import random
import socket
import struct
import logging
import selectors2 as selectors

from contextlib import closing

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
lib_dir = os.path.realpath(os.path.join(current_dir, '../../lib'))
sys.path.insert(0, lib_dir)

logger = logging.getLogger(__name__)

import messaging
from config import ConfigManager

active_client = None  # needs to be refactored: Singleton \ factory callbacks


class Client(object):
    """
    Client base class provides config loading, communication with server (including automatic reconnection, broadcast listening and binding). You can inherit this class in order to extend functionality for practical applications.
    
    Attributes:
        server_connection (ConnectionManager) - connection to the server.
        connected (bool) - whether the client is connected to the server.
        client_id (string) - ID of the client.
        config (ConfigManager) - contains loaded client configuration.
        config_path (string) -  path to configuration file. There also should be config specification file at 'config_path\config\configspec_client.ini'.
    """

    def __init__(self, config_path=os.path.join(current_dir, os.pardir, "config", "client.ini")):
        """
        Initializtion
        ```python
        client = Client(config_path)
        ```
        
        Args:
            config_path (string, optional): Path to the file with configuration.  There also should be config specification file at `<config_path>\config\configspec_client.ini`. Defaults to `<current_dir>\os.pardir\config\client.ini`.
        """
        self.selector = selectors.DefaultSelector()
        self.client_socket = None

        self.server_connection = messaging.ConnectionManager()

        self.connected = False
        self.client_id = None

        # Init configs
        self.config = ConfigManager()
        self.config_path = config_path

        global active_client
        active_client = self

    def load_config(self):
        """
        Loads or reloads config from file specified in 'config_path' attribute.
        """
        self.config.load_config_and_spec(self.config_path)

        config_id = self.config.id.lower()
        if config_id == '/default':
            self.client_id = 'copter' + str(random.randrange(9999)).zfill(4)
            self.config.set('', 'id', self.client_id, write=True) # set and write
        elif config_id == '/hostname':
            self.client_id = socket.gethostname()
        elif config_id == '/ip':
            self.client_id = messaging.get_ip_address()
        else:
            self.client_id = config_id

        logger.info("Config loaded")

    @staticmethod
    def get_ntp_time(ntp_host, ntp_port):
        """Gets and returns time from specified host and port of NTP server.

        Args:
            ntp_host (string): hostname or address of the NTP server.
            ntp_port (int): port of the NTP server.

        Returns:
            int: Current time recieved from the NTP server
        """
        NTP_PACKET_FORMAT = "!12I"
        NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
        NTP_QUERY = '\x1b' + 47 * '\0'

        with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as s:
            s.sendto(bytes(NTP_QUERY), (ntp_host, ntp_port))
            msg, address = s.recvfrom(1024)
        unpacked = struct.unpack(NTP_PACKET_FORMAT, msg[0:struct.calcsize(NTP_PACKET_FORMAT)])
        return unpacked[10] + float(unpacked[11]) / 2 ** 32 - NTP_DELTA

    def time_now(self):
        """gets and returns system time or NTP time depending on the config.

        Returns:
            int: Current time.
        """
        if self.config.ntp_use:
            timenow = self.get_ntp_time(self.config.ntp_host, self.config.ntp_port)
        else:
            timenow = time.time()
        return timenow

    def start(self):
        """
        Reloads config and starts infinite loop of connecting to the server and processing said connection. Calling of this method will indefinitely halt execution of any subsequent code.
        """
        self.load_config()

        logger.info("Starting client")
        messaging.NotifierSock().init(self.selector)

        try:
            while True:
                self._reconnect()
                self._process_connections()

        except (KeyboardInterrupt, ):
            logger.critical("Caught interrupt, exiting!")
            self.selector.close()

    def _reconnect(self, timeout=2.0, attempt_limit=3):  # TODO reconnecting broadcast listener in another thread
        logger.info("Trying to connect to {}:{} ...".format(self.config.server_host, self.config.server_port))
        attempt_count = 0
        while not self.connected:
            logger.info("Waiting for connection, attempt {}".format(attempt_count))
            try:
                self.client_socket = socket.socket()
                self.client_socket.settimeout(timeout)
                messaging.set_keepalive(self.client_socket)
                self.client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.client_socket.connect((self.config.server_host, self.config.server_port))
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
        self.selector.register(self.client_socket, selectors.EVENT_READ, data=self.server_connection)
        self.server_connection.connect(self.selector, self.client_socket,
                                       (self.config.server_host, self.config.server_port))

    def broadcast_bind(self, timeout=2.0, attempt_limit=3):
        broadcast_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        broadcast_client.settimeout(timeout)
        try:
            broadcast_client.bind(("", self.config.broadcast_port))
        except socket.error as error:
            logger.error("Error during broadcast listening binding: {}".format(error))
            return

        attempt_count = 0
        try:
            while attempt_count <= attempt_limit:
                try:
                    data, addr = broadcast_client.recvfrom(self.config.server_buffer_size)
                except socket.error as error:
                    logger.warning("Could not receive broadcast due error: {}".format(error))
                    attempt_count += 1
                else:
                    message = messaging.MessageManager()
                    message.income_raw = data
                    message.process_message()
                    if message.content and message.jsonheader["action"] == "server_ip":
                        logger.info("Received broadcast message {} from {}".format(message.content, addr))

                        kwargs = message.content["kwargs"]
                        self.config.set("SERVER", "port", int(kwargs["port"]))
                        self.config.set("SERVER", "host", kwargs["host"])
                        self.config.write()

                        logger.info("Binding to new IP: {}:{}".format(
                            self.config.server_host, self.config.server_port))
                        self.on_broadcast_bind()
                        break
        finally:
            broadcast_client.close()

    def on_broadcast_bind(self):  # TODO move ALL binding code here
        """
        Method called on binding to the server by broadcast. Override that method in order to add functionality.
        """
        pass

    def _process_connections(self):
        while True:
            events = self.selector.select(timeout=1)
            for key, mask in events:
                connection = key.data
                if connection is not None:
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
            try:
                mapping_fds = self.selector.get_map().keys() # file descriptors
                notifier_fd = messaging.NotifierSock().get_sock().fileno()
            except (KeyError, RuntimeError) as e:
                logger.error("Exception {} occurred when getting connections map!".format(e))
                logger.error("Connections changed during getting connections map, passing")
            else:
                notify_only= len(mapping_fds) == 1 and notifier_fd in mapping_fds
                if notify_only or not mapping_fds:
                    logger.warning("No active connections left!")
                    return


@messaging.message_callback("config")
def _command_config_write(*args, **kwargs):
    mode = kwargs.get("mode", "modify")
    # exceptions would be risen in case of incorrect config
    if mode == "rewrite":
        active_client.config.load_from_dict(kwargs["config"], configspec=active_client.config_path)  # with validation
    elif mode == "modify":
        new_config = ConfigManager()
        new_config.load_from_dict(kwargs["config"])
        active_client.config.merge(new_config, validate=True)

    active_client.config.write()
    logger.info("Config successfully updated from command")
    active_client.load_config()

@messaging.request_callback("config")
def _response_config(*args, **kwargs):
    send_configspec = kwargs.get("send_configspec", False)
    response = {"config": active_client.config.full_dict()}
    if send_configspec:
        response.update({"configspec": dict(active_client.config.config.configspec)})
    return response

@messaging.request_callback("clover_dir")
def _response_clover_dir(*args, **kwargs):
    return active_client.config.clover_dir

@messaging.request_callback("id")
def _response_id(*args, **kwargs):
    new_id = kwargs.get("new_id", None)
    if new_id is not None:
        active_client.config.set("PRIVATE", "id", new_id, True)
        active_client.load_config()
        # TODO renaming here

    return active_client.client_id


@messaging.request_callback("time")
def _response_time(*args, **kwargs):
    return active_client.time_now()


if __name__ == "__main__":
    startup_cwd = os.getcwd()

    import threading

    print(Client.get_ntp_time("ntp1.stratum2.ru", 123))


    def restart():  # move to core
        args = sys.argv[:]
        logging.info('Restarting {}'.format(args))
        args.insert(0, sys.executable)
        if sys.platform == 'win32':
            args = ['"%s"' % arg for arg in args]
        os.chdir(startup_cwd)
        os.execv(sys.executable, args)

    def mock_telem():
        while True:
            time.sleep(5)
            #t = dict([('fcu_status', None), ('current_position', [-2.89, 2.12, 3.64, 15.22, 'aruco_map']), ('animation_id', 'two_drones_test'), ('selfcheck', 'OK'), ('battery', None), ('git_version', '01bf95e'), ('calibration_status', None), ('start_position', [0.2, 0.2, 0.0]), ('mode', 'MANUAL'), ('time_delta', 1581338473.438682), ('armed', False), ('config_version', None), ('last_task', 'No task')])
            t = dict([('fcu_status', 'STANDBY'), ('current_position', [-1.17, 2.04, 3.45, 0, "11"]), ('animation_id', 'two_drones_test'), ('selfcheck', 'OK'), ('battery', [12.2, 1.0]), ('git_version', '42aee96'), ('calibration_status', None), ('start_position', [0.2, 0.2, 0.0]), ('mode', 'MANUAL'), ('time_delta', 1581342970.889573), ('armed', False), ('config_version', 'Copter config V0.0'), ('last_task', 'No task')])
            if active_client.connected:
                active_client.server_connection.send_message("telemetry", kwargs={"value": t})

    logging.basicConfig(level=logging.DEBUG)
    client = Client()
    tr = threading.Thread(target=mock_telem)
    tr.start()
    client.start()

