import time
import errno
import random
import socket
import struct
import logging
import selectors2 as selectors
import ConfigParser
from contextlib import closing

import os,sys,inspect
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 

from messaging_lib import Message
random.seed()

logging.basicConfig(  # TODO all prints as logs
    level=logging.DEBUG, # INFO
    format="%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
    handlers=[
        logging.FileHandler("client_logs.log"),
        logging.StreamHandler()
    ])


class Client:
    def __init__(self, config_path="client_config.ini"):
        self.selector = selectors.DefaultSelector()
        self.client_socket = None
        self.server_host = None
        self.server_port = None
        self.broadcast_port = None

        self.connected = False
        self.client_id = None

        # Init configs
        self.config_path = config_path
        self.config = ConfigParser.ConfigParser()
        self.load_config()

    def load_config(self):
        self.config.read(self.config_path)

        self.broadcast_port = self.config.getint('SERVER', 'broadcast_port')
        self.server_port = self.config.getint('SERVER', 'port')
        self.server_host = self.config.get('SERVER', 'host')
        self.BUFFER_SIZE = self.config.getint('SERVER', 'buffer_size')
        self.USE_NTP = self.config.getboolean('NTP', 'use_ntp')
        self.NTP_HOST = self.config.get('NTP', 'host')
        self.NTP_PORT = self.config.getint('NTP', 'port')

        files_directory = self.config.get('FILETRANSFER', 'files_directory')

        #FRAME_ID = self.config.get('COPTERS', 'frame_id')  # TODO in play_animation
        #self.TAKEOFF_HEIGHT = self.config.getfloat('COPTERS', 'takeoff_height')
        #self.TAKEOFF_TIME = self.config.getfloat('COPTERS', 'takeoff_time')
        #self.RFP_TIME = self.config.getfloat('COPTERS', 'reach_first_point_time')
        #self.SAFE_TAKEOFF = self.config.getboolean('COPTERS', 'safe_takeoff')

        #self.X0_COMMON = self.config.getfloat('COPTERS', 'x0_common')
        #self.Y0_COMMON = self.config.getfloat('COPTERS', 'y0_common')
        #self.X0 = self.config.getfloat('PRIVATE', 'x0')
        #self.Y0 = self.config.getfloat('PRIVATE', 'y0')

        #self.USE_LEDS = config.getboolean('PRIVATE', 'use_leds')
        #play_animation.USE_LEDS = USE_LEDS  # TODO in copter_client

        self.client_id = self.config.get('PRIVATE', 'id')
        if self.client_id == 'default':
            client_id = 'copter' + str(random.randrange(9999)).zfill(4)
            #write_to_config('PRIVATE', 'id', client_id)
        elif self.client_id == '/hostname':
            self.client_id = socket.gethostname()

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

    def reconnect(self, timeout=2, attempt_limit=5):
        logging.info("Trying to connect to {}:{} ...".format(self.server_host, self.server_port))
        attempt_count = 0
        while not self.connected:
            logging.info("Waiting for connection, attempt {}".format(attempt_count))
            try:
                self.client_socket = socket.socket()
                self.client_socket.settimeout(timeout)
                self.client_socket.connect((self.server_host, self.server_port))
            except socket.error as error:
                if error.errno != errno.EINTR:
                    logging.warning("Can not connect due error: {}".format(error))
                    attempt_count += 1
                    time.sleep(timeout)
                else:
                    logging.critical("Shutting down on keyboard interrupt")
                    raise KeyboardInterrupt
            else:
                self.connected = True
                #self.client_socket.settimeout(None)
                self.client_socket.setblocking(False)
                events = selectors.EVENT_READ | selectors.EVENT_WRITE
                self.selector.register(self.client_socket, events, data=None)
                logging.info("Connection to server successful!")
                break

            if attempt_count >= attempt_limit:
                logging.info("Too many attempts. Trying to get new server IP")
                self.broadcast_bind()
                attempt_count = 0

    def broadcast_bind(self):
        broadcast_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        broadcast_client.bind(("", self.broadcast_port))
        try:
            while True:
                data, addr = broadcast_client.recvfrom(1024)
                message = Message()
                message.income_raw = data
                message.process_message()
                if message.content:
                    logging.info("Received broadcast message {} from {}".format(message.content, addr))
                    if message.content["command"] == "server_ip":
                        args = message.content["args"]
                        self.server_host = args["host"]
                        self.server_port = int(args["port"])
                        logging.info("Binding to new IP: {}:{}".format(self.server_host, self.server_port))
                        #write_to_config("SERVER", "port", port)
                        #write_to_config("SERVER", "host", host)  # TODO
                        break
        finally:
            broadcast_client.close()

    def service_connection(self, key, mask):
        sock = key.fileobj
        data = key.data
        if mask & selectors.EVENT_READ:
            recv_data = sock.recv(1024)  # Should be ready to read
            if recv_data:
                print("received", repr(recv_data), "from connection", )
            if not recv_data:
                print("closing connection",)
                self.selector.unregister(sock)
                self.client_socket.close()
        if mask & selectors.EVENT_READ:
            pass


    def mainloop(self):
        #self.client_socket.send("104771313759739")
        try:
            while True:
                events = self.selector.select(timeout=1)
                if events:
                    for key, mask in events:
                        self.service_connection(key, mask)
                        pass

                if not self.selector.get_map():
                    logging.warning("No active connections left")
                    self.reconnect()
        except KeyboardInterrupt:
            print("caught keyboard interrupt, exiting")
        finally:
            self.selector.close()


if __name__ == "__main__":
    client = Client()
    client.reconnect()
    client.mainloop()
