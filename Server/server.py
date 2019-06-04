import sys
import time
import socket
import random
import logging
import threading
import selectors
import collections
import configparser

import os,sys,inspect  # Add parent dir to PATH to import messaging_lib
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir) 
import messaging_lib as messaging

# All imports sorted in pyramid just because

random.seed()

logging.basicConfig(  # TODO all prints as logs
    level=logging.DEBUG,
    format="%(asctime)s [%(name)-7.7s] [%(threadName)-19.19s] [%(levelname)-7.7s]  %(message)s",
    handlers=[
        logging.FileHandler("server_logs.log"),
        logging.StreamHandler()
    ])

ConfigOption = collections.namedtuple("ConfigOption", ["section", "option", "value"])


class Server:
    BUFFER_SIZE = 1024

    def __init__(self, server_id=None, config_path="server_config.ini"):
        self.id = server_id if server_id else str(random.randint(0, 9999)).zfill(4)

        # Init socket
        self.sel = selectors.DefaultSelector()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = socket.gethostname()
        self.ip = Server.get_ip_address()

        # Init configs
        self.config_path = config_path
        self.config = configparser.ConfigParser()
        self.load_config()

        # Init threads
        self.autoconnect_thread = threading.Thread(target=self._client_processor, daemon=True,
                                                   name='Client processor')
        self.client_processor_thread_running = threading.Event()  # Can be used for manual thread killing

        self.broadcast_thread = threading.Thread(target=self._ip_broadcast, daemon=True,
                                                 name='IP broadcast sender')
        self.broadcast_thread_running = threading.Event()

        self.listener_thread = threading.Thread(target=self._broadcast_listen, daemon=True,
                                                name='IP broadcast listener')
        self.listener_thread_running = threading.Event()

    def load_config(self):
        self.config.read(self.config_path)
        self.port = int(self.config['SERVER']['port'])  # TODO try, init def
        self.broadcast_port = int(self.config['SERVER']['broadcast_port'])
        self.BROADCAST_DELAY = int(self.config['SERVER']['broadcast_delay'])
        Server.BUFFER_SIZE = int(self.config['SERVER']['buffer_size'])

        self.USE_NTP = self.config.getboolean('NTP', 'use_ntp')
        self.NTP_HOST = self.config['NTP']['host']
        self.NTP_PORT = int(self.config['NTP']['port'])

    def start(self):  # do_auto_connect=True, do_ip_broadcast=True, do_listen_broadcast=False
        logging.info("Starting server with id: {} on {}:{} !".format(self.id, self.ip, self.port))
        logging.info("Starting server socket!")
        self.server_socket.bind((self.ip, self.port))

        logging.info("Starting client processor thread!")
        self.client_processor_thread_running.set()
        self.autoconnect_thread.start()

        logging.info("Starting broadcast sender thread!")
        self.broadcast_thread_running.set()
        self.broadcast_thread.start()

        logging.info("(not) Starting broadcast listener thread!")
        self.listener_thread_running.set()
        # listener_thread.start()

    def stop(self):
        logging.info("Stopping server")
        self.client_processor_thread_running.clear()
        self.broadcast_thread_running.clear()
        self.listener_thread_running.clear()
        self.server_socket.close()
        self.sel.close()
        logging.info("Server stopped")

    @staticmethod
    def get_ip_address():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ip_socket:
                ip_socket.connect(("8.8.8.8", 80))
                return ip_socket.getsockname()[0]
        except OSError:
            logging.warning("No network connection detected, starting on localhost")
            return "localhost"

    @staticmethod
    def get_ntp_time(ntp_host, ntp_port):
        NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
        NTP_QUERY = b'\x1b' + bytes(47)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ntp_socket:
            ntp_socket.sendto(NTP_QUERY, (ntp_host, ntp_port))
            msg, _ = ntp_socket.recvfrom(1024)
        return int.from_bytes(msg[-8:], 'big') / 2 ** 32 - NTP_DELTA

    def time_now(self):
        if self.USE_NTP:
            timenow = self.get_ntp_time(self.NTP_HOST, self.NTP_PORT)
        else:
            timenow = time.time()
        return timenow

    # noinspection PyArgumentList
    def _client_processor(self):
        logging.info("Client processor (selector) thread started!")
        self.server_socket.listen()
        self.server_socket.setblocking(False)
        self.sel.register(self.server_socket, selectors.EVENT_READ | selectors.EVENT_WRITE, data=None)

        while self.client_processor_thread_running.is_set():
            events = self.sel.select(timeout=None)
            for key, mask in events:
                if key.data is None:
                    self._connect_client(key.fileobj)
                else:
                    client = key.data
                    try:
                        client.process_events(mask)
                    except Exception as error:
                        logging.error("Exception {} occurred for {}! Resetting connection!".format(error, client.addr))
                        client.close()

        logging.info("Client autoconnect thread stopped!")

    def _connect_client(self, sock):
        conn, addr = sock.accept()
        logging.info("Got connection from: {}".format(str(addr)))
        conn.setblocking(False)

        if not any(client_addr == addr[0] for client_addr in Client.clients.keys()):
            client = Client(addr[0])
            logging.info("New client")
        else:
            client = Client.clients[addr[0]]
            logging.info("Reconnected client")
        self.sel.register(conn, selectors.EVENT_READ, data=client)
        client.connect(self.sel, conn, addr)

    def _ip_broadcast(self):
        logging.info("Broadcast sender thread started!")
        msg = messaging.MessageManager.create_simple_message(
            "server_ip", {"host": self.ip, "port": str(self.port), "id": self.id})
        broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        logging.info("Formed broadcast message: {}".format(msg))

        try:
            while self.broadcast_thread_running.is_set():
                time.sleep(self.BROADCAST_DELAY)
                broadcast_sock.sendto(msg, ('255.255.255.255', self.broadcast_port))
                logging.debug("Broadcast sent")
        finally:
            broadcast_sock.close()
            logging.info("Broadcast sender thread stopped, socked closed!")

    def _broadcast_listen(self):
        logging.info("Broadcast listener thread started!")
        broadcast_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            broadcast_client.bind(("", self.broadcast_port))
        except OSError:
            logging.critical("Another server is running on this computer, shutting down!")
            sys.exit()

        try:
            while self.listener_thread_running.is_set():
                data, addr = broadcast_client.recvfrom(1024)
                message = messaging.MessageManager()
                message.income_raw = data
                message.process_message()
                if message.content:
                    if message.content["command"] == "server_ip":
                        if message.content["args"]["id"] != self.id:
                            logging.critical("Another server detected on network, shutting down")
                            sys.exit()
                else:
                    logging.warning("Got wrong broadcast message from {}".format(addr))
        finally:
            broadcast_client.close()
            logging.info("Broadcast listener thread stopped, socked closed!")

    def send_starttime(self, copter, dt=0):
        timenow = self.time_now()
        print('Now:', time.ctime(timenow), "+ dt =", dt)
        copter.send_message("start", {"time": str(timenow + dt)})


def requires_connect(f):
    def wrapper(*args, **kwargs):
        if args[0].connected:
            return f(*args, **kwargs)
        else:
            logging.warning("Function requires client to be connected!")
    return wrapper


def requires_any_connected(f):
    def wrapper(*args, **kwargs):
        if Client.clients:
            return f(*args, **kwargs)
        else:
            logging.warning("No clients were connected!")
    return wrapper


class Client(messaging.ConnectionManager):
    clients = {}

    on_connect = None  # Use as callback functions
    on_first_connect = None
    on_disconnect = None

    def __init__(self, ip):
        super(Client, self).__init__()
        self.copter_id = None
        self.connected = False

        self.clients[ip] = self

    @staticmethod
    def get_by_id(copter_id):
        for client in Client.clients.values():
            if client.copter_id == copter_id:
                return client

    def connect(self, client_selector, client_socket, client_addr):
        logging.info("Client connected")
        if not self.resume_queue:
            self._send_queue = collections.deque()

        super(Client, self).connect(client_selector, client_socket, client_addr)

        self.connected = True

        if self.copter_id is None:
            self.get_response("id", self._got_id)

        if self.on_connect:
            self.on_connect(self)

    def _got_id(self, value):
        logging.info("Got copter id: {} for client {}".format(value, self.addr))
        self.copter_id = value
        if Client.on_first_connect:
            Client.on_first_connect(self)

    def close(self):
        self.connected = False

        if Client.on_disconnect:
            Client.on_disconnect(self)

        super(Client, self).close()

    @requires_connect
    def _send(self, data):
        super(Client, self)._send(data)
        logging.debug("Queued data to send: {}".format(data))

    def send_config_options(self, *options: ConfigOption, reload_config=True):
        logging.info("Sending config options: {} to {}".format(options, self.addr))
        sending_options = [{'section': option.section, 'option': option.option, 'value': option.value}
                           for option in options]
        print(sending_options)
        self.send_message(
            'config_write', {"options": sending_options, "reload": reload_config}
        )

    @staticmethod
    @requires_any_connected
    def broadcast(message, force_all=False):
        for client in Client.clients.values():
            if client.connected or force_all:
                client._send(message)

    @classmethod
    @requires_any_connected
    def broadcast_message(cls, command, args=None, force_all=False):
        cls.broadcast(messaging.MessageManager.create_simple_message(command, args), force_all)


if __name__ == '__main__':
    server = Server()
    server.start()
