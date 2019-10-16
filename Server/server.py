import sys
import time
import socket
import random
import logging
import threading
import selectors
import collections
import configparser

import os, inspect  # Add parent dir to PATH to import messaging_lib

current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)
import messaging_lib as messaging

random.seed()

logging.basicConfig(  # TODO all prints as logs
    level=logging.DEBUG,
    format="%(asctime)s [%(name)-7.7s] [%(threadName)-19.19s] [%(levelname)-7.7s]  %(message)s",
    handlers=[
        logging.FileHandler("server_logs.log"),
        logging.StreamHandler()
    ])

ConfigOption = collections.namedtuple("ConfigOption", ["section", "option", "value"])


class Server(messaging.Singleton):
    def __init__(self, server_id=None, config_path="server_config.ini", on_stop=None):
        self.id = server_id if server_id else str(random.randint(0, 9999)).zfill(4)
        self.time_started = 0

        self.on_stop = on_stop

        # Init socket
        self.sel = selectors.DefaultSelector()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = socket.gethostname()
        self.ip = messaging.get_ip_address()

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
        self.BUFFER_SIZE = int(self.config['SERVER']['buffer_size']) # TODO connect to connection manager

        self.remove_disconnected = self.config.getboolean('SERVER', 'remove_disconnected')

        self.use_broadcast = self.config.getboolean('BROADCAST', 'use_broadcast')
        self.broadcast_port = int(self.config['BROADCAST']['broadcast_port'])
        self.BROADCAST_DELAY = int(self.config['BROADCAST']['broadcast_delay'])

        self.USE_NTP = self.config.getboolean('NTP', 'use_ntp')
        self.NTP_HOST = self.config['NTP']['host']
        self.NTP_PORT = int(self.config['NTP']['port'])

    def start(self, do_ip_broadcast=None):  # do_auto_connect=True, , do_listen_broadcast=False
        self.time_started = time.time()

        if do_ip_broadcast is None:
            do_ip_broadcast = self.use_broadcast

        logging.info("Starting server with id: {} on {}:{} !".format(self.id, self.ip, self.port))
        logging.info("Starting server socket!")
        self.server_socket.bind((self.ip, self.port))

        logging.info("Starting client processor thread!")
        self.client_processor_thread_running.set()
        self.autoconnect_thread.start()

        if do_ip_broadcast:
            logging.info("Starting broadcast sender thread!")
            self.broadcast_thread_running.set()
            self.broadcast_thread.start()

        logging.info("Starting broadcast listener thread!")
        self.listener_thread_running.set()
        self.listener_thread.start()

    def stop(self):
        logging.info("Stopping server")
        self.client_processor_thread_running.clear()
        self.broadcast_thread_running.clear()
        self.listener_thread_running.clear()
        self.server_socket.close()
        self.sel.close()
        logging.info("Server stopped")

        if self.on_stop is not None:
            self.on_stop()

        sys.exit("Stopped")

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
        self.sel.register(self.server_socket, selectors.EVENT_READ, data=None) #| selectors.EVENT_WRITE

        messaging.NotifierSock().bind((self.ip, self.port))

        while self.client_processor_thread_running.is_set():
            events = self.sel.select()
            logging.error('tick')
            for key, mask in events:
                # logging.error(mask)
                # logging.error(str(key.data))
                client = key.data
                if client is None:
                    self._connect_client(key.fileobj)
                elif isinstance(client, messaging.ConnectionManager):
                    try:
                        client.process_events(mask)
                    except Exception as error:
                        logging.error("Exception {} occurred for {}! Resetting connection!".format(error, client.addr))
                        client.close(True)
                else:  # Notifier
                    client.process_events(mask)

        logging.info("Client autoconnect thread stopped!")

    def _connect_client(self, sock):
        conn, addr = sock.accept()
        logging.info("Got connection from: {}".format(str(addr)))
        conn.setblocking(False)

        if addr[0] == self.ip and messaging.NotifierSock().addr is None:
            client = messaging.NotifierSock()
            logging.info("Notifier sock client")

        elif not any([client_addr == addr[0] for client_addr in Client.clients.keys()]):
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
            "server_ip", {"host": self.ip, "port": str(self.port), "id": self.id, "start_time": str(self.time_started)})
        broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        logging.info("Formed broadcast message: {}".format(msg))

        time.sleep(self.BROADCAST_DELAY)
        try:
            while self.broadcast_thread_running.is_set():
                broadcast_sock.sendto(msg, ('255.255.255.255', self.broadcast_port))
                logging.debug("Broadcast sent")
                time.sleep(self.BROADCAST_DELAY)

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
            # TODO popup and as function
            self.stop()

        try:
            while self.listener_thread_running.is_set():
                data, addr = broadcast_client.recvfrom(1024)  # TODO nonblock
                message = messaging.MessageManager()
                message.income_raw = data
                message.process_message()
                if message.content:
                    if message.content["command"] == "server_ip":
                        if message.content["args"]["id"] != str(self.id) \
                                and float(message.content["args"]["start_time"]) <= self.time_started:

                            # younger server should shut down
                            logging.critical("Another server detected over the network, shutting down!")
                            # TODO popup
                            self.stop()

                else:
                    logging.warning("Got wrong broadcast message from {}".format(addr))
        finally:
            broadcast_client.close()
            logging.info("Broadcast listener thread stopped, socked closed!")

    def send_starttime(self, copter, start_time):
        print('start_time: {}'.format(start_time))
        copter.send_message("start", {"time": str(start_time)})


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
        if self.on_first_connect:
            self.on_first_connect(self)

    def close(self, inner=False):
        self.connected = False

        if self.on_disconnect:
            self.on_disconnect(self)

        if inner:
            super(Client, self)._close()
        else:
            super(Client, self).close()

        logging.info("Connection to {} closed!".format(self.copter_id))

    def remove(self):
        if self.connected:
            self.close()
        if self.clients:
            self.clients.pop(self.addr[0])
        logging.info("Client {} successfully removed!".format(self.copter_id))

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

    while True:
        pass