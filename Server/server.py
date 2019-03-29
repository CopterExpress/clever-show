import os
import sys
import math
import time
import json
import struct
import socket
import random
import logging
import threading
import collections
import configparser

# All imports sorted in pyramid just because

random.seed()

logging.basicConfig(  # TODO all prints as logs
    level=logging.INFO,
    format="%(asctime)s [%(name)-7.7s] [%(threadName)-19.19s] [%(levelname)-7.7s]  %(message)s",
    handlers=[
        logging.FileHandler("server_logs.log"),
        logging.StreamHandler()
    ])


class ConfigOption:
    def __init__(self, section, option, value):
        self.section = section
        self.option = option
        self.value = value


class Server:
    BUFFER_SIZE = 1024

    def __init__(self, server_id=None, config_path="server_config.ini"):

        self.id = server_id if server_id else str(random.randint(0, 9999)).zfill(4)

        # Init socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.host = socket.gethostname()
        self.ip = Server.get_ip_address()

        # Init configs
        self.config_path = config_path
        self.config = configparser.ConfigParser()
        self.config.read(self.config_path)
        self.load_config()

        # Init threads
        self.autoconnect_thread = threading.Thread(target=self._auto_connect, daemon=True,
                                                   name='Client auto-connect')
        self.autoconnect_thread_running = threading.Event()  # Can be used for manual thread killing

        self.broadcast_thread = threading.Thread(target=self._ip_broadcast, daemon=True,
                                                 name='IP broadcast sender')
        self.broadcast_thread_running = threading.Event()

        self.listener_thread = threading.Thread(target=self._broadcast_listen, daemon=True,
                                                name='IP broadcast listener')
        self.listener_thread_running = threading.Event()

    def load_config(self):
        self.port = int(self.config['SERVER']['port'])
        self.broadcast_port = int(self.config['SERVER']['broadcast_port'])
        self.BROADCAST_DELAY = int(self.config['SERVER']['broadcast_delay'])
        Server.BUFFER_SIZE = int(self.config['SERVER']['buffer_size'])

        self.USE_NTP = self.config.getboolean('NTP', 'use_ntp')
        self.NTP_HOST = self.config['NTP']['host']
        self.NTP_PORT = int(self.config['NTP']['port'])

    def start(self):  # do_auto_connect=True, do_ip_broadcast=True, do_listen_broadcast=False
        logging.info("Starting server with id: {} on {} !".format(self.id, self.ip))
        logging.info("Starting server socket!")
        self.server_socket.bind((self.ip, self.port))

        logging.info("Starting client autoconnect thread!")
        self.autoconnect_thread_running.set()
        self.autoconnect_thread.start()

        logging.info("Starting broadcast sender thread!")
        self.broadcast_thread_running.set()
        self.broadcast_thread.start()

        logging.info("(not) Starting broadcast listener thread!")
        self.listener_thread_running.set()
        # listener_thread.start()

    def stop(self):
        logging.info("Stopping server")
        self.autoconnect_thread_running.clear()
        self.broadcast_thread_running.clear()
        self.listener_thread_running.clear()
        self.server_socket.close()
        logging.info("Server stopped")

    @staticmethod
    def get_ip_address():
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ip_socket:
            ip_socket.connect(("8.8.8.8", 80))
            return ip_socket.getsockname()[0]

    @staticmethod
    def get_ntp_time(ntp_host, ntp_port):
        NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
        NTP_QUERY = b'\x1b' + bytes(47)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ntp_socket:
            ntp_socket.sendto(NTP_QUERY, (ntp_host, ntp_port))
            msg, _ = ntp_socket.recvfrom(1024)
        return int.from_bytes(msg[-8:], 'big') / 2 ** 32 - NTP_DELTA

    def _auto_connect(self):
        logging.info("Client autoconnect thread started!")
        while self.autoconnect_thread_running.is_set():
            self.server_socket.listen(1)
            c, addr = self.server_socket.accept()
            logging.info("Got connection from: {}".format(str(addr)))
            if not any(client_addr == addr[0] for client_addr in Client.clients.keys()):
                client = Client(addr[0])
                logging.info("New client")
            else:
                logging.info("Reconnected client")
            Client.clients[addr[0]].connect(c, addr)
        logging.info("Client autoconnect thread stopped!")

    def _ip_broadcast(self):
        logging.info("Broadcast sender thread started!")
        msg = bytes(Client.form_message(
            "server_ip", {"host": self.ip, "port": str(self.port), "id": self.id}
        ), "UTF-8")
        broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        logging.info("Formed broadcast message: {}".format(msg))

        while self.broadcast_thread_running.is_set():
            time.sleep(self.BROADCAST_DELAY)
            broadcast_sock.sendto(msg, ('255.255.255.255', self.broadcast_port))
            logging.debug("Broadcast sent")
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

        while self.listener_thread_running.is_set():
            data, addr = broadcast_client.recvfrom(1024)
            command, args = Client.parse_message(data.decode("UTF-8"))
            if command == "server_ip":
                if args["id"] != self.id:
                    logging.critical("Another server detected on network, shutting down")
                    sys.exit()
        broadcast_client.close()
        logging.info("Broadcast listener thread stopped, socked closed!")

    def send_starttime(self, copter, dt=0):
        if self.USE_NTP:
            timenow = Server.get_ntp_time(self.NTP_HOST, self.NTP_PORT)
        else:
            timenow = time.time()
        print('Now:', time.ctime(timenow), "+ dt =", dt)
        copter.send(Client.form_message("starttime", {"time": str(timenow + dt)}))


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


class Client:
    resume_quee = True

    clients = {}

    on_connect = None  # Use as callback functions
    on_first_connect = None
    on_disconnect = None

    def __init__(self, ip):
        self.socket = None
        self.addr = None

        self._send_queue = collections.deque()
        self._received_queue = collections.deque()
        self._request_queue = collections.OrderedDict()

        self._send_lock = threading.Lock()
        self._request_lock = threading.Lock()

        self.copter_id = None
        self.selected = False  # Use to select copters for certain purposes DEPRECATED

        Client.clients[ip] = self

        self.connected = False

    def connect(self, client_socket, client_addr):
        print("Client connected")
        if not Client.resume_quee:
            self._send_queue = collections.deque()

        self.socket = client_socket
        self.addr = client_addr

        self.socket.setblocking(0)
        self.connected = True
        client_thread = threading.Thread(target=self._run, name="Client {} thread".format(self.addr))
        client_thread.start()
        if self.copter_id is None:
            self.copter_id = self.get_response("id")
            print("Got copter id:", self.copter_id)
            if Client.on_first_connect:
                Client.on_first_connect(self)

        if Client.on_connect:
            Client.on_connect(self)

    def _send_all(self, msg):
        self.socket.sendall(struct.pack('>I', len(msg)) + msg)

    def _receive_all(self, n):
        data = b''
        while len(data) < n:
            packet = self.socket.recv(min(n - len(data), Server.BUFFER_SIZE))
            print("Receiving packet {}; full data is {}".format(packet, data))
            if not packet:
                return None
            data += packet
        return data

    def _receive_message(self):
        raw_msglen = self._receive_all(4)
        if not raw_msglen:
            print("No valid msg")
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        msg = self._receive_all(msglen)
        return msg

    def _run(self):
        while self.connected:
            try:
                if self._send_queue:
                    with self._send_lock:
                        msg = self._send_queue.popleft()
                    try:
                        self._send_all(msg)
                        print("Send", msg, "to", self.addr)
                    except socket.error as e:
                        logging.warning("Attempt to send failed: {}".format(e))
                        with self._send_lock:
                            self._send_queue.appendleft(msg)
                        raise e

                try:  # check if data in buffer
                    check = self.socket.recv(Server.BUFFER_SIZE, socket.MSG_PEEK)
                    if check:
                        received = self._receive_message()
                        if received:
                            received = received.decode("UTF-8")
                            print("Received", received, "from", self.addr)
                            command, args = Client.parse_message(received)
                            if command == "response":
                                with self._request_lock:
                                    for key, value in self._request_queue.items():
                                        if not value and key == args["value_name"]:
                                            self._request_queue[key] = args['value']
                                            print("Request successfully closed")
                                            break
                                    else:
                                        print("Unexpected request")
                            else:
                                self._received_queue.appendleft(received)
                except socket.error:
                    pass

            except socket.error as e:
                logging.warning("Client error: {}, disconnected".format(e))
                self.connected = False
                self.socket.close()
                if Client.on_disconnect:
                    Client.on_disconnect(self)
                break
            # time.sleep(0.05)

    @staticmethod
    def form_message(command: str, dict_arguments: dict = None):
        if dict_arguments is None:
            dict_arguments = {}
        msg_dict = {command: dict_arguments}
        msg = json.dumps(msg_dict)
        return msg

    @staticmethod
    def parse_message(msg):
        try:
            j_message = json.loads(msg)
        except ValueError:
            print("Json string not in correct format")
            return None, None
        str_command = list(j_message.keys())[0]
        dict_arguments = list(j_message.values())[0]

        return str_command, dict_arguments

    @requires_connect
    def send(self, *messages):
        for message in messages:
            with self._send_lock:
                self._send_queue.append(bytes(message, "UTF-8"))

    @requires_connect
    def get_response(self, requested_value):
        with self._request_lock:
            self._request_queue[requested_value] = ""
        self.send(Client.form_message("request", {"value": requested_value}))

        while not self._request_queue[requested_value]:
            pass

        with self._request_lock:
            return self._request_queue.pop(requested_value)

    @requires_connect
    def send_file(self, filepath, dest_filename, clever_restart = False):
        print("Sending file ", dest_filename)
        chunk_count = math.ceil(os.path.getsize(filepath) / Server.BUFFER_SIZE)
        self.send(Client.form_message("writefile", {"filesize": chunk_count, "filename": dest_filename, "clever_restart": clever_restart}))
        with open(filepath, 'rb') as file:
            chunk = file.read(Server.BUFFER_SIZE)
            while chunk:
                with self._send_lock:
                    self._send_queue.append(chunk)
                chunk = file.read(Server.BUFFER_SIZE)

        self.send(Client.form_message("/endoffile"))  # TODO mb remove
        print("File sent")

    @staticmethod
    @requires_any_connected
    def send_to_selected(message):  # DEPRECATED
        for client in Client.clients.values():
            if client.connected and client.selected:
                client.send(message)

    @staticmethod
    @requires_any_connected
    def request_to_selected(requested_value):  # DEPRECATED
        for client in Client.clients.values():
            if client.connected and client.selected:
                client.get_response(requested_value)

    @staticmethod
    @requires_any_connected
    def broadcast(message, force_all=False):
        for client in Client.clients.values():
            if client.connected or force_all:
                client.send(message)

    def send_config_options(self, *options: ConfigOption):
        for option in options:
            self.send(
                Client.form_message('config_write',
                                    {'section': option.section, 'option': option.option, 'value': option.value}))
        self.send(Client.form_message("config_reload"))

    @staticmethod
    def get_by_id(copter_id):
        for copter in Client.clients.values():
            if copter.copter_id == copter_id:
                return copter


if __name__ == '__main__':
    server = Server()
    server.start()
