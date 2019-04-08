import sys
import time
import socket
import random
import logging
import threading
import selectors
import collections
import configparser

from messaging_lib import Message

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
PendingRequest = collections.namedtuple("PendingRequest", ["value", "requested_value",  # "expires_on",
                                                           "callback", "callback_args", "callback_kwargs"
                                                           ])


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

    # noinspection PyArgumentList
    def _client_processor(self):
        logging.info("Client processor (selector) thread started!")
        self.server_socket.listen()
        self.server_socket.setblocking(False)
        self.sel.register(self.server_socket, selectors.EVENT_READ, data=None)

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
        msg = Message.create_simple_message(
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
                message = Message()
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

    def time_now(self):
        if self.USE_NTP:
            timenow = Server.get_ntp_time(self.NTP_HOST, self.NTP_PORT)
        else:
            timenow = time.time()
        return timenow

    def send_starttime(self, copter, dt=0):
        timenow = self.time_now()
        print('Now:', time.ctime(timenow), "+ dt =", dt)
        copter.send(Message.create_simple_message("starttime", {"time": str(timenow + dt)}))  # TODO change start_on
        # TODO all commands as tasks with timestamp and priority


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
    resume_queue = True

    clients = {}

    on_connect = None  # Use as callback functions
    on_first_connect = None
    on_disconnect = None

    def __init__(self, ip):
        self.selector = None
        self.socket = None
        self.addr = None

        self._recv_buffer = b""
        self._send_buffer = b""

        self._send_queue = collections.deque()
        self._received_queue = collections.deque()
        self._request_queue = collections.OrderedDict()

        self._send_lock = threading.Lock()
        self._request_lock = threading.Lock()

        self.copter_id = None

        self.clients[ip] = self

        self.connected = False

    def _set_selector_events_mask(self, mode):
        """Set selector to listen for events: mode is 'r', 'w', 'rw'."""
        if mode == "r":
            events = selectors.EVENT_READ
        elif mode == "w":
            events = selectors.EVENT_WRITE
        elif mode == "rw":
            events = selectors.EVENT_READ | selectors.EVENT_WRITE
        else:
            raise ValueError("Invalid events mask mode {}.".format(mode))
        self.selector.modify(self.socket, events, data=self)

    def connect(self, client_selector, client_socket, client_addr):
        logging.info("Client connected")
        if not Client.resume_queue:
            self._send_queue = collections.deque()

        self.selector = client_selector
        self.socket = client_socket
        self.addr = client_addr

        self.connected = True

        if self.copter_id is None:
            self.get_response("id", self._got_id)

        if self.on_connect:
            self.on_connect(self)

    def _got_id(self):
        logging.info("Got copter id: {} for client {}".format(self.copter_id, self.addr))
        if Client.on_first_connect:
            Client.on_first_connect(self)

    def close(self):
        logging.info("Closing connection to {}".format(self.addr))
        try:
            self.selector.unregister(self.socket)
        except AttributeError:
            pass
        except Exception as error:
            logging.error("{}: Error during selector unregistering: {}".format(self.addr, error))
        finally:
            self.selector = None

        try:
            self.socket.close()
        except AttributeError:
            pass
        except OSError as error:
            logging.error("{}: Error during socket closing: {}".format(self.addr, error))
        finally:
            self.socket = None

    def process_events(self, mask):
        print(self.socket, self.selector, mask)
        if mask & selectors.EVENT_READ:
            self.read()
        if mask & selectors.EVENT_WRITE:
            self.write()

        with self._send_lock:
            if (not self._send_buffer) and self._send_queue:
                message = self._send_queue.popleft()
                self._send_buffer += message
                self._set_selector_events_mask('rw')

    def process_received(self):
        if self._received_queue:
            if self._received_queue[-1].content:
                message = self._received_queue.pop()
                message_type = message.jsonheader["message-type"]
                logging.debug("Received message! Header: {}, content: {}".format(message.jsonheader, message.content))
                if message_type == "message":
                    pass
                elif message_type == "response":
                    request_id, requested_value = message.content["requst_id"], message.content["requested_value"]
                    with self._request_lock:
                        for key, value in self._request_queue.items():
                            if (key == request_id) and (value.requested_value == requested_value):
                                request = self._request_queue.pop(key)
                                request.value = message.content["value"]
                                logging.debug(
                                    "Request successfully closed with value {}".format(message.content["value"])
                                )
                                request.callback(request.value, *request.callback_args, **request.callback_kwargs)
                                break
                        else:
                            logging.warning("Unexpected request response!")
                elif message_type == "request":
                    pass

    def read(self):
        self._read()
        if self._recv_buffer:
            if not self._received_queue or (self._received_queue[0].content is not None):
                self._received_queue.appendleft(Message())

            self._received_queue[0].income_raw += self._recv_buffer
            self._received_queue[0].process_message()

            if self._received_queue[0].content and self._received_queue[0].income_raw:
                self._recv_buffer = self._received_queue[0].income_raw + self._recv_buffer
                self._received_queue[0].income_raw = b''

            self.process_received()

    def write(self):
        self._write()
        if not (self._send_buffer and self._send_queue):
            self._set_selector_events_mask("r")

    def _read(self):
        try:
            data = self.socket.recv(Server.BUFFER_SIZE)
        except BlockingIOError:
            # Resource temporarily unavailable (errno EWOULDBLOCK)
            pass
        else:
            if data:
                self._recv_buffer += data
                logging.debug("Received {} from {}".format(data, self.addr))
            else:
                logging.warning("Connection to {} lost!".format(self.addr))
                self.connected = False
                if not Client.resume_queue:
                    self._recv_buffer = b''

                if Client.on_disconnect:
                    Client.on_disconnect(self)

                raise RuntimeError("Peer closed.")

    def _write(self):
        if self._send_buffer:
            try:
                sent = self.socket.send(self._send_buffer)
            except BlockingIOError:
                # Resource temporarily unavailable (errno EWOULDBLOCK)
                pass
            except socket.error as error:
                logging.warning("Attempt to send message {} to {} failed due error: {}".format(
                    self._send_buffer, self.addr, error))

                if not Client.resume_queue:
                    self._send_buffer = b''

                raise error
            else:
                print(sent)
                logging.debug("Sent {} to {}".format(self._send_buffer[:sent], self.addr))
                self._send_buffer = self._send_buffer[sent:]
                print(self._send_buffer)
                time.sleep(0.1)

    @requires_connect
    def send(self, *messages):
        for message in messages:
            with self._send_lock:
                self._send_queue.append(message)

    def send_message(self, command, args=None):
        self.send(Message.create_simple_message(command, args))

    @staticmethod
    @requires_any_connected
    def broadcast(message, force_all=False):
        for client in Client.clients.values():
            if client.connected or force_all:
                client.send(message)

    @classmethod
    @requires_any_connected
    def broadcast_message(cls, command, args=None, force_all=False):
        cls.broadcast(Message.create_simple_message(command, args), force_all)

    def get_response(self, requested_value, callback, request_args=None,  # timeout=30,
                     callback_args=(), callback_kwargs: dict=None):
        if request_args is None:
            request_args = {}
        if callback_kwargs is None:
            callback_kwargs = {}

        request_id = str(random.randint(0, 9999)).zfill(4)
        with self._request_lock:
            self._request_queue[request_id] = PendingRequest(
                requested_value=requested_value,
                value=None,
                # expires_on=Server.time_now()+timeout,
                callback=callback,
                callback_args=callback_args,
                callback_kwargs=callback_kwargs,
            )
        self.send(Message.create_request(requested_value, request_id, request_args))

    def send_file(self, filepath, dest_filepath):  # clever_restart=False
        try:
            with open(filepath, 'rb') as file:
                data = file.read()
        except OSError as error:
            logging.warning("File can not be opened due error: ".format(error))
        else:
            logging.info("Sending file {} to {} (as: {})".format(filepath, self.addr, dest_filepath))
            self.send(Message.create_message(data, "binary", "filetransfer", "binary", {"path": dest_filepath}))

    def send_config_options(self, *options: ConfigOption):
        logging.info("Sending config options: {} to {}".format(options, self.addr))
        for option in options:
            self.send_message(
                'config_write',
                {'section': option.section, 'option': option.option, 'value': option.value}
            )
        self.send_message("config_reload")

    @staticmethod
    def get_by_id(copter_id):
        for copter in Client.clients.values():
            if copter.copter_id == copter_id:
                return copter


if __name__ == '__main__':
    server = Server()
    server.start()
