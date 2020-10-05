import os
import sys
import time
import socket
import random
import logging
import datetime
import threading
import selectors
import collections
import traceback

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.realpath(os.path.join(current_dir, os.pardir, os.pardir, 'lib')))

# Import modules from lib dir
import messaging
from config import ConfigManager

random.seed()

now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

log_path = os.path.join(current_dir, os.pardir, "server_logs")
if not os.path.exists(log_path):
    try:
        os.mkdir(log_path)
    except OSError:
        print("Creation of the directory {} failed".format(log_path))
    else:
        print("Successfully created the directory {}".format(log_path))

logger = logging.getLogger(__name__)

class Server(messaging.Singleton):
    def __init__(self, config_path=os.path.join(current_dir, os.pardir, "config", "server.ini"), server_id=None):
        self.id = server_id if server_id else str(random.randint(0, 9999)).zfill(4)
        self.time_started = 0

        # Init socket
        self.sel = selectors.DefaultSelector()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        messaging.set_keepalive(self.server_socket)
        self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self.host = socket.gethostname()
        self.ip = messaging.get_ip_address()

        # Init configs
        self.config = ConfigManager()
        self.config_path = config_path

        # Init threads
        self.autoconnect_thread = threading.Thread(target=self._client_processor, daemon=True,
                                                   name='Client processor')
        self.client_processor_thread_running = threading.Event()  # Can be used for manual thread killing

        self.broadcast_thread = threading.Thread(target=self._ip_broadcast, daemon=True,
                                                 name='IP broadcast sender')
        self.broadcast_thread_running = threading.Event()  # TODO replace by interrupt
        self.broadcast_thread_interrupt = threading.Event()

        self.listener_thread = threading.Thread(target=self._broadcast_listen, daemon=True,
                                                name='IP broadcast listener')
        self.listener_thread_running = threading.Event()

    def load_config(self):
        self.config.load_config_and_spec(self.config_path)

    def start(self):
        # load config on startup
        self.load_config()

        self.time_started = time.time()

        logging.info("Starting server with id: {} on {}:{} ({})!".format(self.id, self.ip, self.config.server_port,
                                                                         socket.gethostname()))
        logging.info("Binding server socket!")
        self.server_socket.bind((self.ip, self.config.server_port))

        logging.info("Starting client processor thread!")
        self.client_processor_thread_running.set()
        self.autoconnect_thread.start()

        if self.config.broadcast_send:
            logging.info("Starting broadcast sender thread!")
            self.broadcast_thread_running.set()
            self.broadcast_thread.start()

        if self.config.broadcast_listen:
            logging.info("Starting broadcast listener thread!")
            self.listener_thread_running.set()
            self.listener_thread.start()

    def stop(self):
        logging.info("Stopping server")

        self.client_processor_thread_running.clear()

        self.broadcast_thread_interrupt.set()
        self.broadcast_thread_running.clear()

        self.listener_thread_running.clear()

        messaging.NotifierSock().notify()

        self.server_socket.close()
        self.sel.close()

        messaging.NotifierSock().close()

        logging.info("Server stopped")

    def terminate(self, reason="Terminated"):
        self.stop()
        logging.critical(reason)

    @staticmethod
    def get_ntp_time(ntp_host, ntp_port):
        NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
        NTP_QUERY = b'\x1b' + bytes(47)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ntp_socket:
            ntp_socket.sendto(NTP_QUERY, (ntp_host, ntp_port))
            msg, _ = ntp_socket.recvfrom(1024)
        return int.from_bytes(msg[-8:], 'big') / 2 ** 32 - NTP_DELTA

    def time_now(self):
        if self.config.ntp_use:
            return self.get_ntp_time(self.config.ntp_host, self.config.ntp_port)

        return time.time()

    # noinspection PyArgumentList
    def _client_processor(self):
        logging.info("Client processor (selector) thread started!")

        messaging.NotifierSock().init(self.sel)

        self.server_socket.listen()
        self.server_socket.setblocking(False)
        self.sel.register(self.server_socket, selectors.EVENT_READ, data=None)

        while self.client_processor_thread_running.is_set():
            events = self.sel.select(timeout=1)
            for key, mask in events:
                client = key.data
                if client is None:
                    self._connect_client(key.fileobj)
                elif isinstance(client, messaging.ConnectionManager):
                    try:
                        client.process_events(mask)
                    except Exception as error:
                        logging.error("Exception {} occurred for {}! Resetting connection!".format(error, client.addr))
                        traceback.print_exc()
                        client.close(True)
                else:  # Notifier
                    client.process_events(mask)

        logging.info("Client autoconnect thread stopped!")

    def _connect_client(self, sock):
        try:
            conn, addr = sock.accept()
        except OSError:
            logging.error("Error while connecting socket!")
            return

        logging.info("Got connection from: {}".format(str(addr)))
        conn.setblocking(False)

        if not any([client_addr == addr[0] for client_addr in Client.clients.keys()]):
            client = Client(addr[0])
            client.buffer_size = self.config.server_buffer_size
            logging.info("New client")
        else:
            client = Client.clients[addr[0]]
            client.close(True)  # to ensure in unregistering
            logging.info("Reconnected client")
        self.sel.register(conn, selectors.EVENT_READ, data=client)
        client.connect(self.sel, conn, addr)

    def _ip_broadcast(self):
        logging.info("Broadcast sender thread started!")
        msg = messaging.MessageManager.create_action_message(
            "server_ip", kwargs={"host": self.ip, "port": str(self.config.server_port), "id": self.id,
                                 "start_time": str(self.time_started)})
        logging.debug("Formed broadcast message to {}:{}: {}".format(self.config.broadcast_send_ip, self.config.broadcast_port, msg))

        broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        try:
            while self.broadcast_thread_running.is_set():
                self.broadcast_thread_interrupt.wait(timeout=self.config.broadcast_delay)
                try:
                    broadcast_sock.sendto(msg, (self.config.broadcast_send_ip, self.config.broadcast_port))
                except OSError as e:
                    logging.error(f"Cannot send broadcast due error {e}")
                else:
                    logging.debug("Broadcast sent")
        except Exception as e:
            logging.error(f"Unexpected error {e}!")
            raise

    def _broadcast_listen(self):
        logging.info("Broadcast listener thread started!")
        broadcast_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        # broadcast_client.settimeout(1)
        try:
            broadcast_client.bind(("", self.config.broadcast_port))
        except OSError:
            self.terminate("Another server is running on this computer, shutting down!")
            return

        try:
            while self.listener_thread_running.is_set():
                try:
                    data, addr = broadcast_client.recvfrom(1024)  # TODO nonblock
                except OSError:
                    logging.error(f"Cannot receive broadcast due error {e}")
                    continue

                message = messaging.MessageManager()
                message.income_raw = data
                message.process_message()
                content = message.content

                right_command = (content and message.jsonheader["action"] == "server_ip")

                if right_command:
                    different_id = content["kwargs"]["id"] != str(self.id)
                    self_younger = float(content["kwargs"]["start_time"]) <= self.time_started

                    if different_id and self_younger:
                        # younger server should shut down
                        self.terminate("Another server detected over the network, shutting down!")

                else:
                    logging.warning("Got wrong broadcast message from {}".format(addr))

        except Exception as e:
            logging.error(f"Unexpected error {e}!")
            raise

        finally:
            broadcast_client.close()
            logging.info("Broadcast listener thread stopped, socked closed!")

    def send_starttime(self, copter, start_time):
        copter.send_message("start", kwargs={"time": str(start_time)})


def requires_connect(f):
    def wrapper(*args, **kwargs):
        if args[0].connected:
            return f(*args, **kwargs)
        logging.warning("Function requires client to be connected!")

    return wrapper


def requires_any_connected(f):
    def wrapper(*args, **kwargs):
        if Client.clients:
            return f(*args, **kwargs)
        logging.warning("No clients were connected!")

    return wrapper

# TODO do a factory class for clients\connection managers with common properties
class Client(messaging.ConnectionManager):
    clients = {}

    on_connect = None  # Use as callback functions
    on_first_connect = None
    on_disconnect = None

    def __init__(self, ip):
        super().__init__()
        self.copter_id = None
        self.clover_dir = None
        self.connected = False

        self.clients[ip] = self

    @staticmethod
    def get_by_id(copter_id):
        for client in Client.clients.values():  # TODO filter
            if client.copter_id == copter_id:
                return client

    def connect(self, client_selector, client_socket, client_addr):
        logging.info("Client connected")
        if not self.resume_queue:
            self._send_queue = collections.deque()

        super().connect(client_selector, client_socket, client_addr)

        self.connected = True

        #if self.copter_id is None:
        self.get_response("id", self._got_id)

        if self.on_connect:
            self.on_connect(self)

    def _got_id(self, _client, value):  # TODO make as regular comand
        logging.info("Got copter id: {} for client {}".format(value, self.addr))
        old_id = self.copter_id
        self.copter_id = value

        if old_id is None:
            self.get_response("clover_dir", self._got_clover_dir)

        if old_id is None and self.on_first_connect:  # TODO merge
            self.on_first_connect(self)

    def _got_clover_dir(self, _client, value):
        self.clover_dir = value

    def close(self, inner=False):
        self.connected = False

        if self.on_disconnect:
            self.on_disconnect(self)

        if inner:
            super()._close()
        else:
            super().close()

        logging.info("Connection to {} closed!".format(self.copter_id))

    def remove(self):
        if self.connected:
            self.close()

        try:
            self.clients.pop(self.addr[0])
        except KeyError as e:
            logging.error(e)

        logging.info("Client {} successfully removed!".format(self.copter_id))

    @requires_connect
    def _send(self, data):
        super()._send(data)
        logging.debug("Queued data to send (first 256 bytes): {}".format(data[:256]))

    @staticmethod
    @requires_any_connected
    def broadcast(message, force_all=False):
        for client in Client.clients.values():
            if client.connected or force_all:
                client._send(message)

    @classmethod
    @requires_any_connected
    def broadcast_message(cls, command, args=(), kwargs=None, force_all=False):
        cls.broadcast(messaging.MessageManager.create_action_message(command, args, kwargs), force_all)


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(name)-7.7s] [%(threadName)-19.19s] [%(levelname)-7.7s]  %(message)s",
        handlers=[
            logging.FileHandler(os.path.join(log_path, "{}.log".format(now))),
            logging.StreamHandler()
        ])

    server = Server()
    server.start()

    while True:
        pass
