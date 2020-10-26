import os
import sys
import time

import socket
import asyncio
import random
import logging
import datetime
import collections
import traceback

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.realpath(os.path.join(current_dir, os.pardir, os.pardir)))

# Import modules from lib dir
import lib.messaging as messaging
from lib.config import ConfigManager

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


class Server:
    def __init__(self, config_path=os.path.join(current_dir, os.pardir, "config", "server.ini"), server_id=None):
        self.id = server_id if server_id else str(random.randint(0, 9999)).zfill(4)
        self.time_started = 0

        self._tcp_server = None
        self.callbacks = messaging.CallbackManager()
        self._clients = dict()


        self.host = socket.gethostname()
        self.ip = messaging.get_ip_address()

        self.config = ConfigManager()
        self.config_path = config_path

        self._broadcast_send_task = None
        self._broadcast_listen_task = None

    def load_config(self):
        self.config.load_config_and_spec(self.config_path)

    async def start(self):
        loop = asyncio.get_event_loop()
        loop.set_debug(True)

        self.time_started = time.time()

        # load config on startup
        self.load_config()  # TODO async

        if self.config.broadcast_send:
            self._broadcast_send_task = loop.create_task(self._broadcast_send())

        if self.config.broadcast_listen:
            self._broadcast_send_task = loop.create_task(self._broadcast_listen())

        logging.info(f"Starting server with id: {self.id} on {self.host} ({self.ip})")

        self._tcp_server = await loop.create_server(asyncio.Protocol(),
                                                    port=self.config.server_port,
                                                    reuse_address=True,
                                                    start_serving=False)
        for sock in self._tcp_server.sockets:  # to handle multiple interfaces (IPv4\IPv6)
            self._configure_sock(sock)
            sockname = sock.getsockname()
            logging.info(f"Running server socket on {sockname[0]} : {sockname[1]}")

        logging.info("Starting serving")
        await self._tcp_server.start_serving()

    async def stop(self):
        logging.info("Stopping server")
        self._tcp_server.close()
        to_await = [self._tcp_server.wait_closed()]

        if self._broadcast_send_task is not None:
            logging.info("Cancelling broadcast sending")
            self._broadcast_send_task.cancel()
            to_await.append(self._broadcast_send_task)

        if self._broadcast_listen_task is not None:
            logging.info("Cancelling broadcast listening")
            self._broadcast_listen_task.cancel()
            to_await.append(self._broadcast_listen_task)

        await asyncio.gather(*to_await, return_exceptions=True)
        logging.info("Server stopped")

    def terminate(self, reason="Terminated"):
        self.stop()
        logging.critical(reason)

    def time_now(self):
        if self.config.ntp_use:
            return messaging.get_ntp_time(self.config.ntp_host, self.config.ntp_port)

        return time.time()

    @staticmethod
    def _configure_sock(sock):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        messaging.set_keepalive(sock)

    def _connect_client(self, sock):


        if not any(client_addr == addr[0] for client_addr in Client.clients.keys()):
            client = Client(self.callbacks, addr[0])
            client.buffer_size = self.config.server_buffer_size
            logging.info("New client")
        else:
            client = Client.clients[addr[0]]
            client.close(True)  # to ensure in unregistering
            logging.info("Reconnected client")
        self.sel.register(conn, selectors.EVENT_READ, data=client)
        client.connect(self.sel, conn, addr)

    async def _broadcast_send(self):
        logging.info("Broadcast sender task started!")
        msg = messaging.MessageManager.create_action_message(
            "server_ip", kwargs={"host": self.ip, "port": self.config.server_port,
                                 "id": self.id, "start_time": self.time_started})
        await asyncio.sleep(0)
        logging.debug(
            f"Formed broadcast message to {self.config.broadcast_send_ip}:{self.config.broadcast_port}: {msg}")

        broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        while True:
            try:
                await asyncio.sleep(self.config.broadcast_delay)
                broadcast_sock.sendto(msg, (self.config.broadcast_send_ip, self.config.broadcast_port))
            except OSError as e:
                logging.error(f"Cannot send broadcast due error {e}")
            except asyncio.CancelledError:
                print("cans")
                raise
            else:
                logging.debug("Broadcast sent")

    async def _broadcast_listen(self):
        logging.info("Broadcast listener task started!")
        broadcast_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_client.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        loop = asyncio.get_event_loop()
        transport, protocol = await loop.create_datagram_endpoint(messaging.BroadcastProtocol,
                                                                  local_addr=('', self.config.broadcast_port))
        # broadcast_client.settimeout(1)
        # try:
        #     broadcast_client.bind(("", self.config.broadcast_port))
        # except OSError:
        #     self.terminate("Another server is running on this computer, shutting down!")
        #     return
        #
        # finally:
        #     broadcast_client.close()
        #     logging.info("Broadcast listener thread stopped, socked closed!")

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


class Client(messaging.ConnectionManager):
    clients = {}

    on_connect = None  # Use as callback functions
    on_first_connect = None
    on_disconnect = None

    def __init__(self, callbacks, ip):
        super().__init__(callbacks)
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

        # if self.copter_id is None:
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
    loop = asyncio.get_event_loop()
    loop.set_debug(True)
    print(loop)

    try:
        loop.run_until_complete(server.start())
        loop.run_until_complete(asyncio.sleep(4))
        loop.run_until_complete(server.stop())
        print(3)
    finally:
        print(1)
    print(4)
    #loop.run_forever()


