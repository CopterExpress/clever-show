"""
Is a client-side module containing base Client class, utility functions and basic callbacks declarations. Main focus of the module is client-specific communication without reliance on `clover` Raspberry Pi environment.
"""

import os
import sys
import time
import errno
import random
import socket
import asyncio
import logging

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.realpath(os.path.join(current_dir, os.pardir, os.pardir)))

logger = logging.getLogger(__name__)

import lib.messaging as messaging
from lib.config import ConfigManager

class ServerPeer(messaging.PeerProtocol):
    def connection_lost(self, exc):
        super().connection_lost(exc)
        if exc is not None:
            loop = asyncio.get_event_loop()
            loop.call_soon(self._parent.reconnect)
            self._parent._server_connection = None

class Client:
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

        self.callbacks = messaging.CallbackManager()

        self._server_connection: messaging.PeerProtocol = None

        self.client_id = None

        # Init configs
        self.config = ConfigManager()
        self.config_path = config_path

        self._reconnect_task = None

        self._stopping: asyncio.Future = None
        self._stopped: asyncio.Future = None

    @property
    def connected(self):
        return self._server_connection is not None

    def load_config(self):
        """
        Loads or reloads config from file specified in 'config_path' attribute.
        """
        self.config.load_config_and_spec(self.config_path)

        config_id = self.config.id.lower()
        if config_id == '/default':
            self.client_id = 'copter' + str(random.randrange(9999)).zfill(4)
            self.config.set('', 'id', self.client_id, write=True)  # set and write
        elif config_id == '/hostname':
            self.client_id = socket.gethostname()
        elif config_id == '/ip':
            self.client_id = messaging.get_ip_address()
        else:
            self.client_id = config_id

        logger.info("Config loaded")

    def time_now(self):
        """gets and returns system time or NTP time depending on the config.

        Returns:
            int: Current time.
        """
        if self.config.ntp_use:
            timenow = messaging.get_ntp_time(self.config.ntp_host, self.config.ntp_port)
        else:
            timenow = time.time()
        return timenow

    def serve_forever(self):
        asyncio.run(self.run(serve_forever=True))

    async def run(self, serve_forever=False):
        """
        Reloads config and starts infinite loop of connecting to the server and processing said connection. Calling of this method will indefinitely halt execution of any subsequent code.
        """
        loop = asyncio.get_event_loop()

        self._stopping = loop.create_future()
        self._stopped = loop.create_future()

        self.load_config()
        self.register_callbacks()

        logger.info(f"Starting client with id: '{self.client_id}' on '{socket.gethostname()}'"
                    f" ({messaging.get_ip_address()})")

        self.reconnect()

        if serve_forever:
            await self._stopped

    async def stop(self, reason: str=''):
        if self._stopping.done():
            logging.error("Client is already stopping")
            return

        self._stopping.set_result(True)
        logging.info("Stopping client")
        self._server_connection.transport.close()
        to_await = [self._server_connection.closed]

        if self._reconnect_task is not None:
            logging.info("Cancelling reconnection")
            self._reconnect_task.cancel()
            to_await.append(self._reconnect_task)

        await asyncio.gather(*to_await, return_exceptions=True)  # wait until everything shuts down

        if not self._stopped.done():
            self._stopped.set_result(True)

        logging.info(f"Client stopped: {reason}")

    def reconnect(self):
        if self._reconnect_task is not None:
            logger.warning("Reconnection task is already running")

        logger.info("Starting reconnection task")
        loop = asyncio.get_event_loop()
        self._reconnect_task = loop.create_task(self._reconnect())  # todo args

    async def _reconnect(self, attempt_limit=3, timeout=20):
        logger.info(f"Reconnection task started")

        try:
            while not self.connected:
                logger.info(f"Trying to connect to {self.config.server_host}:{self.config.server_port} ...")
                for attempt_count in range(1, attempt_limit+1):
                    logger.info(f"Waiting for connection, attempt {attempt_count}/{attempt_limit}")
                    await self._connect()
                    if self.connected:
                        return
                else:
                    logger.info("Too many attempts. Trying to get new server IP")
                    await self._broadcast_listen(timeout)
        finally:
            logging.info("Reconnection task stopped")
            self._reconnect_task = None

    async def _connect(self):
        loop = asyncio.get_event_loop()

        try:
            transport, protocol = await loop.create_connection(lambda: ServerPeer(self, self.callbacks),
                                                               host=self.config.server_host,
                                                               port=self.config.server_port,
                                                               )

        except OSError as e:
            logger.error(f"Cannot connect to server due error: {e}")
            self._server_connection = None
        else:
            logger.info("Connection to server successful!")

            messaging.set_keepalive(transport.get_extra_info('socket'))
            self._server_connection = protocol

    async def _broadcast_listen(self, listen_timeout=None):
        logging.info("Broadcast listener started")

        loop = asyncio.get_event_loop()
        try:
            transport, protocol = await loop.create_datagram_endpoint(
                lambda: messaging.BroadcastProtocol(self._on_broadcast_bind),
                local_addr=('', self.config.broadcast_port),
                family=socket.AF_INET)
        except OSError as e:
            logging.info(f"Broadcast listener exited: port is busy: {e}")
            return

        try:
            await asyncio.wait_for(protocol.closed, timeout=listen_timeout)
        except asyncio.TimeoutError:
            logging.warning("Broadcast listener timed out")
        finally:
            transport.close()
            await protocol.closed
            logging.info("Broadcast listener stopped")

    def _on_broadcast_bind(self, message: messaging.MessageManager):
        """
        Method called on binding to the server by broadcast. Override that method in order to add functionality.
        """
        kwargs = message.content["kwargs"]
        self.config.set("SERVER", "port", int(kwargs["port"]))
        self.config.set("SERVER", "host", kwargs["host"])
        self.config.write()

        logger.info(f"Got new server IP: {self.config.server_host}:{self.config.server_port}")

    def register_callbacks(self):
        @self.callbacks.action_callback("config")
        def _command_config_write(*args, **kwargs):
            mode = kwargs.get("mode", "modify")
            # exceptions would be risen in case of incorrect config
            if mode == "rewrite":
                self.config.load_from_dict(kwargs["config"],
                                           configspec=self.config_path)  # with validation
            elif mode == "modify":
                new_config = ConfigManager()
                new_config.load_from_dict(kwargs["config"])
                self.config.merge(new_config, validate=True)

            self.config.write()
            logger.info("Config successfully updated from command")
            self.load_config()

        @self.callbacks.request_callback("config")
        def _response_config(*args, **kwargs):
            send_configspec = kwargs.get("send_configspec", False)
            response = {"config": self.config.full_dict()}
            if send_configspec:
                response.update({"configspec": dict(self.config.config.configspec)})
            return response

        @self.callbacks.request_callback("clover_dir")
        def _response_clover_dir(*args, **kwargs):
            return self.config.clover_dir

        @self.callbacks.request_callback("id")
        def _response_id(*args, **kwargs):
            new_id = kwargs.get("new_id", None)
            if new_id is not None:
                self.config.set("PRIVATE", "id", new_id, True)
                self.load_config()
                # TODO renaming here

            return self.client_id

        @self.callbacks.request_callback("time")
        def _response_time(*args, **kwargs):
            return self.time_now()


if __name__ == "__main__":
    startup_cwd = os.getcwd()

    # print(Client.get_ntp_time("ntp1.stratum2.ru", 123))

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
            # t = dict([('fcu_status', None), ('current_position', [-2.89, 2.12, 3.64, 15.22, 'aruco_map']), ('animation_id', 'two_drones_test'), ('selfcheck', 'OK'), ('battery', None), ('git_version', '01bf95e'), ('calibration_status', None), ('start_position', [0.2, 0.2, 0.0]), ('mode', 'MANUAL'), ('time_delta', 1581338473.438682), ('armed', False), ('config_version', None), ('last_task', 'No task')])
            t = dict([('fcu_status', 'STANDBY'), ('current_position', [-1.17, 2.04, 3.45, 0, "11"]),
                      ('animation_id', 'two_drones_test'), ('selfcheck', 'OK'), ('battery', [12.2, 1.0]),
                      ('git_version', '42aee96'), ('calibration_status', None), ('start_position', [0.2, 0.2, 0.0]),
                      ('mode', 'MANUAL'), ('time_delta', 1581342970.889573), ('armed', False),
                      ('config_version', 'Copter config V0.0'), ('last_task', 'No task')])
            if active_client.connected:
                active_client.server_connection.send_message("telemetry", kwargs={"value": t})


    logging.basicConfig(level=logging.DEBUG)
    client = Client()
    # tr = threading.Thread(target=mock_telem)
    # tr.start()
    client.serve_forever()
    #asyncio.run(client.run())
