"""
`messaging` is an universal for server and clients module (running both on Python 2.7 and 3.6+). This module contains utility functions and classes implementing high level protocol for TCP socket communication.
"""
import os
import random
import asyncio
# import aiofiles
import logging
import traceback

import messages
import exceptions
from messages import MessageDecoder, Request, ReceivedRequest
from protocols import PeerProtocol
from utils import Callback, KeyQueue

logger = logging.getLogger("connections")

class CallbackManager:
    def __init__(self):
        self.action_callbacks = dict()
        self.request_callbacks = dict()

        self.connected_callback = None
        self.disconnected_callback = None

    @staticmethod
    def _register_function(d, key):
        def inner(f):
            if not callable(f):
                raise TypeError("f should be callable")
            d[key] = f
            logger.debug("Registered callback function {} for {}".format(f, key))
            return f

        return inner

    def action_callback(self, key):
        return self._register_function(self.action_callbacks, key)

    def request_callback(self, key):
        return self._register_function(self.request_callbacks, key)

class temp:
    def _process_received(self, message: MessageDecoder):
        message_type = message.message_type
        # content = message.content if message.content_type != "binary" else message.content[:256]
        # logger.debug(f"Received message! Header: {message.jsonheader}, content: {content}")

    def _process_message(self, message: MessageDecoder):
        if message.jsonheader["action"] == "filetransfer":
            self._write_file(message.jsonheader["filepath"], message.content)
        else:
            self._process_action(message)

    def _process_action(self, message: MessageDecoder):
        action = message.header["action"]
        args = message.content["args"]
        kwargs = message.content["kwargs"]
        try:
            callback = self._callbacks.action_callbacks[action]
        except KeyError:
            logger.warning(f"Action '{action}' does not exist!")
            return

        try:
            callback(self, *args, **kwargs)
        except Exception as error:
            logger.error("Error during action {} execution: {}".format(action, error))
            # traceback.print_exc()

    def _process_request(self, message):
        requested_value = message.content["requested_value"]
        request_id = message.content["request_id"]
        args = message.content["args"]
        kwargs = message.content["kwargs"]

        filetransfer = requested_value == "filetransfer"
        try:
            if filetransfer:
                value = self._read_file(kwargs["filepath"])
            else:
                try:
                    callback = self._callbacks.request_callbacks[requested_value]
                except KeyError:
                    logger.warning(f"Request '{requested_value}' does not exist!")
                    raise

                value = callback(self, *args, **kwargs)
        except Exception as error:  # TODO send response error\cancel
            logger.error("Error during request {} processing: {}".format(requested_value, error))
            # self._send_status_response(error.message, error.args)
        else:
            self._send_response(requested_value, request_id, value, filetransfer)

    @staticmethod
    def _read_file(filepath):
        with open(filepath, mode='rb') as f:
            return f.read()

    def _write_file(self, filepath, content):
        try:
            with open(filepath, 'wb') as f:
                f.write(content)
        except OSError as error:
            logger.error("File {} can not be written due error: {}".format(filepath, error))
        else:
            logger.info("File {} successfully received ".format(filepath))
            # if self.whoami == "pi":
            #     logger.info("Return rights to pi:pi after file transfer")
            #     os.system("chown pi:pi {}".format(filepath))
            # process = await

    # Sending api functions

class Connection:
    def __init__(self, request_callback=None):
        self.request_callback = Callback(request_callback)

        self.protocol: PeerProtocol = None
        self.transport = None

        self._sent_requests = dict()  # holds requests awaiting reply
        self._recv_requests = KeyQueue()

    def connect(self, transport, protocol: PeerProtocol):
        self.transport = transport
        self.protocol = protocol

        self.protocol.message_callback.add(self._receive)
        self.protocol.disconnected_callback.add(self._on_disconnect)

    def _on_disconnect(self, protocol):
        self._recv_requests.cancel()

    async def close(self):
        self.transport.close()
        await self.protocol.closed

    async def send(self, request: Request) -> messages.MessageDecoder:
        request.set_chain_id()
        self._sent_requests[request.chain_id] = request

        try:
            await self.protocol.send(request)
            response = await request.response
        except asyncio.TimeoutError:
            self._sent_requests.pop(request.chain_id, None)
            request.sent.cancel("Request timed out")
            request.response.set_exception(exceptions.ConnectionClosedError)
            raise
        else:
            return response

    async def send_request(self, name, args=(), kwargs=None, callback=None):
        request = Request(name, args, kwargs, callback)
        return await self.send(request)

    async def send_response(self, request, data=None, progress=None, err=None):
        pass

    async def receive(self, name: str =None) -> messages.ReceivedRequest:
        try:
            return await self._recv_requests.get(key=name)
        except asyncio.CancelledError:
            raise exceptions.ConnectionClosedError

    def _receive(self, protocol, msg):
        if msg.message_type == messages.MessageTypes.RESPONSE:
            self._process_response(msg)
        elif msg.message_type == messages.MessageTypes.REQUEST:
            self._process_request(msg)
        else:
            logger.warning(f"Got unknown message type {msg.message_type} for message {msg}")
            return True  # put msg in protocol rcv queue
        return False  # don't put msg in protocol rcv queue

    def _process_response(self, response: MessageDecoder):
        logger.debug(f"Received response {response} with id {response.chain_id}")

        try:
            request = self._sent_requests.pop(response.chain_id)
        except KeyError:
            logger.error(f"Unexpected response {response} with id {response.chain_id}")
            return

        try:
            response_type = response.header["response-type"]
        except KeyError:
            logger.error("Missing 'response-type' header in response")
            return

        if response_type == messages.ResponseTypes.RESULT:
            result = response.content
            request.response.set_result(result)
        elif response_type == messages.ResponseTypes.ERROR:  # TODOOOO
            error = None
            request.response.set_result(error)
        elif response_type == messages.ResponseTypes.STATUS:
            pass
        else:
            logger.error(f"Unknown response type '{response_type}' in response {response}")

    def _process_request(self, request_msg):
        request = ReceivedRequest(self, request_msg)
        logger.debug(f"Received request {request_msg} with id {request_msg.chain_id} for {request.name}")

        put_msg = self.request_callback.gather_bool(self, request)
        if put_msg:
            self._recv_requests.put_nowait(request, key=request.name)

async def connect(port, host, connection_factory=Connection, protocol_factory=PeerProtocol, **kwargs):
    loop = asyncio.get_event_loop()

    transport, protocol = await loop.create_connection(protocol_factory, host=host, port=port, **kwargs)
    connection = connection_factory()
    connection.connect(transport, protocol)
    return connection

async def create_server(port, host="", connection_factory=Connection, protocol_factory=PeerProtocol,
                        reuse_connections=True, **kwargs):
    loop = asyncio.get_event_loop()
    new_connections = asyncio.Queue()

    def factory(addr=None):
        nonlocal new_connections
        protocol = protocol_factory()

        def callback(p):
            nonlocal new_connections
            connection = connection_factory()
            connection.connect(p.transport, p)
            new_connections.put_nowait(connection)

        protocol.connected_callback.add(callback)

        return protocol

    server = await loop.create_server(factory, host=host, port=port, **kwargs)

    return server, new_connections.get()


class LegacyConnectionManager:
    """
    This class represents high-level protocol of TCP connection.

    Attributes:
        addr (str): Address of the peer.
        buffer_size (int): Size of the sending/receiving buffer.
        resume_queue (bool): Whether to resume sending queue upon peer reconnection.
        resend_requests (bool): Whether to resend unanswered requests in queue to reconnected client.
    """

    def __init__(self, callbacks, whoami="computer"):
        """
        Args:
            whoami (str, optional): What type of system the ConnectionManager is running on (`computer` or `pi`). Defaults to "computer".
        
        Example:

        ```python
        connection = ConnectionManager(whoami)
        connection.connect(client_selector, client_socket, client_addr)
        ```
        """
        self.whoami = whoami


    def _process_response(self, message):
        if requested_value == "filetransfer":
            value = True
            self._process_filetransfer(message.content, request.callback_kwargs["filepath"])
            logger.debug(
                "Request {} successfully closed with file bytes {}...".format(request, message.content[:256])
            )
        else:
            value = message.content["value"]
            logger.debug(
                "Request {} successfully closed with value {}".format(request, message.content["value"])
            )
        if request.callback is not None:
            try:
                request.callback(self, value, *request.callback_args, **request.callback_kwargs)
            except Exception as error:
                logger.error("Error during response {} processing: {}".format(request, error))
        else:
            logger.info("No callback were registered for response: {}".format(request))

    def get_response(self, requested_value, callback,  # timeout=30,
                     request_args=(), request_kwargs=None,
                     callback_args=(), callback_kwargs=None, ):
        """
        Sends request to the client and adds it to the request queue. The callback will be called upon receiving the response (see example below).

        Args:
            requested_value (string): Name of requested value.
            callback (function): Callable object (function, binded method, etc.) that would be called upon receiving response to this request or None.
            request_args (tuple, optional): Arguments for the request. Defaults to ().
            request_kwargs (dict, optional): Keyword arguments for the request. Defaults to None.
            callback_args (tuple, optional): Arguments for the callback. Defaults to ().
            callback_kwargs (dict, optional): Keyword arguments for the callback. Defaults to None.

        Example of a callback:

        ```python
        def callback(client, value, *args, **kwargs):
            print(value, args, kwargs)
        ```

        First argument passed to callback function is an instance of `ConnectionManager`, representing connection by which the message was received.
        Second arguments is received value. Arguments and keyword arguments from response  will also be passed.
        """

    def get_file(self, client_filepath, filepath=None, callback=None,
                 callback_args=(), callback_kwargs=None, ):
        """
        Requests file from peer located at `client_filepath`. Received file will be written to the `filepath` if specified.

        Args:
            client_filepath (str): Path to file to retrieve from peer.
            filepath (str, optional): Path to write file upon receiving. If `None` - file will not be written. Defaults to None.
            callback (function): Callable object (function, binded method, etc.) that would be called upon receiving response to this request or None.
            callback_args (tuple, optional): Arguments for the callback. Defaults to ().
            callback_kwargs (dict, optional): Keyword arguments for the callback. Defaults to None.

        """
        if callback_kwargs is None:
            callback_kwargs = {}

        if filepath is None:
            filepath = os.path.split(client_filepath)[1]

        request_kwargs = {"filepath": client_filepath}
        callback_kwargs.update({"filepath": filepath})

        self.get_response("filetransfer", callback, request_kwargs=request_kwargs,
                          callback_args=callback_args, callback_kwargs=callback_kwargs)

    def send_file(self, filepath, dest_filepath):  # clever_restart=False
        """
        Sends to peer a file from `filepath` to write it on `dest_filepath`.

        Args:
            filepath (str): Path of the file to send.
            dest_filepath (str): Path on peer where recieved file will be written to.
        """
        try:
            with open(filepath, 'rb') as f:
                data = f.read()
        except OSError as error:
            logger.warning("File can not be opened due error: ".format(error))
        else:
            logger.info("Sending file {} to {} (as: {})".format(filepath, self.addr, dest_filepath))
            self._send(Message.create_message(data, "binary", "message",
                                              additional_headers={"action": "filetransfer",
                                                                         "filepath": dest_filepath}))
