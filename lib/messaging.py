"""
`messaging` is an universal for server and clients module (running both on Python 2.7 and 3.6+). This module contains utility functions and classes implementing high level protocol for TCP socket communication.
"""
import io
import os
import sys
import json
import socket
import struct
import random
import asyncio
import logging
import platform
import traceback
import collections

from contextlib import closing

try:
    import selectors
except ImportError:
    import selectors2 as selectors


class Namespace:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __getitem__(self, key):
        return self.__dict__[key]

    def __setitem__(self, key, value):
        self.__dict__[key] = value


class PendingRequest(Namespace): pass


logger = logging.getLogger(__name__)


def get_ip_address():
    """
    Returns the IP address of current computer or `localhost` if no network connection present.

    Returns:
        string: IP address of current computer or `localhost` if no network connection present
    """    
    try:
        with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as ip_socket:
            ip_socket.connect(("8.8.8.8", 80))
            return ip_socket.getsockname()[0]
    except OSError:
        logger.warning("No network connection detected, using localhost")
        return "localhost"


def get_ntp_time(ntp_host, ntp_port):
    NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
    NTP_QUERY = b'\x1b' + bytes(47)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ntp_socket:
        ntp_socket.sendto(NTP_QUERY, (ntp_host, ntp_port))
        msg, _ = ntp_socket.recvfrom(1024)
    return int.from_bytes(msg[-8:], 'big') / 2 ** 32 - NTP_DELTA


def set_keepalive(sock, after_idle_sec=1, interval_sec=3, max_fails=5):
    """
    Sets `keepalive` parameters of given socket.

    Args:
        sock (socket): Socket which parameters will be changed.
        after_idle_sec (int, optional): Start sending keepalive packets after this amount of seconds. Defaults to 1.
        interval_sec (int, optional): Interval of keepalive packets in seconds. Defaults to 3.
        max_fails (int, optional): Count of fails leading to socket disconnect. Defaults to 5.

    Raises:
        NotImplementedError: for unknown platform.
    """     
    current_platform = platform.system()  # could be empty
    
    if current_platform == "Linux":
        return _set_keepalive_linux(sock, after_idle_sec, interval_sec, max_fails)
    if current_platform == "Windows":
        return _set_keepalive_windows(sock, after_idle_sec, interval_sec)
    if current_platform == "Darwin":
        return _set_keepalive_osx(sock, interval_sec)

    raise NotImplementedError

def _set_keepalive_linux(sock, after_idle_sec, interval_sec, max_fails):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, after_idle_sec)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, interval_sec)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, max_fails)

def _set_keepalive_windows(sock, after_idle_sec, interval_sec):
    sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, after_idle_sec*1000, interval_sec*1000))

def _set_keepalive_osx(sock, interval_sec):
    TCP_KEEPALIVE = 0x10
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    sock.setsockopt(socket.IPPROTO_TCP, TCP_KEEPALIVE, interval_sec)


class BroadcastProtocol:
    def __init__(self, on_broadcast):
        self._on_broadcast = on_broadcast

    def datagram_received(self, data, addr):
        message = MessageManager()
        message.append_data(data)
        message.process_message()
        content = message.content

        is_ip_broadcast = (content is not None and message.jsonheader["action"] == "server_ip")

        if is_ip_broadcast:
            asyncio.get_event_loop().call_soon(self._on_broadcast, message)
            # different_id = content["kwargs"]["id"] != str(self.id)
            # self_younger = float(content["kwargs"]["start_time"]) <= self.time_started
        else:
            logging.warning("Got wrong broadcast message from {}".format(addr))

    # def error_received(self, exc):
    #     logging.error("Error received")

class BroadcastSendProtocol:
    pass

class MessageManager:
    """
    MessageManager class represents single incoming by TCP stream message and contains methods to decode and extract data from incoming data. It also contains static class methods for encoding various types of messages.

    Messages in protocol implemented by this class consists of 3 parts:

    * Fixed-length (2 bytes) protoheader - contains length of json header
    * json header - contains information about message contents: length, encoding, byteorder, type of message and contents, etc.
    * content - contains actual contents of message (json information, bytes, etc.)


    Attributes:
        _income_raw (bytes string): Holds incoming data bytes. Append incoming data to this attribute. It may not be empty after processing.
        jsonheader (dict): Headers dictionary with information about message encoding and purpose. Would be populated when receiving and processing of the json header will be completed.
        content (object): Would be populated when receiving and processing of the message will be completed. Defaults to None.
    """
    def __init__(self):
        """
        ```python
        message = MessageManager()
        ```
        """
        self._income_raw = b"" #todo bytearrray
        self._jsonheader_len = None
        self.jsonheader = None
        self.content = None

    @staticmethod
    def _json_encode(obj, encoding="utf-8"):
        return json.dumps(obj, ensure_ascii=False).encode(encoding)

    @staticmethod
    def _json_decode(json_bytes, encoding="utf-8"):
        with io.TextIOWrapper(io.BytesIO(json_bytes), encoding=encoding, newline="") as tiow:
            obj = json.load(tiow, object_pairs_hook=collections.OrderedDict)
        return obj

    @classmethod
    def create_message(cls, content_bytes, content_type, message_type, content_encoding="utf-8",
                       additional_headers=None):
        """Returns encoded message in bytes. It is recommended use other encoding functions for general purposes.
        Args:
            content_bytes (byte string): Content of the message.
            content_type (str): Type of the message content  (json, bytes, etc.).
            message_type (str): Type of the message (command, request, etc.).
            content_encoding (str, optional): Encoding of the message content. Defaults to "utf-8".
            additional_headers (dict, optional): Optional dict argument, additional json headers of the message. Defaults to None.

        Returns:
            bytes: encoded message
        """

        jsonheader = {
            "byteorder": sys.byteorder,
            "content-type": content_type,
            "content-encoding": content_encoding,
            "content-length": len(content_bytes),
            "message-type": message_type,
        }
        if additional_headers:
            jsonheader.update(additional_headers)

        jsonheader_bytes = cls._json_encode(jsonheader, "utf-8")
        message_hdr = struct.pack(">H", len(jsonheader_bytes))
        message = message_hdr + jsonheader_bytes + content_bytes
        return message

    @classmethod
    def create_json_message(cls, contents, additional_headers=None):
        """Returns encoded message with json-encoded content in bytes.

        Args:
            contents (object): Any object convertible to json, content of the message.
            additional_headers (dict, optional): Optional dict argument, additional json headers of the message. Defaults to None.

        Returns:
            bytes: encoded message
        """
        message = cls.create_message(cls._json_encode(contents), "json", "message",
                                     additional_headers=additional_headers)
        return message

    @classmethod
    def create_action_message(cls, action, args=(), kwargs=None):
        """
        Returns encoded command with arguments as json-encoded message in bytes.

        Args:
            action (str): action(command) to perform upon receiving. Should correspond with `action_string` of function registered in `message_callback()` on the peer.
            args (tuple, optional): Arguments for the command. Defaults to ().
            kwargs (dict, optional): Keyword arguments for the command. Defaults to None.

        Returns:
            bytes: encoded message
        """
        if kwargs is None:
            kwargs = {}
        message = cls.create_json_message({"args": args, "kwargs": kwargs}, {"action": action, })
        return message

    @classmethod
    def create_request(cls, requested_value, request_id, args=(), kwargs=None):
        """Returns encoded request with arguments as json-encoded message in bytes.

        Args:
            requested_value (str): name of requested value. Should correspond with `string_command` of function registered in `request_callback()` on the peer. 
            request_id (int): unique ID of the request.
            args (tuple, optional): Arguments for the request.. Defaults to ().
            kwargs (dict, optional): Keyword arguments for the request. Defaults to None.

        Returns:
            bytes: encoded message
        """
        if kwargs is None:
            kwargs = {}
        contents = {"requested_value": requested_value,
                    "request_id": request_id,
                    "args": args,
                    "kwargs": kwargs,
                    }
        message = cls.create_message(cls._json_encode(contents), "json", "request")
        return message

    @classmethod
    def create_response(cls, requested_value, request_id, value, filetransfer=False):
        """ Returns encoded response to request in bytes.

        Args:
            requested_value (str): name of requested value. Should correspond with received one.
            request_id (int): unique ID of the request. Should correspond with received one.
            value: returned value or bytes to send back.
            filetransfer (bool, optional):  Whether `value` of response contains file bytes or actual value.. Defaults to False.

        Returns:
            bytes: encoded message
        """
        headers = {"requested_value": requested_value,
                   "request_id": request_id,  # TODO status
                   }
        if filetransfer:
            contents = value
        else:
            contents = cls._json_encode({"value": value, })
        message = cls.create_message(contents, "binary" if filetransfer else "json",
                                     "response", additional_headers=headers)
        return message

    def _process_protoheader(self):
        header_len = 2
        if len(self._income_raw) >= header_len:
            self._jsonheader_len = struct.unpack(">H", self._income_raw[:header_len])[0]
            self._income_raw = self._income_raw[header_len:]

    def _process_jsonheader(self):
        header_len = self._jsonheader_len
        if len(self._income_raw) >= header_len:
            self.jsonheader = self._json_decode(self._income_raw[:header_len], "utf-8")
            self._income_raw = self._income_raw[header_len:]
            for reqhdr in (
                    "byteorder",
                    "content-length",
                    "content-type",
                    "content-encoding",
                    "message-type",
            ):
                if reqhdr not in self.jsonheader:
                    raise ValueError('Missing required header {}'.format(reqhdr))

    def _process_content(self):
        content_len = self.jsonheader["content-length"]
        if not len(self._income_raw) >= content_len:
            return
        data = self._income_raw[:content_len]
        self._income_raw = self._income_raw[content_len:]
        if self.jsonheader["content-type"] == "json":
            encoding = self.jsonheader["content-encoding"]
            self.content = self._json_decode(data, encoding)
        else:
            self.content = data

    def append_data(self, data):
        self._income_raw += data

    def get_leftovers(self):
        return self._income_raw

    def process_message(self):
        """
        Attempts processing the message. Chunks of `income_raw` would be consumed as different parts of the message will be processed. The result of processing (body of the message) will be available at `content` and `jsonheader`.
        """
        if self._jsonheader_len is None:
            self._process_protoheader()

        if self._jsonheader_len is not None:
            if self.jsonheader is None:
                self._process_jsonheader()

        if self.jsonheader:
            if self.content is None:
                self._process_content()

class CallbackManager:
    def __init__(self):
        self.action_callbacks = dict()
        self.request_callbacks = dict()

        self.connected_callback = None

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

class PeerProtocol:
    def __init__(self, parent, calabacks):
        self._parent = parent
        self._callbacks = calabacks

        self._recv_msg_queue = asyncio.Queue

    def connection_made(self, transport):
        peername = transport.get_extra_info('peername')

        self._resend_requests()

        self.transport = transport


    def data_received(self, data):

        while self._recv_buffer:
            # add new message object if queue is empty or last message already processed
            if self._recv_msg_queue.empty() or (self._received_queue[0].content is not None):
                self._recv_msg_queue.put(MessageManager())

            last_message = self._received_queue[0]

            last_message.income_raw += self._recv_buffer
            self._recv_buffer = b''
            last_message.process_message()

            # if something left after processing message - put it back
            if last_message.content is not None and last_message.income_raw:
                self._recv_buffer = last_message.income_raw + self._recv_buffer
                last_message.income_raw = b''

            if self._received_queue and last_message.content is not None:
                self.process_received(self._received_queue.popleft())

    def connection_lost(self, exc):
        pass
    #        logger.info("Closing connection to {}".format(self.addr))


class ConnectionManager(object):
    """
    This class represents high-level protocol of TCP connection.

    Attributes:
        selector (selector): Related selector object.
        socket (socket): Socket object of the connection.
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
        self.callbacks = callbacks

        self.selector = None
        self.socket = None
        self.addr = None

        self._should_close = False

        self._recv_buffer = b""
        self._send_buffer = b""

        self.whoami = whoami


    def _clear(self):
        if not self.resume_queue:  # maybe needs locks
            self._recv_buffer = b''
            self._send_buffer = b''
            self._received_queue.clear()
            self._send_queue.clear()



    def read(self):
        self._read()


    def _read(self):
        try:
            data = self.socket.recv(self.buffer_size)
        except io.BlockingIOError:
            # Resource temporarily unavailable (errno EWOULDBLOCK)
            pass
        else:
            if data:
                self._recv_buffer += data
                logger.debug("Received {} bytes from {}".format(len(data), self.addr))
            else:
                logger.warning("Connection to {} lost!".format(self.addr))

                raise RuntimeError("Peer closed.")

    def process_received(self, message):
        message_type = message.jsonheader["message-type"]
        content = message.content if message.jsonheader["content-type"] != "binary"\
            else message.content[:256]
        logger.debug(
            "Received message! Header: {}, content: {}".format(message.jsonheader, content))

        if message_type == "message":
            self._process_message(message)
        elif message_type == "response":
            self._process_response(message)
        elif message_type == "request":
            self._process_request(message)

    def _process_message(self, message):
        if message.jsonheader["action"] == "filetransfer":
            self._process_filetransfer(message.content, message.jsonheader["filepath"])
        else:
            self._process_action(message)

    def _process_action(self, message):
        action = message.jsonheader["action"]
        args = message.content["args"]
        kwargs = message.content["kwargs"]
        callback = self.callbacks.action_callbacks.get(action, None)
        if callback is None:
            logger.warning("Action {} does not exist!".format(action))
            return
        try:
            callback(self, *args, **kwargs)
        except Exception as error:
            logger.error("Error during action {} execution: {}".format(action, error))
            traceback.print_exc()

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
                callback = self.callbacks.request_callbacks.get(requested_value, None)
                if callback is None:
                    logger.warning("Request {} does not exist!".format(requested_value))
                    return

                value = callback(self, *args, **kwargs)
        except Exception as error:  # TODO send response error\cancel
            logger.error("Error during request {} processing: {}".format(requested_value, error))
        else:
            self._send_response(requested_value, request_id, value, filetransfer)

    def _process_response(self, message):
        request_id, requested_value = message.jsonheader["request_id"], message.jsonheader["requested_value"]

        with self._request_lock:
            request = self._request_queue.pop(request_id, None)
        if (request is None) or (request.requested_value != requested_value):
            logger.warning("Unexpected response!")
            return

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

    @staticmethod
    def _read_file(filepath):
        with open(filepath, mode='rb') as f:
            return f.read()

    def _process_filetransfer(self, content, filepath):
        try:
            with open(filepath, 'wb') as f:
                f.write(content)
        except OSError as error:
            logger.error("File {} can not be written due error: {}".format(filepath, error))
        else:
            logger.info("File {} successfully received ".format(filepath))
            if self.whoami == "pi":
                logger.info("Return rights to pi:pi after file transfer")
                os.system("chown pi:pi {}".format(filepath))

    def write(self):
        with self._send_lock:
            if (not self._send_buffer) and self._send_queue:
                message = self._send_queue.popleft()
                self._send_buffer += message
        if self._send_buffer:
            self._write()
        else:
            self._set_selector_events_mask('r')  # we're done writing

    def _write(self):
        try:
            sent = self.socket.send(self._send_buffer[:self.buffer_size])
        except io.BlockingIOError:
            # Resource temporarily unavailable (errno EWOULDBLOCK)
            pass
        except Exception as error:
            logger.warning(
                "Attempt to send message {} to {} failed due error: {}".format(self._send_buffer, self.addr, error))

            raise error
        else:
            self._send_buffer = self._send_buffer[sent:]
            left = len(self._send_buffer)
            logger.debug("Sent message to {}: sent {} bytes, {} bytes left.".format(self.addr, sent, left))

    def _send(self, data):
        with self._send_lock:
            self._send_queue.append(data)

        if self.selector.get_key(self.socket).events != selectors.EVENT_WRITE:
            self._set_selector_events_mask('rw')
            NotifierSock().notify()

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
        if request_kwargs is None:
            request_kwargs = {}
        if callback_kwargs is None:
            callback_kwargs = {}

        request_id = str(random.randint(0, 9999)).zfill(4)  # maybe hash
        with self._request_lock:
            self._request_queue[request_id] = PendingRequest(
                requested_value=requested_value,
                value=None,
                # expires_on=Server.time_now()+timeout, #TODO
                callback=callback,
                callback_args=callback_args,
                callback_kwargs=callback_kwargs,
                request_args=request_args,
                request_kwargs=request_kwargs,
                resend=True,
            )
        self._send(MessageManager.create_request(requested_value, request_id, request_args, request_kwargs))

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

    def _resend_requests(self):
        with self._request_lock:
            for request_id, request in self._request_queue.items():  # TODO filter
                if request.resend:
                    self._send(MessageManager.create_request(
                        request.requested_value, request_id, request.request_kwargs.update(resend=request.resend))
                    )
                    request.resend = False

    def send_message(self, action, args=(), kwargs=None):
        """
        Sends to peer message with specified action,  arguments and keyword arguments.

        Args:
            action (str): action(command) to perform upon receiving. Should correspond with `action_string` of function registered in `message_callback()` on the peer.
            args (tuple, optional): Arguments for the command. Defaults to ().
            kwargs (dict, optional): Keyword arguments for the command. Defaults to None.
        """
        self._send(MessageManager.create_action_message(action, args, kwargs))

    def _send_response(self, requested_value, request_id, value, filetransfer=False):
        self._send(MessageManager.create_response(requested_value, request_id, value, filetransfer))

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
            self._send(MessageManager.create_message(data, "binary", "message",
                       additional_headers={"action": "filetransfer", "filepath": dest_filepath}))