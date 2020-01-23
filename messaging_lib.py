import io
import os
import sys
import json
import socket
import struct
import random
import logging
import threading
import collections
import traceback

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
    try:
        with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as ip_socket:
            ip_socket.connect(("8.8.8.8", 80))
            return ip_socket.getsockname()[0]
    except OSError:
        logger.warning("No network connection detected, using localhost")
        return "localhost"


class _Singleton(type):
    """ A metaclass that creates a Singleton base class when called. """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(_Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class Singleton(_Singleton('SingletonMeta', (object,), {})): pass


class MessageManager:
    def __init__(self):
        self.income_raw = b""
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
        message = cls.create_message(cls._json_encode(contents), "json", "message",
                                     additional_headers=additional_headers)
        return message

    @classmethod
    def create_action_message(cls, action, args=(), kwargs=None):
        if kwargs is None:
            kwargs = {}
        message = cls.create_json_message({"args": args, "kwargs": kwargs}, {"action": action, })
        return message

    @classmethod
    def create_request(cls, requested_value, request_id, args=(), kwargs=None):
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
        if len(self.income_raw) >= header_len:
            self._jsonheader_len = struct.unpack(">H", self.income_raw[:header_len])[0]
            self.income_raw = self.income_raw[header_len:]

    def _process_jsonheader(self):
        header_len = self._jsonheader_len
        if len(self.income_raw) >= header_len:
            self.jsonheader = self._json_decode(self.income_raw[:header_len], "utf-8")
            self.income_raw = self.income_raw[header_len:]
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
        if not len(self.income_raw) >= content_len:
            return
        data = self.income_raw[:content_len]
        self.income_raw = self.income_raw[content_len:]
        if self.jsonheader["content-type"] == "json":
            encoding = self.jsonheader["content-encoding"]
            self.content = self._json_decode(data, encoding)
        else:
            self.content = data

    def process_message(self):
        if self._jsonheader_len is None:
            self._process_protoheader()

        if self._jsonheader_len is not None:
            if self.jsonheader is None:
                self._process_jsonheader()

        if self.jsonheader:
            if self.content is None:
                self._process_content()


def message_callback(action_string):
    def inner(f):
        ConnectionManager.messages_callbacks[action_string] = f
        logger.debug("Registered message function {} for {}".format(f, action_string))

        def wrapper(*args, **kwargs):
            return f(*args, **kwargs)

        return wrapper

    return inner


def request_callback(string_command):
    def inner(f):
        ConnectionManager.requests_callbacks[string_command] = f
        logger.debug("Registered callback function {} for {}".format(f, string_command))

        def wrapper(*args, **kwargs):
            return f(*args, **kwargs)

        return wrapper

    return inner


class ConnectionManager(object):
    messages_callbacks = {}
    requests_callbacks = {}

    def __init__(self, whoami="computer"):
        self.selector = None
        self.socket = None
        self.addr = None

        self._should_close = False

        self._recv_buffer = b""
        self._send_buffer = b""

        self.whoami = whoami

        self._send_queue = collections.deque()
        self._received_queue = collections.deque()
        self._request_queue = collections.OrderedDict()

        self._send_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._close_lock = threading.Lock()

        self.buffer_size = 1024
        self.resume_queue = False
        self.resend_requests = True

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

        key = self.selector.modify(self.socket, events, data=self)
        logger.debug("Switched selector of {} to mode {}".format(self.addr, key.events))
        return key

    def connect(self, client_selector, client_socket, client_addr):
        self.selector = client_selector
        self.socket = client_socket
        self.addr = client_addr

        self._clear()

        self._set_selector_events_mask('r')
        if self.resend_requests:
            self._resend_requests()

    def _clear(self):
        if not self.resume_queue:  # maybe needs locks
            self._recv_buffer = b''
            self._send_buffer = b''
            self._received_queue.clear()
            self._send_queue.clear()

    def close(self):
        with self._close_lock:
            self._should_close = True

        self._set_selector_events_mask('w')
        NotifierSock().notify()

    def _close(self):
        logger.info("Closing connection to {}".format(self.addr))

        try:
            logger.info("Unregistering selector of {}".format(self.addr))
            self.selector.unregister(self.socket)
        except AttributeError:
            pass
        except Exception as error:
            logger.error("{}: Error during selector unregistering: {}".format(self.addr, error))
        finally:
            self.selector = None

        try:
            logger.info("Closing socket of of {}".format(self.addr))
            self.socket.close()
        except AttributeError:
            pass
        except OSError as error:
            logger.error("{}: Error during socket closing: {}".format(self.addr, error))
        finally:
            self.socket = None

        with self._close_lock:
            self._should_close = False

        self._clear()
        logger.info("CLOSED connection to {}".format(self.addr))

    def process_events(self, mask):
        with self._close_lock:
            close = self._should_close
        if close:
            self._close()
            return

        if mask & selectors.EVENT_READ:
            self.read()
        if mask & selectors.EVENT_WRITE:
            self.write()

    def read(self):
        self._read()
        while self._recv_buffer:
            if not self._received_queue or (self._received_queue[0].content is not None):
                self._received_queue.appendleft(MessageManager())

            self._received_queue[0].income_raw += self._recv_buffer
            self._recv_buffer = b''
            self._received_queue[0].process_message()

            # if something left after processing message - put it back
            if self._received_queue[0].content and self._received_queue[0].income_raw:
                self._recv_buffer = self._received_queue[0].income_raw + self._recv_buffer
                self._received_queue[0].income_raw = b''

            if self._received_queue:
                if self._received_queue[0].content:
                    self.process_received(self._received_queue.popleft())

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
        callback = self.messages_callbacks.get(action, None)
        if callback is None:
            logger.warning("Action {} does not exist!".format(action))
            return
        try:
            callback(self, *args, **kwargs)
        except Exception as error:
            logger.error("Error during action {} execution: {}".format(action, error))

    def _process_request(self, message):
        requested_value = message.content["requested_value"]
        request_id = message.content["request_id"]
        args = message.content["args"]
        kwargs = message.content["kwargs"]

        callback = self.requests_callbacks.get(requested_value, None)
        if callback is None:
            logger.warning("Request {} does not exist!".format(requested_value))
            return
        filetransfer = requested_value == "filetransfer"
        try:
            if filetransfer:
                value = self._read_file(kwargs["filepath"])
            else:
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
        self._send(MessageManager.create_action_message(action, args, kwargs))

    def _send_response(self, requested_value, request_id, value, filetransfer=False):
        self._send(MessageManager.create_response(requested_value, request_id, value, filetransfer))

    def send_file(self, filepath, dest_filepath):  # clever_restart=False
        try:
            with open(filepath, 'rb') as f:
                data = f.read()
        except OSError as error:
            logger.warning("File can not be opened due error: ".format(error))
        else:
            logger.info("Sending file {} to {} (as: {})".format(filepath, self.addr, dest_filepath))
            self._send(MessageManager.create_message(data, "binary", "message",
                       additional_headers={"action": "filetransfer", "filepath": dest_filepath}))


class NotifierSock(Singleton):
    def __init__(self):
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        self._server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self._sending_sock = socket.socket()
        self._send_lock = threading.Lock()

        self._receiving_sock = None

    def init(self, selector, port=26000):
        port += random.randint(0, 100)  # local testing fix

        self._server_socket.bind(('', port))
        self._server_socket.listen(1)
        self._sending_sock.connect(('127.0.0.1', port))
        self._receiving_sock, _ = self._server_socket.accept()
        logger.info("Notify socket: connected")

        selector.register(self._receiving_sock, selectors.EVENT_READ, data=self)
        logger.info("Notify socket: selector registered")

    def get_sock(self):
        return self._receiving_sock

    def notify(self):
        with self._send_lock:
            if self._receiving_sock is None:
                return
            self._sending_sock.sendall(bytes(1))
            logger.debug("Notify socket: notified")

    def process_events(self, mask):
        if mask & selectors.EVENT_READ and self._receiving_sock is not None:
            try:
                self._receiving_sock.recv(1024)
                logger.debug("Notify socket: received")
            except io.BlockingIOError:
                pass
            except Exception as e:
                logger.error(e)

    def close(self):
        try:
            self._server_socket.close()
            self._sending_sock.close()
            self._receiving_sock.close()
        except (OSError, KeyError) as error:
            logger.error("Error during unregistring notifier socket: {}".format(error))
