import io
import sys
import json
import struct
import random
import logging
import threading
import collections

try:
    import selectors
except:
    import selectors2 as selectors

PendingRequest = collections.namedtuple("PendingRequest", ["value", "requested_value",  # "expires_on",
                                                           "callback", "callback_args", "callback_kwargs"
                                                           ])


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
            obj = json.load(tiow)
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
    def create_json_message(cls, contents):
        message = cls.create_message(cls._json_encode(contents), "json", "message")
        return message

    @classmethod
    def create_simple_message(cls, command, args=None):
        if args is None:
            args = {}
        message = cls.create_json_message({"command": command, "args": args})
        return message

    @classmethod
    def create_request(cls, requested_value, request_id, args=None):
        if args is None:
            args = {}
        contents = {"requested_value": requested_value,
                    "request_id": request_id,
                    "args": args,
                    }
        message = cls.create_message(cls._json_encode(contents), "json", "request")
        return message

    @classmethod
    def create_response(cls, requested_value, request_id, value):
        contents = {"requested_value": requested_value,
                    "request_id": request_id,
                    "value": value,
                    }
        message = cls.create_message(cls._json_encode(contents), "json", "response")
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


def message_callback(string_command):
    def inner(f):
        ConnectionManager.requests_callbacks.update({string_command: f})

        def wrapper(*args, **kwargs):
            return f(*args, **kwargs)
        return wrapper
    return inner


class ConnectionManager(object):
    messages_callbacks = {}
    requests_callbacks = {}

    def __init__(self):
        self.selector = None
        self.socket = None
        self.addr = None

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

        self.BUFFER_SIZE = 1024
        self.resume_queue = True

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
        self.selector = client_selector
        self.socket = client_socket
        self.addr = client_addr

        self._set_selector_events_mask('rw')

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

    def read(self):
        self._read()
        if self._recv_buffer:
            if not self._received_queue or (self._received_queue[0].content is not None):
                self._received_queue.appendleft(MessageManager())

            self._received_queue[0].income_raw += self._recv_buffer
            self._received_queue[0].process_message()

            if self._received_queue[0].content and self._received_queue[0].income_raw:
                self._recv_buffer = self._received_queue[0].income_raw + self._recv_buffer
                self._received_queue[0].income_raw = b''

        self.process_received()

    def _read(self):
        try:
            data = self.socket.recv(self.BUFFER_SIZE)
        except io.BlockingIOError:
            # Resource temporarily unavailable (errno EWOULDBLOCK)
            pass
        else:
            if data:
                self._recv_buffer += data
                logging.debug("Received {} from {}".format(data, self.addr))
            else:
                logging.warning("Connection to {} lost!".format(self.addr))
                if not self.resume_queue:
                    self._recv_buffer = b''

                raise RuntimeError("Peer closed.")

    def process_received(self):
        if self._received_queue:
            if self._received_queue[-1].content:
                message = self._received_queue.pop()
                message_type = message.jsonheader["message-type"]
                logging.debug("Received message! Header: {}, content: {}".format(message.jsonheader, message.content))
                if message_type == "message":
                    self._process_message(message)
                elif message_type == "response":
                    self._process_response(message)
                elif message_type == "request":
                    self._process_request(message)
                elif message_type == "filetransfer":
                    self._process_filetransfer(message)

    def _process_message(self, message):
        command = message.content["command"]
        args = message.content["args"]
        self.messages_callbacks[command](**args)

    def _process_request(self, message):
        command = message.content["requested_value"]
        request_id = message.content["request_id"]
        args = message.content["args"]
        value = self.requests_callbacks[command](**args)
        self._send_response(command, request_id, value)

    def _process_response(self, message):
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
                logging.warning("Unexpected  response!")

    def _process_filetransfer(self, message):
        if message.jsonheader["content-type"] == "binary":
            filepath = message.jsonheader["filepath"]
            try:
                with open(filepath, 'wb') as f:
                    f.write(message.content)
            except OSError as error:
                logging.warning("File can not be written due error: ".format(error))
            else:
                logging.info("File {} successfully received ".format(filepath))

    def write(self):
        with self._send_lock:
            if (not self._send_buffer) and self._send_queue:
                message = self._send_queue.popleft()
                self._send_buffer += message
        if self._send_buffer:
            self._write()

    def _write(self):
        try:
            sent = self.socket.send(self._send_buffer)
        except io.BlockingIOError:
            # Resource temporarily unavailable (errno EWOULDBLOCK)
            pass
        except Exception as error:
            logging.warning("Attempt to send message {} to {} failed due error: {}".format(
                self._send_buffer, self.addr, error))

            if not self.resume_queue:
                self._send_buffer = b''

            raise error
        else:
            logging.debug("Sent {} to {}".format(self._send_buffer[:sent], self.addr))
            self._send_buffer = self._send_buffer[sent:]

    def _send(self, data):
        with self._send_lock:
            self._send_queue.append(data)

    def get_response(self, requested_value, callback, request_args=None,  # timeout=30,
                     callback_args=(), callback_kwargs=None):
        if request_args is None:
            request_args = {}
        if callback_kwargs is None:
            callback_kwargs = {}

        request_id = str(random.randint(0, 9999)).zfill(4)
        with self._request_lock:
            self._request_queue[request_id] = PendingRequest(
                requested_value=requested_value,
                value=None,
                # expires_on=Server.time_now()+timeout, #TODO
                callback=callback,
                callback_args=callback_args,
                callback_kwargs=callback_kwargs,
            )
        self._send(MessageManager.create_request(requested_value, request_id, request_args))

    def send_message(self, command, args=None):
        self._send(MessageManager.create_simple_message(command, args))

    def _send_response(self, requested_value, request_id, value):
        self._send(MessageManager.create_response(requested_value, request_id, value))

    def send_file(self, filepath, dest_filepath):  # clever_restart=False
        try:
            with open(filepath, 'rb') as f:
                data = f.read()
        except OSError as error:
            logging.warning("File can not be opened due error: ".format(error))
        else:
            logging.info("Sending file {} to {} (as: {})".format(filepath, self.addr, dest_filepath))
            self._send(MessageManager.create_message(
                data, "binary", "filetransfer", "binary", {"filepath": dest_filepath}
            ))