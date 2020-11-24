import io
import abc
import json
import uuid
import copy
import struct
import random
import asyncio
import functools

random.seed()

class Codec(abc.ABC):
    def encode(self, data) -> bytes:
        raise NotImplementedError

    def decode(self, data: bytes):
        raise NotImplementedError


class BytesCodec(Codec):
    def encode(self, data: bytes) -> bytes:
        return data

    def decode(self, data: bytes):
        return data

class JsonCodec(Codec):
    def __init__(self, encoding="utf-8"):
        self.encoding = encoding

    def encode(self, data) -> bytes:
        return json.dumps(data, ensure_ascii=False).encode(self.encoding)

    def decode(self, data):
        with io.TextIOWrapper(io.BytesIO(data), encoding=self.encoding, newline="") as tiow:
            obj = json.load(tiow)
        return obj


default_codec = JsonCodec()

class ContentTypes:
    BINARY = "binary"
    ENCODED = "encoded"

class MessageTypes:
    REQUEST = "request"
    RESPONSE = "response"

class ResponseTypes:
    RESULT = "result"
    ERROR = "error"
    STATUS = "status"

class MessageDecoder:
    """
    MessageManager class represents single incoming by TCP stream message and contains methods to decode and extract data from incoming data. It also contains static class methods for encoding various types of messages.

    Messages in protocol implemented by this class consists of 3 parts:

    * Fixed-length (2 bytes) protoheader - contains length of json header
    * header - contains information about message contents: length, encoding, type of message and contents, etc.
    * content - contains actual contents of message (json information, bytes, etc.)


    Attributes:
        header (dict): Headers dictionary with information about message encoding and purpose. Would be populated when receiving and processing of the json header will be completed.
        content (object): Would be populated when receiving and processing of the message will be completed. Defaults to None.
    """

    def __init__(self, data, codec=default_codec):
        """
        ```python
        message = MessageManager()
        ```
        """

        self.codec = codec
        self._income_raw = None

        self._header_len = None
        self.header = None

        self.message_type = None
        self.content_type = None
        self.chain_id = None

        self.content = None

        self.set_buffer(data)

    def _process_protoheader(self):
        header_len = 2
        if len(self._income_raw) >= header_len:
            self._header_len = struct.unpack(">H", self._income_raw[:header_len])[0]
            self._income_raw = self._income_raw[header_len:]

    def _process_header(self):
        header_len = self._header_len
        if not len(self._income_raw) >= header_len:
            return
        self.header = self.codec.decode(self._income_raw[:header_len])
        self._income_raw = self._income_raw[header_len:]

        for reqhdr in (
                "content-length",
                "content-type",
                "message-type",
                "chain-id",
        ):
            if reqhdr not in self.header:
                raise ValueError('Missing required header {}'.format(reqhdr))

        self.content_type = self.header["content-type"]
        self.message_type = self.header["message-type"]
        self.chain_id = self.header["chain-id"]

    def _process_content(self):
        content_len = self.header["content-length"]
        if not len(self._income_raw) >= content_len:
            return
        data = self._income_raw[:content_len]
        self._income_raw = self._income_raw[content_len:]

        if self.content_type == ContentTypes.ENCODED:
            self.content = self.codec.decode(data)
        elif self.content_type == ContentTypes.BINARY:
            self.content = data

    def set_buffer(self, data):
        self._income_raw = memoryview(data)

    def get_buffer(self):
        return bytearray(self._income_raw)

    def reset_buffer(self):
        self._income_raw = None

    def process_message(self):
        """
        Attempts processing the message. Chunks of `income_raw` would be consumed as different parts of the message will be processed. The result of processing (body of the message) will be available at `content` and `jsonheader`.
        """
        if self._header_len is None:
            self._process_protoheader()

        if self._header_len is not None:
            if self.header is None:
                self._process_header()

        if self.header:
            if not self.processed:
                self._process_content()

    @property
    def processed(self):
        return self.content is not None

class MessageEncoder:
    def __init__(self, codec=default_codec):
        self.chain_id = None

        self.codec = codec

    def encode(self, *args, **kwargs):
        return self.encode_raw_message(*args, **kwargs)

    def encode_raw_message(self, content: bytes, content_type, message_type, chain_id=None, additional_headers=None):
        """Returns encoded message in bytes. It is recommended use other encoding functions for general purposes.
        Args:
            content (byte string): Content of the message.
            content_type (str): Type of the message content  (json, bytes, etc.).
            message_type (str): Type of the message (action, request, etc.).
            chain_id:
            additional_headers (dict, optional): Optional dict argument, additional json headers of the message. Defaults to None.

        Returns:
            bytes: encoded message
        """

        if chain_id is None:
            chain_id = uuid.uuid4().int
        elif self.chain_id is not None:
            chain_id = self.chain_id
        else:
            self.chain_id = chain_id

        header = {
            "content-length": len(content),
            "content-type": content_type,
            "message-type": message_type,
            "chain-id": chain_id
        }
        if additional_headers:
            header.update(additional_headers)

        header_bytes = self.codec.encode(header)
        protoheader = struct.pack(">H", len(header_bytes))
        message = protoheader + header_bytes + content

        return message

    def encode_message(self, content, message_type, chain_id=None, additional_headers=None):
        """Returns encoded message with encoded content in bytes.

        Args:
            content (object): Any object convertible to json, content of the message.
            additional_headers (dict, optional): Optional dict argument, additional json headers of the message. Defaults to None.

        Returns:
            bytes: encoded message
        """
        message = self.encode_raw_message(self.codec.encode(content), ContentTypes.ENCODED, message_type, chain_id,
                                          additional_headers=additional_headers)
        return message

    # @classmethod
    # def create_action_message(cls, action, args=(), kwargs=None):
    #     """
    #     Returns encoded command with arguments as json-encoded message in bytes.
    #
    #     Args:
    #         action (str): action(command) to perform upon receiving. Should correspond with `action_string` of function registered in `message_callback()` on the peer.
    #         args (tuple, optional): Arguments for the command. Defaults to ().
    #         kwargs (dict, optional): Keyword arguments for the command. Defaults to None.
    #
    #     Returns:
    #         bytes: encoded message
    #     """
    #     if kwargs is None:
    #         kwargs = {}
    #     message = cls.create_json_message({"args": args, "kwargs": kwargs}, {"action": action, })
    #     return message
    #
    # @classmethod
    # def create_response(cls, requested_value, request_id, value, filetransfer=False):
    #     """ Returns encoded response to request in bytes.
    #
    #     Args:
    #         requested_value (str): name of requested value. Should correspond with received one.
    #         request_id (int): unique ID of the request. Should correspond with received one.
    #         value: returned value or bytes to send back.
    #         filetransfer (bool, optional):  Whether `value` of response contains file bytes or actual value.. Defaults to False.
    #
    #     Returns:
    #         bytes: encoded message
    #     """
    #     headers = {"requested_value": requested_value,
    #                "request_id": request_id,  # TODO status
    #                }
    #     if filetransfer:
    #         contents = value
    #     else:
    #         contents = cls._json_encode({"value": value, })
    #     message = cls.create_message(contents, "binary" if filetransfer else "json",
    #                                  "response", additional_headers=headers)
    #     return message

class PendingMessage(MessageEncoder):
    def __init__(self, codec=default_codec):
        super().__init__(codec)
        self._sent = asyncio.Future()

    @property
    def sent(self):
        return self._sent

    async def send(self, connection):
        if self._sent:
            raise RuntimeError("This message was already sent, create another one")
        return connection.send(self)

class Response(PendingMessage):
    def __init__(self, chain_id, result, type, codec=default_codec):
        super().__init__(codec)

        self._chain_id = chain_id
        self._result = result

    def encode(self):
        contents = {"value": self._result}
        return self.encode_message(contents, MessageTypes.RESPONSE, chain_id=self._chain_id)


class Request(PendingMessage):
    def __init__(self, name, args=(), kwargs=None, callback=None, codec=default_codec):
        super().__init__(codec)

        self._name = name
        self._args = args

        if kwargs is None:
            kwargs = {}
        self._kwargs = kwargs

        self.callback = callback
        self._response = asyncio.Future()

    @property
    def response(self):
        return self._response

    def encode(self):
        contents = {"name": self._name,
                    "args": self._args,
                    "kwargs": self._kwargs,
                    }
        return self.encode_message(contents, MessageTypes.REQUEST)

    def __copy__(self):
        return self.__class__(self._name, self._args, self._kwargs, self.callback, self.codec)

class RequestBatch:
    def __init__(self):
        self._request_dict = dict()

    def from_dict(self, d):
        self._request_dict = d.copy()

    def from_prototype(self, connections, request):
        self._request_dict = {connection: copy.copy(request) for connection in connections}

    def set_request(self, connection, request):
        self._request_dict[connection] = request

    async def send(self):
        for connection, request in self._request_dict.items():
            connection.send(request)

    @property
    def request_dict(self):
        return self._request_dict.copy()

    @property
    def response_dict(self):
        return {connection: request.response for connection, request in self._request_dict.items()}

    @property
    def all_responses(self):
        return asyncio.gather(*self.request_dict.values(), return_exceptions=True)

class ReceivedRequest:
    def __init__(self, connection, message):
        super().__init__()
        self.connection = connection
        self.message: MessageDecoder = message

    async def reply(self, data):
        reply = Response(self.message.chain_id)

    async def reply_processing(self, progress: float=0):
        progress = max(0, min(1, progress))

    async def reply_error(self, err: Exception):
        pass
