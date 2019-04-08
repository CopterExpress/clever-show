import io
import sys
import json
import struct


class Message:
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
                    "requst_id": request_id,
                    "args": args,
                    }
        message = cls.create_message(cls._json_encode(contents), "json", "request")
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
