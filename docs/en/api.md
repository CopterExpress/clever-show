# API

Clever-show can be

## messaging

Location: `clever-show/lib/lib.py`
`messaging` is an universal for server and clients module (running both on Python 2.7 and 3.6+). This module contains utility functions and classes implementing high level protocol for TCP socket communication.

## Utility functions

* `get_ip_address()` - returns IP address of current computer or `localhost` if no network connection present.
* `set_keepalive(sock, after_idle_sec=1, interval_sec=3, max_fails=5)` - sets `keepalive` parameters of given socket. Please note, this function is cross-platform but execution and exact effects can vary based on the platform.

## `MessageManager` class

MessageManager class represents single incoming by TCP stream message and contains methods to decode and extract data from incoming data. It also contains static class methods for encoding various types of messages.

### Initialization

```python
message = MessageManager()
```

### Public attributes

* `income_raw` - bytes string, append incoming data to this attribute.
* `content` - string (by default `None`). Would be populated when receiving and processing of the message will be completed.

#### Public API methods

* `process_message()` - calling this method will lead to attempt of processing the message. Chunks of `income_raw` would be consumed as different parts of the message will be processed. The result of processing (body of the message) will be available at `content`.

#### Static class methods

* `create_message(content_bytes, content_type, message_type, content_encoding, additional_headers)` - returns encoded message in bytes. It is recommended use other encoding functions for general purposes.
  * `content_bytes` - bytes string, content of the message.
  * `content_type` - string, type of the message content  (json, bytes, etc.).
  * `message_type` - string, type of the message.
  * `content_encoding` - optional string argument, encoding of the message content
  * `additional_headers` - optional dict argument, additional json headers of the message

## client_core

Location: `clever-show/drone/modules/client_core.py`
`client_core` is a client-side module (meant to be run on Python 2.7) containing base Client class, utility functions and basic callbacks declarations. Main focus of the module is client-specific communication without reliance on `clover` Raspberry Pi environment.

## `Client` class

Client base class provides config loading, communication with server (including automatic reconnection, broadcast listening and binding). You can inherit this class in order to extend functionality for practical applications.

### Initialization

```python
client = Client(config_path)
```

config_path - string optional argument. Path to the file with configuration.  There also should be config specification file at `config_path\config\configspec_client.ini`.

### Public attributes

* `server_connection` - ConnectionManager object representing connation to the server.
* `connected` - read-only boolean, whether client is connected to the server.
* `client_id` - read-only string, ID of the client
* `config` - ConfigManager object, containing loaded client configuration.
* `config_path` - path to configuration file. There also should be config specification file at `config_path\config\configspec_client.ini`.

#### Public API methods

* `on_broadcast_bind()` - method called on binding to the server by broadcast. Override that method in order to add functionality.
* `load_config()` - loads or reloads config from file specified in `config_path`.

* `time_now()` - gets and returns system time or NTP time depending on config.
* `start()` - reloads config and starts infinite loop of connecting to the server and processing said connection. Calling of this method *will* indefinitely halt execution of any subsequent code.

#### Static methods

* `get_ntp_time(ntp_host, ntp_port)` - gets and returns time from specified NTP server.
