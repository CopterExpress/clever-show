# API

`clever-show` can be modified or used as set of modules to repurpose the software or implement different behaviors, expand functionality.


## messaging

Location: `clever-show/lib/lib.py`
`messaging` is an universal for server and clients module (running both on Python 2.7 and 3.6+). This module contains utility functions and classes implementing high level protocol for TCP socket communication.

## Utility functions

* `get_ip_address()` - returns IP address of current computer or `localhost` if no network connection present.
* `set_keepalive(sock, after_idle_sec=1, interval_sec=3, max_fails=5)` - sets `keepalive` parameters of given socket. Please note, this function is cross-platform but execution and exact effects can vary based on the platform.

### Decorators
In order to register your function as callback for message or command, you can use those decorators:

* `message_callback(action_string)`- callback decorator. Functions registered by this decorator will be called upon receiving action message with `action_string` as action.

Example:

```python
@messaging.message_callback("test_command")
  def test_command(client, *args, **kwargs):
    print(client, args, kwargs)
```

First argument passed to decorated function is instance of `ConnectionManager`, representing connection by which the message was received. Arguments and keyword arguments from message will also be passed.

* `request_callback(string_command)`- callback decorator. Functions registered by this decorator will be called upon receiving request message with `string_command` as requested value.

Example:

```python
@messaging.request_callback("time")
  def test_respose(client, *args, **kwargs):
    return time.time()
```

First argument passed to decorated function is an instance of `ConnectionManager`, representing connection by which the message was received. Arguments and keyword arguments from message will also be passed. Functions registered by this decorator must return requested value or raise an exception. Returned value will be sent back.

## `MessageManager` class

MessageManager class represents single incoming by TCP stream message and contains methods to decode and extract data from incoming data. It also contains static class methods for encoding various types of messages.

Messages in protocol implemented by this class consists of 3 parts:

* Fixed-length (2 bytes) protoheader - contains length of json header
* json header - contains information about message contents: length, encoding, byteorder, type of message and contents, etc.
* content - contains actual contents of message (json information, bytes, etc.)

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
  * `content_encoding` - optional string argument, encoding of the message content.
  * `additional_headers` - optional dict argument, additional json headers of the message.

* `create_json_message(contents, additional_headers=None)` - returns encoded message with json-encoded content in bytes.
  * `contents` - any object convertible to json, content of the message.
  * `additional_headers` - optional dict argument, additional json headers of the message.
* `create_action_message(action, args, kwargs)`  - returns encoded command with arguments as json-encoded message in bytes.
  * `action` - string, action(command) to perform upon receiving.
  * `args` - optional set, arguments for the command
  * `kwargs` - optional dict, keyword arguments for the command.
* `create_request(requested_value, request_id, args, kwargs)` - returns encoded request with arguments as json-encoded message in bytes.
  * `requested_value` - string, name of requested value.
  * `request_id` - unique ID of the request.
  * `args` - optional set, arguments for the request.
  * `kwargs` - optional dict, keyword arguments for the request.
* `create_response(requested_value, request_id, value, filetransfer=False)` - returns encoded response to request in bytes.
  * `requested_value` - string, name of requested value.
  * `request_id` - unique ID of the request.
  * `value` - returned value or bytes to send back.
  * `filetransfer` - optional boolean, whether `value` of response contains file bytes or actual value.

### `ConnectionManager` class

This class represents high-level protocol of TCP connection

### Initialization

```python
connection = ConnectionManager(whoami)
connection.connect(client_selector, client_socket, client_addr)
```

`whoami` - optional argument - `computer` or `pi`

### Public attributes

* `selector` - related selector object.
* `socket` - socket object of the connection.
* `addr` - address of the peer.
* `buffer_size` - size of the sending\receiving buffer.
* `resume_queue` - whether to resume sending queue upon peer reconnection.
* `resend_requests` - whether to resend unanswered requests in queue to reconnected client.

### Public API methods

* `connect(client_selector, client_socket, client_addr)` - establishes connection to client, writes `selector`, `socket`, `addr`attributes.
* `close()` - closes connection with peer.
* `process_events(mask)` - processes read\write events with given mask.
* `get_response(requested_value, callback, request_args=(), request_kwargs=None, callback_args=(), callback_kwargs=None)` - sends request to the client and adds it to the request queue. The callback will be called upon receiving the response (see example below).
  * `requested_value` - string, name of requested value.
  * `callback` - callable object (function, binded method, etc.) that would be called upon receiving response to this request.
  * `request_args` - optional set, arguments for the request.
  * `request_kwargs` - optional dict, keyword arguments for the request.
  * `callback_args` - optional set, arguments for the callback.
  * `callback_kwargs` - optional dict, keyword arguments for the callback.

Example of callback:

```python
def callback(client, value, *args, **kwargs):
  print(value, args, kwargs)
```

First argument passed to callback function is an instance of `ConnectionManager`, representing connection by which the message was received.
Second arguments is received value. Arguments and keyword arguments from response  will also be passed.

* `get_file(client_filepath, filepath=None, callback=None, callback_args=(), callback_kwargs=None)` - requests file from client located at `client_filepath`. Received file will be written to the `filepath` if specified.
  * `client_filepath` - string, path to file to retrieve from client.
  * `filepath` - string, path to write it upon receiving. If `None` - file will not be written.
  * `callback` - callable object (function, binded method, etc.) that would be called upon receiving response to this request (see example above). `True` will be passed as `value` keyword argument.
  * `callback_args` - optional set, arguments for the callback.
  * `callback_kwargs` - optional dict, keyword arguments for the callback.

* `send_message(action, args=(), kwargs=None)` - sends to client message with specified action,  arguments and keyword arguments.
* `send_file(filepath, dest_filepath)` - sends to client file from `filepath` to write it on `dest_filepath`.

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

### Public API methods

* `on_broadcast_bind()` - method called on binding to the server by broadcast. Override that method in order to add functionality.
* `load_config()` - loads or reloads config from file specified in `config_path`.

* `time_now()` - gets and returns system time or NTP time depending on config.
* `start()` - reloads config and starts infinite loop of connecting to the server and processing said connection. Calling of this method *will* indefinitely halt execution of any subsequent code.

### Static methods

* `get_ntp_time(ntp_host, ntp_port)` - gets and returns time from specified host and port of NTP server.
