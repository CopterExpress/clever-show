# Module [lib.messaging](../../../lib/messaging.py)

`messaging` is an universal for server and clients module (running both on Python 2.7 and 3.6+). This module contains utility functions and classes implementing high level protocol for TCP socket communication.

## Module functions
### [get\_ip\_address](../../../lib/messaging.py#L42)
```py
def get_ip_address()
```
Returns the IP address of current computer or `localhost` if no network connection present.

Returns:
    string: IP address of current computer or `localhost` if no network connection present
### [message\_callback](../../../lib/messaging.py#L312)
```py
def message_callback(action_string)
```
In order to register your function as callback for message or command, you can use those decorators:

Callback decorator. Functions registered by this decorator will be called upon receiving action message.

Args:
    action_string (str): Functions registered by this decorator will be called upon receiving action message with `action_string` as action.

Example:

```python
@messaging.message_callback("test_command")
def test_command(client, *args, **kwargs):
    print(client, args, kwargs)
```

First argument passed to decorated function is instance of `ConnectionManager`, representing connection by which the message was received. Arguments and keyword arguments from message will also be passed.
### [request\_callback](../../../lib/messaging.py#L343)
```py
def request_callback(string_command)
```
In order to register your function as callback for message or command, you can use those decorators:

Callback decorator. Functions registered by this decorator will be called upon receiving request message.

Args:
    string_command (str): Functions registered by this decorator will be called upon receiving request message with `string_command` as requested value.

Example:

```python
@messaging.request_callback("time")
def test_respose(client, *args, **kwargs):
    return time.time()
```

First argument passed to decorated function is an instance of `ConnectionManager`, representing connection by which the message was received. Arguments and keyword arguments from message will also be passed. Functions registered by this decorator must return requested value or raise an exception. Returned value will be sent back.
### [set\_keepalive](../../../lib/messaging.py#L58)
```py
def set_keepalive(sock, after_idle_sec=1, interval_sec=3, max_fails=5)
```
Sets `keepalive` parameters of given socket.

Args:
    sock (socket): Socket which parameters will be changed.
    after_idle_sec (int, optional): Start sending keepalive packets after this amount of seconds. Defaults to 1.
    interval_sec (int, optional): Interval of keepalive packets in seconds. Defaults to 3.
    max_fails (int, optional): Count of fails leading to socket disconnect. Defaults to 5.

Raises:
    NotImplementedError: for unknown platform.
## Class [ConnectionManager](../../../lib/messaging.py#L374)
This class represents high-level protocol of TCP connection.

Attributes:
    selector (selector): Related selector object.
    socket (socket): Socket object of the connection.
    addr (str): Address of the peer.
    buffer_size (int): Size of the sending/receiving buffer.
    resume_queue (bool): Whether to resume sending queue upon peer reconnection.
    resend_requests (bool): Whether to resend unanswered requests in queue to reconnected client.
### [\_\_init\_\_](../../../lib/messaging.py#L389)
```py
def __init__(self, whoami='computer')
```
Args:
    whoami (str, optional): What type of system the ConnectionManager is running on (`computer` or `pi`). Defaults to "computer".

Example:

```python
connection = ConnectionManager(whoami)
connection.connect(client_selector, client_socket, client_addr)
```
### [close](../../../lib/messaging.py#L464)
```py
def close(self)
```
Closes connection with the peer.
### [connect](../../../lib/messaging.py#L439)
```py
def connect(self, client_selector, client_socket, client_addr)
```
[summary]

Args:
    client_selector (selector): Related selector object.
    client_socket (socket): Socket object of the connection.
    client_addr (str): Address of the peer.
### [get\_file](../../../lib/messaging.py#L735)
```py
def get_file(self, client_filepath, filepath=None, callback=None, callback_args=(), callback_kwargs=None)
```
Requests file from peer located at `client_filepath`. Received file will be written to the `filepath` if specified.

Args:
    client_filepath (str): Path to file to retrieve from peer.
    filepath (str, optional): Path to write file upon receiving. If `None` - file will not be written. Defaults to None.
    callback (function): Callable object (function, binded method, etc.) that would be called upon receiving response to this request or None.
    callback_args (tuple, optional): Arguments for the callback. Defaults to ().
    callback_kwargs (dict, optional): Keyword arguments for the callback. Defaults to None.
### [get\_response](../../../lib/messaging.py#L691)
```py
def get_response(self, requested_value, callback, request_args=(), request_kwargs=None, callback_args=(), callback_kwargs=None)
```
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
### [process\_events](../../../lib/messaging.py#L503)
```py
def process_events(self, mask)
```
Processes read\write events with given mask.

Args:
    mask (bytes): mask of the selector events.
### [process\_received](../../../lib/messaging.py#L556)
```py
def process_received(self, message)
```

### [read](../../../lib/messaging.py#L520)
```py
def read(self)
```

### [send\_file](../../../lib/messaging.py#L783)
```py
def send_file(self, filepath, dest_filepath)
```
Sends to peer a file from `filepath` to write it on `dest_filepath`.

Args:
    filepath (str): Path of the file to send.
    dest_filepath (str): Path on peer where recieved file will be written to.
### [send\_message](../../../lib/messaging.py#L769)
```py
def send_message(self, action, args=(), kwargs=None)
```
Sends to peer message with specified action,  arguments and keyword arguments.

Args:
    action (str): action(command) to perform upon receiving. Should correspond with `action_string` of function registered in `message_callback()` on the peer.
    args (tuple, optional): Arguments for the command. Defaults to ().
    kwargs (dict, optional): Keyword arguments for the command. Defaults to None.
### [write](../../../lib/messaging.py#L657)
```py
def write(self)
```



## Class [MessageManager](../../../lib/messaging.py#L116)
MessageManager class represents single incoming by TCP stream message and contains methods to decode and extract data from incoming data. It also contains static class methods for encoding various types of messages.

Messages in protocol implemented by this class consists of 3 parts:

* Fixed-length (2 bytes) protoheader - contains length of json header
* json header - contains information about message contents: length, encoding, byteorder, type of message and contents, etc.
* content - contains actual contents of message (json information, bytes, etc.)


Attributes:
    income_raw (bytes string): Holds incoming data bytes. Append incoming data to this attribute. It may not be empty after processing.
    jsonheader (dict): Headers dictionary with information about message encoding and purpose. Would be populated when receiving and processing of the json header will be completed.
    content (object): Would be populated when receiving and processing of the message will be completed. Defaults to None.
### [\_\_init\_\_](../../../lib/messaging.py#L132)
```py
def __init__(self)
```
```python
message = MessageManager()
```
### [process\_message](../../../lib/messaging.py#L296)
```py
def process_message(self)
```
Attempts processing the message. Chunks of `income_raw` would be consumed as different parts of the message will be processed. The result of processing (body of the message) will be available at `content` and `jsonheader`.


## Class [Namespace](../../../lib/messaging.py#L25)
...
### [\_\_init\_\_](../../../lib/messaging.py#L26)
```py
def __init__(self, **kwargs)
```
Initialize self.  See help(type(self)) for accurate signature.


## Class [NotifierSock](../../../lib/messaging.py#L802)
Singleton base class.
### [\_\_init\_\_](../../../lib/messaging.py#L803)
```py
def __init__(self)
```
Initialize self.  See help(type(self)) for accurate signature.
### [close](../../../lib/messaging.py#L846)
```py
def close(self)
```

### [get\_sock](../../../lib/messaging.py#L826)
```py
def get_sock(self)
```

### [init](../../../lib/messaging.py#L814)
```py
def init(self, selector, port=26000)
```

### [notify](../../../lib/messaging.py#L829)
```py
def notify(self)
```

### [process\_events](../../../lib/messaging.py#L836)
```py
def process_events(self, mask)
```



## Class [PendingRequest](../../../lib/messaging.py#L36)
...
### [\_\_init\_\_](../../../lib/messaging.py#L26)
```py
def __init__(self, **kwargs)
```
Initialize self.  See help(type(self)) for accurate signature.


## Class [Singleton](../../../lib/messaging.py#L109)
Singleton base class.


