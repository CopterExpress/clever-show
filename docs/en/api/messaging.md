# Module [lib.messaging](..\..\..\lib\messaging.py)
## Module functions
### [get\_ip\_address](..\..\..\lib\messaging.py#L39)
```py
def get_ip_address()
```
Returns the IP address of current computer or `localhost` if no network connection present.

Returns:
    string: IP address of current computer or `localhost` if no network connection present
### [message\_callback](..\..\..\lib\messaging.py#L232)
```py
def message_callback(action_string)
```
Callback decorator. Functions registered by this decorator will be called upon receiving action message.

Args:
    action_string ([type]): [description]
### [request\_callback](..\..\..\lib\messaging.py#L250)
```py
def request_callback(string_command)
```

### [set\_keepalive](..\..\..\lib\messaging.py#L54)
```py
def set_keepalive(sock, after_idle_sec=1, interval_sec=3, max_fails=5) -> None
```
Sets keepalive parameters of given socket.

Args:
    sock (socket): socket
    after_idle_sec (int, optional): Start sending keepalive packets after this amount of seconds. Defaults to 1.
    interval_sec (int, optional): Interval of keepalive packets in seconds. Defaults to 3.
    max_fails (int, optional): Count of fails leading to socket disconnect. Defaults to 5.

Raises:
    NotImplementedError: for unknown platform
## Class [ConnectionManager](..\..\..\lib\messaging.py#L263)
...
### [\_\_init\_\_](..\..\..\lib\messaging.py#L267)
```py
def __init__(self, whoami='computer')
```
Initialize self.  See help(type(self)) for accurate signature.
### [close](..\..\..\lib\messaging.py#L324)
```py
def close(self)
```

### [connect](..\..\..\lib\messaging.py#L306)
```py
def connect(self, client_selector, client_socket, client_addr)
```

### [get\_file](..\..\..\lib\messaging.py#L566)
```py
def get_file(self, client_filepath, filepath=None, callback=None, callback_args=(), callback_kwargs=None)
```

### [get\_response](..\..\..\lib\messaging.py#L543)
```py
def get_response(self, requested_value, callback, request_args=(), request_kwargs=None, callback_args=(), callback_kwargs=None)
```

### [process\_events](..\..\..\lib\messaging.py#L360)
```py
def process_events(self, mask)
```

### [process\_received](..\..\..\lib\messaging.py#L408)
```py
def process_received(self, message)
```

### [read](..\..\..\lib\messaging.py#L372)
```py
def read(self)
```

### [send\_file](..\..\..\lib\messaging.py#L595)
```py
def send_file(self, filepath, dest_filepath)
```

### [send\_message](..\..\..\lib\messaging.py#L589)
```py
def send_message(self, action, args=(), kwargs=None)
```

### [write](..\..\..\lib\messaging.py#L509)
```py
def write(self)
```



## Class [MessageManager](..\..\..\lib\messaging.py#L105)
MessageManager class represents single incoming by TCP stream message and contains methods to decode and extract data from incoming data. It also contains static class methods for encoding various types of messages.

Attributes:
    income_raw (bytes string) - Append incoming data to this attribute.
    content: Would be populated when receiving and processing of the message will be completed. Deafults to None.
### [\_\_init\_\_](..\..\..\lib\messaging.py#L112)
```py
def __init__(self)
```
Initialize self.  See help(type(self)) for accurate signature.
### [process\_message](..\..\..\lib\messaging.py#L217)
```py
def process_message(self)
```
[summary]
        


## Class [Namespace](..\..\..\lib\messaging.py#L22)
...
### [\_\_init\_\_](..\..\..\lib\messaging.py#L23)
```py
def __init__(self, **kwargs)
```
Initialize self.  See help(type(self)) for accurate signature.


## Class [NotifierSock](..\..\..\lib\messaging.py#L607)
...
### [\_\_init\_\_](..\..\..\lib\messaging.py#L608)
```py
def __init__(self)
```
Initialize self.  See help(type(self)) for accurate signature.
### [close](..\..\..\lib\messaging.py#L651)
```py
def close(self)
```

### [get\_sock](..\..\..\lib\messaging.py#L631)
```py
def get_sock(self)
```

### [init](..\..\..\lib\messaging.py#L619)
```py
def init(self, selector, port=26000)
```

### [notify](..\..\..\lib\messaging.py#L634)
```py
def notify(self)
```

### [process\_events](..\..\..\lib\messaging.py#L641)
```py
def process_events(self, mask)
```



## Class [PendingRequest](..\..\..\lib\messaging.py#L33)
...
### [\_\_init\_\_](..\..\..\lib\messaging.py#L23)
```py
def __init__(self, **kwargs)
```
Initialize self.  See help(type(self)) for accurate signature.


## Class [Singleton](..\..\..\lib\messaging.py#L102)
...


