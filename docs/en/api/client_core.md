# Module [drone.modules.client_core](..\..\..\drone\modules\client_core.py)
## Module functions
## Class [Client](..\..\..\drone\modules\client_core.py#L25)
Client base class provides config loading, communication with server (including automatic reconnection, broadcast listening and binding).

Attributes:
    server_connection (ConnectionManager) - connection to the server.
    connected (bool) - whether the client is connected to the server.
    client_id (string) - ID of the client.
    config (ConfigManager) - contains loaded client configuration.
    config_path (string) -  path to configuration file. There also should be config specification file at 'config_path\config\configspec_client.ini'.
### [\_\_init\_\_](..\..\..\drone\modules\client_core.py#L37)
```py
def __init__(self, config_path='C:\\Users\\artem\\Documents\\GitHub\\COEX-clever-swarm\\clever-show\\drone\\modules\\..\\config\\client.ini')
```
Initialize self.  See help(type(self)) for accurate signature.
### [broadcast\_bind](..\..\..\drone\modules\client_core.py#L160)
```py
def broadcast_bind(self, timeout=2.0, attempt_limit=3)
```

### [get\_ntp\_time](..\..\..\drone\modules\client_core.py#L72)
```py
def get_ntp_time(ntp_host, ntp_port)
```
Gets and returns time from specified host and port of NTP server.

Args:
    ntp_host (string): hostname or address of the NTP server.
    ntp_port (int): port of the NTP server.

Returns:
    int: Current time recieved from the NTP server
### [load\_config](..\..\..\drone\modules\client_core.py#L53)
```py
def load_config(self)
```
Loads or reloads config from file specified in 'config_path'.
### [on\_broadcast\_bind](..\..\..\drone\modules\client_core.py#L197)
```py
def on_broadcast_bind(self)
```
Method called on binding to the server by broadcast. Override that method in order to add functionality.
        
### [start](..\..\..\drone\modules\client_core.py#L105)
```py
def start(self)
```
Reloads config and starts infinite loop of connecting to the server and processing said connection. Calling of this method will indefinitely halt execution of any subsequent code.
        
### [time\_now](..\..\..\drone\modules\client_core.py#L93)
```py
def time_now(self)
```
gets and returns system time or NTP time depending on the config.

Returns:
    int: Current time.


