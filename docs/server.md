# Настройка сервера
## Файл конфигурации
Конфигурация сервера задаётся в файле 	Server/server_config.ini, имеющем вид:
```ini
[SERVER]  
port = 25000  
broadcast_port = 8181  
broadcast_delay = 5  
buffer_size = 1024  
  
[NTP]  
use_ntp = False  
host = ntp1.stratum2.ru  
port = 123
``` 
### Раздел 'Server'
В этом разделе задаются параметры сетевого взаимодействия сервера, доступны следующие параметры:

 * `port` - TCP порт, на котором будут приниматься входящие соединения от клиентов (коптеров)
 * `broadcast_port` - UDP 

 
<!--stackedit_data:
eyJoaXN0b3J5IjpbOTM2NTcxMzg4LDg3MjY4MDYxOF19
-->