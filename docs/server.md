# Настройка сервера
## Файл конфигурации
Конфигурация сервера задаётся в файле 	Server/server_config.ini, имеющем вид:
```ini
[SERVER]  
port = 25000  
buffer_size = 1024  
  
[BROADCAST]  
use_broadcast = True  
broadcast_port = 8181  
broadcast_delay = 5  
  
[NTP]  
use_ntp = False  
host = ntp1.stratum2.ru  
port = 123
``` 
### Раздел 'Server'
В этом разделе задаются параметры сетевого взаимодействия сервера, доступны следующие параметры:

 * `port` - TCP порт, на котором будут приниматься входящие соединения от клиентов (коптеров). При использовании broadcast данный порт будет сконфигурирован у клиента автоматически. Рекомендуется изменить значение по умолчанию в целях безопасности (любое пятизначное чис
 * `buffer_size` - размер буфера при приёме и передаче данных, не рекомендуется изменять

 * `broadcast_port` - UDP 

 
<!--stackedit_data:
eyJoaXN0b3J5IjpbMjI2MDI1OTM0LDkzNjU3MTM4OCw4NzI2OD
A2MThdfQ==
-->