# Установка ПО clever-show на образ clever v0.21

Настройте подключение к WiFi сети вашего роутера согласно [инструкции](https://clover.coex.tech/ru/network.html#%D0%BF%D0%B5%D1%80%D0%B5%D0%BA%D0%BB%D1%8E%D1%87%D0%B5%D0%BD%D0%B8%D0%B5-%D0%B0%D0%B4%D0%B0%D0%BF%D1%82%D0%B5%D1%80%D0%B0-%D0%B2-%D1%80%D0%B5%D0%B6%D0%B8%D0%BC-%D0%BA%D0%BB%D0%B8%D0%B5%D0%BD%D1%82%D0%B0)

Для установки hostname (опционально) воспользуйтесь командами для редактирования следующих файлов:
```bash
sudo nano /etc/hostname
sudo nano /etc/hosts
```
После установки hostname перезагрузите Raspberry:
```bash
sudo reboot
```

Обновите репозитории apt:
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update -qq --allow-releaseinfo-change
```

Загрузите репозиторий clever-show и установите необходимые для клиента зависимости
```
git clone https://github.com/CopterExpress/clever-show.git
sudo pip install -r /home/pi/clever-show/drone/requirements.txt
```

Установите chrony и установите конфигурацию по умолчанию:
```bash
sudo apt-get install -y chrony
```

```bash
sudo cp /home/pi/clever-show/examples/chrony/client.conf /etc/chrony/chrony.conf
sudo systemctl restart chrony
```

Скопируйте файлы сервисов clever-show и запустите их:
```bash 
sudo cp /home/pi/clever-show/builder/assets/clever-show.service /lib/systemd/system/
sudo systemctl enable clever-show.service
sudo systemctl start clever-show.service


sudo cp /home/pi/clever-show/builder/assets/failsafe.service /lib/systemd/system/
sudo systemctl enable failsafe.service
sudo systemctl start failsafe.service
```

На новых ревизиях Raspbery Pi может не работать светодиодная лента
Для исправления, обновите пакет `rpi_ws281x` и перезагрузите Raspbery Pi: 
```bash
pip install rpi_ws281x --upgrade
sudo apt-get update
sudo apt-get install ros-melodic-ws281x
sudo reboot
```
Проверка работы светодиодной ленты:
```bash
rosservice call /led/set_effect "{effect: 'rainbow'}"
```

# Установка ПО clever-show на образ clever v0.23
Настройте подключение к WiFi сети вашего роутера согласно [инструкции](https://clover.coex.tech/ru/network.html#%D0%BF%D0%B5%D1%80%D0%B5%D0%BA%D0%BB%D1%8E%D1%87%D0%B5%D0%BD%D0%B8%D0%B5-%D0%B0%D0%B4%D0%B0%D0%BF%D1%82%D0%B5%D1%80%D0%B0-%D0%B2-%D1%80%D0%B5%D0%B6%D0%B8%D0%BC-%D0%BA%D0%BB%D0%B8%D0%B5%D0%BD%D1%82%D0%B0)

Для установки hostname (опционально) воспользуйтесь командами для редактирования следующих файлов:
```bash
sudo nano /etc/hostname
sudo nano /etc/hosts
```
После установки hostname перезагрузите Raspberry:
```bash
sudo reboot
```

Обновите репозитории apt:
```bash
sudo apt-get update
```

Загрузите репозиторий clever-show и установите необходимые для клиента зависимости
```
git clone https://github.com/CopterExpress/clever-show.git --branch python3
sudo pip3 install -r /home/pi/clever-show/drone/requirements.txt --upgrade
```

Установите chrony и установите конфигурацию по умолчанию:
```bash
sudo apt-get install -y chrony
```

```bash
sudo cp /home/pi/clever-show/examples/chrony/client.conf /etc/chrony/chrony.conf
sudo systemctl restart chrony
```

Скопируйте файлы сервисов clever-show и запустите их:
```bash 
sudo cp /home/pi/clever-show/builder/assets/clever-show.service /lib/systemd/system/
sudo systemctl enable clever-show.service
sudo systemctl start clever-show.service


sudo cp /home/pi/clever-show/builder/assets/failsafe.service /lib/systemd/system/
sudo systemctl enable failsafe.service
sudo systemctl start failsafe.service
```
