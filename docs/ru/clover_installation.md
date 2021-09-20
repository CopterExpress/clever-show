# Установка ПО clever-show на образ clever v0.21

Настройте подключение к WiFi сети согласно [инструкции](https://clover.coex.tech/ru/network.html#%D0%BF%D0%B5%D1%80%D0%B5%D0%BA%D0%BB%D1%8E%D1%87%D0%B5%D0%BD%D0%B8%D0%B5-%D0%B0%D0%B4%D0%B0%D0%BF%D1%82%D0%B5%D1%80%D0%B0-%D0%B2-%D1%80%D0%B5%D0%B6%D0%B8%D0%BC-%D0%BA%D0%BB%D0%B8%D0%B5%D0%BD%D1%82%D0%B0)

Для установки hostname воспользуйтесь командами:
```bash
sudo nano /etc/hostname
sudo nano /etc/hosts
```

Обновите репозитории apt:
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update -qq --allow-releaseinfo-change
```

Зашрузите репозиторий clever-show и установите необходимые для клиента зависимости
```
git clone https://github.com/CopterExpress/clever-show.git
sudo pip install -r /home/pi/clever-show/drone/requirements.txt
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

Установите chrony и установите конфигурацию по умолчанию:
```bash
sudo apt-get install -y chrony
sudo cp /home/pi/clever-show/examples/chrony/client.conf /etc/chrony/chrony.conf
```
