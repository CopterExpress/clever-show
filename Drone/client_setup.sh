#!/bin/bash

# $1 - ssid, $2 - password of wifi router
# $3 - hostname of rpi
# $4 - server ip 

# check if enough arguments
if [[ $# -ne 4 ]] ; then
    echo -e "\nPlease, enter arguments: ssid, wifi password, copter id and server ip"
    echo -e "\nExample: sudo ./$0 clever-swarm swarmwifi clever-1 192.168.1.100\n"
    exit 0
fi

# stop and disable dnsmasq service
systemctl stop dnsmasq
systemctl disable dnsmasq

# enable get auto ip
sed -i 's/interface wlan0//' /etc/dhcpcd.conf
sed -i 's/static ip_address=192.168.11.1\/24//' /etc/dhcpcd.conf

# make backup of wpa_supplicant.conf
if ! [ -f "/etc/wpa_supplicant/wpa_supplicant.conf.OLD" ] ; then
    cp /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.OLD
fi

# ssid and password of the router
cat << EOF | tee /etc/wpa_supplicant/wpa_supplicant.conf
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB

network={
    ssid="$1"
    psk="$2"
}
EOF

# set hostname for linux
cat << EOF | tee /etc/hostname
$3
EOF
sed -i "/127.0.1.1/c 127.0.1.1       $3" /etc/hosts

# set hostname for ROS
sed -i "/ROS_HOSTNAME/c ROS_HOSTNAME=\'$3\'" /home/pi/.bashrc
sed -i "/ROS_HOSTNAME/c ROS_HOSTNAME=$3" /lib/systemd/system/roscore.env

# set sudoers variables to make sudo works with ros:
grep -qxF 'Defaults        env_keep += "ROS_LOG_DIR"' /etc/sudoers || cat << EOT >> /etc/sudoers

Defaults        env_keep += "PYTHONPATH"
Defaults        env_keep += "PATH"
Defaults        env_keep += "ROS_ROOT"
Defaults        env_keep += "ROS_MASTER_URI"
Defaults        env_keep += "ROS_PACKAGE_PATH"
Defaults        env_keep += "ROS_LOCATIONS"
Defaults        env_keep += "ROS_HOME"
Defaults        env_keep += "ROS_LOG_DIR"
EOT

# copy aruco.launch and clever.launch: TODO???

# install samba and winbind
apt-get -y install samba
apt-get -y install winbind

# set /etc/nsswitch.conf
sed -i '/hosts:/c hosts:         files dns wins' /etc/nsswitch.conf

# install chrony
apt-get -y install chrony

# configure chrony as client
cat << EOF | tee /etc/chrony/chrony.conf
server $4 iburst
driftfile /var/lib/chrony/drift
makestep 1.0 3
rtcsync
EOF

# change server ip in client_config
sed -i "0,/^host/s/\(^h.*\)/host = $4/" client_config.ini

# restart clever
reboot