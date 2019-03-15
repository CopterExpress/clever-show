#!/bin/bash

# $1 - ssid, $2 - password of wifi router
# $3 - hostname of rpi 

# check if enough arguments
if [[ $# -ne 3 ]] ; then
    echo -e "\nPlease, enter arguments: ssid, password of wifi and hostname."
    echo -e "\nExample: sudo $0 clever-swarm swarmwifi clever-1\n"
    exit 0
fi

# stop and disable dnsmasq service
systemctl stop dnsmasq
systemctl disable dnsmasq

# enable get auto ip
sed -i 's/interface wlan0//' /etc/dhcpcd.conf
sed -i 's/static ip_address=192.168.11.1\/24//' /etc/dhcpcd.conf

# make backup of wpa_supplicant.conf
cp /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.OLD

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

# set sudoers variables to make sudo works with ros: TODO

# copy aruco.launch and clever.launch: TODO???

# install samba and winbind
apt-get -y install samba
apt-get -y install winbind

# set /etc/nsswitch.conf
sed -i '/hosts:/c hosts:         files dns wins' /etc/nsswitch.conf

# restart clever
reboot