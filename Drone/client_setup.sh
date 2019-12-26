#!/bin/bash

# $1 - ssid, $2 - password of wifi router
# $3 - hostname of rpi
# $4 - server ip 

if [ $(whoami) != "root" ]; then
  echo -e "\nThis should be run as root!\n"
  exit 1
fi

# check if enough arguments
if [[ $# -ne 4 ]] ; then
    echo -e "\nPlease, enter arguments: router ssid, wifi password, copter id and server ip"
    echo -e "\nExample: sudo $0 clever-swarm swarmwifi clever-1 192.168.1.100\n"
    exit 0
fi

# stop and disable dnsmasq service (to set wifi in client mode)
systemctl stop dnsmasq
systemctl disable dnsmasq

# enable getting auto ip
sed -i 's/interface wlan0//' /etc/dhcpcd.conf
sed -i 's/static ip_address=192.168.11.1\/24//' /etc/dhcpcd.conf

# make backup of wpa_supplicant.conf
if ! [ -f "/etc/wpa_supplicant/wpa_supplicant.conf.OLD" ] ; then
    cp /etc/wpa_supplicant/wpa_supplicant.conf /etc/wpa_supplicant/wpa_supplicant.conf.OLD
fi

# set ssid and password of the router
cat << EOF | tee /etc/wpa_supplicant/wpa_supplicant.conf
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=GB

network={
    ssid="$1"
    psk="$2"
    scan_ssid=1    
}
EOF

# restart dhcpcd to connect to wifi as a client
systemctl restart dhcpcd

# set hostname for linux
cat << EOF | tee /etc/hostname
$3
EOF
sed -i "/127.0.1.1/c 127.0.1.1       $3 $3.local" /etc/hosts

# set hostname for ROS
sed -i "/ROS_HOSTNAME/c ROS_HOSTNAME=\'$3\'" /home/pi/.bashrc

# set ssh message
cat << EOF | tee /etc/motd

$3

EOF

# configure chrony as client
cat << EOF | tee /etc/chrony/chrony.conf
server $4 iburst
driftfile /var/lib/chrony/drift
makestep 1.0 -1
rtcsync
EOF

# change server ip in client_config
sed -i "0,/^host/s/\(^h.*\)/host = $4/" client_config.ini

# enable clever show service and visual_pose_watchdog service
systemctl enable clever-show.service
systemctl enable visual_pose_watchdog.service

# restart clever
reboot