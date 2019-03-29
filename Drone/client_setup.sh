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
    echo -e "\nExample: sudo $0 clever-swarm swarmwifi clever-1 192.168.1.100"
    echo -e "\nWarning: this script requires wifi router to be connected to internet!\n"
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
}
EOF

# restart dhcpcd to connect to wifi as a client
systemctl restart dhcpcd

# set hostname for linux
cat << EOF | tee /etc/hostname
$3
EOF
sed -i "/127.0.1.1/c 127.0.1.1       $3" /etc/hosts

# set hostname for ROS
sed -i "/ROS_HOSTNAME/c ROS_HOSTNAME=\'$3\'" /home/pi/.bashrc
sed -i "/ROS_HOSTNAME/c ROS_HOSTNAME=$3" /lib/systemd/system/roscore.env

# set ssh message
cat << EOF | tee /etc/motd

$3

EOF

# add sudoers variables to make sudo works with ros (for led strip)
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

# configure aruco.launch and clever.launch (for positioning with aruco map)
sed -i '/<arg name="aruco_map"/c \    <arg name="aruco_map" default="true"/>' /home/pi/catkin_ws/src/clever/clever/launch/aruco.launch
sed -i '/<arg name="aruco_vpe"/c \    <arg name="aruco_vpe" default="true"/>' /home/pi/catkin_ws/src/clever/clever/launch/aruco.launch
sed -i '/<param name="map"/c \        <param name="map" value="\$\(find aruco_pose\)/map/animation_map.txt"/>' /home/pi/catkin_ws/src/clever/clever/launch/aruco.launch
sed -i '/<arg name="aruco"/c \    <arg name="aruco" default="true"/>' /home/pi/catkin_ws/src/clever/clever/launch/clever.launch

# copy office map to animation map if there is no animation map file
if ! [ -f "/home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt" ] ; then
    sudo -u pi cp /home/pi/catkin_ws/src/clever/aruco_pose/map/office.txt /home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt
fi

# install samba and winbind (for hostname resolving)
# apt-get -y install samba
# apt-get -y install winbind

# set /etc/nsswitch.conf
sed -i '/hosts:/c hosts:         files dns wins' /etc/nsswitch.conf

# install chrony (for time syncing)
# apt-get -y install chrony

# configure chrony as client
cat << EOF | tee /etc/chrony/chrony.conf
server $4 iburst
driftfile /var/lib/chrony/drift
makestep 1.0 3
rtcsync
EOF

# change server ip in client_config
sed -i "0,/^host/s/\(^h.*\)/host = $4/" client_config.ini

# set roscore service to restart on-failure
sed -i '/Restart=/c Restart=on-failure\
RestartSec=3' /lib/systemd/system/roscore.service

# install python pause module
pip install pause

# restart clever
reboot