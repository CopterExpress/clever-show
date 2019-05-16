#! /usr/bin/env bash

set -e

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  TEXT="\e[1m${TEXT}\e[0m" # BOLD

  case "$2" in
    SUCCESS)
    TEXT="\e[32m${TEXT}\e[0m";; # GREEN
    ERROR)
    TEXT="\e[31m${TEXT}\e[0m";; # RED
    *)
    TEXT="\e[34m${TEXT}\e[0m";; # BLUE
  esac
  echo -e ${TEXT}
}

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

# configure aruco.launch and clever.launch (for positioning with aruco map and optical flow)
sed -i '/<arg name="aruco_map"/c \    <arg name="aruco_map" default="true"/>' /home/pi/catkin_ws/src/clever/clever/launch/aruco.launch
sed -i '/<arg name="aruco_vpe"/c \    <arg name="aruco_vpe" default="true"/>' /home/pi/catkin_ws/src/clever/clever/launch/aruco.launch
sed -i '/<param name="map"/c \        <param name="map" value="\$\(find aruco_pose\)/map/animation_map.txt"/>' /home/pi/catkin_ws/src/clever/clever/launch/aruco.launch
sed -i '/<arg name="aruco"/c \    <arg name="aruco" default="true"/>' /home/pi/catkin_ws/src/clever/clever/launch/clever.launch
sed -i '/<arg name="rangefinder_vl53l1x"/c \    <arg name="rangefinder_vl53l1x" default="true"/>' /home/pi/catkin_ws/src/clever/clever/launch/clever.launch
sed -i '/<arg name="optical_flow"/c \    <arg name="optical_flow" default="true"/>' /home/pi/catkin_ws/src/clever/clever/launch/clever.launch

# copy office map to animation map if there is no animation map file
if ! [ -f "/home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt" ] ; then
    sudo -u pi cp /home/pi/catkin_ws/src/clever/aruco_pose/map/office.txt /home/pi/catkin_ws/src/clever/aruco_pose/map/animation_map.txt
fi

echo_stamp "Image was configured!" "SUCCESS"

