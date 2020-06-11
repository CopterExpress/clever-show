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

# rename wifi ssid
sed -i "s/NEW_SSID='clover/NEW_SSID='clever-show/" /root/init_rpi.sh

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

echo_stamp "Image was configured!" "SUCCESS"

echo "Move /etc/ld.so.preload back to its original position"
mv /etc/ld.so.preload.disabled-for-build /etc/ld.so.preload
