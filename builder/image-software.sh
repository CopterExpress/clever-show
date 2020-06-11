#! /usr/bin/env bash

#
# Script for install software to the image.
#

set -e # Exit immidiately on non-zero result

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

# https://gist.github.com/letmaik/caa0f6cc4375cbfcc1ff26bd4530c2a3
# https://github.com/travis-ci/travis-build/blob/master/lib/travis/build/templates/header.sh
my_travis_retry() {
  local result=0
  local count=1
  while [ $count -le 3 ]; do
    [ $result -ne 0 ] && {
      echo -e "\n${ANSI_RED}The command \"$@\" failed. Retrying, $count of 3.${ANSI_RESET}\n" >&2
    }
    # ! { } ignores set -e, see https://stackoverflow.com/a/4073372
    ! { "$@"; result=$?; }
    [ $result -eq 0 ] && break
    count=$(($count + 1))
    sleep 1
  done

  [ $count -gt 3 ] && {
    echo -e "\n${ANSI_RED}The command \"$@\" failed 3 times.${ANSI_RESET}\n" >&2
  }

  return $result
}

echo_stamp "Move /etc/ld.so.preload out of the way"
mv /etc/ld.so.preload /etc/ld.so.preload.disabled-for-build

echo_stamp "Update apt cache"
apt-get update -qq

echo_stamp "Software installing"
apt-get install -y \
chrony \
&& echo_stamp "Everything was installed!" "SUCCESS" \
|| (echo_stamp "Some packages wasn't installed!" "ERROR"; exit 1)

echo_stamp "Install python libs"
cd /home/pi/clever-show/drone
my_travis_retry pip install -r requirements.txt

echo_stamp "Install catkin packages"
cd /home/pi/catkin_ws/src
git clone https://github.com/CopterExpress/clever_tools.git
cd ..
source devel/setup.bash
catkin_make --pkg clever_flight_routines
source devel/setup.bash

echo_stamp "Change clever-show and catkin_ws owner to pi"
chown -Rf pi:pi /home/pi/clever-show/
chown -Rf pi:pi /home/pi/catkin_ws/

echo_stamp "End of software installation"
