#! /usr/bin/env bash

set -e # Exit immidiately on non-zero result

SOURCE_IMAGE="https://github.com/CopterExpress/clover/releases/download/v0.21.2/clover_v0.21.2.img.zip"

export DEBIAN_FRONTEND=${DEBIAN_FRONTEND:='noninteractive'}
export LANG=${LANG:='C.UTF-8'}
export LC_ALL=${LC_ALL:='C.UTF-8'}

echo_stamp() {
  # TEMPLATE: echo_stamp <TEXT> <TYPE>
  # TYPE: SUCCESS, ERROR, INFO

  # More info there https://www.shellhacks.com/ru/bash-colors/

  TEXT="$(date '+[%Y-%m-%d %H:%M:%S]') $1"
  TEXT="\e[1m$TEXT\e[0m" # BOLD

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

REPO_DIR="/mnt"
SCRIPTS_DIR="${REPO_DIR}/builder"
CONFIG_DIR="${SCRIPTS_DIR}/clover-config"
IMAGES_DIR="${REPO_DIR}/images"

[[ ! -d ${SCRIPTS_DIR} ]] && (echo_stamp "Directory ${SCRIPTS_DIR} doesn't exist" "ERROR"; exit 1)
[[ ! -d ${IMAGES_DIR} ]] && mkdir ${IMAGES_DIR} && echo_stamp "Directory ${IMAGES_DIR} was created successful" "SUCCESS"
[[ ! -d ${CONFIG_DIR} ]] && mkdir ${CONFIG_DIR} && echo_stamp "Directory ${CONFIG_DIR} was created successful" "SUCCESS"

if [[ -z ${TRAVIS_TAG} ]]; then IMAGE_VERSION="$(cd ${REPO_DIR}; git log --format=%h -1)"; else IMAGE_VERSION="${TRAVIS_TAG}"; fi
# IMAGE_VERSION="${TRAVIS_TAG:=$(cd ${REPO_DIR}; git log --format=%h -1)}"
REPO_URL="$(cd ${REPO_DIR}; git remote --verbose | grep origin | grep fetch | cut -f2 | cut -d' ' -f1 | sed 's/git@github\.com\:/https\:\/\/github.com\//')"
REPO_NAME="$(basename -s '.git' ${REPO_URL})"
echo_stamp "REPO_NAME=${REPO_NAME}" "INFO"
IMAGE_NAME="${REPO_NAME}_${IMAGE_VERSION}.img"
echo_stamp "IMAGE_NAME=${IMAGE_NAME}" "INFO"
IMAGE_PATH="${IMAGES_DIR}/${IMAGE_NAME}"
echo_stamp "IMAGE_PATH=${IMAGE_PATH}" "INFO"

get_image() {
  # TEMPLATE: get_image <IMAGE_PATH> <RPI_DONWLOAD_URL>
  local BUILD_DIR=$(dirname $1)
  echo_stamp "BUILD_DIR=${BUILD_DIR}" "INFO"
  local RPI_ZIP_NAME=$(basename $2)
  echo_stamp "RPI_ZIP_NAME=${RPI_ZIP_NAME}" "INFO"
  local RPI_IMAGE_NAME=$(echo ${RPI_ZIP_NAME} | sed 's/.zip//')
  echo_stamp "RPI_IMAGE_NAME=${RPI_IMAGE_NAME}" "INFO"

  if [ ! -e "${BUILD_DIR}/${RPI_ZIP_NAME}" ]; then
    echo_stamp "Downloading original clover distribution"
    wget --progress=dot:giga -O ${BUILD_DIR}/${RPI_ZIP_NAME} $2
    echo_stamp "Downloading complete" "SUCCESS"
  else echo_stamp "Clever distribution already downloaded" "INFO"; fi

  echo_stamp "Unzipping clever distribution image" \
  && unzip -p ${BUILD_DIR}/${RPI_ZIP_NAME} ${RPI_IMAGE_NAME} > $1 \
  && echo_stamp "Unzipping complete" "SUCCESS" \
  || (echo_stamp "Unzipping was failed!" "ERROR"; exit 1)
}

get_image ${IMAGE_PATH} ${SOURCE_IMAGE}

# Make free space
img-resize ${IMAGE_PATH} max '6G'

# Reconfiguring clever show repository for simplier unshallowing
# git config remote.origin.fetch "+refs/heads/*:refs/remotes/origin/*"

# Checkout to tag's branch if built with travis tag
if [[ ! -z ${TRAVIS_TAG} ]]; then
  cd ${REPO_DIR}
  REMOTE_BRANCH="$(git branch -r --contains ${TRAVIS_TAG} | sed -n 1p | cut -d ' ' -f 5)"
  BRANCH="$(echo ${REMOTE_BRANCH} | cut -d '/' -f 2)"
  echo_stamp "Checkout to ${REMOTE_BRANCH} from ${TRAVIS_TAG}" "INFO"
  git branch ${BRANCH} ${REMOTE_BRANCH}
  git checkout ${BRANCH}
  cd /
fi

# Copy cloned repository to the image
# Include dotfiles in globs (asterisks)
shopt -s dotglob

echo_stamp "Mount loop-image: ${IMAGE_PATH}"
DEV_IMAGE=$(losetup -Pf ${IMAGE_PATH} --show)
sleep 0.5

MOUNT_POINT=$(mktemp -d --suffix=.builder_image)
echo_stamp "Mount dirs ${MOUNT_POINT} & ${MOUNT_POINT}/boot"
mount "${DEV_IMAGE}p2" ${MOUNT_POINT}
mount "${DEV_IMAGE}p1" ${MOUNT_POINT}/boot

mkdir -p ${MOUNT_POINT}'/home/pi/clever-show/'
for dir in ${REPO_DIR}/*; do
  if [[ $dir != *"images" && $dir != *"imgcache" ]]; then
    cp -r $dir ${MOUNT_POINT}'/home/pi/clever-show/'$(basename $dir)
  fi;
done

umount -fR ${MOUNT_POINT}
losetup -d ${DEV_IMAGE}

# Install software
img-chroot ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-software.sh'

# Copy service files for clever show client and visual_pose_watchdog
img-chroot ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/clever-show.service' '/lib/systemd/system/'
img-chroot ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/failsafe.service' '/lib/systemd/system/'

# Copy client-setup script to /usr/local/bin to provide wide access
img-chroot ${IMAGE_PATH} copy ${SCRIPTS_DIR}'/assets/client-setup' '/usr/local/bin/'

# Copy basic animation
img-chroot ${IMAGE_PATH} copy ${REPO_DIR}'/examples/animations/basic/basic.csv' '/home/pi/clever-show/drone/animation.csv'

# Copy chrony configuration
img-chroot ${IMAGE_PATH} copy ${REPO_DIR}'/examples/chrony/client.conf' '/etc/chrony/chrony.conf'

# Configure image
img-chroot ${IMAGE_PATH} exec ${SCRIPTS_DIR}'/image-configure.sh'

# Copy config files for clover
if [[ -d "${CONFIG_DIR}/launch" ]]; then img-chroot ${IMAGE_PATH} copy ${CONFIG_DIR}'/launch' '/home/pi/catkin_ws/src/clover/clover'; fi
if [[ -d "${CONFIG_DIR}/map" ]]; then img-chroot ${IMAGE_PATH} copy ${CONFIG_DIR}'/map' '/home/pi/catkin_ws/src/clover/aruco_pose'; fi
if [[ -d "${CONFIG_DIR}/camera_info" ]]; then img-chroot ${IMAGE_PATH} copy ${CONFIG_DIR}'/camera_info' '/home/pi/catkin_ws/src/clover/clover'; fi

# Shrink image
img-resize ${IMAGE_PATH}





