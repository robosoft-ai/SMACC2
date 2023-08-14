#!/bin/sh
xhost +

DIR="$(dirname "$(realpath "$0")")"
ROOT_DIR=`realpath $DIR/../../../..`
DRIVER=$(apt list --installed | grep nvidia-driver)

sudo nvidia-docker run --net host -it -e DISPLAY  -e QT_X11_NO_MITSHM=1 --privileged --name ue_editor_rclue -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix ue_editor_rclue:humble /bin/bash  -c "./nvidia-driver-check.sh '$DRIVER' && bash"
