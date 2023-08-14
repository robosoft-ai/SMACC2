#!/bin/sh
xhost +

DIR="$(dirname "$(realpath "$0")")"
ROOT_DIR=`realpath $DIR/../../../..`

sudo nvidia-docker run --net host -it -e DISPLAY  -e QT_X11_NO_MITSHM=1 --privileged --name ue_editor_rclue -v $ROOT_DIR/SMACC2:/home/ros2_ws/src/SMACC2 -v $ROOT_DIR/SMACC2_RTA:/home/ros2_ws/src/SMACC2_RTA  -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix ue_editor_rclue:humble /bin/bash
