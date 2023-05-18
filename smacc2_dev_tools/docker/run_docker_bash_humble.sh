#!/bin/sh
xhost +

DIR="$(dirname "$(realpath "$0")")"
ROOT_DIR=`realpath $DIR/../..`

sudo nvidia-docker  run -it -e DISPLAY  -e QT_X11_NO_MITSHM=1 -v $ROOT_DIR:/home/ros2_ws/src/SMACC2  -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix smacc2:humble /bin/bash
