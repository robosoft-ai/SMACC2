#!/bin/sh
xhost +

DIR="$(dirname "$(realpath "$0")")"
ROOT_DIR=`realpath $DIR/../../../..`
DRIVER=$(apt list --installed | grep nvidia-driver)

sudo nvidia-docker run --net host --ipc=host -it -e DISPLAY  -e QT_X11_NO_MITSHM=1 --privileged --name ue_editor_rclue -v $ROOT_DIR/ue_project_1:/home/ros2_ws/src/ue_project_1 -v $ROOT_DIR/ue_project_2:/home/ros2_ws/src/ue_project_2 -v $ROOT_DIR/SMACC2:/home/ros2_ws/src/SMACC2 -v $ROOT_DIR/UE-Plugins:/home/ros2_ws/src/ue_project_1/Plugins/RapyutaSimulationPlugins -v $ROOT_DIR/rclUE:/home/ros2_ws/src/ue_project_1/Plugins/rclUE -v $ROOT_DIR/UE-Plugins:/home/ros2_ws/src/ue_project_2/Plugins/RapyutaSimulationPlugins -v $ROOT_DIR/rclUE:/home/ros2_ws/src/ue_project_2/Plugins/rclUE -v /dev/dri:/dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix ue_editor_rclue:humble /bin/bash  -c "./nvidia-driver-check.sh '$DRIVER' && bash"
