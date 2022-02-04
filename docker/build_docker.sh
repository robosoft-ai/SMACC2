#!/bin/sh
DIR="$(dirname "$(realpath "$0")")"
echo $DIR
cd $DIR/..
echo `pwd`

ROS_DISTRO=$1
#ROS_VERSION_NAME="rolling"

sudo docker build --build-arg ROS_DISTRO=$ROS_DISTRO -t smacc2 -f docker/Dockerfile .
