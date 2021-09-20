#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ../../../..
echo "Building DOCKER from directory `pwd`"

#TODO: Implement user selection of ROS2 Distro
#Ex sudo docker build --build-arg ROS2_DISTRO=galactic

sudo docker build -f $DIR/Dockerfile -t smacc2_rolling_ubuntu_2004 .
cd $DIR
