#!/bin/sh
DIR="$(dirname "$(realpath "$0")")"
echo $DIR
cd $DIR/..
echo `pwd`

ROS_DISTRO=$1
GITBRANCH=$2
#ROS_VERSION_NAME="rolling"

echo "git branch: $GITBRANCH"
sudo docker build --build-arg ROS_DISTRO=$ROS_DISTRO --build-arg GITBRANCH=$GITBRANCH -t smacc2 -f docker/Dockerfile .
