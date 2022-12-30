#!/bin/sh
DIR="$(dirname "$(realpath "$0")")"
echo $DIR
cd $DIR/..
echo `pwd`

ROS_DISTRO=$1
GIT_BRANCH=$2
UBUNTU_VERSION=$3
NOCACHE="--no-cache"
NOCACHE=

echo "ros distro: $ROS_DISTRO"
echo "git branch: $GIT_BRANCH"
echo "ubuntu version: $UBUNTU_VERSION"
sudo docker build --build-arg ROS_DISTRO=$ROS_DISTRO --build-arg GIT_BRANCH=$GIT_BRANCH --build-arg UBUNTU_VERSION=$UBUNTU_VERSION -t smacc2:$ROS_DISTRO -f docker/Dockerfile . $NOCACHE
