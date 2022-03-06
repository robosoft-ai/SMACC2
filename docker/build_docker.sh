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
echo "git branch: $GITBRANCH"
sudo docker build --build-arg ROS_DISTRO=$ROS_DISTRO --build-arg GIT_BRANCH=$GIT_BRANCH -t smacc2:$ROS_DISTRO -f docker/Dockerfile . $NOCACHE
