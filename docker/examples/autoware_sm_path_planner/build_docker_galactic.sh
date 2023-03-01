#!/bin/bash
DIR="$(dirname "$(realpath "$0")")"
echo $DIR

GIT_BRANCH=galactic
ROS_DISTRO=galactic
echo "ros distro: $ROS_DISTRO"
echo "git branch: $GIT_BRANCH"
which docker

cd $DIR/../../..
pwd

docker build --build-arg ROS_DISTRO=$ROS_DISTRO --build-arg GIT_BRANCH=$GIT_BRANCH -t smacc2_autoware_sm_path_planner:$ROS_DISTRO -f docker/examples/autoware_sm_path_planner/Dockerfile .

#build  -t smacc2:$ROS_DISTRO -f docker/Dockerfile .
