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

ROOT_DIR=`realpath $DIR/../../..`

echo "ros distro: $ROS_DISTRO"
echo "git branch: $GIT_BRANCH"
echo "ubuntu version: $UBUNTU_VERSION"
echo "root path: $ROOT_DIR"

UE_FOLDER=$DIR/UE5.1/UnrealEngine
# if DIRECTORY does exist show message error
if [ -d "$UE_FOLDER" ]; then
  echo "UE5.1 folder exists"
else
  echo "UE5.1 folder does not exist"
  echo "Please download UE5.1 from https://www.unrealengine.com/en-US/download"
  echo "and extract it to $UE_FOLDER"
  exit 1
fi

#it is expected to exist $UE_FOLDER/Engine
if [ -d "$UE_FOLDER/Engine" ]; then
  echo "UE5.1 Engine folder exists"
else
  echo "UE5.1 Engine folder does not exist"
  echo "Please download UE5.1 from https://www.unrealengine.com/en-US/download"
  echo "and extract it to $UE_FOLDER"
  exit 1
fi


cd $ROOT_DIR
sudo docker build --build-arg ROS_DISTRO=$ROS_DISTRO --build-arg GIT_BRANCH=$GIT_BRANCH --build-arg UBUNTU_VERSION=$UBUNTU_VERSION -t ue_editor_rclue:$ROS_DISTRO -f $ROOT_DIR/smacc2_sm_reference_library/sm_dancebot_ue/docker/Dockerfile . $NOCACHE
