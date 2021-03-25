#!/bin/bash
export ROS_DISTRO=rolling
sudo apt-get update
sudo apt-get install xterm python3-rosdistro python3-rosdep
sudo pip3 install -U vcstool

echo "installing ros2 repos"

wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
mkdir ros2_repos
vcs import ros2_repos < ros2.repos
vcs pull ros2_repos

echo "assumming we are located in the smacc folder"
if [ ! -d dependencies ] 
then
mkdir dependencies 
fi
cd dependencies
mkdir ros2

vcs import < ../submodules/navigation2/tools/underlay.repos 
cd ..
#rosdep update > /dev/null
#rosdep install -q -y --from-paths submodules/navigation2/ --skip-keys "slam_toolbox"