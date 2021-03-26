#!/bin/bash
sudo apt-get update
sudo apt-get install -y python3-pip xterm python3-rosdistro python3-rosdep python3-vcstools
sudo pip3 install -U vcstool

echo "installing ros2 repos"

if [ ! -d ros2_repos ] 
then
mkdir ros2_repos 
fi

wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos

vcs import ros2_repos < ros2.repos
vcs pull ros2_repos

echo "assumming we are located in the smacc folder"

vcs import ros2_repos < submodules/navigation2/tools/underlay.repos 

