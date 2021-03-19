#!/bin/bash
export ROS_DISTRO=rolling
sudo apt-get update
sudo apt-get install xterm python3-rosdistro python3-rosdep
sudo pip3 install -U vcstool

echo "assumming we are located in the smacc folder"
if [ ! -d dependencies ] 
then
mkdir dependencies 
fi
cd dependencies
vcs import < ../submodules/navigation2/tools/underlay.repos 
cd ..
rosdep update > /dev/null
rosdep install -q -y --from-paths ../submodules/navigation2/ --skip-keys "slam_toolbox"