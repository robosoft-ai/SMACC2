#!/bin/bash
sudo apt-get update
sudo apt-get install xterm python3-rosdistro python3-rosdep
sudo pip3 install -U vcstool

echo "assumming we are located in the smacc folder"
if [ ! -d dependencies ] 
then
mkdir dependencies 
fi
cd dependencies
/usr/bin/vcs import < ../submodules/navigation2/tools/underlay.repos 
cd ..
/usr/bin/rosdep install -q -y --from-paths . --skip-keys "slam_toolbox" > /dev/null