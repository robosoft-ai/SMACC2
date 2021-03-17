#!/bin/bash
apt-get install xterm python3-rosdistro python3-rosdep2
echo "assumming we are located in the smacc folder"
if [ ! -d dependencies ] 
then
mkdir dependencies 
fi
cd dependencies
vcs import < ../submodules/navigation2/tools/underlay.repos 
cd ..
rosdep install -q -y --from-paths . --skip-keys "slam_toolbox"