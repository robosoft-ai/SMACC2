#!/bin/sh

# Installation
# See this website for lttng instation tutorial: https://lttng.org/docs/v2.12/#doc-ubuntu-ppa

sudo apt-add-repository ppa:lttng/stable-2.12
sudo apt-get update

sudo apt-get install lttng-tools
sudo apt-get install lttng-modules-dkms
sudo apt-get install liblttng-ust-dev

# for tracing Java applications

sudo apt-get install liblttng-ust-agent-java

# for tracing Python 3 applications

sudo apt-get install python3-lttngust

# Kernel Fix 8/30/2021

sudo newgrp tracing
#the above command results in a switch to root user.
#added exit to return to user terminal
#exit
# ^not effective
sudo usermod -a -G tracing $user

#sudo adduser $user tracing
#reboot

#START of reboot cheat explained by Denis
# this should update the existing current groups
sudo su $user
#END reboot cheat

echo "Current groups displayed below. Is \"tracing\" present?"
groups | grep tracing

#install lttng kernel modules:
#sudo apt-get install lttng-modules-dkms

# SOME DEPENDENCIES WE SHOULD HAVE IN THE ROS2/package.xml file in exec_depends
sudo apt-get install ros-rolling-tracetools-launch ros-rolling-ros2trace ros-rolling-tracetools*

#source ros rolling to run trace tools
sudo source opt/ros/rolling/setup.bash
