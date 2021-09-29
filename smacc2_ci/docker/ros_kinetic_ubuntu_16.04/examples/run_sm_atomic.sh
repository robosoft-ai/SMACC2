#!/bin/bash
./build.sh
xhost +
sudo docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it smacc_kinetic_ubuntu_1604 bash -c "source /opt/ros/kinetic/setup.bash; roslaunch sm_atomic sm_atomic.launch & rosrun smacc_viewer smacc_viewer_node.py"
