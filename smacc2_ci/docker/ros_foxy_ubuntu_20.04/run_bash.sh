#!/bin/bash
./build.sh
xhost +
sudo docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it smacc2_foxy_ubuntu_2004 bash
