#!/bin/bash
./build.sh
xhost +
sudo docker run --gpus device=0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it smacc_kinetic_ubuntu_1604 bash
