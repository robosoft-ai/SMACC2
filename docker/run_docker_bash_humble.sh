#!/bin/sh
#sudo docker run -it smacc2:humble /bin/bash
xhost +
sudo nvidia-docker  run -it -e DISPLAY  -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix smacc2:humble /bin/bash
