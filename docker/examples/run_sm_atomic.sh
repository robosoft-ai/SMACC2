#!/bin/sh
sudo docker run -e DISPLAY -it smacc2 /bin/bash -c 'source /opt/ros/rolling/setup.sh && source /home/ros2_ws/install/setup.sh && ros2 launch sm_atomic sm_atomic.launch'
