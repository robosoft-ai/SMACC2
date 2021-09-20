
#!/bin/bash
./buildRolling.sh
xhost +
sudo docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -it smacc2_rolling_ubuntu_2004 bash
