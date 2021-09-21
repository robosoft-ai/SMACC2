## How to use this Dockerfile

Docker allows you to run your applications in a lean environment known as a container.

This dockerfile allows you to select your preferred ROS distro as an argument at runtime.
To build the docker image use this command:


    sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/


ROS rolling will act as default if no argument is provided.
