#FROM osrf/ros2:rolling-desktop
# ^^ not yet supported

# argument to allow selection of ROS distro at runtime
# use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
# rolling will act as default if no arg is provided
ARG ROS2_DISTRO=rolling

FROM ros:$ROS2_DISTRO

ARG ROS2_DISTRO=${ROS2_DISTRO}

ENV HOME /root
ENV DEBIAN_FRONTEND noninteractive
ENV DISTRO=${ROS2_DISTRO}



RUN echo "Using ROS distro: ${ROS2_DISTRO}"
RUN echo "Using ROS distro: ${DISTRO}"
RUN apt-get update; apt-get -y install cmake apt-utils
RUN apt install python3-colcon-common-extensions python3-vcstool
RUN apt-get -y upgrade && apt-get update

# Setup workspace
RUN mkdir -p /root/smacc2_ws/src
WORKDIR /root/smacc2_ws/src
RUN git clone https://github.com/robosoft-ai/SMACC2.git


WORKDIR /root/smacc2_ws

# Resolve dependencies and build workspace
#RUN vcs import src --skip-existing --input src/SMACC2/smacc2.repos
RUN rosdep install --ignore-src --from-paths src -y -r
RUN bash -c "source /opt/ros/${DISTRO}/setup.bash; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"


# source setup file then run sm_atomic
WORKDIR /root
#RUN bash -c "source /root/smacc2_ws/install/setup.bash; ros2 launch sm_atomic sm_atomic.launch"


##Beyond this point is a work in progress
## see https://github.com/robosoft-ai/SMACC2/blob/master/tracing.md

# #set up trace
# RUN lttng list --userspace
# RUN ros2 run smacc2 trace.sh
