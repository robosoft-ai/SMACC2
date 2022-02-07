ARG ROS_DISTRO=rolling
ARG GIT_BRANCH=master

FROM osrf/ros:$ROS_DISTRO-desktop

ARG ROS_DISTRO
ARG GIT_BRANCH

RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    && apt-get clean \
    && apt upgrade -y --with-new-pkgs


WORKDIR "/home/ros2_ws/src"
RUN echo "git clone -b $GIT_BRANCH"
RUN git clone -b $GIT_BRANCH https://github.com/robosoft-ai/SMACC2.git /home/ros2_ws/src/SMACC2

# WORKDIR "/home/ros2_ws"

WORKDIR "/home/ros2_ws"

# install dependencies and build
RUN ls src && vcs import  src --skip-existing --input src/SMACC2-not-released.rolling.repos \
  && ls src

RUN rosdep install --from-paths src -r -y
RUN apt-get update && apt-get install -q -y --no-install-recommends xterm

RUN bash -c "source /opt/ros/rolling/setup.sh && colcon build --merge-install"
