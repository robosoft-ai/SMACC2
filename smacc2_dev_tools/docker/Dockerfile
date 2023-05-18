ARG ROS_DISTRO=rolling
ARG GIT_BRANCH=master
ARG UBUNTU_VERSION=focal

FROM ros:$ROS_DISTRO-ros-base-$UBUNTU_VERSION

ARG ROS_DISTRO
ARG GIT_BRANCH
ARG LOCAL_FOLDER_SOURCE=1

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

RUN echo "copying current code version to docker image:"
ADD . /home/ros2_ws/src/SMACC2
WORKDIR "/home/ros2_ws"

# install dependencies and build
RUN vcs import  src --skip-existing --input src/SMACC2/.github/SMACC2-not-released.$ROS_DISTRO.repos
RUN ls src

RUN apt update
RUN rosdep install --from-paths src --ignore-src -r -y
RUN apt-get update && apt-get install -q -y --no-install-recommends xterm

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.sh && colcon build --merge-install --parallel-workers 1"
