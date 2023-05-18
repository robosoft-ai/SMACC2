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

# # -------- moveit ----------
# RUN git clone https://github.com/ros-planning/moveit2_tutorials.git -b humble src/moveit2_tutorials
# RUN vcs import < src/moveit2_tutorials/moveit2_tutorials.repos

# RUN apt-get update
# RUN rosdep install --from-paths src --ignore-src -r -y
RUN apt-get update && apt-get install -q -y --no-install-recommends xterm

WORKDIR "/home/ros2_ws"
#ADD . /home/ros2_ws/src/SMACC2
RUN git clone https://github.com/robosoft-ai/SMACC2.git -b humble src/SMACC2

# install dependencies and build
RUN vcs import  src --skip-existing --input src/SMACC2/.github/SMACC2-not-released.$ROS_DISTRO.repos
RUN ls src
RUN rosdep install --from-paths src --ignore-src -r -y

RUN git clone https://github.com/ros2/ros_testing.git src/ros_testing

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.sh && colcon build --merge-install --parallel-workers 1"
