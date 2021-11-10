FROM osrf/ros:rolling-desktop

# Make sure everything is up to date before building from source
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get clean

RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-colcon-ros \
    && apt-get clean

RUN mkdir -p /home/ros2_ws/src

WORKDIR "/home/ros2_ws/src"

#RUN git clone https://github.com/robosoft-ai/SMACC2.git
ADD . "/home/ros2_ws/src"

WORKDIR "/home/ros2_ws"

# install dependencies and build
RUN ls src && vcs import  src --skip-existing --input src/SMACC2-not-released.rolling.repos \
  && ls src

RUN rosdep install --from-paths src -r -y
RUN apt-get update && apt-get install -q -y --no-install-recommends xterm

RUN bash -c "source /opt/ros/rolling/setup.sh && colcon build --merge-install"

