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

RUN git clone https://github.com/robosoft-ai/SMACC2.git

WORKDIR "/home/ros2_ws"

RUN vcs import src --skip-existing --input src/SMACC2/SMACC2-not-released.rolling.repos \
  && rosdep install --ignore-src --from-paths src -y -r \
  && bash -c "source /opt/ros/rolling/setup.sh && colcon build --merge-install"

# ENTRYPOINT /bin/bash -c 'source /opt/ros/rolling/setup.sh && source /home/ros2_ws/install/setup.sh && ros2 launch sm_atomic sm_atomic.launch'
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
