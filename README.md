# SMACC2

SMACC2 is a state-machine framework for ROS2-based applications written in C++.


## Build Status

 Rolling |
:------: |
[master](https://github.com/robosoft-ai/SMACC2/tree/master) |
[![Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/ci-build-binary.yml/badge.svg)](https://github.com/robosoft-ai/SMACC2/actions/workflows/ci-build-binary.yml) |
[![Semi-Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/ci-build-semi-binary.yml/badge.svg)](https://github.com/robosoft-ai/SMACC2/actions/workflows/ci-build-semi-binary.yml) |
[![Source Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/ci-build-source.yml/badge.svg)](https://github.com/robosoft-ai/SMACC2/actions/workflows/ci-build-source.yml)


## Repository Structure

- `smacc2` - core library of SMACC2.
- `smacc_client_library` - client libraries for SMACC2, e.g., Navigation2 (`move_base_z_client`), MoveIt2 (`move_group_interface_client`).
- `smacc_event_generators` - ...
- `smacc_msgs` - ROS2 messages for SMACC2 framework.
- `smacc_sm_reference_library` - libraries with reference implementations of state-machines used for demonstaration and testing of functionalities.
- `↓smacc_state_reactor_library` - ...


## Getting started

1. [Install ROS2 Rolling](https://index.ros.org/doc/ros2/Installation/Rolling/Linux-Install-Debians/).

2. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

3. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/workspace/ros_ws_rolling_smacc
   mkdir -p $COLCON_WS/src
   ```

4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
   ```
   cd $COLCON_WS
   git clone https://github.com/robosoft-ai/SMACC2.git src/SMACC2
   vcs import src --skip-existing --input src/SMACC2/SMACC2-not-released.rolling.repos
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Examples

To demonstrate some functionalities of the SMACC2 you can use following examples:

### sm_atomic
TBD: add some description
```
ros2 launch sm_atomic sm_atomic.py
```

### sm_dance_bot
TBD: add some description
```
ros2 launch sm_dance_bot sm_dance_bot_launch.py
```
