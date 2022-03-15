





# SMACC2

SMACC2 is an event-driven, asynchronous, behavioral state machine library for real-time ROS2 (Robotic Operating System) applications written in C++, designed to allow programmers to build robot control applications for multicomponent robots, in an intuitive and systematic manner.

SMACC was inspired by Harel's statecharts and the [SMACH ROS package](http://wiki.ros.org/smach). SMACC is built on top of the [Boost StateChart library](https://www.boost.org/doc/libs/1_53_0/libs/statechart/doc/index.html).


## Repository Status, Packages and Documentation

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Foxy** | [`foxy`](https://github.com/robosoft-ai/SMACC2/tree/foxy) | [![Foxy Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/foxy-binary-build.yml/badge.svg?branch=foxy)](https://github.com/robosoft-ai/SMACC2/actions/workflows/foxy-binary-build.yml?branch=foxy) <br /> [![Foxy Semi-Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/foxy-semi-binary-build.yml/badge.svg?branch=foxy)](https://github.com/robosoft-ai/SMACC2/actions/workflows/foxy-semi-binary-build.yml?branch=foxy) <br /> [![Foxy Source Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/foxy-source-build.yml/badge.svg?branch=foxy)](https://github.com/robosoft-ai/SMACC2/actions/workflows/foxy-source-build.yml?branch=foxy) | [![Doxygen Doc Deployment](https://github.com/robosoft-ai/SMACC2/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/robosoft-ai/SMACC2/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://robosoft-ai.github.io/SMACC2_Documentation/foxy/html/namespacesmacc2.html) | [SMACC2](https://index.ros.org/p/SMACC2/#foxy)
**Galactic** | [`galactic`](https://github.com/robosoft-ai/SMACC2/tree/galactic) | [![Galactic Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/galactic-binary-build.yml/badge.svg?branch=galactic)](https://github.com/robosoft-ai/SMACC2/actions/workflows/galactic-binary-build.yml?branch=galactic) <br /> [![Galactic Semi-Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/galactic-semi-binary-build.yml/badge.svg?branch=galactic)](https://github.com/robosoft-ai/SMACC2/actions/workflows/galactic-semi-binary-build.yml?branch=galactic) <br /> [![Galactic Source Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/galactic-source-build.yml/badge.svg?branch=galactic)](https://github.com/robosoft-ai/SMACC2/actions/workflows/galactic-source-build.yml?branch=galactic) | [![Doxygen Doc Deployment](https://github.com/robosoft-ai/SMACC2/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/robosoft-ai/SMACC2/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://robosoft-ai.github.io/SMACC2_Documentation/galactic/html/namespacesmacc2.html) | [SMACC2](https://index.ros.org/p/SMACC2/#galactic)
**Rolling - 20.04 (Focal)** | [`rolling`](https://github.com/robosoft-ai/SMACC2/tree/rolling) | [![Rolling Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-binary-build-last-focal.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-semi-binary-build-last-focal.yml/badge.svg?branch=master)](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-semi-binary-build-last-focal.yml?branch=master) | [![Doxygen Doc Deployment](https://github.com/robosoft-ai/SMACC2/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/robosoft-ai/SMACC2/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://robosoft-ai.github.io/SMACC2_Documentation/master/html/namespacesmacc2.html) | [SMACC2](https://index.ros.org/p/SMACC2/#rolling)
**Rolling - 22.04 (Jammy)** | [`rolling`](https://github.com/robosoft-ai/SMACC2/tree/rolling) | [![Rolling Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-binary-build.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-semi-binary-build.yml?branch=master) <br /> [![Rolling Source Build](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-source-build.yml/badge.svg?branch=master)](https://github.com/robosoft-ai/SMACC2/actions/workflows/rolling-source-build.yml?branch=master) | [![Doxygen Doc Deployment](https://github.com/robosoft-ai/SMACC2/actions/workflows/doxygen-deploy.yml/badge.svg)](https://github.com/robosoft-ai/SMACC2/actions/workflows/doxygen-deploy.yml) <br /> [Generated Doc](https://robosoft-ai.github.io/SMACC2_Documentation/master/html/namespacesmacc2.html) | [SMACC2](https://index.ros.org/p/SMACC2/#rolling)

**NOTE**: There are three build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/SMACC2/SMACC2-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/SMACC2/SMACC2.repos`

1. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.


## Getting started - ROS Rolling + Ubuntu 20.04 (Focal)

1. [Install ROS2 Rolling](https://index.ros.org/doc/ros2/Installation/Rolling/Linux-Install-Debians/).

2. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```
3. Create a new ROS2 workspace if necessary:
   ```
   export COLCON_WS=~/workspace/rolling_ws
   mkdir -p $COLCON_WS/src
   ```
4. Or just navigate to your workspace source folder:
   ```
   cd ~/workspace/rolling_ws/src
   ```
5. Clone the repo:
   ```
   git clone https://github.com/robosoft-ai/SMACC2.git
   ```
6. Navigate to the workspace:
   ```
   cd ~/workspace/rolling_ws
   ```
7. Update System:
   ```
   sudo apt update
   sudo apt upgrade
   ```
8. Source the workspace:
   ```
   source /opt/ros/rolling/setup.bash
   ```
9. Update dependencies:
   ```
   rosdep update
   ```
10. Pull relevant packages and install dependencies:
   ```
   vcs import src --skip-existing --input src/SMACC2/SMACC2-not-released.rolling.repos
   rosdep install --ignore-src --from-paths src -y -r
   ```
11. Compile:
   ```
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## Getting started - ROS Galactic

1. [Install ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

2. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```
3. Create a new ROS2 workspace if necessary:
   ```
   export COLCON_WS=~/workspace/galactic_ws
   mkdir -p $COLCON_WS/src
   ```
4. Or just navigate to your workspace source folder:
   ```
   cd ~/workspace/galactic_ws/src
   ```
5. Clone the repo:
   ```
   git clone https://github.com/robosoft-ai/SMACC2.git
   ```
6. Checkout the Galactic branch:
   ```
   cd ~/workspace/galactic_ws/src/SMACC2
   git checkout galactic
   ```
7. Navigate to the workspace:
   ```
   cd ~/workspace/galactic_ws
   ```
8. Update System:
   ```
   sudo apt update
   sudo apt upgrade
   ```
9. Source the workspace:
   ```
   source /opt/ros/galactic/setup.bash
   ```
10. Update dependencies:
   ```
   rosdep update
   ```
11. Pull relevant packages and install dependencies:
   ```
   vcs import src --skip-existing --input src/SMACC2/SMACC2.galactic.repos
   rosdep install --ignore-src --from-paths src -y -r
   ```
12. Compile:
   ```
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## Getting started - ROS Foxy

1. [Install ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).

2. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```
3. Create a new ROS2 workspace if necessary:
   ```
   export COLCON_WS=~/workspace/foxy_ws
   mkdir -p $COLCON_WS/src
   ```
4. Or just navigate to your workspace source folder:
   ```
   cd ~/workspace/foxy_ws/src
   ```
5. Clone the repo:
   ```
   git clone https://github.com/robosoft-ai/SMACC2.git
   ```
6. Checkout the Foxy branch:
   ```
   cd ~/workspace/foxy_ws/src/SMACC2
   git checkout foxy
   ```
7. Navigate to the workspace:
   ```
   cd ~/workspace/foxy_ws
   ```
8. Update System:
   ```
   sudo apt update
   sudo apt upgrade
   ```
9. Source the workspace:
   ```
   source /opt/ros/foxy/setup.bash
   ```
10. Update dependencies:
   ```
   rosdep update
   ```
11. Pull relevant packages and install dependencies:
   ```
   vcs import src --skip-existing --input src/SMACC2/SMACC2.foxy.repos
   rosdep install --ignore-src --from-paths src -y -r
   ```
12. Compile:
   ```
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

## Features
 *  ***Powered by ROS2:*** SMACC2 has been developed specifically to work with ROS2. It supports ROS2 topics, services and actions, right out of the box.
 *   ***Written in C++:*** Until now, ROS2 has lacked a library to develop task-level behavioral state machines in C++. Although libraries have been developed in scripting languages such as python, these are unsuitable for real-world industrial environments where real-time requirements are demanded.
 *   ***Orthogonals:*** Originally conceived by David Harel in 1987, orthogonality is absolutely crucial to developing state machines for complex robotic systems. This is because complex robots are always a collection of hardware devices which require communication protocols, start-up determinism, etc. With orthogonals, it is an intuitive and relatively straight forward exercise (at least conceptually;) to code a state machine for a robot comprising a mobile base, a robotic arm, a gripper, two lidar sensors, a gps transceiver and an imu, for instance.
 *  ***Static State Machine Checking:*** One of the features that SMACC2 inherits from Boost Statechart is that you get compile time validation checking. This benefits developers in that the amount of runtime testing necessary to ship quality software that is both stable and safe is dramatically reduced. Our philosophy is "Wherever possible, let the compiler do it".
 *  ***State Machine Reference Library:*** With a constantly growing library of out-of-the-box reference state machines, (found in the folder [sm_reference_library](smacc2_sm_reference_library)) guaranteed to compile and run, you can jumpstart your development efforts by choosing a reference machine that is closest to your needs, and then customize and extend to meet the specific requirements of your robotic application. All the while knowing that the library supports advanced functionalities that are practically universal among actual working robots.
 *  ***SMACC2 Client Library:*** SMACC2 also features a constantly growing library of [clients](smacc2_client_library) that support ROS2 Action Servers, Service Servers and other nodes right out-of-the box. The clients within the SMACC2 Client library have been built utilizing a component based architecture that allows for developer to build powerful clients of their own. Current clients of note include MoveBaseZ, a full featured Action Client built to integrate with Nav2, the ros_timer_client, the multi_role_sensor_client, and a keyboard_client used extensively for state machine drafting & debugging.
  *  ***Extensive Documentation:*** Although many ROS users are familiar with doxygen, our development team has spent a lot of time researching the more advanced features of doxygen such as uml style class diagrams and call graphs, and we've used them to document the SMACC2 library. Have a look to [our doxygen sites](https://robosoft-ai.github.io/SMACC2_Documentation/master/html/namespaces.html) and we think you'll be blown away at what Doxygen looks like when [it's done right](https://robosoft-ai.github.io/SMACC2_Documentation/master/html/classsmacc2_1_1ISmaccStateMachine.html) and it becomes a powerful tool to research a codebase.
  *  ***SMACC2 Runtime Analyzer:*** The SMACC2 library works out of the box with the SMACC2 RTA. This allows developers to visualize and runtime debug the state machines they are working on. The SMACC2 RTA is closed source, but is free for individual and academic use. It can be found [here](https://robosoft.ai/product-category/smacc2-runtime-analyzer/).


## Repository Structure
- `smacc2` - core library of SMACC2.
- `smacc2_ci` - ...
- `smacc2_client_library` - client libraries for SMACC2, e.g., Navigation2 (`nav2z_client`), MoveIt2 (`move_group_interface_client`).
- `smacc2_event_generators` - ...
- `smacc2_msgs` - ROS2 messages for SMACC2 framework.
- `smacc2_sm_reference_library` - libraries with reference implementations of state-machines used for demonstaration and testing of functionalities.
- `↓smacc2_state_reactor_library` - ...
- `smacc2_performance_tools` - ...



## SMACC2 applications
From it's inception, SMACC2 was written to support the programming of multi-component, complex robots. If your project involves small, solar-powered insect robots, that simply navigate towards a light source, then SMACC2 might not be the right choice for you. But if you are trying to program a robot with a mobile base, a robotic arm, a gripper, two lidar sensors, a gps transceiver and an imu, then you've come to the right place.


## Run a State Machine
The easiest way to get started is by selecting one of the state machines in our [reference library](smacc2_sm_reference_library), and then hacking it to meet your needs.

Each state machine in the reference library comes with it's own README.md file, which contains the appropriate operating instructions, so that all you have to do is simply copy & paste some commands into your terminal.


  *  If you are looking for a minimal example, we recommend [sm_atomic](smacc2_sm_reference_library/sm_atomic).

  *  If you are looking for a minimal example but with a looping superstate, try [sm_three_some](smacc2_sm_reference_library/sm_three_some).

  *  If you want to get started with the ROS Navigation stack right away, try [sm_dance_bot](smacc2_sm_reference_library/sm_dance_bot).

  *  If you want to get started with ROS Navigation and exploring the orthogonal read-write cycle, then try [sm_dance_bot_strikes_back](smacc2_sm_reference_library/sm_dance_bot_strikes_back).


Operating instructions can be found in each reference state machines readme file.


## Writing your State Machines
There is a [state machine generator in the reference library](smacc2_sm_reference_library/create-sm-package.bash).
To use it go to the `src` folder of your ROS2 workspace and execute:
  ```
  smacc2/smacc2_sm_reference_library/create-sm-package.bash <name_of_state_machine>
  ```
After than compile your workspace and source it to set paths of the new package.
Check `README.md` in new package about instructions to start newly created state machine.

Happy Coding!

## Support
If you are interested in getting involved or need a little support, feel free to contact us by emailing techsupport@robosoft.ai
