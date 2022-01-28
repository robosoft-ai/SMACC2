 <h2>Build Instructions</h2>

First, source your chosen ros2 distro.
```
source /opt/ros/rolling/setup.bash
```
```
source /opt/ros/galactic/setup.bash
```

Before you build, make sure you've installed all the dependencies...

```
rosdep install --ignore-src --from-paths src -y -r
```

Then build with colcon build...

```
colcon build
```
 <h2>Operating Instructions</h2>
After you build, remember to source the proper install folder...

```
source ~/<ros2_ws>/install/setup.bash
```

And then run the launch file...

```
ros2 launch sm_hercules_1 sm_hercules_1.launch
```

 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>


 <h2>Hercules testing instructions</h2>

"Hercules" robot consists of Husky mobile base and two UR manipulators.

To start simulation with test setup start the following commands.


 <h3>Dual UR manipulators</h3>

```
ros2 launch sm_hercules_1 dual_ur_sim_control.launch.py
```

Start test script to see arm movement with the following command (wait 6 seconds until arm start to move):

```
ros2 launch sm_hercules_1 test_dual_joint_trajectory_controller.launch.py
```

 <h3>Hercules</h3>

```
ros2 launch sm_hercules_1 hercules_sim_control.launch.py
```

Start test script to see arm movement with the following command (wait 6 seconds until arm start to move):

```
ros2 launch sm_hercules_1 test_dual_joint_trajectory_controller.launch.py
```

Send command for the base:

```
ros2 topic pub /base_velocity_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.5"
```

Send command for grippers:
```
ros2 topic pub /port_gripper_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.6"
```

```
ros2 topic pub /starboard_gripper_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.6"
```



  <h3> Running Ignition simulation </h3>

To run simulation in Ignition add the following arguments to launch files:

```
sim_gazebo:=false sim_ignition:=true
```

**NOTE:** Hercules is currently not working in Ignition, but only in Gazebo.
