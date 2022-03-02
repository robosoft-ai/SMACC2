 <h2>State Machine Diagram</h2>

 ! New Image required

 <h2>Description</h2> A full-featured state machine example, that highlights the capabilities of SMACC2, ROS2 & Nav2 via the Nav2z Client. In the original sm_dance_bot_strikes_back state machine, the variables that controlled the movements of the robot were entirely static, but in this example we made them dynamic, in that the robot uses a lidar sensor to find the range of the wall in front of it, then moves forward that value minus some margin. This also shows the "Orthogonal Read-Write Cycle" where we are reading data from one orthogonal (in this case the lidar client, "cl_lidar" is inside the obstacle perception orthogonal "or_obstacle_perception" and writes via the client "cl_nav2z" in the navigation orthogonal "or_navigation".<br></br>
<a href="https://robosoft-ai.github.io/SMACC2_Documentation/master/html/namespacesm__dance__bot__strikes__back.html">Doxygen Namespace & Class Reference</a>

! Updated video url required
To see a video of this state machine in action click <a href="https://www.youtube.com/watch?v=ucMr5Dg6UpU">here</a>.
<br></br>

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
source ~/colcon_ws/install/setup.bash
```

And then run the launch file...

```
ros2 launch sm_dance_bot_strikes_back sm_dance_bot_strikes_back_launch.py
```

 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>.
