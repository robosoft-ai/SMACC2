 <h2>State Machine Diagram</h2>
<img  src="https://github.com/reelrbtx/SMACC/blob/master/smacc_sm_reference_library/sm_dance_bot_strikes_back/docs/smacc_state_machine_20200222-122229.dot.svg" width="950" align="center" border="10"/>

<h2>Description</h2> A full-featured state machine example, that highlights the capabilities of SMACC & the ROS Navigation Stack via the MoveBaseZ Client. In the original sm_dance_bot state machine, the variables that controlled the movements of the robot were entirely static, but in this example we made them dynamic, in that the robot uses a lidar sensor to find the range of the wall in front of it, then moves forward that value minus some margin. This also shows the "Orthogonal Read-Write Cycle" where we are reading data from one orthogonal (in this case the lidar client, "cl_lidar" is inside the obstacle perception orthogonal "or_obstacle_perception" and writes via the client "cl_move_base_z" in the navigation orthogonal "or_navigation".<br></br>

<a href="https://reelrbtx.github.io/SMACC_Documentation/master/html/namespacesm__dance__bot__strikes__back.html">Doxygen Namespace & Class Reference</a>

To see a video of this state machine in action click <a href="https://www.youtube.com/watch?v=ucMr5Dg6UpU">here</a>.
<br></br>

<p align="center">
 <img src="https://github.com/reelrbtx/SMACC/blob/master/smacc_sm_reference_library/sm_dance_bot_strikes_back/docs/sm_dance_bot_strikes_back.JPG" width="800"/> 
 </p>
 <br></br>

<h2>Build Instructions</h2>
Before you build, make sure you've installed all the dependencies...

```
rosdep install --from-paths src --ignore-src -r -y 
```

Then you build with either catkin build or catkin make...

```
catkin build
```
<h2>Operating Instructions</h2>
After you build, remember to source the proper devel folder...

```
source ~/catkin_ws/devel/setup.bash
```

And then run the launch file for the full gazebo demo...

```
roslaunch sm_dance_bot_strikes_back sm_dance_bot_strikes_back.launch
```

Or, you can run the lightweight demo...

```
roslaunch sm_dance_bot_strikes_back sm_dance_bot_strikes_back.launch sm_xterm:=nice server_nodes_xterms:=nice show_gz_client:=false show_smacc_viewer:=false
```

This launch command starts the process by hiding the xterminals for all the servers, the state machine, the smacc_viewer and the gazebo client.

<h2>Viewer Instructions</h2>
If you have the SMACC Viewer installed then type...

```
rosrun smacc_viewer smacc_viewer_node.py
``` 

If you don't have the SMACC Viewer installed, click <a href="http://smacc.ninja/smacc-viewer/">here</a> for instructions.


