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

After you built the project don't forget to setup your environment: First source the gazebo setup via

```
source /opt/ros/galactic/share/gazebo_ros/local_setup.bash
```
Then source the proper install folder...

```
source ~/<ros2_ws>/install/setup.bash
```

And then run the launch file...

```
ros2 launch sm_test_moveit_ur5_sim sm_test_moveit_ur5_sim.launch
```

If the simulation is not starting up properly check if your GAZEBO_RESOURCE_PATH is set correctly (echo $GAZEBO_RESOURCE_PATH). Otherwise simply set it with:

```
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-<your_gazebo_version>/
```

Note: Sometimes the Gazebo server is not shutdown correctly. If you cannot restart the simulation you might have to look if there is still a instance of the server (gzserver) running. You then have to end it manually. Or just run:
```
killall -9 gzserver
```

 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>
