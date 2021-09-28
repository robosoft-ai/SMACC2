 <h2>State Machine Diagram</h2>

 ! New Image required

 <h2>Description</h2> ! New Description required.<br></br>
<a href="https://robosoft-ai.github.io/SMACC2_Documentation/master/html/namespacesm__atomic__subscribers__performance__test.html">Doxygen Namespace & Class Reference</a>

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
ros2 launch sm_atomic_subscribers_performance_test sm_atomic_subscribers_performance_test.launch
```

 !
 [ERROR] [sm_atomic_subscribers_performance_test_node-1]: process has died [pid 19996, exit code -11, cmd '/home/dec/workspace/rolling_ws/install/sm_atomic_subscribers_performance_test/lib/sm_atomic_subscribers_performance_test/sm_atomic_subscribers_performance_test_node --ros-args -r __node:=sm_atomic_performance_test --params-file /home/dec/workspace/rolling_ws/install/sm_atomic_performance_test/share/sm_atomic_performance_test/config/sm_atomic_config.yaml'].

! Are there more steps to running this sm?

 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>.
