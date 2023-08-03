 <h2>State Machine Diagram</h2>

 ! New Image required

 <h2>Description</h2> ! New Description required.<br></br>
<a href="https://robosoft-ai.github.io/smacc2_doxygen/master/html/namespacesm__atomic__subscribers__performance__test.html">Doxygen Namespace & Class Reference</a>

 <h2>Build Instructions</h2>

First, source your ros2 installation.
```
source /opt/ros/humble/setup.bash
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
After you build, remember to source the proper workspace...

```
source ~/workspace/humble_ws/install/setup.sh
```

And then run the launch file...

```
ros2 launch sm_atomic_subscribers_performance_test sm_atomic_subscribers_performance_test.launch
```
 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>.
