 <h2>State Machine Diagram</h2>

 ![sm_multi_stage_1](docs/SmMultiStage1_2021-11-29_0916.svg)

 <h2>Description</h2> A large multi stage state machine that demonstrates that the NxN problem is not as formidable as some would have you believe.<br></br>

<a href="https://robosoft-ai.github.io/SMACC2_Documentation/master/html/namespacesm__multi__stage__1.html">Doxygen Namespace & Class Reference</a>

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
ros2 launch sm_multi_stage_1 sm_multi_stage_1.launch
```

 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>.
