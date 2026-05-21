 <h2>State Machine Diagram</h2>

 ![sm_data_sharing_1](docs/SmDataSharing1_2026-5-21_125317.svg)

 <h2>Description</h2> Demonstrates sharing data across multiple states using a state-machine-scoped component. A local client (<code>ClData</code>) hosts a <code>CpMissionData</code> component that holds <code>initialPosition</code> and <code>targetPosition</code> fields. Client behaviors in each state call <code>requiresComponent(missionData_)</code> to obtain a pointer to the component — the framework searches all orthogonals and clients globally, so any behavior can access any component regardless of which orthogonal or client owns it. <code>CbStoreData1</code> stores the initial position, <code>CbStoreData2</code> stores the target position, and <code>CbProcessData</code> reads both, computes the Euclidean distance, logs the result, and resets the data for the next cycle.<br></br>

 <h2>Build Instructions</h2>

First, source your ros2 installation.
```
source /opt/ros/jazzy/setup.bash
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
After you build, remember to source the workspace...

```
source install/setup.bash
```

And then run the launch file...

```
ros2 launch sm_data_sharing_1 sm_data_sharing_1.launch.py
```

 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>.
