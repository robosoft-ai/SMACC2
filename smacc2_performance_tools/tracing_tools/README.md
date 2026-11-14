# SMACC2 Tracing

## Timing summary in the terminal

`smacc2_trace.sh` runs one state machine node under an LTTng userspace session, converts the trace
to text with `babeltrace` and prints a timing summary: polling loop rate and jitter, state and client
behavior entry and exit durations, `update()` durations, transition latency and the time spent in
each state. It needs `lttng-tools` and `babeltrace`; it needs no root, no `tracing` group and no
graphical viewer.

```
smacc2_trace.sh <out-dir> <seconds> <node-binary> [node args...]
```

For example, from a sourced workspace:

```
smacc2_performance_tools/tracing_tools/smacc2_trace.sh /tmp/trace_sm_atomic 20 \
  install/sm_atomic/lib/sm_atomic/sm_atomic_node --ros-args \
  --params-file install/sm_atomic/share/sm_atomic/config/sm_atomic_config.yaml -r __node:=sm_atomic
```

The output directory receives the binary trace (`trace/`), the text trace (`trace.txt`, one event per
line with a nanosecond timestamp), the node's log and `summary.txt`. `trace_summary.py <trace.txt>`
summarizes an existing text trace again. Run the node binary directly and give it the node name its
launch file uses, or its config file does not apply.

## Setting up and recording by hand

This small example shows the steps to follow to trace a SMACC2 library.
First, make sure that smacc and tracetools installed (or built in your workspace) in your system.

See README.md for setting up a SMACC workspace.

Setting up LTTng

```
sudo apt-add-repository ppa:lttng/stable-2.12
sudo apt-get update
sudo apt-get install lttng-tools lttng-modules-dkms liblttng-ust-dev
```

Create and add your user to a new tracing group

```
newgrp tracing
sudo usermod -aG tracing `whoami`
```
Reboot or log out and back in to your user to update user groups

```
sudo su $USER
```
Install Python packages for reading and setting up tracing sessions

```
sudo apt-get install python3-babeltrace python3-lttng
```
In your SMACC2 workspace, install the following tracing repos.

```
wget https://gitlab.com/ros-tracing/ros2_tracing/raw/master/tracing.repos
vcs import src < tracing.repos
```
Finally build your workspace

```
colcon build
source install/setup.bash
```
Then start your smacc application:

```
ros2 launch sm_three_some sm_three_some.launch
```
Once the application is running you can check if the tracepoints are available via the lttng command:

```
lttng list --userspace
```
You should get the list of nodes with tracepoints available. Below it is shown a chunk of the output got with that command, we see how the smacc and the default ros2 tracepoint are available:

```
[...]
ID: 1663722 - Name: /home/geus/Desktop/smacc_tracing/install/sm_three_some/lib/sm_three_some/sm_three_some_node
      ros2:smacc2_client_behavior_on_exit_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_client_behavior_on_exit_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_client_behavior_on_entry_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_client_behavior_on_entry_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_state_onExit_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_state_onExit_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_state_onEntry_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_state_onEntry_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_state_onRuntimeConfigure_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_state_onRuntimeConfigure_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_state_update_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_state_update_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc2_event (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:spinOnce (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:callback_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:callback_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rclcpp_callback_register (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rclcpp_timer_callback_added (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rcl_timer_init (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rcl_client_init (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rclcpp_service_callback_added (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rcl_service_init (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rclcpp_subscription_callback_added (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rclcpp_subscription_init (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rcl_subscription_init (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rcl_publisher_init (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rcl_node_init (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:rcl_init (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      [...]
```
Now lets start a recording session. We use a custom command ```trace.sh```, it is essentially an extended version of ```ros2 trace``` but also adding the smacc tracepoints to be recorded:

```
ros2 run smacc2 trace.sh
```
We press a key, record some seconds and then stop the recording pressing again some key:

```
ros2 run smacc2 trace.sh
UST tracing enabled (33 events)
kernel tracing enabled (4 events)
context (3 names)
writing tracing session to: /home/geus/.ros/tracing/session-20210528232348
press enter to start...
press enter to stop...
stopping & destroying tracing session
```
We can check the tracing database using the babeltrace tool or the tracecompass tool.
In our case we use trace compass. Importing the data from the tracing output directory.

![image](https://user-images.githubusercontent.com/13334595/120043632-9b258500-c00c-11eb-9e64-cb5507c46f49.png)
