# SMACC2 Tracing

This small example shows the steps to follow to trace a smacc application.
First, make sure that smacc and tracetools installed (or built in your workspace) in your system.
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
      ros2:client_behavior_on_exit_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:client_behavior_on_exit_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:client_behavior_on_entry_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:client_behavior_on_entry_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:state_onExit_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:state_onExit_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:state_onEntry_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:state_onEntry_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:state_onRuntimeConfigure_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:state_onRuntimeConfigure_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:update_end (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:update_start (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
      ros2:smacc_event (loglevel: TRACE_DEBUG_LINE (13)) (type: tracepoint)
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

First ensure you have the necessary package installed.
```
sudo apt-get install ros-rolling-ros2trace
```

Then run this command.
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
