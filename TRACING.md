# SMACC2 LTTng Tracing

SMACC2 instruments key lifecycle points using
[LTTng-UST](https://lttng.org/docs/v2.13/#doc-liblttng-ust) tracepoints.
When a trace session is active the library emits high-resolution timestamped
events for every state entry/exit, client behavior entry/exit,
`runtimeConfigure` call, periodic `update` call, and state machine event.
These can be correlated with standard ROS 2 rcl/rclcpp tracepoints to
produce a complete execution timeline.

---

## Prerequisites

### System packages

```bash
sudo apt install \
  liblttng-ust-dev \
  lttng-tools \
  babeltrace2
```

| Package | Purpose |
|---------|---------|
| `liblttng-ust-dev` | LTTng user-space tracer — required to **build** `smacc2` |
| `lttng-tools` | Session daemon (`lttng-sessiond`) and `lttng` CLI |
| `babeltrace2` | Read CTF trace files after capture (recommended) |

> `babeltrace` (v1) is also functional but produces less readable output.
> Install `babeltrace2` where possible.

### ROS 2 packages

```bash
sudo apt install \
  ros-jazzy-tracetools \
  ros-jazzy-tracetools-trace
```

| Package | Purpose |
|---------|---------|
| `ros-jazzy-tracetools` | Tracepoint declarations consumed by `smacc2` |
| `ros-jazzy-tracetools-trace` | Provides `ros2 run tracetools_trace trace` session tool |

---

## Starting the LTTng Session Daemon

The LTTng session daemon must be running before any trace session can be
created.  It is typically started automatically on login for desktop systems,
but in headless or Docker environments start it manually:

```bash
lttng-sessiond --daemonize
```

Verify it is available:

```bash
lttng list   # should print "Currently no available recording session"
```

---

## Running a Trace

### Option A — convenience script

The `smacc2` package installs a script that enables all SMACC2 tracepoints
plus the most useful rcl/rclcpp tracepoints:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run smacc2 trace.sh
```

The script blocks until you press **Enter** to start, then blocks again until
you press **Enter** to stop.

### Option B — manual session

Choose only the tracepoints you need:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run tracetools_trace trace \
  -s my_session \
  -u ros2:smacc2_state_onEntry_start \
     ros2:smacc2_state_onEntry_end \
     ros2:smacc2_state_onExit_start \
     ros2:smacc2_state_onExit_end \
     ros2:smacc2_state_onRuntimeConfigure_start \
     ros2:smacc2_state_onRuntimeConfigure_end \
     ros2:smacc2_state_update_start \
     ros2:smacc2_state_update_end \
     ros2:smacc2_client_behavior_on_entry_start \
     ros2:smacc2_client_behavior_on_entry_end \
     ros2:smacc2_client_behavior_on_exit_start \
     ros2:smacc2_client_behavior_on_exit_end \
     ros2:smacc2_event
```

Press **Enter** to start collecting, launch your state machine in a separate
terminal, then press **Enter** again to stop.

### Option C — add rcl/rclcpp context

Append these events to either option above to capture the full ROS 2
execution context alongside SMACC2 events:

```
ros2:rcl_init
ros2:rcl_node_init
ros2:rcl_publisher_init
ros2:rcl_subscription_init
ros2:rcl_timer_init
ros2:rclcpp_timer_callback_added
ros2:rclcpp_callback_register
ros2:callback_start
ros2:callback_end
```

---

## Trace Output Location

By default, sessions are written to:

```
~/.ros/tracing/<session-name>/
```

Use `-p /path/to/dir` to write to a custom location:

```bash
ros2 run tracetools_trace trace -s my_session -p /tmp/traces ...
```

---

## Reading Trace Data

### babeltrace2 (recommended)

```bash
babeltrace2 ~/.ros/tracing/my_session
```

Filter to SMACC2 events only:

```bash
babeltrace2 ~/.ros/tracing/my_session \
  --component filter.utils.muxer \
  2>/dev/null | grep smacc2
```

Or with a simple shell filter:

```bash
babeltrace2 ~/.ros/tracing/my_session | grep smacc2
```

### babeltrace (v1 fallback)

```bash
babeltrace ~/.ros/tracing/my_session
```

### lttng view

While a session is **still active** you can stream events live:

```bash
lttng view my_session
```

This requires `babeltrace` (v1) to be installed.  It is not usable on
sessions that have already been stopped.

---



All tracepoints use the `ros2` LTTng provider.

| Tracepoint | Fields | Description |
|------------|--------|-------------|
| `ros2:smacc2_state_onEntry_start` | `state_name` | Fired just before `onEntry()` runs |
| `ros2:smacc2_state_onEntry_end` | `state_name` | Fired just after `onEntry()` returns |
| `ros2:smacc2_state_onExit_start` | `state_name` | Fired just before `onExit()` runs |
| `ros2:smacc2_state_onExit_end` | `state_name` | Fired just after `onExit()` returns |
| `ros2:smacc2_state_onRuntimeConfigure_start` | `state_name` | Fired before `runtimeConfigure()` |
| `ros2:smacc2_state_onRuntimeConfigure_end` | `state_name` | Fired after `runtimeConfigure()` |
| `ros2:smacc2_state_update_start` | `updatable_element_name` | Fired before each `update()` tick |
| `ros2:smacc2_state_update_end` | `updatable_element_name` | Fired after each `update()` tick |
| `ros2:smacc2_client_behavior_on_entry_start` | `state_name`, `orthogonal_name`, `client_behavior_name` | Client behavior `onEntry()` begins |
| `ros2:smacc2_client_behavior_on_entry_end` | `state_name`, `orthogonal_name`, `client_behavior_name` | Client behavior `onEntry()` complete |
| `ros2:smacc2_client_behavior_on_exit_start` | `state_name`, `orthogonal_name`, `client_behavior_name` | Client behavior `onExit()` begins |
| `ros2:smacc2_client_behavior_on_exit_end` | `state_name`, `orthogonal_name`, `client_behavior_name` | Client behavior `onExit()` complete |
| `ros2:smacc2_event` | `event_type` | Fired each time the state machine receives an event |
| `ros2:spinOnce` | — | Fired each SignalDetector polling cycle |

---

## Example Output

The following is a representative excerpt from a `sm_cl_ros2_timer_unit_test_1`
trace session:

```
[16:05:15.597417] system76-pc ros2:smacc2_state_onEntry_start:
    { state_name = "sm_cl_ros2_timer_unit_test_1::State1" }

[16:05:15.597512] (+0.000095) system76-pc ros2:smacc2_state_onRuntimeConfigure_start:
    { state_name = "sm_cl_ros2_timer_unit_test_1::State1" }

[16:05:15.597632] (+0.000120) system76-pc ros2:smacc2_client_behavior_on_entry_start:
    { state_name = "sm_cl_ros2_timer_unit_test_1::State1",
      orthogonal_name = "sm_cl_ros2_timer_unit_test_1::OrTimer",
      client_behavior_name = "cl_ros2_timer::CbTimerCountdownLoop" }

[16:05:18.624434] (+3.026802) system76-pc ros2:smacc2_event:
    { event_type = "cl_ros2_timer::EvTimer<cl_ros2_timer::CbTimerCountdownLoop, ...>" }

[16:05:20.625084] (+2.000651) system76-pc ros2:smacc2_state_onExit_start:
    { state_name = "sm_cl_ros2_timer_unit_test_1::State1" }
```

---

## Troubleshooting

**`lttng list` reports "No session daemon is available"**
Run `lttng-sessiond --daemonize` to start a per-user daemon.

**Trace session starts and stops but no events appear in the output**
Ensure the state machine binary was linked against the `smacc2` library built
with LTTng enabled (`TRACETOOLS_LTTNG_ENABLED` and
`TRACETOOLS_TRACEPOINT_PROVIDER=ros2` must be defined at compile time — this
is handled automatically by the `smacc2` CMakeLists.txt).

**`ros2 trace` is not a recognized command**
The `ros2 trace` CLI verb does not exist. Use
`ros2 run tracetools_trace trace` instead.

**`babeltrace2` not found**
Install it: `sudo apt install babeltrace2`. The older `babeltrace` (v1) also
works but produces less detailed output.
