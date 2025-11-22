# sm_multithread_test_1 - SMACC2 Multi-threaded Executor Demonstration

**A reference implementation demonstrating SMACC2's multi-threaded executor capability.**

> 🎯 **Addresses GitHub Issue [#571](https://github.com/robosoft-ai/SMACC2/issues/571)**: "Multithread executor example"

---

## 📋 Table of Contents

- [What is This?](#what-is-this)
- [Quick Start](#quick-start)
- [The Problem This Solves](#the-problem-this-solves)
- [What You'll Observe](#what-youll-observe)
- [How It Works](#how-it-works)
- [When to Use Multi-threaded Mode](#when-to-use-multi-threaded-mode)
- [Important Limitations](#important-limitations)
- [Code Deep Dive](#code-deep-dive)
- [Build Instructions](#build-instructions)
- [Troubleshooting](#troubleshooting)
- [Further Reading](#further-reading)

---

## What is This?

This state machine demonstrates **SMACC2's multi-threaded executor feature** by running four concurrent timers with simulated work. The logs clearly show the difference between single-threaded and multi-threaded execution through thread IDs and overlapping timestamps.

**Key Features:**
- ✅ Multi-threaded ROS2 callback processing
- ✅ Visual demonstration of concurrency via logs
- ✅ Side-by-side comparison with single-threaded mode
- ✅ Comprehensive documentation of usage and limitations

---

## Quick Start

### Run Multi-threaded Version (Main Demo)

```bash
# Build the package
colcon build --packages-select sm_multithread_test_1

# Source the workspace
source install/setup.bash

# Launch multi-threaded version
ros2 launch sm_multithread_test_1 sm_multithread_test_1.py
```

### Run Single-threaded Version (For Comparison)

```bash
# Launch single-threaded version
ros2 launch sm_multithread_test_1 sm_multithread_test_1_single.py
```

The state machine runs for **15 seconds** then enters a terminal state. Press `Ctrl+C` to exit.

---

## The Problem This Solves

### Background

By default, SMACC2 uses a **single-threaded executor** where ROS2 callbacks (timers, subscribers, action clients) execute sequentially. This is simple and deterministic, but can be a bottleneck when:

- Multiple callbacks have significant processing time
- Callbacks involve I/O operations (network, disk, sensors)
- You want maximum throughput for concurrent operations

### Solution

SMACC2 supports ROS2's **Multi-threaded Executor** which allows callbacks to execute concurrently on multiple threads, improving throughput for I/O-bound and concurrent operations.

**Enabling it is trivial** – just one parameter change:

```cpp
// Single-threaded (default)
smacc2::run<MyStateMachine>();

// Multi-threaded
smacc2::run<MyStateMachine>(smacc2::ExecutionModel::MULTI_THREAD_SPINNER);
                            //  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                            //  THIS IS THE ONLY CHANGE NEEDED!
```

---

## What You'll Observe

###  Multi-threaded Mode Output

```
[Timer-A] Tick   1 START (Thread: 12345678) - simulating  50ms work
[Timer-B] Tick   1 START (Thread: 87654321) - simulating 100ms work  ← Different thread!
[Timer-A] Tick   1 END   (Thread: 12345678)
[Timer-C] Tick   1 START (Thread: 11223344) - simulating 150ms work  ← Concurrent!
[Timer-A] Tick   2 START (Thread: 12345678) - simulating  50ms work
[Timer-B] Tick   1 END   (Thread: 87654321)
[Timer-A] Tick   2 END   (Thread: 12345678)
[Timer-D] Tick   1 START (Thread: 99887766) - simulating 200ms work
```

**Observations:**
- ✅ **Different thread IDs** (12345678, 87654321, 11223344, 99887766)
- ✅ **Overlapping START/END timestamps**
- ✅ **True parallel execution**

### Single-threaded Mode Output

```
[Timer-A] Tick   1 START (Thread: 12345678) - simulating  50ms work
[Timer-A] Tick   1 END   (Thread: 12345678)
[Timer-A] Tick   2 START (Thread: 12345678) - simulating  50ms work
[Timer-A] Tick   2 END   (Thread: 12345678)
[Timer-B] Tick   1 START (Thread: 12345678) - simulating 100ms work  ← Same thread
[Timer-B] Tick   1 END   (Thread: 12345678)
[Timer-C] Tick   1 START (Thread: 12345678) - simulating 150ms work  ← Serial execution
```

**Observations:**
- ✅ **Same thread ID for all** (12345678)
- ✅ **No overlapping timestamps**
- ✅ **Serial execution** (one callback at a time)

---

## How It Works

### Architecture

The state machine consists of:

1. **4 Orthogonals** – each with a timer at a different rate:
   - `OrTimerA`: 100ms period
   - `OrTimerB`: 250ms period
   - `OrTimerC`: 500ms period
   - `OrTimerD`: 1000ms period

2. **Custom Client Behavior** – `CbTimerWithWorkSimulation`:
   - Simulates work with `std::this_thread::sleep_for()`
   - Logs thread ID on each callback
   - Logs START/END timestamps
   - Makes concurrency visually obvious

3. **Two States**:
   - `StConcurrentOperation`: Runs timers for 15 seconds
   - `StComplete`: Terminal state with summary

### Timer Configuration

| Timer | Period | Work Duration | Purpose |
|-------|--------|---------------|---------|
| A | 100ms | 50ms | Fast, light work |
| B | 250ms | 100ms | Medium, medium work |
| C | 500ms | 150ms | Slow, heavy work |
| D | 1000ms | 200ms | Very slow, very heavy work |

The different rates and work durations create interesting interleaving patterns that clearly demonstrate concurrency.

---

## When to Use Multi-threaded Mode

### ✅ Use Multi-threaded When:

1. **Multiple concurrent ROS2 callbacks**
   - Multiple timer callbacks
   - Multiple subscriber callbacks
   - Parallel action client callbacks

2. **Callbacks have blocking operations**
   - Network I/O
   - Disk I/O
   - Sensor data acquisition
   - Long computations

3. **Need maximum throughput**
   - High-frequency data streams
   - Real-time sensor fusion
   - Parallel goal processing

4. **Using callback-based patterns only**
   - Timers ✅
   - Subscribers ✅
   - Action clients ✅
   - Service clients ✅

### ❌ Avoid Multi-threaded When:

1. **Using `ISmaccUpdatable` pattern**
   - Components/behaviors with `update()` method
   - Polling-based patterns
   - Custom periodic logic

2. **Simple state machines**
   - Minimal concurrent operations
   - No performance bottleneck

3. **Debugging complex behavior**
   - Single-threaded is easier to trace
   - More deterministic execution

4. **Need strict determinism**
   - Single-threaded provides guaranteed order
   - Multi-threaded has non-deterministic scheduling

---

## Important Limitations

### ⚠️ ISmaccUpdatable Pattern Does NOT Work

**The `update()` method is never called in multi-threaded mode!**

**Why?** The SignalDetector's `pollOnce()` function (which calls `update()` on all `ISmaccUpdatable` objects) is only invoked in single-threaded mode's polling loop. Multi-threaded mode uses `executor.spin()` which never calls `pollOnce()`.

**Affected Patterns:**
- ❌ Components inheriting from `ISmaccUpdatable`
- ❌ Behaviors inheriting from `ISmaccUpdatable`
- ❌ States inheriting from `ISmaccUpdatable`
- ❌ Any custom `update()` methods

**Solution:**
Use ROS2 callback-based patterns instead:
- ✅ Timer callbacks via `ClRos2Timer`
- ✅ Subscriber callbacks via `CpTopicSubscriber`
- ✅ Action client callbacks
- ✅ Service client callbacks

**Example Migration:**

```cpp
// ❌ DON'T: This won't work in multi-threaded mode
class MyCb : public SmaccClientBehavior, public ISmaccUpdatable
{
  void update() override {
    // This is NEVER called in multi-threaded mode!
    checkCondition();
  }
};

// ✅ DO: Use timer callbacks instead
class MyCb : public SmaccClientBehavior
{
  void onEntry() override {
    requiresClient(timerClient_);
    timerComponent->onTimerTick(&MyCb::onTimerCallback, this);
  }

  void onTimerCallback() {
    // This WILL be called in multi-threaded mode
    checkCondition();
  }
};
```

---

## Code Deep Dive

### The One-Line Change

The **only difference** between single-threaded and multi-threaded SMACC2 is in the main node file:

**Multi-threaded** (`sm_multithread_test_1_node.cpp`):
```cpp
smacc2::run<SmMultithreadTest1>(
  smacc2::ExecutionModel::MULTI_THREAD_SPINNER  // ← ADD THIS
);
```

**Single-threaded** (`sm_multithread_test_1_node_single.cpp`):
```cpp
smacc2::run<SmMultithreadTest1>(
  smacc2::ExecutionModel::SINGLE_THREAD_SPINNER  // ← Or omit (default)
);
// Equivalent to:
smacc2::run<SmMultithreadTest1>();
```

### Implementation Details

#### Signal Detector Logic

From `smacc2/src/smacc2/signal_detector.cpp:365-386`:

```cpp
if (this->executionModel_ == ExecutionModel::SINGLE_THREAD_SPINNER)
{
  RCLCPP_INFO_STREAM(getLogger(), "[SignalDetector] Running in single-threaded mode.");

  rclcpp::Rate r(loop_rate_hz);  // Default 20Hz
  while (rclcpp::ok() && !end_)
  {
    pollOnce();              // ✅ Calls update() on ISmaccUpdatable objects
    rclcpp::spin_some(nh);   // Process callbacks
    r.sleep();
  }
}
else
{
  RCLCPP_INFO_STREAM(getLogger(), "[SignalDetector] Running in multi-threaded mode.");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(nh);
  executor.spin();  // ❌ pollOnce() NEVER CALLED
}
```

**Key Insight:** Multi-threaded mode skips `pollOnce()`, so `ISmaccUpdatable::update()` is never invoked.

#### Custom Behavior Implementation

`cb_timer_with_work_simulation.hpp` demonstrates the pattern:

```cpp
class CbTimerWithWorkSimulation : public smacc2::SmaccClientBehavior
{
  void onEntry() override {
    // Connect to timer callback
    timerComponent->onTimerTick(&CbTimerWithWorkSimulation::onTimerCallback, this);
  }

  void onTimerCallback() {
    auto thread_id = std::this_thread::get_id();  // Log thread ID
    std::size_t thread_hash = std::hash<std::thread::id>{}(thread_id);

    RCLCPP_INFO(getLogger(), "[Timer-%s] Tick %d START (Thread: %zu)",
                timerName_.c_str(), tickCount_, thread_hash);

    std::this_thread::sleep_for(workDuration_);  // Simulate work

    RCLCPP_INFO(getLogger(), "[Timer-%s] Tick %d END   (Thread: %zu)",
                timerName_.c_str(), tickCount_, thread_hash);
  }
};
```

---

## Build Instructions

### Prerequisites

- ROS 2 (jazzy or later)
- SMACC2 framework installed
- `cl_ros2_timer` client library

### Building

```bash
# Navigate to your workspace
cd ~/your_workspace

# Build the package
colcon build --packages-select sm_multithread_test_1

# Source the workspace
source install/setup.bash
```

### Running

```bash
# Multi-threaded version (main demonstration)
ros2 launch sm_multithread_test_1 sm_multithread_test_1.py

# Single-threaded version (for comparison)
ros2 launch sm_multithread_test_1 sm_multithread_test_1_single.py
```

---

## Troubleshooting

### "No matching function for call to 'run'"

**Solution:** Ensure you're using a recent version of SMACC2 that supports the `ExecutionModel` parameter.

### Logs show same thread ID in multi-threaded mode

**Possible causes:**
1. Launched the wrong executable (single-threaded variant)
2. ROS 2 executor configuration issue

**Check:** Look for this log line at startup:
```
[SignalDetector] Running in multi-threaded mode.
```

If you see "single-threaded mode", you launched the wrong variant.

### Behaviors not executing

**Check if they inherit from `ISmaccUpdatable`** – this pattern doesn't work in multi-threaded mode. Migrate to timer/subscriber callbacks instead.

---

## Further Reading

### SMACC2 Documentation
- [SMACC2 Official Docs](https://smacc2.robosoft.ai/)
- [SMACC2 GitHub Repository](https://github.com/robosoft-ai/SMACC2)

### Related Issues
- [GitHub Issue #571](https://github.com/robosoft-ai/SMACC2/issues/571) - Original request for multithread executor documentation

### ROS 2 Documentation
- [ROS 2 Executors](https://docs.ros.org/en/jazzy/Concepts/About-Executors.html)
- [MultiThreadedExecutor API](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1executors_1_1MultiThreadedExecutor.html)

### Code Files
- `sm_multithread_test_1_node.cpp` - Multi-threaded entry point
- `sm_multithread_test_1_node_single.cpp` - Single-threaded entry point
- `cb_timer_with_work_simulation.hpp` - Custom behavior with thread logging
- `signal_detector.cpp` - SignalDetector implementation with execution model selection

---

## License

Copyright 2025 RobosoftAI Inc.

Licensed under the Apache License, Version 2.0
