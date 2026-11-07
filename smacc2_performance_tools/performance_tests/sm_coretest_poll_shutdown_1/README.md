# sm_coretest_poll_shutdown_1

Core test for the signal detector's polling loop under a concurrent rclcpp shutdown. It makes
the shutdown land, in most runs, between the loop's `rclcpp::ok()` check and its
`rclcpp::Rate::sleep()`, which is the window in which `rclcpp::Clock::sleep_until` throws
`std::runtime_error("context cannot be slept with because it's invalid")`.

## Design

- `ClSlowUpdate` (in `OrSlow`) is an `ISmaccUpdatable` client whose `update()` blocks for
  `slow_update_ms` (default 150). `SignalDetector::pollOnce` calls it on the polling thread,
  inside the window.
- The polling period is set with the core parameter `signal_detector_loop_freq`; the launch
  file uses 5 Hz. With a 150 ms update in a 200 ms period, 75 % of each period is window.
- The update must stay shorter than the period. `rclcpp::Rate::sleep()` returns without
  sleeping when the iteration overran, and then never reaches the throwing call; a run that
  overruns cannot show the defect and cannot prove its absence.
- `StIdle` is the only state and has no transitions, so the state machine thread is idle and
  the polling thread is the only one doing work.

## Pass criteria

Send SIGINT at a random moment while `StIdle` is active and wait for the process to exit.

- Exit status 0, with `Polling loop exited`, `Shutdown complete` and `Finishing State
  Machine` in the log.
- No `terminate called` line and no `context cannot be slept` line.
- Under AddressSanitizer the leak check runs at exit.

The failure signature is `terminate called after throwing an instance of 'std::runtime_error'`
with `what():  context cannot be slept with because it's invalid`, exit status 134, logged right
after the shutdown callback's `Terminating scheduler...` line and with no `Polling loop exited`
line. With the launch file's parameters it appears in about three runs out of four.

## Running

Plain:

```bash
ros2 launch sm_coretest_poll_shutdown_1 sm_coretest_poll_shutdown_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_poll_shutdown_1_latest.log`.

For sanitizer runs, build in a separate build and install base with the sanitizer flags and
run the node binary directly, passing the parameters on the command line:

```bash
install_asan/sm_coretest_poll_shutdown_1/lib/sm_coretest_poll_shutdown_1/sm_coretest_poll_shutdown_1_node \
  --ros-args --log-level INFO -p signal_detector_loop_freq:=5.0 -p slow_update_ms:=150
```

`ASAN_OPTIONS=handle_abort=1` prints the stack of the aborting thread, which shows where the
exception came from.
