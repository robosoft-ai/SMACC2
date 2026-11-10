# sm_coretest_callback_rate_1

Core test for the rate at which the signal detector's polling loop delivers callbacks to the
entities of the state machine node. It measures, once a second, how many messages a subscription
received and how many times a node timer fired, against how many messages were published, when
both run faster than the polling loop.

## Design

- `ClCallbackCounter` (in `OrCounter`) publishes `std_msgs/UInt32` on `/callback_rate_test` from
  its own thread at `publish_rate_hz` (default 200), subscribes to the same topic with a history
  depth of 10, and runs a node wall timer at `timer_rate_hz` (default 100). The subscription and
  timer callbacks run wherever the polling loop spins the node's executor; the publishing thread
  runs regardless.
- `update()`, called by `SignalDetector::pollOnce` once per loop iteration on the polling thread,
  logs the counts of the last second: published, received, timer ticks and loop iterations, plus the
  totals since start.
- `StIdle` is the only state and has no transitions; the state machine thread is idle.
- The polling rate is the core parameter `signal_detector_loop_freq` (default 20 Hz).

## What it shows

With the loop spinning the executor once per iteration (`spin_some`), a subscription receives at
most one message per iteration and a timer fires at most once per iteration, whatever their rates:
at 20 Hz polling the report reads about 20 received and 20 ticks per second against 200 published.
A loop that drains the executor each iteration delivers every message and every due tick: about
200 received and 100 ticks per second, with the same 20 loop iterations.

A publishing rate far above what the loop can drain (`publish_rate_hz:=5000`) is the saturation
case: it shows what the loop iterations per second and the process CPU do when the drain is bounded
by the loop period.

## Running

```bash
ros2 launch sm_coretest_callback_rate_1 sm_coretest_callback_rate_1.launch.py
ros2 launch sm_coretest_callback_rate_1 sm_coretest_callback_rate_1.launch.py publish_rate_hz:=5000.0
```

The launch file tees the node output to `/tmp/sm_coretest_callback_rate_1_latest.log`. The test is a
measurement, not a pass/fail gate: read the `[ClCallbackCounter] last ... s:` lines.
