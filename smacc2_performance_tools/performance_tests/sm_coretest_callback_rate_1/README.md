# sm_coretest_callback_rate_1

Core test for the rate at which the signal detector's polling loop delivers callbacks to the
entities of the state machine node. It measures, once a second, how many messages a subscription
received and how many times a node timer fired, against how many messages were published, when
both run faster than the polling loop.

## Design

- `ClCallbackCounter` (in `OrCounter`) publishes `std_msgs/UInt32` on `/callback_rate_test` from
  its own thread at `publish_rate_hz` (default 200), subscribes to the same topic with a history
  depth of `subscription_depth` (default 10) and `callback_work_us` (default 0) of busy work per
  message, and runs a node wall timer at `timer_rate_hz` (default 100). The subscription and
  timer callbacks run wherever the polling loop spins the node's executor; the publishing thread
  runs regardless.
- `update()`, called by `SignalDetector::pollOnce` once per loop iteration on the polling thread,
  logs the counts of the last second: published, received, timer ticks and loop iterations, plus the
  totals since start.
- `StIdle` is the only state and has no transitions; the state machine thread is idle.
- The polling rate is the core parameter `signal_detector_loop_freq` (default 20 Hz).

## What it shows

With the loop spinning the executor once per iteration (`spin_some`), a subscription receives at
most one message per iteration, whatever its rate: at 20 Hz polling the report reads about 20
received per second against 200 published. A loop that drains the executor each iteration
(`spin_all` bounded by the loop period) delivers every message the subscription's history holds:
about 200 received per second, with the same 20 loop iterations. A timer fires at most once per
iteration either way, because callbacks run only while the loop spins and an overdue timer fires
once: the 100 Hz timer ticks about 20 times a second under both loops.

The saturation case needs a backlog that costs real time: `publish_rate_hz:=5000
subscription_depth:=1000 callback_work_us:=300` queues about 250 messages per 50 ms period at
300 us each, more than the period can drain. It shows what the loop iterations per second, the
received count and the process CPU do when the drain hits its budget, and the core's throttled
saturation warning. With the default depth of 10 the backlog is capped at 10 messages per
iteration and the loop never saturates, whatever the publishing rate.

## Running

```bash
ros2 launch sm_coretest_callback_rate_1 sm_coretest_callback_rate_1.launch.py
ros2 launch sm_coretest_callback_rate_1 sm_coretest_callback_rate_1.launch.py publish_rate_hz:=5000.0 subscription_depth:=1000 callback_work_us:=300
```

The launch file tees the node output to `/tmp/sm_coretest_callback_rate_1_latest.log`. The test is a
measurement, not a pass/fail gate: read the `[ClCallbackCounter] last ... s:` lines.
