# sm_coretest_idle_load_1

Core test for memory. The machine enters one state and stays in it while the polling loop works. It
is not a reference state machine; it exists so that
`smacc2_performance_tools/core_test_tools/memory_tests/idle_load_soak.sh` can watch the process's
memory, threads and file descriptors over hundreds of thousands of loop iterations, timer ticks and
messages. Most robots spend most of their time like this: in one state, with data arriving.

## What runs while `StIdle` is active

| Object | What it does |
|--|--|
| `CpRos2Timer` | fires at 500 Hz |
| `CpLoadDriver` | a component connected to the timer signal; publishes a 1 kB `std_msgs/String` on each tick, and is an updatable |
| `CpTopicPublisher`, `CpTopicSubscriber` | the same topic, `/sm_coretest_idle_load_1/data`; the subscriber posts `EvTopicMessage` for every message, which no state consumes, and emits its signal |
| `CbMessageCounter` | a behavior connected to the subscriber's signal, and an updatable; logs a count at FATAL every 1000 messages |

With `signal_detector_loop_freq` at 2000 the machine receives 500 messages and runs 2000 updates of
each updatable a second.

## The deliberate leak

With `SM_CORETEST_IDLE_LEAK=1` in the environment `CbMessageCounter` keeps every message it
receives, about 0.5 MB a second. It is there to check the soak itself: a run with it set must fail.

## Run

```bash
ros2 launch sm_coretest_idle_load_1 sm_coretest_idle_load_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_idle_load_1_latest.log`.

## Not in the sanitizer gate

`.github/scripts/sanitizer_core_tests.sh` does not run this machine. It was run by hand for 60
seconds under TSan and under ASan on 2026-11-14 (29,000 messages each) with no report.
