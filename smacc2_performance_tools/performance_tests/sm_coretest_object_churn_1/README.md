# sm_coretest_object_churn_1

Core test for memory. Two states follow each other without pause, and each creates and destroys one
of every state-scoped object. It is not a reference state machine; it exists so that
`smacc2_performance_tools/core_test_tools/memory_tests/object_churn_soak.sh` can watch the process's
memory, threads and file descriptors while these objects are created and destroyed hundreds of
thousands of times.

## What each state holds

| Object | What it does |
|--|--|
| `CbTickSubscriber` | a synchronous behavior that is also an updatable and connects to the signal of a 500 Hz timer |
| `CbFinish` | an asynchronous behavior whose thread posts `EvCbSuccess` and ends |
| `EgConditionalGenerator` | posts `EvTrue` on its first update |
| `SrAllEventsGo` | waits for both events and posts `EvAllGo`, the event the state leaves on |
| the state itself | connects to the same timer signal with `this` |

`StChurnA` logs the number of round trips at FATAL every 1000, so the count is printed with logging
otherwise off. One round trip is two state lifetimes. The rate is one transition per polling loop
iteration: about 1000 round trips a second with `signal_detector_loop_freq` at 2000.

## Run

```bash
ros2 launch sm_coretest_object_churn_1 sm_coretest_object_churn_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_object_churn_1_latest.log`.

## Not in the sanitizer gate

`.github/scripts/sanitizer_core_tests.sh` does not run this machine. It was run by hand for 60
seconds under TSan (30,000 round trips) and ASan (52,000) on 2026-11-14 with no report.
