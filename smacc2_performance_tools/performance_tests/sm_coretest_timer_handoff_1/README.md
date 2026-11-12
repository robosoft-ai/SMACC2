# sm_coretest_timer_handoff_1

Core test for the hand-off of a timer created on the state machine thread to the
executor in the signal detector's polling loop. It is `sm_coretest_forced_finish_1`'s
cycle run at 100 ms instead of 1 s, which is the shape that reproduced the missed
hand-off (finding W): a plain two-state timer ping-pong with no forced finish did not.

## What it does

```
StA  ──EvCountdownEnd──▶  StB  ──EvTimer<CbTimerCountdownLoop>──▶  StA
```

- `StA` runs `CbSleepFor(30s)` in `OrSleep` and `CbTimerCountdownOnce(100ms)` in
  `OrTimer`, with an `SrEventCountdown(1)` reactor on the timer event. The timer
  always wins, so every `StA` exit forces the sleep to finish: the state machine
  thread holds the exit for the forced-finish wait, then enters `StB`, whose
  `CbTimerCountdownLoop(100ms)` creates a new ROS timer in its `onEntry`.
- The polling loop's executor has to pick that timer up. A hand-off it misses leaves
  the timer ready and never executed, and the machine sits in `StB` for good.
- About three cycles a second; a healthy 60 s run logs some 300 `Entered` lines
  with no gap longer than a cycle. The gate's invariant is the gap: no two
  consecutive state entries more than 5 s apart, and the last one within 5 s of the
  interrupt.

## Running it

```
ros2 launch sm_coretest_timer_handoff_1 sm_coretest_timer_handoff_1.launch.py
```

Output is teed to `/tmp/sm_coretest_timer_handoff_1_latest.log`. The plain build is
the one that matters: the sanitizer allocators delay the reuse of freed memory, and
the race this test exists for has only ever shown in a plain build.
