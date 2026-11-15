# sm_coretest_action_client_1

Core test for the action client component (`CpActionClient`) and the action behavior base
(`CbActionClientBehaviorBase`), with no simulator. It is not a reference state machine; it exists so
the code every Nav2 and MoveIt behavior is built on runs under the sanitizers.

## What it does

`OrAction` holds a Fibonacci action server on the state machine's own node (`CpFibonacciServer`) and
two action client components of the same action type, told apart by name: `server`, aimed at that
server, and `no_server`, aimed at an action nothing serves. Five states follow each other, one per
outcome of a goal:

| State | Goal | What must happen | Next |
|--|--|--|--|
| `StSuccess` | order 5 | five feedback messages, then success: `EvCbSuccess` | `StRejected` |
| `StRejected` | order 13 | the server rejects it; `sendGoal` posts `EvCbFailure` | `StAborted` |
| `StAborted` | order 21 | accepted, aborted after two steps: `EvCbFailure` | `StNoServer` |
| `StNoServer` | through `no_server` | no server; `sendGoal` posts `EvCbFailure` without waiting | `StCancelled` |
| `StCancelled` | order 1000, against a 500 ms timer | the state exits with the goal in flight and the behavior's `onExit` cancels it | `StSuccess` |

Each state sends the outcome it must not see to `StTrap`, which is terminal and logs `TRAP ENTERED`.

`StSuccess` follows `StCancelled` on purpose. The cancelled goal's result reaches the client about
100 ms after `StSuccess` has sent its own goal through the same component. Two things must hold, and
the test found both broken:

- the cancellation must stop only the goal it was meant for, not the next state's goal;
- the cancelled goal's result must not be taken for the result of the next state's goal.

## Run

```bash
ros2 launch sm_coretest_action_client_1 sm_coretest_action_client_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_action_client_1_latest.log`.

## In the gate

`.github/scripts/sanitizer_core_tests.sh` runs the machine for 20 s under TSan, ASan and a plain
build, then interrupts it at five random moments. It fails if the machine completes fewer than five
cycles, if `StTrap` is entered, if any outcome's count is not within one of the cycle count, if a run
ends with an uncaught exception or a hang, or on a sanitizer report with a SMACC2 frame.

## In the memory tests

`smacc2_performance_tools/core_test_tools/memory_tests/action_goal_soak.sh` runs the same machine
with logging off and two environment variables that shorten the cycle from about 1.1 s to about
37 ms: `SM_CORETEST_ACTION_STEP_MS` (the server's step, 50 ms by default; the soak uses 2) and
`SM_CORETEST_ACTION_CANCEL_MS` (the timer in `StCancelled`, 500 ms by default; the soak uses 20).
`StSuccess` logs the cycle count at FATAL every 100 cycles so the soak can report it.

The server joins the worker threads that have finished each time it accepts a goal. It used to keep
every worker until shutdown, one exited thread and its stack per goal, which the soak reported as
growth of about 70 kB a second.
