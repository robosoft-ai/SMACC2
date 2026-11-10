# sm_coretest_post_during_exit_1

Core test that reproduces, without a simulator, the race between an event posted from the executor
thread and a state exit on the state machine thread (Next Level Study NL-2). `ISmaccStateMachine::postEvent`
reads `currentState_.back()` and walks that state's reactors on the posting thread, while the state machine
thread pushes and pops `currentState_` and destroys the state under `m_mutex_`, which `postEvent` does not
hold. The nav2 machines showed it under ThreadSanitizer on every action-driven transition; this test shows
it in a few seconds with no simulator.

## Design

- `CpEventPulse` (on `ClPulse` in `OrPulse`) posts `EvPulse` at `pulse_rate_hz` (default 200) from a node
  wall timer, that is from the executor thread, the posting thread of every subscription, timer and action
  callback.
- `StA` and `StB` alternate on the natural end of a 50 ms `CbSleepFor` in `OrSleep`, about twenty
  transitions a second. No state reacts to `EvPulse`.
- `StA` creates a `SrEventCountdown` on `EvPulse` with a count it never reaches, so every post walks into a
  reactor object owned by `StA`, whether `StA` is running or being destroyed.

## What it shows

Under ThreadSanitizer on the current core: data races pairing `propagateEventToStateReactors` (the
executor thread, inside `postEvent<EvPulse>`) with `StA`'s destruction on the state machine thread (the
vptr write in `~SmaccState`, the deallocation of the state, and the reactor's `eventCount_`). After a fix
that moves the reactor walk onto the state machine thread, none. The plain run is the regression check: the
`StA`/`StB` alternation and the pulse count, with no `TRAP`-style failure, since no transition depends on
`EvPulse`.

## Running

```bash
ros2 launch sm_coretest_post_during_exit_1 sm_coretest_post_during_exit_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_post_during_exit_1_latest.log`. For the
sanitizer run, build in the sanitizer base and run the node binary directly with the sanitizer environment
of the CI gate script (`.github/scripts/sanitizer_core_tests.sh`).
