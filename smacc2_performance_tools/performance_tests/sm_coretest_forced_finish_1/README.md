# sm_coretest_forced_finish_1

Core test for the forced finish of an asynchronous client behavior. It is not a
reference state machine; it exists to drive one core code path on demand, like
`sm_coretest_transition_speed_1` next to it.

## What it does

```
StA  ──EvCountdownEnd──▶  StRest  ──EvTimer<CbTimerCountdownLoop>──▶  StA
 │                          │
 └── EvCbSuccess/EvCbFinished<CbSleepFor, OrSleep> ──▶ StTrap (terminal) ◀──┘
```

- `StA` runs `CbSleepFor(30s)` in `OrSleep` and `CbTimerCountdownOnce(1s)` in
  `OrTimer`, plus an `SrEventCountdown(1)` reactor on the timer event. The timer
  always wins, so on every `StA` exit the state machine thread calls
  `requestForceFinish()` on the sleep behavior while its `onEntry` thread polls
  `isShutdownRequested()`. The behavior then posts `EvCbSuccess` and
  `EvCbFinished` with `CURRENT_STATE` lifetime while the machine is
  `STATE_EXITING`, so both hit the discard path in `ISmaccStateMachine::postEvent`.
- `StRest` waits 1 s on a `CbTimerCountdownLoop`. It is deliberately a different
  timer behavior than `StA`'s: `StA`'s own `EvTimer<CbTimerCountdownOnce>` is still
  queued behind the reactor's `EvCountdownEnd` when `StRest` is entered and would
  otherwise fire `StRest`'s reaction immediately.
- `StTrap` is terminal and logs `FATAL "TRAP ENTERED"`. Both `StA` and `StRest`
  transition to it on any sleep event. The sleep can never finish within `StA`'s
  1 s lifetime, so a sleep event processed anywhere is one that crossed a state
  exit instead of being discarded.

One cycle takes 2 s. Expected per cycle: one `requestForceFinish`, two
`CURRENT STATE SCOPED EVENT DISCARDED` warnings, zero `TRAP ENTERED`.

## Running

```bash
ros2 launch sm_coretest_forced_finish_1 sm_coretest_forced_finish_1.launch.py
# output is teed to /tmp/sm_coretest_forced_finish_1_latest.log
```

Quick check of a log:

```bash
L=/tmp/sm_coretest_forced_finish_1_latest.log
echo "cycles: $(grep -ac '\[StRest\] Entered' $L)  discards: $(grep -ac 'EVENT DISCARDED' $L)  trap: $(grep -ac 'TRAP ENTERED' $L)"
```

## Under sanitizers

Build the core and this package into a separate build/install base with
`-fsanitize=thread,undefined` or `-fsanitize=address,undefined -fsanitize-recover=address`,
then run the node binary directly (not through `ros2 run` with a `stdbuf` prefix,
which preloads a library ahead of the sanitizer runtime). ThreadSanitizer needs
`setarch x86_64 -R` on kernels with 32-bit mmap randomization. For LeakSanitizer
to report, send SIGINT while the machine is in `StRest`, when no behavior thread
is alive. The full procedure and the recorded results are in
`docs/smacc2_core_defect_fix_study.md` at the repository root.
