# sm_coretest_signal_key_1

Core test for the `stateCallbackConnections` map: the key path (defect E) and the
state-entry finalization (defect F) in `smacc2_core_defect_fix_study.md`.

`CbMixinFirstSubscriber` is declared `: public ISmaccUpdatable, public SmaccClientBehavior`, so
its `ISmaccClientBehavior` subobject is not at offset 0. In `StA` it subscribes to a 5 Hz
`CpRos2Timer::onTimerTick` signal with `this`. `StA` exits after 1 s and the behavior is
disposed and destroyed; the tick keeps firing into `StRest`.

- Unfixed core: the dispose erases under the base-subobject address, finds nothing, and the
  Boost connection stays live. The next tick calls `onTick` on freed memory: ASan reports
  `heap-use-after-free`; the plain build logs `TICK` lines during `StRest` or crashes.
- Fixed core: both sides key on `dynamic_cast<void *>`, the semaphore is finalized on dispose,
  and no `TICK` line appears while `StRest` is active.

Run the binary directly for sanitizer runs (see the study's "Builds" section); the launch file
tees to `/tmp/sm_coretest_signal_key_1_latest.log`.

## Defect F: the state subscribes too

`StA` subscribes to the same tick with `this` from its own `onEntry`. Boost.Statechart destroys
the state object right after `exit()`.

- Unfixed core: nothing erases the state's map entry, so the connection stays live and the next
  tick calls `StA::onStateTick` on freed memory: `[StA] STATE TICK` lines during `StRest`, ASan
  `heap-use-after-free`.
- Fixed core: `notifyOnStateExited` finalizes the state's entry last, after behaviors, reactors
  and generators; no `STATE TICK` line appears while `StRest` is active.
