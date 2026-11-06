# sm_coretest_signal_key_1

Core test for the `stateCallbackConnections` key path (defect E in
`smacc2_core_defect_fix_study.md`).

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
