# sm_coretest_signal_key_1

Core test for signal-connection cleanup of state-scoped subscribers. It checks that when a
client behavior or a state subscribes to a signal with `this`, the connection is finalized when
the subscriber is destroyed, so that later emissions never reach a dead object.

## Design

- `ClTickSource` (in `OrTick`) owns a free-running `CpRos2Timer` at 5 Hz. The component lives
  for the whole state machine, so its `onTimerTick` signal keeps firing after any state has
  exited.
- `CbMixinFirstSubscriber` is deliberately declared `: public ISmaccUpdatable, public
  SmaccClientBehavior`, with the non-SMACC base first. That layout places the
  `ISmaccClientBehavior` subobject away from the start of the object, so the pointer the
  behavior subscribes with and the base pointer the orthogonal disposes it with are different
  addresses. The behavior subscribes to the tick in `onEntry` and logs `[CbMixinFirstSubscriber]
  TICK n`.
- `StA` runs the behavior for 1 s and also subscribes to the same tick itself, with `this`,
  from its own `onEntry`, logging `[StA] STATE TICK n`. Boost.Statechart destroys the state
  object right after `exit()`.
- `StRest` (1 s) has no subscriber of either kind. The machine cycles `StA -> StRest -> StA`
  every 2 s.

## Pass criteria

- No `TICK` or `STATE TICK` line is logged while `StRest` is active. Any such line is a
  callback delivered into a destroyed behavior or state.
- Each `StA` lifetime logs about 5 ticks for the behavior and about 5 for the state, reported
  at exit (`onExit, ticks=` and `Exiting. state ticks=`).
- Under AddressSanitizer: no `heap-use-after-free`, no `SEGV`.

The failure signature, if the cleanup regresses, is a `heap-use-after-free` inside the bound
signal lambda under `CpRos2Timer::timerCallback`, with the memory freed either by the
orthogonal's `shared_ptr` release (behavior case) or by `boost::statechart::detail::deallocate`
(state case). In a plain build the state case segfaults on the first tick after the first
`StA` exit.

## Running

Plain:

```bash
ros2 launch sm_coretest_signal_key_1 sm_coretest_signal_key_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_signal_key_1_latest.log`.

For sanitizer runs, build in a separate build and install base with the sanitizer flags and
run the node binary directly, without a launch prefix (a `stdbuf` prefix preloads a library
ahead of the sanitizer runtime and ASan refuses to start):

```bash
install_asan/sm_coretest_signal_key_1/lib/sm_coretest_signal_key_1/sm_coretest_signal_key_1_node --ros-args --log-level INFO
```

Twelve seconds, six cycles, is enough to see every pass criterion.

## Note

`StA` reaches the timer component through the state-level `requiresComponent`, once by type and
once by name (the empty default name), and logs an error if the two disagree. Until the
forwarders were fixed the state had to go through
`getOrthogonal<OrTick>()->getClients().front()->getComponent<CpRos2Timer>()`.
