# sm_coretest_reactor_order_1

Core test that records whether a state reactor's output event is processed before or after the event
that triggered it. The order is a property of `ISmaccStateMachine::postEvent`, which today runs the
reactors of the current state synchronously at post time, so a reactor's output event is queued before
the triggering event itself. Machines depend on it: `sm_retry_logic_1` goes from `SsAttemptTask` straight to
`StExhausted` on the third timeout only because the countdown's end is processed before the timer event
that `StAttempting` also reacts to.

## Design

- `CpEventPulse` (on `ClPulse` in `OrPulse`) posts `EvX` every `pulse_period_ms` (default 500) from a node
  wall timer, that is from the executor thread, which is the posting thread of every subscription and
  timer callback.
- `StA` creates a `SrEventCountdown` with count 1 on `EvX`, whose output is `EvCountdownEnd`, and reacts
  to both: `EvX` → `StViaTrigger`, `EvCountdownEnd` → `StViaCountdown`.
- `StViaCountdown` and `StViaTrigger` return to `StA` on the next `EvX`, so every pulse makes one pass
  through `StA` and records one outcome in the log.

## What it shows

On the current core every pass enters `StViaCountdown`: the countdown's output is queued before the
trigger, so `EvCountdownEnd` wins. (The leftover `EvX` then lands in `StViaCountdown` and takes the machine
back to `StA` at once, the "events firing in the next state" rule.) A core change that processes the
trigger first would enter `StViaTrigger` instead, and `EvCountdownEnd` would land there with no reaction
until the next pulse. The count of each is the measure: before and after a change to the event path, the
`StViaTrigger` count must stay at zero.

## Running

```bash
ros2 launch sm_coretest_reactor_order_1 sm_coretest_reactor_order_1.launch.py
```

The launch file tees the node output to `/tmp/sm_coretest_reactor_order_1_latest.log`. Count the
`[StViaCountdown] Entered` and `[StViaTrigger] Entered` lines against the `[CpEventPulse] pulse` lines.
