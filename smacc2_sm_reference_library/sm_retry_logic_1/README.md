# sm_retry_logic_1

Demonstrates a **state reactor (`SrEventCountdown`) defined in a superstate**. The reactor
counts failures across inner state transitions, implementing a 3-attempt retry limit before
giving up. This pattern answers the question: *can a parent state accumulate event history
that persists while its inner states cycle?* The answer is yes — and this machine shows how.

## What it demonstrates

- `SrEventCountdown(3)` lives in `SsAttemptTask` (a superstate), not in any leaf state.
- The reactor counts **timer timeouts** and **explicit 'f' keypresses** as failures.
- The failure count **persists across `StIdle ↔ StAttempting` inner transitions** — it is not reset when the inner states change.
- After 3 total failures (any combination), the superstate transitions to `StExhausted`.
- Pressing `s` at any point during an attempt — or while waiting in `StIdle` — transitions immediately to `StSuccess`.

## State hierarchy

```
SmRetryLogic1
├── OrTimer    (ClRos2Timer)
└── OrKeyboard (ClKeyboard)

SmRetryLogic1
├── SsAttemptTask  ←  superstate; owns SrEventCountdown(3)
│   │              Reactor input events: EvTimer, EvKeyPressF
│   │              Reactor output event: EvCountdownEnd → StExhausted
│   │              Also reacts: EvKeyPressS → StSuccess
│   │
│   ├── StIdle       (initial)  — waiting for 'n' to start an attempt
│   └── StAttempting            — attempt in progress; 10s timeout
│
├── StSuccess    — reached by pressing 's' at any time
└── StExhausted  — reached after 3 total failures
```

## Build

```bash
cd /home/brettpac/workspaces/isaac_ros-dev
colcon build --packages-select sm_retry_logic_1
source install/setup.bash
```

## Launch

```bash
ros2 launch sm_retry_logic_1 sm_retry_logic_1.launch.py
```

This opens two konsole terminals: the state machine node and the keyboard server.

## Keyboard controls

| Key | Effect | Active in |
|-----|--------|-----------|
| `n` | Start an attempt (`StIdle → StAttempting`) | StIdle |
| `f` | Manually fail the current attempt (`StAttempting → StIdle`, counts as 1 failure) | StAttempting |
| `s` | Succeed immediately (`→ StSuccess`) | StIdle or StAttempting |
| *(timer)* | 10-second timeout = auto-fail (`StAttempting → StIdle`, counts as 1 failure) | StAttempting |

Send keystrokes via ROS 2 topic (if not using the konsole keyboard server):

```bash
ros2 topic pub /keyboard_unicode std_msgs/msg/UInt16 "data: 110" --once  # n
ros2 topic pub /keyboard_unicode std_msgs/msg/UInt16 "data: 102" --once  # f
ros2 topic pub /keyboard_unicode std_msgs/msg/UInt16 "data: 115" --once  # s
```

## Example walkthroughs

### Reach StExhausted via 3 manual failures

```
Start → StIdle
  → press n → StAttempting
  → press f → StIdle          (failure count: 2 remaining)
  → press n → StAttempting
  → press f → StIdle          (failure count: 1 remaining)
  → press n → StAttempting
  → press f → StExhausted     (failure count: 0, EvCountdownEnd fires)
```

### Reach StExhausted via 3 timer timeouts

```
Start → StIdle
  → press n → StAttempting → wait 10s → StIdle   (failure count: 2 remaining)
  → press n → StAttempting → wait 10s → StIdle   (failure count: 1 remaining)
  → press n → StAttempting → wait 10s → StExhausted
```

### Reach StSuccess after 2 failures

```
Start → StIdle
  → press n → StAttempting
  → press f → StIdle          (failure count: 2 remaining)
  → press n → StAttempting
  → press f → StIdle          (failure count: 1 remaining)
  → press n → StAttempting
  → press s → StSuccess
```

### Reach StSuccess immediately (no attempts needed)

```
Start → StIdle
  → press s → StSuccess
```

## Monitoring

```bash
# Current state hierarchy
ros2 topic echo /sm_retry_logic_1/smacc/status

# State transitions as they fire
ros2 topic echo /sm_retry_logic_1/smacc/transition_log

# All posted events (including EvCountdownEnd)
ros2 topic echo /sm_retry_logic_1/smacc/event_log
```

Watch the node console output for `SB COUNTDOWN (N)` log lines — these show the remaining
failure count after each failure event is received by the reactor.
