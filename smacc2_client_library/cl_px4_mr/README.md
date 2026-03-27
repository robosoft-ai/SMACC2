# PX4 Multirotor Client (cl_px4_mr)

SMACC2 client library for PX4 multirotor control via ROS 2 XRCE-DDS.

## Architecture

The `cl_px4_mr` follows a pure component-based architecture where the client orchestrates seven specialized components:

```
ClPx4Mr (Client - Orchestrator)
├── CpVehicleCommand        - Sends vehicle commands (arm, disarm, mode, land, takeoff)
├── CpVehicleStatus         - Monitors arming state, landing, and nav mode
├── CpVehicleLocalPosition  - Tracks vehicle position in NED frame
├── CpTrajectorySetpoint    - Publishes position setpoints for offboard control
├── CpOffboardKeepAlive     - Maintains offboard mode via periodic heartbeat
├── CpVehicleCommandAck     - Monitors command acknowledgement responses
└── CpGoalChecker           - Detects when vehicle reaches target position
```

### Components

#### CpVehicleCommand
**Responsibility**: Sends PX4 vehicle commands via `/fmu/in/vehicle_command`

- `arm()` / `forceArm()` / `disarm()` - Vehicle arming control
- `setOffboardMode()` - Switch to offboard control mode
- `takeoff(altitude)` / `land()` - Flight commands
- `sendCommand(...)` - Generic 7-parameter command interface

**Signals**: None

#### CpVehicleStatus
**Responsibility**: Monitors vehicle state from `/fmu/out/vehicle_status_v1`

- Tracks arming state and navigation mode transitions
- Detects armed/disarmed and landed state changes

**Signals**: `onArmed_`, `onDisarmed_`, `onModeChanged_`, `onLanded_`

#### CpVehicleLocalPosition
**Responsibility**: Tracks position from `/fmu/out/vehicle_local_position`

- Provides NED position (x, y, z) and heading
- Thread-safe accessors with validity checking

**Signals**: `onPositionReceived_`

#### CpTrajectorySetpoint
**Responsibility**: Publishes position commands to `/fmu/in/trajectory_setpoint`

- `setPositionNED(x, y, z, yaw)` - Set target position
- `hold()` - Hold current position
- `republishLast()` - Re-send last setpoint
- Thread-safe with cached last setpoint

**Signals**: None

#### CpOffboardKeepAlive
**Responsibility**: Publishes offboard control mode heartbeat to `/fmu/in/offboard_control_mode`

- Inherits from `ISmaccUpdatable` for periodic ~20Hz updates
- `enable()` / `disable()` - Toggle heartbeat publishing
- PX4 requires continuous offboard heartbeat to stay in offboard mode

**Signals**: None

#### CpVehicleCommandAck
**Responsibility**: Monitors command responses from `/fmu/out/vehicle_command_ack`

- Tracks last command ID and result code

**Signals**: `onAckReceived_`

#### CpGoalChecker
**Responsibility**: Monitors position and fires when goal is reached

- Inherits from `ISmaccUpdatable` for periodic distance checking
- `setGoal(x, y, z, xy_tolerance, z_tolerance)` - Set target with tolerances
- `clearGoal()` - Deactivate goal checking
- Default tolerances: 0.5m XY, 0.3m Z

**Signals**: `onGoalReached_`

### Signal Flow

```
PX4 SITL (via XRCE-DDS)
    ↓ (subscriptions)
CpVehicleStatus         → onArmed_ / onDisarmed_ / onLanded_
CpVehicleLocalPosition  → onPositionReceived_
CpVehicleCommandAck     → onAckReceived_
CpGoalChecker (update)  → onGoalReached_
    ↓ (signal connections)
Client Behaviors (CbArmPX4, CbTakeOff, CbLand, CbHoldPosition, CbFollowWaypoints, etc.)
    ↓ (postSuccessEvent / postFailureEvent)
State Machine Transitions
```

```
Client Behaviors
    ↓ (component method calls)
CpVehicleCommand       → /fmu/in/vehicle_command
CpTrajectorySetpoint   → /fmu/in/trajectory_setpoint
CpOffboardKeepAlive    → /fmu/in/offboard_control_mode
    ↓ (publications)
PX4 SITL (via XRCE-DDS)
```

## Client Behaviors

| Behavior | Base Class | Purpose | Parameters |
|----------|-----------|---------|------------|
| CbConnectMicroRosAgent | SmaccAsyncClientBehavior | Launch agent, detect node, wait for health | `timeoutSec` (default 30.0) |
| CbArmPX4 | SmaccAsyncClientBehavior | Enable offboard, set mode, arm with retry | None (5 retries, force-arm after 2) |
| CbDisarmPX4 | SmaccAsyncClientBehavior | Disarm vehicle | None (3 retries) |
| CbTakeOff | SmaccAsyncClientBehavior | Enter offboard mode and climb to altitude | `targetAltitude` (default 5.0m) |
| CbGoToLocation | SmaccAsyncClientBehavior | Fly to NED position | `targetX`, `targetY`, `targetZ`, `yaw` (optional) |
| CbOrbitLocation | SmaccAsyncClientBehavior + ISmaccUpdatable | Orbit a center point | `centerX`, `centerY`, `altitude`, `radius`, `angularVelocity`, `numOrbits` |
| CbLand | SmaccAsyncClientBehavior | Disable offboard and land | None (detects landing via disarm signal) |
| CbHoldPosition | SmaccAsyncClientBehavior + ISmaccUpdatable | Hold current position for a duration | `durationSeconds` (default 5.0) |
| CbYawRotate | SmaccAsyncClientBehavior + ISmaccUpdatable | Rotate to a target heading | `targetYawRad`, `relative` (default false) |
| CbChangeAltitude | SmaccAsyncClientBehavior | Ascend or descend to a target altitude | `targetAltitude` (positive meters above ground) |
| CbFollowWaypoints | SmaccAsyncClientBehavior | Visit a sequence of NED waypoints | `waypoints` (vector of {x,y,z,yaw}), `xyTol` (0.5), `zTol` (0.3) |
| CbFigureEight | SmaccAsyncClientBehavior + ISmaccUpdatable | Fly a lemniscate figure-8 pattern | `centerX`, `centerY`, `altitude`, `size` (5.0), `speed` (0.5), `numLoops` (1) |
| CbReturnToHome | SmaccAsyncClientBehavior | Return to a specified home position | `homeX`, `homeY`, `homeZ`, `homeYaw` |
| CbSpiralPattern | SmaccAsyncClientBehavior + ISmaccUpdatable | Fly an expanding Archimedean spiral (search & rescue) | `centerX`, `centerY`, `altitude`, `maxRadius` (20.0), `spacing` (3.0), `speed` (2.0) |

### CbConnectMicroRosAgent

Launches the micro_ros_agent (if not already running) and waits for PX4 readiness:

1. **Phase 1 — Node detection:** Polls `get_node_names()` at 5 Hz for the `/px4_micro_xrce_dds` node
2. **Phase 2 — Health check:** Subscribes to `/fmu/out/failsafe_flags` and polls at 2 Hz until PX4's EKF has converged (`attitude_invalid`, `local_altitude_invalid`, and `local_position_invalid` all false)
3. Posts `EvCbSuccess` when both phases pass, `EvCbFailure` on timeout or shutdown
4. Resets the failsafe subscription on exit to prevent dangling callbacks

### CbArmPX4

Enables offboard mode and arms the vehicle:

1. Enables `CpOffboardKeepAlive` (starts publishing `offboard_control_mode` at ~20 Hz)
2. Sends `setOffboardMode()` command
3. Waits 2 s for PX4 to register the offboard signal
4. Attempts arming with retry and force-arm escalation:
   - Attempts 1-2: Standard arm command
   - Attempts 3-5: Force-arm (param2=21196)
5. Posts `EvCbSuccess` on armed, `EvCbFailure` after all retries exhausted
6. Connects to `CpVehicleStatus::onArmed_` signal

**Note:** PX4 ignores the force-arm flag for external commands (`from_external=true`), so the offboard keepalive step is essential — without it, `canArm()` fails because `offboard_control_signal_lost` is true when `nav_state == OFFBOARD`.

### CbTakeOff

Full offboard entry sequence:
1. Enable offboard keepalive heartbeat (if not already enabled)
2. Send hold command (2s)
3. Switch to offboard mode
4. Set position setpoint at target altitude
5. Posts `EvCbSuccess` when goal checker reports altitude reached

### CbOrbitLocation

Continuous orbit using `ISmaccUpdatable::update()`:
- Computes start angle from current position via `atan2`
- Advances angle at specified angular velocity each update cycle
- Sends `setPositionNED` on circular path around center
- Posts `EvCbSuccess` after completing `numOrbits` full rotations

### CbLand

Landing sequence:
1. Disable offboard keepalive
2. Send land command
3. Posts `EvCbSuccess` when vehicle disarms (PX4 auto-disarms after touchdown)

### CbHoldPosition

Hold current position for a specified duration:
- Calls `CpTrajectorySetpoint::hold()` to lock current position
- Uses `ISmaccUpdatable::update()` to monitor elapsed time
- Posts `EvCbSuccess` when duration reached

### CbYawRotate

Rotate in place to a target heading:
- Supports absolute or relative yaw targets
- Commands position setpoint with new yaw at current XY/Z
- Monitors heading convergence (within ~0.1 rad tolerance)
- Posts `EvCbSuccess` when target heading reached

### CbChangeAltitude

Ascend or descend to a target altitude while maintaining XY position:
- Gets current XY and heading, sends setpoint with new Z = `-targetAltitude`
- Uses `CpGoalChecker` signal for completion detection
- Posts `EvCbSuccess` when altitude reached

### CbFollowWaypoints

Visit a sequence of NED waypoints in order:
- Each waypoint is `{x, y, z, yaw}` — yaw = NaN to maintain current heading
- Advances to next waypoint on each `CpGoalChecker::onGoalReached_` callback
- Posts `EvCbSuccess` after the last waypoint is reached

### CbFigureEight

Fly a figure-8 (lemniscate of Bernoulli) pattern:
- Parametric equations trace a lemniscate centered at the given point
- Yaw faces direction of travel via derivatives
- Uses `ISmaccUpdatable::update()` for continuous trajectory streaming
- Posts `EvCbSuccess` after completing `numLoops` full loops

### CbReturnToHome

Return to a stored home position:
- Home coordinates passed explicitly by the state machine
- Uses `CpGoalChecker` signal for completion detection
- Posts `EvCbSuccess` when home position reached

### CbSpiralPattern

Fly an expanding Archimedean spiral for search and rescue area coverage:
- `r(θ) = (spacing / 2π) × θ` — each revolution expands radius by `spacing` meters
- Adaptive angular velocity maintains constant linear ground speed
- Yaw faces direction of travel
- Posts `EvCbSuccess` when `maxRadius` is reached

## Usage

### Orthogonal Setup

```cpp
#include <cl_px4_mr/cl_px4_mr.hpp>

class OrPx4 : public smacc2::Orthogonal<OrPx4>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_px4_mr::ClPx4Mr>();
  }
};
```

### State with Behavior

```cpp
#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>

class StGoToWaypoint : public smacc2::SmaccState<StGoToWaypoint, MsInFlight>
{
public:
  using SmaccState::SmaccState;

  using reactions = boost::mpl::list<
    Transition<EvCbSuccess<cl_px4_mr::CbGoToLocation, OrPx4>, StNextState>
  >;

  static void staticConfigure()
  {
    // Fly to position (10, 0) at 5m altitude (NED: z = -5.0)
    configure_orthogonal<OrPx4, cl_px4_mr::CbGoToLocation>(10.0f, 0.0f, -5.0f);
  }
};
```

### Direct Component Access

```cpp
class CbCustomFlight : public smacc2::SmaccAsyncClientBehavior
{
public:
  void onEntry() override
  {
    this->requiresComponent(trajectorySetpoint_);
    this->requiresComponent(goalChecker_);

    this->getStateMachine()->createSignalConnection(
      goalChecker_->onGoalReached_, &CbCustomFlight::onGoalReached, this);

    trajectorySetpoint_->setPositionNED(5.0f, 5.0f, -3.0f);
    goalChecker_->setGoal(5.0f, 5.0f, -3.0f, 0.5f, 0.3f);
  }

  void onExit() override { goalChecker_->clearGoal(); }

private:
  void onGoalReached() { this->postSuccessEvent(); }
  cl_px4_mr::CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  cl_px4_mr::CpGoalChecker * goalChecker_ = nullptr;
};
```

## PX4 XRCE-DDS Topics

| Direction | Topic | Message Type | Component / Behavior |
|-----------|-------|-------------|-----------|
| Subscribe | `/fmu/out/vehicle_status_v1` | VehicleStatus | CpVehicleStatus |
| Subscribe | `/fmu/out/vehicle_local_position` | VehicleLocalPosition | CpVehicleLocalPosition |
| Subscribe | `/fmu/out/vehicle_command_ack` | VehicleCommandAck | CpVehicleCommandAck |
| Subscribe | `/fmu/out/failsafe_flags` | FailsafeFlags | CbConnectMicroRosAgent (phase 2) |
| Publish | `/fmu/in/vehicle_command` | VehicleCommand | CpVehicleCommand |
| Publish | `/fmu/in/trajectory_setpoint` | TrajectorySetpoint | CpTrajectorySetpoint |
| Publish | `/fmu/in/offboard_control_mode` | OffboardControlMode | CpOffboardKeepAlive |

### QoS Settings
- Publishers: `rclcpp::QoS(1).best_effort()`
- Subscribers: `rclcpp::SensorDataQoS()`

### Coordinate Frame
PX4 uses NED (North-East-Down): altitude of 5m above ground = z = -5.0

## Dependencies

- **smacc2**: State machine framework
- **px4_msgs**: PX4 ROS 2 message definitions
- **rclcpp**: ROS 2 C++ client library

## PX4 SITL Requirements

- PX4 v1.15+ publishes vehicle status on `vehicle_status_v1` (not `vehicle_status`)
- XRCE-DDS agent must be running: `ros2 run micro_ros_agent micro_ros_agent udp4 -p 8888`
- `CbConnectMicroRosAgent` manages agent launch automatically and waits for PX4 EKF convergence via failsafe flags before proceeding
- PX4 ignores the force-arm flag (param2=21196) for external commands; `CbArmPX4` satisfies arming requirements by enabling offboard keepalive and setting offboard mode before arming

## Testing

Two reference state machines are available:

**sm_cl_px4_mr_test_1** — Basic flight: arm, takeoff, go-to, orbit, return, land.
```bash
ros2 launch sm_cl_px4_mr_test_1 sm_cl_px4_mr_test_1.launch.py
```

**sm_cl_px4_mr_test_2** — Extended behaviors: hold position, yaw rotate, change altitude, spiral pattern, follow waypoints, figure-eight, return to home.
```bash
ros2 launch sm_cl_px4_mr_test_2 sm_cl_px4_mr_test_2.launch.py
```

See [sm_cl_px4_mr_test_1 README](../../smacc2_sm_reference_library/sm_cl_px4_mr_test_1/README.md) for full launch instructions.

## License

Copyright 2024-2025 RobosoftAI Inc.

Licensed under the Apache License, Version 2.0.
