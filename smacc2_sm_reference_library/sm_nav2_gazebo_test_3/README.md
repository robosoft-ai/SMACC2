# sm_nav2_gazebo_test_3

SMACC2 state machine exercising the **Nav2 behavior server primitives** through
`CbActionClientBehaviorBase` (smacc2 core `client_behavior_bases/`): the whole
motion mission runs on collision-checked behavior server actions — no
planner/controller stack involved after bring-up.

## Mission

```
StAllSensorsGo → StSetInitialPose → SsPrimitiveLoop
                                          │ EvLoopEnd
                                          v
      StSpinToPillar → StDriveAtPillar → StBackUpSafe → StFinalState
```

`SsPrimitiveLoop` (3 iterations):

```
StiPrimitiveLoopStart ──EvLoopContinue──> StiDriveOnHeading  (CbDriveOnHeading 1.2 m, odometry-relative)
                                                │ success
                                                v
                                          StiSpin            (CbSpin 360°, footprint collision-checked)
                                                │ success
                                                v
                                          StiBackUp          (CbBackUp 1.2 m back to start)
                                                │ success
                                                v
                                          StiPrimitiveLoopStart
```

Each wrapper behavior is ~20 lines on `CbActionClientBehaviorBase<TAction>`:
goal in `onEntry`, `sendGoal` (server-ready guard + rejection watchdog), result
signals wired on the state machine thread, state-scoped
`EvCbSuccess`/`EvCbFailure` transitions.

## Build & Run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sm_nav2_gazebo_test_3
source install/setup.bash
ros2 launch sm_nav2_gazebo_test_3 sm_nav2_gazebo_test_3.py
```

**Spin directions alternate per lap** (counter-clockwise, clockwise, counter-clockwise)
via `runtimeConfigure`, exercising negative target yaw and runtime goal
configuration.

**Collision-abort finale**: after the loop the robot turns to face the pillar and
commands a 2.5 m drive straight at it (obstacle at ~1.15 m). The behavior
server's collision checking must abort the goal — the EXPECTED outcome is
`EvCbFailure` (transition tag `COLLISION_ABORT_OK`), validating the genuine
server-abort path. The mission then backs away and finishes.

**Early-exit cancellation**: pressing `N` while a primitive is executing cancels
the in-flight goal (`CbActionClientBehaviorBase::onExit`) — the robot stops
instead of continuing under an abandoned goal.

Keyboard `N` advances any state manually.
