# sm_nav2_gazebo_test_4

SMACC2 state machine that runs the `cl_nav2z` client behaviors the other Nav2 reference machines
(`sm_nav2_gazebo_test_1`, `_2`, `_3`) do not use, one behavior per state, in the Nav2 TurtleBot3
Gazebo simulation. It exists so that these behaviors are executed, not only compiled.

The arena world, its map and the Nav2 parameters are those of `sm_nav2_gazebo_test_2`, which must be
built as well.

## Mission

| State | Behavior under test | What happens |
|--|--|--|
| `StSetInitialPose` | `CbWaitPose` | publishes the AMCL initial pose, waits for a map pose |
| `StAcquireSensors` | `CbWaitNav2Nodes` | waits for the Nav2 nodes |
| `StWaitTransform` | `CbWaitTransform` | waits for the `map` to `base_link` transform |
| `StLoadMissingFile` | `CbLoadWaypointsFile` | a path that does not exist: the behavior must post its failure |
| `StLoadWaypoints` | `CbLoadWaypointsFile` | loads `config/waypoints.yaml` through the `waypoints_file` parameter |
| `StNavigateWaypoints` | `CbNavigateNextWaypoint`, `EvWaypointN` | waypoints alpha, bravo, charlie, one per entry |
| `StRotateAtWaypoint` | `CbRotate` | 90 degrees in place after alpha and bravo |
| `StReloadWaypoints` | `CbLoadWaypointsFile` | the same file a second time; the list must come out the same |
| `StNavigateNamedWaypoint` | `CbNavigateNamedWaypoint` | to echo, skipping the five waypoints before it |
| `StCheckAtEcho` | | the robot must be at echo and not at delta |
| `StUnknownWaypoint` | `CbNavigateNamedWaypoint` | a name that is not in the file: the behavior must post its failure |
| `StSeekWaypoint` | `CbSeekWaypoint` | by name, moves the waypoint index back to alpha, the first |
| `StSeekByCount` | `CbSeekWaypoint` | by count, three forward and two back, to bravo |
| `StNavigateUntilReached` | `CbNavigateNextWaypointUntilReached` | bravo, charlie, the unnamed waypoint, delta |
| `StForwardCutShort` | `CbNavigateForward` | 2 m forward, cut short by a 3 s timer |
| `StAbortNavigation` | `CbAbortNavigation` | cancels the goal in flight and leaves when Nav2 has finished cancelling |
| `StRetryForward` | `CbRetry<CbNavigateForward>` | completes the forward motion that was cut short |
| `StNavigateBackwards` | `CbNavigateBackwards`, `CbTrackPathOdometry` | 1 m in reverse |
| `StRotateLookAt` | `CbRotateLookAt`, `CbTrackPathSLAM` | turns to face a point |
| `StPositionControl` | `CbPositionControlFreeSpace` | drives `/cmd_vel` directly to a pose 1 m ahead |
| `StNextWaypointFree` | `CbNavigateNextWaypointFree` | the same direct control, to the current waypoint (echo) |
| `StStopNavigation` | `CbStopNavigation` | sends the current pose as the goal |
| `StActiveStop` | `CbActiveStop` | publishes zero velocity for 3 s |
| `StNavigateLastWaypoint` | `CbNavigateNextWaypoint`, `EvWaypointFinal` | to foxtrot, the last waypoint; left by the navigator's last-waypoint event |
| `StFinalState` | | every behavior succeeded |
| `StMissionFailed` | | reached from any behavior failure or safety timer |

The waypoint file has seven waypoints; the fourth has no name, so that the names after it are seen to
stay with their own waypoints. `CbExpectPosition` and `CbExpectWaypointIndex`, in this package, are
the checks the states above use.

The `n` key skips to the next state. Not covered here: the SLAM behaviors (`CbPauseSlam`,
`CbResumeSlam`, `CbSaveSlamMap`), which need slam_toolbox instead of AMCL, `CbSpiralMotion`, and
`CbDockRobot` and `CbUndockRobot`, which need a dock.

## Build

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --ignore-src --from-paths src -y -r
colcon build --packages-up-to sm_nav2_gazebo_test_4
source install/setup.bash
```

## Run

```bash
ros2 launch sm_nav2_gazebo_test_4 sm_nav2_gazebo_test_4.py
```

Headless, with the machine's exit status and a report (see
`smacc2_performance_tools/nav2_test_tools/README.md`):

```bash
bash src/SMACC2/smacc2_performance_tools/nav2_test_tools/nav2_run.sh install /tmp/nav2_4 4
```
