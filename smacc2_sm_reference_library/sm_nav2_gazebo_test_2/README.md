# sm_nav2_gazebo_test_2

SMACC2 state machine that exercises **undo path backwards navigation**
(`CbUndoPathBackwards` + `UndoPathGlobalPlanner` + `BackwardLocalPlanner`) in the
Nav2 TurtleBot3 Gazebo simulation.

## Mission

```
StAllSensorsGo → StSetInitialPose → StNavigateToWaypoint1 → SsRadialPattern1 → StFinalState
```

`SsRadialPattern1` is a superstate that loops 4 times (radial pattern, modeled on
`sm_nav2_test_7` from the nova_carter_sm_library):

```
StiRadialLoopStart ──EvLoopContinue──> StiRadialRotate     (CbAbsoluteRotate, pure spinning)
                                             │ success
                                             v
                                       StiRadialEndPoint   (CbNavigateForward 1 m, odom tracker RECORDS path)
                                             │ success
                                             v
                                       StiRadialReturn     (CbUndoPathBackwards retraces the recorded path)
                                             │ success
                                             v
                                       StiRadialLoopStart  (next ray: 45°, 135°, 225°, 315°)
```

After 4 iterations `EvLoopEnd` exits the superstate to `StFinalState`.

The package provides its own `config/nav2_params.yaml` (stock Jazzy params extended
with the SMACC2 planner/controller plugin family and the cl_nav2z goal checkers) and
`config/default_nav_to_pose_bt.xml` (behavior tree with PlannerSelector,
ControllerSelector and GoalCheckerSelector nodes). Without these, plugin switching —
and therefore undo navigation — cannot work.

## Build

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --ignore-src --from-paths src -y -r
colcon build --packages-select sm_nav2_gazebo_test_2
source install/setup.bash
```

## Run

```bash
ros2 launch sm_nav2_gazebo_test_2 sm_nav2_gazebo_test_2.py
```

Optional runtime viewer:

```bash
ros2 run smacc2_rta smacc2_rta
```

## Debug topics

```bash
# recorded forward path (published by CpOdomTracker, consumed by UndoPathGlobalPlanner)
ros2 topic echo /odom_tracker_path --field poses | grep -c position

# undo plan produced by the global planner
ros2 topic echo /undo_path_planner/global_plan

# plugin selection state
ros2 topic echo /planner_selector
ros2 topic echo /controller_selector
ros2 topic echo /goal_checker_selector

# state machine introspection
ros2 topic echo /SmNav2GazeboTest2/smacc/status
ros2 topic echo /SmNav2GazeboTest2/smacc/transition_log
```

Keyboard: press `N` in the keyboard server terminal to advance states manually.
