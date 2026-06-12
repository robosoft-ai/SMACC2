# Typo Fix Backlog

Identified by manual audit. All are in comments/strings — no logic changes required.
Fix as a single dedicated PR after pre-commit / pyupgrade cleanup is merged.

---

## Core `smacc2/`

| File | Line | Wrong | Correct |
|------|------|-------|---------|
| `smacc2/include/smacc2/smacc_state_base.hpp` | 419 | `Initializating` | `Initializing` |
| `smacc2/include/smacc2/impl/smacc_client_behavior_impl.hpp` | 80 | `funcionality` | `functionality` |
| `smacc2/include/smacc2/impl/smacc_orthogonal_impl.hpp` | 74 | `funcionality` | `functionality` |
| `smacc2/src/smacc2/client_bases/smacc_ros_launch_client_2.cpp` | 167 | `Aditional` | `Additional` |

---

## `cl_keyboard`

| File | Line | Wrong | Correct |
|------|------|-------|---------|
| `cl_keyboard/include/cl_keyboard/cl_keyboard.hpp` | 47 | `funcionality` | `functionality` |
| `cl_keyboard/include/cl_keyboard/cl_keyboard.hpp` | 47 | `interated` | `integrated` |

---

## `cl_nav2z` — behaviors & components

| File | Line | Wrong | Correct |
|------|------|-------|---------|
| `.../client_behaviors/cb_spiral_motion.hpp` | 41 | `stae` | `state` |
| `.../client_behaviors/cb_spiral_motion.hpp` | 47 | `usag` | `usage` |
| `.../client_behaviors/cb_spiral_motion.cpp` | 117, 119, 121 | `ellapsed` | `elapsed` (4×) |
| `.../client_behaviors/cb_position_control_free_space.cpp` | 166 | `cummulated` | `cumulated` |
| `.../client_behaviors/cb_pure_spinning.cpp` | 75 | `cummulated` | `cumulated` |
| `.../client_behaviors/cb_save_slam_map.cpp` | 44 | `builded` | `built` |
| `.../components/waypoints_navigator/cp_waypoints_event_dispatcher.hpp` | 29 | `unuseable` | `unusable` |
| `.../components/waypoints_navigator/cp_waypoints_event_dispatcher.hpp` | 1603 | `Doygen` | `Doxygen` |
| `.../components/odom_tracker/cp_odom_tracker.cpp` | 609 | `updatd` | `updated` |

---

## `cl_nav2z` — custom planners

| File | Line | Wrong | Correct |
|------|------|-------|---------|
| `backward_global_planner/src/backward_global_planner.cpp` | 73 | `initializating` | `initializing` |
| `backward_local_planner/include/.../backward_local_planner.hpp` | 91 | `spining` | `spinning` |
| `backward_local_planner/src/backward_local_planner.cpp` | 237 | `funcionality` | `functionality` |
| `forward_local_planner/src/forward_local_planner.cpp` | 635 | `funcionality` | `functionality` |
| `nav2z_planners_common/src/common.cpp` | 42, 43, 52, 57, 68, 74, 83 | `spining` | `spinning` (7×) |
| `pure_spinning_local_planner/src/pure_spinning_local_planner.cpp` | 186 | `funcionality` | `functionality` |
| `undo_path_global_planner/src/undo_path_global_planner.cpp` | 91 | `initializating` | `initializing` |
| `undo_path_global_planner/src/undo_path_global_planner.cpp` | 274 | `POIINT` | `POINT` |

---

## `smacc2_performance_tools`

| File | Line | Wrong | Correct |
|------|------|-------|---------|
| `.../st_state_1.hpp` | 26 | `clases` | `classes` |
| `.../st_state_2.hpp` | 29 | `clases` | `classes` |
