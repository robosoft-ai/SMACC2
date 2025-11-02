Changelog for package sm_atomic
==================================

2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fixes for ros buildfarm issue.

### Contributors
- brettpac
- pabloinigoblasco

2.3.6 (2023-03-12)
--------------------------
No significant changes.

1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release.

### Contributors
- pabloinigoblasco

0.3.0 (2022-04-04)
--------------------------
No significant changes.

0.0.0 (2022-11-09)
---------------------------
### Added
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Ignored packages not to be released.
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2.
  - Progress on navigation rolling.
  - Renamed folders, deleted tracing.md, edited README.md.
  - Added smacc2_performance_tools.
  - Performance tests improvements.
  - Format cleanup for sm_respira_1.
  - Optimized dependencies in move_base_z_planners_common.
  - Renamed event generator library.
  - Added galactic CI setup and renamed rolling files.
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
  - Created new sm from sm_respira_1.
  - Several core improvements during navigation testing.
  - New feature, cb_wait_topic_message: asynchronous client behavior.
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic.
  - Corrected all linters and formatters.

### Contributors
- pabloinigoblasco
- brettpac
- Denis Štogl

0.1.0 (Date: TBD)
-------------------------

- Build-status table
- Detailed install instructions ([source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme))
- `setupTracing.sh` script for installing necessary packages and configuring tracing group

- Default build type set to `Release` for faster build and smaller executables
- Updated examples section

- Resolved missing dependency in `smacc_msgs` and reorganized for better overview
- Fixed bug in `smacc2` component
- Cleaned up formatting in `sm_respira_1` and `sm_atomic_24hr`
- Optimized dependencies in `move_base_z_planners_common`
- Renamed event generator library
- Corrected build-overview table
- Updated and unified CI configurations
- Used `tf_geometry_msgs.h` in Galactic
- Used Galactic branches in `.repos-file`

- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`

Other Changes
-------------
- Reorganized project structure
- Removed test phase from CMake and dependencies from package.xml
- Compiled with navigation and slam_toolbox
- Enabled all packages to compile
- Refactored `getLogger` functionality
- Reverted changes in commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Ignored all packages except `smacc2` and `smacc2_msgs`
- Reactivated `smacc2` nav clients for Rolling via submodules
- Edited tracing.md to reflect new tracing event names
- Performance tests improvements and other related changes
- Do not execute clang-format on `smacc2_sm_reference_library`
- Cleaned up `sm_atomic_24hr`
- More cleanup on `sm_atomic_24hr`

- Denis Štogl
- Pablo Iñigo Blasco
