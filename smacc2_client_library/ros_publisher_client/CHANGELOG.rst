Changelog for package ros_publisher_client
=============================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fixes for ros buildfarm issue.

### Contributors
- brettpac
- pabloinigoblasco

2.3.6 (2023-03-12)
------------------

1.22.1 (2022-11-09)
-------------------
### Added
- Pre-release.

### Contributors
- pabloinigoblasco

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
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
  - Updated smacc2_rta command across readmes.
  - Cleaned up sm_atomic_24hr.
  - Optimized dependencies in move_base_z_planners_common.
  - Renamed event generator library.
  - Added galactic CI setup and renamed rolling files.
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
  - Added new feature, cb_wait_topic_message: asynchronous client behavior.
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic.
  - Corrected all linters and formatters.

### Contributors
- pabloinigoblasco
- brettpac
- Denis Štogl

```rst
Section_2
=========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive, with an option to select specific nodes.
- Base for the `sm_aws_aarehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.
- `Brettpac` branch.
- `mm` improvements.
- Diverse improvements in navigation and performance.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Format fixes for gazebo to show the robot and the lidar.

Fixed
-----
- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.

Removed
-------
- `neo_simulation2` package.

Contributors
------------
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_3
=========

Added
-----
- Diverse improvements in navigation and performance.
- Additional linting and formatting.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introducing slam pausing/resuming functionality in testing sm_dance_bot.
- First working version of sm template and template generator.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Rolling Docker environment to be executed from any environment.
- Initial migration to smacc2.
- Added QOS durability to SmaccPublisherClient.
- More testing on moveit behaviors.
- Husky launch file in sm_dance_bot.
- Waypoint Inputs.

Changed
-------
- Move method after the method it calls to prevent recursion.
- Renamed reference library SMs to smacc2_performance_tools.
- Reworked sm_multi_stage_1 with multistage modes, sequences, and steps.
- Updated dependencies for husky in rolling and galactic.

Fixed
-----
- Minor tuning to mitigate overshot issue cases.
- Fixed launch command for sm_dance_bot_strikes_back in README.md.
- Format fix for Python version in CI.
- Removed node creation and create only a logger.
- Fixed compiling issues.
- Fixed broken master build.

Removed
-------
- Removed merge markers from a Python file.
- Removed parameters smacc.
- Removed test from main moveit CMake.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_4
=========

Added
-----

- Added Brettpac branch (#184) with improvements on sm_dance_bot_warehouse_3 waypoints.
- Added Feature/wharehouse2 dec 14 (#185) introducing warehouse2 enhancements.
- Added Feature/sm warehouse 2 13 dec 2 (#186) for format improvements.
- Added Feature/cb pure spinning (#188) for pure spinning behavior enhancements.
- Added Feature/replanning 16 dec (#193) for replanning improvements.
- Added Feature/undo motion 20 12 (#196) with undo motion navigation enhancements.
- Added Feature/sync 21 12 (#199) for format issue fixes.
- Added Feature/warehouse2 22 12 (#200) for format issue fixes and finishing warehouse2.
- Added Feature/warehouse2 23 12 (#201) for tuning and fixes.
- Added Feature/minor tune (#203) for tuning and fixes.
- Added Feature/undo motion 20 12 (#198) with undo tuning and errors.
- Added Foxy backport (#206) with minor format fixes.
- Added galactic CI build and partial changes for ament_cpplint.
- Added tf2_ros as dependency and satisfied ament_lint_cmake.
- Added missing licences and corrected formatters.
- Added branching example and updated ci-build-source.yml.
- Added workflow for checking doc build and created doxygen-deploy.yml.
- Added workflow for testing prerelease builds and updated doxygen-check-build.yml.
- Added Dockerfile w/ ROS distro as argument.
- Added setupTracing.sh for automated installation of ros-rolling-ros2trace.
- Added alternative ManualTracing and new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------

- Changed "sm_three_some" launch command to "sm_three_some.launch".
- Changed wording from "smacc application" to "SMACC2 library".
- Changed extension of imports and corrected formatting of python file.
- Changed extension of imports and corrected formatting of python file.
- Changed extension of imports and corrected formatting of python file.
- Changed extension of imports and corrected formatting of python file.
- Changed extension of imports and corrected formatting of python file.

Fixed
-----

- Fixed SrConditional formatting and trigger logic.
- Fixed warehouse 3 problems and other core improvements.
- Fixed trailing spaces, codespell, and python linters warnings.
- Fixed weird moveit not downloaded repo and added missing file from warehouse2.
- Fixed minor linking errors for Foxy backport.
- Fixed bug in smacc2 component and reverted markdowns to html.
- Fixed cleanup and edited tracing.md to reflect new tracing event names.
- Fixed build of missing rolling repositories and enabled Navigation2 for semi-binary build.

Removed
-------

- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed tracing directory and moved tracing.md to tracing directory.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.

Co-Authored-By
--------------

- Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_5
=========

Added
-----
- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.

Fixed
-----
- Fixed source CI and corrected README overview. (#62).
- Fixed all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Other
-----
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting improvements.
- Noticed a note that was not removed.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progress in AWS navigation.
- Progressing in AWS navigation.
- Base for the sm_aws_aarehouse navigation.
- More on performance and other issues.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation

Section 6
----------

### Added
- New client behavior `cb_wait_topic_message`: asynchronous behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: waits for nav2 nodes to subscribe to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.

### Changed
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Base for the `sm_aws_warehouse` navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- Going forward in testing `sm_dance_bot` introducing slam pausing/resuming functionality.

### Fixed
- Removed some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.

### Removed
- `neo_simulation2` package.

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_7
=========

Added
-----

- First working version of sm template and template generator. (#127)
- Feature/dance bot s pattern (#129)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Feature/nav2z renaming (#144)
- Added SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Feature/migration moveit client (#151)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)

Changed
-------

- Minor tweaks (#130)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Fix CI: format fix python version (#148)
- Update package list. (#142)
- Update readme (#164)
- Add SM core test (#138)
- Initial migration to smacc2
- Finetuning waypoints (#187)

Fixed
-----

- Waypoints navigator bug (#133)
- Resolve compile warnings (#137)
- Fixing pipeline error
- Fixing broken master build
- SrConditional fixes and formatting (#168)

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Remove node creation and create only a logger. (#149)
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing parameters smacc
- Removing sm_dance_bot_msgs

```rst
Section 8
=========

Added
-----

- Feature/planner changes on 16th December (#191)
- Feature/replanning on 16th December (#193)
- Feature/undo motion on 20th December (#196)
- Feature/undo motion on 20th December (#198)
- Feature/sync on 21st December (#199)
- Feature/warehouse2 on 22nd December (#200)
- Feature/warehouse2 on 23rd December (#201)
- Feature/minor tune (#203)
- Merging code from backport foxy and updates about autoware (#208)
- Foxy backport (#206)
- Add galactic CI build because Navigation2 is broken in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency to find include
- Add workflow for checking doc build
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use manual deployment for now
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Update name of package and package.xml to pass liter
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Open new folder for additional tracing contents
- Move tracing.md to tracing directory
- Add setupTracing.sh
- Create alternative ManualTracing
- Add new sm markdowns
- Add a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Update smacc_sm_reference_library/sm_atomic/README.md
- Enable Navigation2 for semi-binary build

Changed
-------

- Rename header files and correct format
- Update description table
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Changed wording "smacc application" to "SMACC2 library"

Fixed
-----

- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Fix broken build

Removed
-------

- Delete tracing directory
- Removed manual installation of ros-rolling-ros2trace
- Reactivating smacc2 nav clients for rolling via submodules
- Bug in smacc2 component
- Reverted markdowns to html
- Additional cleanup
- Cleanup
```

*pabloinigoblasco*

```rst
Section_9
=========

Added
-----

- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files. (#58)
- Added more Readme Updates (#72).
- Added more Readme (#74).
- Added created new sm from sm_respira_1 (#76).
- Added base for the sm_aws_aarehouse navigation.
- Added progress in aws navigation.
- Added several core improvements during navigation testing.
- Added progress in aws navigation demo.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.
- Added progress in aws navigation demo.
- Added progress in aws navigation demo.
- Added progress in aws navigation demo.

Changed
-------

- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated README.md.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.

Fixed
-----

- Fixed source CI and corrected README overview. (#62).
- Fixed pre-commit.
- Fixed pre-commit.
- Fixed pre-commit.
- Corrected all linters and formatters.

Removed
-------

- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Cleaned up sm_atomic_24hr.
- Removed trailing spaces.
- Removed sm_atomic_performance_trace_1.
- Removed sm_atomic_performance_test_a_1.
- Removed sm_atomic_performance_test_a_2.
- Removed sm_atomic_performance_test_c_1.
- Removed sm_multi_stage_1.

Authors
-------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_10
==========

Added
-----

- New client behavior `cb_wait_topic_message`:
  Asynchronous behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2:
  Waits for nav2 nodes to subscribe to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite`:
  - Visualizes TurtleBot3.
  - Precommit changes.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo improvements.
- `sm_multi_stage_1` doubling.
- `sm_multi_stage_1` working progress.
- `sm_multi_stage_1` improvements.
- Diverse improvements in navigation and performance.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components.
- Introducing slam pausing/resuming functionality in `sm_dance_bot`.
- `smacc2::deep_history` syntax enhancements.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.
- Minor formatting improvements.
- Cleaning and lidar show/hide option in `sm_dance_bot`.
- Format fixes for gazebo visualization.
- Move method after the method it calls to prevent recursion.

Fixed
-----

- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Removed merge markers from a Python file.

Removed
-------

- `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_11

### Added
- First working version of sm template and template generator. (#127)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Rolling Docker environment to be executed from any environment (#154)
- Waypoint Inputs (#178)

### Changed
- Renamed reference library SMs to smacc2_performance_tools (#166)
- Renamed navigation 2 stack to navigation 2 stack renaming
- Renamed sm_pubsub_1 to sm_pubsub_1 part 2 (#170)
- Renamed sm_advanced_recovery_1 to sm_advanced_recovery_1 renaming (#171)
- Reworked sm_multi_stage_1 (#172)
- Refactored husky launch file in sm_dance_bot for AWS navigation (#174)
- Finetuned waypoints (#187)

### Fixed
- Fixed launch command in README.md for sm_dance_bot_strikes_back
- Fixed CI: format fix python version (#148)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error

### Removed
- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed node creation and create only a logger. (#149)
- Removed test from main moveit cmake

### Miscellaneous
- Minor tweaks (#130)
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Minor configuration in moveit migration (#151)
- Minor changes (#175)
- Minor format (#180)
- Minor changes and headless in warehouse 2 (#182)
- Minor changes and headless in warehouse 2 (#186)
- Minor changes and headless in cb pure spinning (#189)

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_12
==========

Added
-----
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Fix rolling builds (#222)
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Foxy backport (#206)
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
- Remove example things from Foxy CI setup. (#214)
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licences.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch

Changed
-------
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green

Fixed
-----
- Minor broken build

Removed
-------
- Merge
- Headless and other fixes
- Default values
- Pure spinning behavior missing files
- More fixes
- Missing
- Missing sm
- Updating subscriber publisher components
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Foxy ci
- Fix
- Minor linking errors foxy
- Minor format
- Minor linking errors foxy
- Weird moveit not downloaded repo
- Minor format issues
- Format issues
- Minor formatting fixes
- Minor format
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes
- Minor changes

```rst
Section_13
==========

Added
~~~~~
- Reactivated smacc2 nav clients for rolling via submodules.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
~~~~~~~
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders.
- Renamed event generator library.

Fixed
~~~~
- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_reference_library.
- Cleaned up sm_atomic_24hr.
- Fixed source CI and corrected README overview.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- Corrected all linters and formatters.

Removed
~~~~~~~
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_14
==========

Added
-----
- Introducing new feature `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior `add` for nav2: waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.
- Base for the `sm_aws_warehouse` navigation.
- Navigation parameters fixes on `sm_dance_bot`.
- New client behavior `cb_pause_slam`.
- Visualizing `turtlebot3` in `sm_dance_bot_lite`.
- Launch option for `gz_lidar` in `sm_dance_bot`.
- Gazebo fixes for showing the robot and lidar in various dance bot features.
- `sm_multi_stage_1` doubling.
- Cleanup and format fixes for various dance bot features.
- AWS demo improvements.
- Removal of `neo_simulation2` package.
- Source build enabled for testing.
- Adjustments in build packages for source CI.
- Diverse improvements in navigation and performance.

Changed
-------
- Minor formatting improvements.

Fixed
-----
- Removed some compile warnings.

Removed
-------
- Redundant and minor format fixes.

Contributors
------------
- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_15

### Added
- Added feature/diverse improvements for navigation performance (#117).
- Added slam toggle and smacc deep history feature (#122).
- Added first working version of sm template and template generator (#127).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient (#163).
- Added sm_pubsub_1 (#169).
- Added sm_pubsub_1 part 2 (#170).
- Added sm_advanced_recovery_1 renaming (#171).
- Added sm_multi_stage_1 reworking (#172).
- Added feature/aws navigation sm dance bot (#174).
- Added warehouse2 (#177).
- Added waypoint inputs (#178).
- Added sm_dance_bot_warehouse_3 (#181).
- Added feature/sm warehouse 2 13 dec 2 (#182).

### Changed
- Changed method order to prevent recursion (#126).
- Changed launch command in README.md for sm_dance_bot_strikes_back (#148).
- Changed node creation to logger creation only (#149).
- Changed Docker environment to be executed from any environment (#154).

### Fixed
- Fixed minor format issues (#134).
- Fixed CI format for Python version (#148).
- Fixed broken master build (#167).
- Fixed pipeline error (#167).
- Fixed broken build in aws navigation (#174).

### Removed
- Removed merge markers from a Python file (#119).
- Removed sm_dance_bot_msgs dependency (#144).
- Removed parameters smacc (#147).
- Removed test from main moveit CMake (#151).

### Miscellaneous
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>.

```rst
Section_16
==========

Added
-----
- Brettpac branch (#184)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)
- Pure spinning behavior missing files
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Several fixes (#194)
- Minor changes (#195)
- Feature/undo motion 20 12 (#196)
- Tuning warehouse3 (#197)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Added missing file from warehouse2 (#205)
- Docker build files for all versions (#225)
- Fix code generators (#221)
- Feature/retry behavior warehouse 1 (#226)
- Foxy backport (#206)

Changed
-------
- Update SM template and make example code clearly visible
- Remove use of node in the sm performance template
- Updated templated to use Blackboard storage
- Update template to resolve the global data correctly
- Update sm_name.hpp
- Correct codespell
- Correct python linters warnings
- Add galactic CI build because Navigation2 is broken in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency to find include
- Disable ament_cpplint
- Disable some packages and update workflows
- Bump ccache version
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licences
- Disable cpplint and cppcheck linters
- Correct formatters
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch

Removed
-------
- Branching example

Authors
-------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: reelrbtx <brett2@reelrobotics.com>
- Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```rst
Section_17
==========

Added
-----
- Added Dockerfile with ROS distro as argument for easier setup.
- Added setupTracing.sh for automated installation of necessary packages and configuration of tracing group.
- Added alternative ManualTracing method.
- Added new SM markdowns.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools for performance testing improvements.
- Added new feature cb_wait_topic_message for asynchronous client behavior.

Changed
-------
- Updated wording from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events for clarity.
- Renamed folders and files for consistency.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Fixed trailing spaces.
- Fixed formatting in various files.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed galactic builds, keeping only rolling, and using .repos file instead of submodules.

Collaborators
-------------
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_18
==========

Added
~~~~~
- Introduce new feature, cb_wait_topic_message: an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for nav2, which waits for nav2 nodes to subscribe to the /bond topic and confirms their activity. Users can optionally select the nodes to wait for.
- Introduce cb_pause_slam client behavior.

Changed
~~~~~~~
- Correct all linters and formatters.
- Minor formatting improvements.

Fixed
~~~~
- Resolve navigation parameters issues on sm_dance_bot.
- Remove some compile warnings.

Removed
~~~~~~~
- Eliminate redundant format improvements.

Other
~~~~~
- Implement base for the sm_aws_warehouse navigation.
- Make progress in AWS navigation demo.
- Implement several core improvements during navigation testing.
- Merge and progress in development.
- Update yaml files.
- Implement hotfix for minor issues.
- Visualize turtlebot3 in sm_dance_bot.
- Add lidar show/hide option in sm_dance_bot.
- Clean files and improve formatting in sm_dance_bot.
- Fix gazebo issues to display the robot and lidar.
- Double sm_multi_stage_1 functionality.
- Clean up precommit process.
- Work on AWS demo functionality.
- Make sm_multi_stage_1 operational.
- Continue development on sm_multi_stage_1.
- Progress in Brettpac branch development.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_19
===========

Added
-----

- Added multistage modes to `sm_multi_stage_1`.
- Added sequences to `sm_multi_stage_1`.
- Added steps to `sm_multi_stage_1`.
- Added sequence d to `sm_multi_stage_1`.
- Added sequence c to `sm_multi_stage_1`.
- Added mode_5_sequence_b.
- Added mode_4_sequence_b.
- Added finishing touches to `sm_multi_stage_1`.
- Added AWS navigation to `sm_dance_bot`.

Changed
-------

- Reworked `sm_multi_stage_1` with improvements.

Fixed
-----

- Fixed a bug where recursion could occur by moving a method after the one it calls.
- Fixed minor format issues.
- Fixed compile warnings.
- Fixed CI format for Python version.
- Fixed launch command in README.md for `sm_dance_bot_strikes_back`.
- Fixed some linting warnings.
- Fixed compiling issues.

Removed
-------

- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs`.
- Removed parameters from `smacc`.

Other
-----

- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, and pabloinigoblasco <pablo@ibrobotics.com>.
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others.
- Updated package list.
- Rolled Docker environment for execution from any environment.
- Moved reference library SMs to `smacc2_performance_tools`.
- Added QOS durability to `SmaccPublisherClient`.
- Added reliability QOS config to `SmaccPublisherClient`.
- Added dependencies for Husky simulation.
- Progressed in navigation, slam toggle client behaviors, and slam_toolbox components.
- Progressed in testing `sm_dance_bot` and `moveit` behaviors.
- Progressed in `moveit` migration testing.
- Progressed in the `sm_dance_bot` tests.
- Progressed in `sm_dance_bot_lite`.
- Progressed in `sm_pubsub_1`.
- Progressed in `sm_advanced_recovery_1` renaming.
- Progressed in `sm_multi_stage_1` reworking.
- Progressed in testing `moveit` behaviors.
- Progressed in `sm_multi_stage_1`.
- Progressed in `sm_multi_stage_1` most.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` c sequence.
- Progressed in `sm_multi_stage_1` steps.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` c sequence.
- Progressed in `sm_multi_stage_1` most.
- Progressed in `sm_dance_bot` introducing slam pausing/resuming functionality.
- Progressed in `sm_dance_bot` and `s-pattern` polishing.
- Progressed in `sm_dance_bot` and `s-pattern` polishing.
- Progressed in `sm_dance_bot` and `s-pattern` polishing.
- Progressed in the moveit migration testing.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed in the sm_dance_bot tests.
- Progressed

```rst
Section_20
==========

Added
-----

- Update dependencies for husky in rolling and galactic.
- Progress on aws navigation and refactorings on navigation clients and behaviors.
- More on aws demo.
- Warehouse2 progress.
- Waypoint Inputs.
- Wharehouse2 progress.
- Sm_dance_bot_warehouse_3.
- Feature/sm warehouse 2 13 dec 2.
- Finetuning waypoints.
- Feature/cb pure spinning.
- Feature/planner changes 16 12.
- Feature/replanning 16 dec.
- Several fixes.
- Feature/undo motion 20 12.
- Improving undo motion navigation warehouse2.
- Tuning warehouse3.
- Feature/sync 21 12.
- Feature/warehouse2 22 12.
- Finishing warehouse2.
- Feature/warehouse2 23 12.
- Feature/minor tune.
- Fixing warehouse 3 problems and other core improvements.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components.
- Autoware demo.
- Docker files for different revisions.
- Retry behavior warehouse 1.
- Update file for fake hardware simulation and add file for gazebo simulation.
- Multiple controllable leds plugin.
- Progress in husky demo.
- Add ignition file and update repos files.
- Improving navigation behaviors.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.

Changed
-------

- Only rolling version should be pre-released on master.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.
- Making models local.

Fixed
-----

- Fix formatting.
- Fixing broken build.
- Fix: some formatting and templating on SrConditional.
- Fix: move trigger logic into headers.
- Fix: lint.
- Fix broken source build.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.

Removed
-------

- Weird moveit not downloaded repo.
- Pure spinning behavior missing files.
- Missing sm.
- Missing file.
- Minor broken build.
- Some reordering fixes.
- Minor format fix.
- Other minor changes.
```

```rst
Section 21
==========

Added
-----
- First ensure you have the necessary package installed:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Use docs/ as source folder and output directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------
- Rename header files and correct format.
- Change extension of imports.
- Correct GitHub branch reference.
- Update name of package and package.xml.
- Update description table.
- Update table.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Renaming of event generator library.
- Update c_cpp_properties.json.

Fixed
-----
- Correct formatting of python file.
- Correct formatters.
- Correct trailing spaces.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Disable further packages.
- Ignore all packages except smacc2 and smacc2_msgs.
- Revert "Ignore all packages except smacc2 and smacc2_msgs".

Other Changes
-------------
- Enable cppcheck.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Update mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivating smacc2 nav clients for rolling via submodules.
- More progress on navigation rolling.
- Performance tests improvements.
- More on performance and other issues.
- More changes on performance tests.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Sm_reference_library reformatting.
- Enable manual deployment for now.
- Update ci-build-source.yml.
- Update doxygen-check-build.yml.
- Update changelogs.
- Update ci-build-source.yml.
- Update tracing/ManualTracing.md.
- Update smacc_sm_reference_library/sm_atomic/README.md.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_rta command across readmes.
- Update smacc2_sm_reference_library/sm_atomic/README.md.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_sm_reference_library/sm_atomic/README.md.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update smacc2_rta command across readmes.
- Update tracing/ManualTracing.md.
- Update tracing.md to tracing directory.
- Update tracing.md to tracing directory.
- Update tracing/ManualTracing.md.
- Update tracing.md to reflect new tracing event names.
```

```rst
Section_22
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection.
- New feature: cb pause slam client behavior.
- New feature: sm_dance_bot_lite.

Changed
-------
- Updated launch command.
- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor hotfixes.

Fixed
-----
- Fixed precommit issues.
- Removed some compile warnings.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

Section 23
-----------

### Added
- Added `sm_dance_bot` visualizing `turtlebot3`.
- Added lidar show/hide option for cleaning.
- Added formatting improvements.

### Changed
- Improved `sm_dance_bot` Lite Gazebo functionality (#104).
- Enhanced Gazebo visualization for the robot and lidar.
- Doubled `sm_multi_stage_1` functionality (#103).
- Improved Gazebo functionality for `sm_dance_bot_strikes_back`.

### Fixed
- Fixed formatting issues.
- Fixed issues with `sm_multi_stage_1`.
- Fixed issues with AWS demo (#108).
- Fixed issues with `sm_dance_bot` strikes back.
- Fixed issues with `sm_dance_bot_lite`.
- Fixed waypoint navigator bug (#133).
- Fixed minor format issues (#134).
- Fixed CI format for Python version (#148).

### Removed
- Removed `neo_simulation2` package (#112).
- Removed `sm_dance_bot_msgs` package.
- Removed unnecessary parameters from `smacc`.

### Miscellaneous
- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored with DecDury <declandury@gmail.com>.
- Co-authored with Denis Štogl <destogl@users.noreply.github.com>.
- Updated package list (#142).
- Updated READMEs with SVGs for atomic, dance_bot, and others (#140).
- Updated Docker environment for execution in any environment (#154).
- Progressed in migration to `smacc2` for MoveIt client (#151).
- Continued improvements and updates in various areas.

Section_24
==========

Added
-----
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)

Changed
-------
- Moved reference library SMs to smacc2_performance_tools (#166)
- Added reliability qos config
- Finishing touches on moveit
- Progress on moveit behaviors
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- Redoing sm_dance_bot_warehouse_3 waypoints
- Improving undo motion navigation warehouse2
- Tuning and fixes
- Improvements in smacc core adding more components mostly developed for autoware demo
- Refining cp subscriber cp publisher
- Several fixes
- Replanning for all our examples
- Tuning and fixes
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Updating subscriber publisher components
- Progress in autoware machine
- Progress in barrel husky
- Progress in barrel demo
- Progress in warehouse2

Fixed
-----
- Add a missing colon
- Fixing pipeline error
- Fixing broken master build
- Fixing broken build
- Fixing startup problems in warehouse 3
- Fixing format and minor
- Fixing docker for foxy and galactic
- Fixing docker build files for all versions
- Fixing broken build

Removed
-------
- Pre-commit cleanup

Co-authored-by
--------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>

```rst
Section_25
==========

Version 0.1.0 (Date: TBD)
-------------------------

### Added
- Build-status table
- Detailed install instructions ([source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme))
- `setupTracing.sh`: Installs necessary packages and configures tracing group

### Changed
- Default build type set to `Release` for faster performance and smaller executables
- Updated examples section

### Fixed
- Resolved missing dependency in `smacc_msgs` and reorganized for better overview
- Fixed broken `sm_respira` code
- Bug in `smacc2` component

### Removed
- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`

### Other Changes
- Refactored the whole project
- Removed test phase from CMake and dependencies from `package.xml`
- Compiled with navigation and `slam_toolbox`
- Enabled all packages to compile
- Improved `getLogger` functionality
- Reset all versions to 0.0.0
- Ignored all packages except `smacc2` and `smacc2_msgs`
- Reverted "Ignore all packages except `smacc2` and `smacc2_msgs`" commit
- Reverted markdowns to HTML
- Edited `tracing.md` to reflect new tracing event names
- Performance tests improvements
- Cleaned up `sm_respira_1` format
- Optimized dependencies in `move_base_z_planners_common`
- Renamed event generator library
- Corrected build-overview table
- Updated and unified CI configurations
- Used `tf_geometry_msgs.h` in Galactic
- Used Galactic branches in `.repos-file`

### Contributors
- Denis Štogl
- Pablo Iñigo Blasco
```
