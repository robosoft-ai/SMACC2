Changelog for package sm_panda_cl_moveit2z_cb_inventory
=======================================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further work on buildfarm problem
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

2.3.6 (2023-03-12)
------------------

1.22.1 (2022-11-09)
-------------------
### Added
- Pre-release
- Contributors: pabloinigoblasco

### Changed
- Improved navigation client behaviors and husky barrel demo (#311)
  - Enhanced navigation client behaviors and husky barrel demo
  - Various improvements in action client and cb sequence for husky barrel search
  - Added more and better navigation behaviors for husky barrel search demo
  - Functionality enhancements in navigation and warehouse 3
  - Format improvements
- Initial fixing of single UR sim. (#302)
  - Updated repos files and README.md for gazebo simulation of UR5
  - Fixed formatting
  - Python flake formatting fixes
  - Co-authored-by: Manuel M <mamueluth@gmail.com>
- Feature/multi UR5 sim (#290)
  - Removed galactic builds from master and kept only rolling
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup for sm_respira_1
  - More performance and other issues
  - Format cleanup for sm_respira_test_2
  - Do not execute clang-format on smacc2_sm_reference_library package
  - Reformatting for sm_reference_library
  - Corrected trailing spaces
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Minor formatting
  - Added galactic CI setup and renamed rolling files (#58)
  - Fixed source CI and corrected README overview (#62)
  - Updated c_cpp_properties.json
  - Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69)
  - Updated doxygen links (#70)
  - More Readme Updates (#72)
  - More Readme (#74)
  - Created new sm from sm_respira_1 (#76)
  - Feature/core and navigation fixes (#78)
  - Progress in aws navigation demo
  - Several core improvements during navigation testing
  - Format improvements
  - Feature/aws demo progress (#80)
  - More on navigation
  - Reworked sm_advanced_recovery_1 (#83)
  - More sm_advanced_recovery_1 work (#85)
  - Round 4 of sm_advanced_recovery_1 (#86)
  - More sm_advanced_recovery_1 (#84)
  - Sm_atomic_performance_test_c_1 (#88)
  - Modifying sm_atomic_performance_test_a_2 (#89)
  - Sm_multi_stage_1 (#90)
  - More sm_multi_stage_1 (#91)
  - Update README.md
  - Wait topic message client behavior (#81)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Attempting precommit fixes
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Feature/wait nav2 nodes client behavior (#82)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait
  - Corrected all linters and formatters

```rst
Section_2
=========

Added
-----

- Feature/aws demo progress (#92)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Added formatting improvements.
  - Developed new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for nav2, allowing waiting for nav2 nodes subscribing to the /bond topic and ensuring they are alive.
  - Continued progress in AWS navigation demo.

- Feature/sm dance bot fixes (#93)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Added formatting improvements.
  - Developed new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for nav2, allowing waiting for nav2 nodes subscribing to the /bond topic and ensuring they are alive.
  - Continued progress in AWS navigation demo.
  - Fixed navigation parameters on sm_dance_bot.

- Feature/sm aws warehouse (#94)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Added formatting improvements.
  - Developed new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for nav2, allowing waiting for nav2 nodes subscribing to the /bond topic and ensuring they are alive.
  - Continued progress in AWS navigation demo.
  - Merged and made progress.

- Feature/sm dance bot fixes (#95)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Added formatting improvements.
  - Developed new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for nav2, allowing waiting for nav2 nodes subscribing to the /bond topic and ensuring they are alive.
  - Continued progress in AWS navigation demo.
  - Fixed navigation parameters on sm_dance_bot.

- Remove some compile warnings. (#96)

- Feature/cb pause slam (#98)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Added formatting improvements.
  - Developed new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for nav2, allowing waiting for nav2 nodes subscribing to the /bond topic and ensuring they are alive.
  - Continued progress in AWS navigation demo.
  - Fixed navigation parameters on sm_dance_bot.
  - Implemented cb pause slam client behavior.

- sm_dance_bot_lite (#99)
  - Updated yaml files.

- Rename doxygen deployment workflow (#100)

- sm_dance_bot visualizing turtlebot3 (#101)

- Feature/dance bot launch gz lidar choice (#102)
  - Improved visualization of turtlebot3.
  - Added cleaning and lidar show/hide option.
  - Enhanced file cleaning and formatting.
  - Implemented more fixes.

- Feature/sm dance bot lite gazebo fixes (#104)
  - Improved visualization of turtlebot3.
  - Added cleaning and lidar show/hide option.
  - Enhanced file cleaning and formatting.
  - Fixed gazebo issues to show the robot and the lidar.
  - Made format fixes.

- sm_multi_stage_1 doubling (#103)

- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Improved visualization of turtlebot3.
  - Added cleaning and lidar show/hide option.
  - Enhanced file cleaning and formatting.
  - Made more fixes.
  - Fixed gazebo issues for sm_dance_bot_strikes_back.

- Precommit cleanup run (#106)

- aws demo (#108)
  - Implemented AWS demo.
  - Made formatting adjustments.

- Brettpac branch (#110)
  - Made progress on sm_multi_stage_1.
  - Continued development on sm_multi_stage_1 with multiple stages.

- Brettpac branch (#111)
  - Made progress on sm_multi_stage_1.
  - Continued development on sm_multi_stage_1 with multiple stages.

- a3 (#113)

Removed
-------

- Removed neo_simulation2 package. (#112)
```

Section_3
==========

Added
-----
- Enable source build on PR for testing.
- Additional linting and formatting.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introducing slam pausing/resuming functionality in sm_dance_bot testing.
- First working version of sm template and template generator.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Rolling Docker environment to be executed from any environment.
- Added SM Atomic SM generator.
- Initial migration to smacc2.
- Added QOS durability to SmaccPublisherClient.
- More testing on moveit behaviors.
- Husky launch file in sm_dance_bot.
- Update dependencies for husky in rolling and galactic.

Changed
-------
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.
- Resolved compile warnings.
- Minor navigation improvements.
- Using local action messages.
- Removed node creation and create only a logger.
- Moved reference library SMs to smacc2_performance_tools.
- Added QOS durability to SmaccPublisherClient.
- Refactored to remove line.
- Added reliability QOS config.

Fixed
-----
- Corrected formatting.
- Remove merge markers from a python file.
- Fix CI: format fix python version.
- Noticed launch command was incorrect in README.md and fixed it.
- Slight waypoint 4 and iterations changes so the robot can complete the course.
- Fixed compiling issues.
- Fixed broken master build.
- Fixing broken build.

Removed
-------
- Remove neo_simulation2 package.
- Remove parameters smacc.
- Remove sm_dance_bot_msgs.
- Remove merge markers from a python file.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

Section 4
----------

Added
-----
- Added Waypoint Inputs (#178)
- Added sm_dance_bot_warehouse_3 (#181)
- Added Brettpac branch (#184)
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files (#189)
- Added planner changes 16 12 (#191)
- Added replanning 16 dec (#193)
- Added undo motion 20 12 (#196)
- Added undo motion 20 12 (#198)
- Added sync 21 12 (#199)
- Added warehouse2 22 12 (#200)
- Added warehouse2 23 12 (#201)
- Added minor tune (#203)
- Added fixing warehouse 3 problems, and other core improvements (#204)
- Added missing file from warehouse2 (#205)
- Added Merging code from backport foxy and updates about autoware (#208)
- Added workflow for checking doc build
- Added doxygen-deploy.yml
- Added workflow for testing prerelease builds
- Added smacc2 and smacc2_msgs
- Added setupTracing.sh
- Added alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- Changed wording "smacc application" to "SMACC2 library"
- Changed html to markdown syntax in smacc_sm_reference_library/sm_atomic/README.md

Fixed
----
- Fixed SrConditional fixes and formatting (#168)
- Fixed some formatting and templating on SrConditional
- Fixed move trigger logic into headers
- Fixed lint
- Fixed several fixes (#194)

Removed
-------
- Removed manual installation of ros-rolling-ros2trace
- Removed tracing directory

Co-authored-by
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <destogl@users.noreply.github.com>

Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_5
=========

Added
-----

- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------

- Renamed tracing events.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed rolling files.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.

Fixed
-----

- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.

Removed
-------

- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.

Other
-----

- Additional cleanup.
- Edited tracing.md to reflect new tracing event names.
- Some progress on navigation rolling.
- Some progress in aws navigation.
- Progress in aws navigation demo.
- Several core improvements during navigation testing.
- Minor formatting improvements.
- More on performance and other issues.
- More changes on performance tests.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
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

```rst
Section_6
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive. Optionally select the nodes to wait.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.
- Gained traction on `sm_multi_stage_1`.
- Diverse improvements in navigation and performance.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- Introducing slam pausing/resuming functionality in testing `sm_dance_bot`.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.
- Format fixes.
- Minor hotfix.
- Cleaning and lidar show/hide option.
- Format improvements.
- Gazebo fixes to show the robot and the lidar.
- Precommit cleanup run.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Remove merge markers from a Python file.

Removed
-------

- Removed `neo_simulation2` package.

Fixed
-----

- Remove some compile warnings.

Authors
-------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_7

### Added
- Added feature "dance bot s pattern" (#128)
- Added first working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added husky launch file in sm_dance_bot for AWS navigation (#174)
- Added warehouse2 progress (#179)
- Added waypoint inputs (#178)
- Added more waypoints to sm_dance_bot_warehouse_3 (#181)
- Added SrConditional fixes and formatting (#168)

### Changed
- Changed "Finnaly" to "Finally"
- Changed method order to prevent recursion (#126)
- Changed navigation 2 stack naming
- Changed launch command in README.md for sm_dance_bot_strikes_back
- Changed node creation to logger only (#149)
- Changed Docker environment to be executed from any environment (#154)
- Changed state machine transition timestamp (#165)
- Changed reference library SMs to smacc2_performance_tools (#166)
- Changed formatting in several places

### Fixed
- Fixed minor tweaks (#130)
- Fixed waypoints navigator bug (#133)
- Fixed overshot issue cases in navigation tuning
- Fixed format issues (#134)
- Fixed CI format for Python version (#148)
- Fixed errors introduced on formatting during migration to smacc2
- Fixed missing dependencies and linting warnings
- Fixed broken master build
- Fixed pipeline error
- Fixed compiling issues
- Fixed broken build for AWS navigation
- Fixed some formatting and templating on SrConditional

### Removed
- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed test from main moveit cmake

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section 8
=========

Added
-----
- Feature/cb pure spinning (#188): Implemented pure spinning behavior with minor changes and headless mode.
- Feature/planner changes 16 12 (#191): Introduced minor changes and fixes to the planner.
- Feature/replanning 16 dec (#193): Improved replanning for all examples with several fixes.
- Feature/undo motion 20 12 (#196): Enhanced undo motion navigation in warehouse2.
- Feature/sync 21 12 (#199): Addressed format issues in synchronization.
- Feature/warehouse2 22 12 (#200): Fixed format issues and completed warehouse2.
- Feature/warehouse2 23 12 (#201): Tuned and fixed warehouse2.
- Feature/minor tune (#203): Tuned and fixed warehouse 3 problems, and made core improvements.
- Add mergify rules file (#209): Added rules for mergify.
- Add Autoware Auto Msgs into not-released dependencies (#220): Included Autoware Auto Msgs in dependencies.
- Fix rolling builds (#222): Resolved issues with rolling builds.
- Foxy backport (#206): Backported changes to Foxy.
- Fix trailing spaces: Corrected trailing spaces.
- Correct codespell: Fixed spelling errors.
- Correct python linters warnings: Addressed Python linter warnings.
- Add galactic CI build: Added Galactic CI build due to Navigation2 issues in rolling.
- Add partial changes for ament_cpplint: Included partial changes for ament_cpplint.
- Add tf2_ros as dependency: Added tf2_ros as a dependency.
- Disable ament_cpplint: Turned off ament_cpplint.
- Disable some packages and update workflows: Deactivated certain packages and updated workflows.
- Bump ccache version: Updated ccache version.
- Ignore further packages: Ignored additional packages.
- Satisfy ament_lint_cmake: Met ament_lint_cmake requirements.
- Add missing licences: Included missing licenses.
- Disable cpplint and cppcheck linters: Turned off cpplint and cppcheck linters.
- Correct formatters: Fixed formatters.
- Enable cppcheck: Enabled cppcheck.
- Correct formatting of python file: Adjusted Python file formatting.
- Included necessary package and edited Threesome launch: Updated package and launch file for Threesome.
- Rename header files and correct format: Renamed headers and fixed formatting.
- Add workflow for checking doc build: Implemented workflow for checking documentation build.
- Update doxygen-check-build.yml: Updated doxygen-check-build.yml.
- Create doxygen-deploy.yml: Created doxygen-deploy.yml.
- Create workflow for testing prerelease builds: Set up workflow for testing prerelease builds.
- Rename to smacc2 and smacc2_msgs: Renamed to smacc2 and smacc2_msgs.
- Correct GitHub branch reference: Fixed GitHub branch reference.
- Update name of package and package.xml: Updated package name and package.xml.
- Execute on master update: Executed update on master branch.
- Reset all versions to 0.0.0: Reset all versions to 0.0.0.
- Ignore all packages except smacc2 and smacc2_msgs: Ignored all packages except smacc2 and smacc2_msgs.
- Update changelogs: Updated changelogs.
- Revert "Ignore all packages except smacc2 and smacc2_msgs": Reverted commit to ignore all packages except smacc2 and smacc2_msgs.
- Update description table: Updated description table.
- Update table: Updated table.
- Copy initial docs: Copied initial documentation.
- Dockerfile w/ ROS distro as argument: Added Dockerfile with ROS distro as argument.
- Opened new folder for additional tracing contents: Created folder for additional tracing contents.
- Delete tracing directory: Removed tracing directory.
- Moved tracing.md to tracing directory: Transferred tracing.md to tracing directory.
- Added setupTracing.sh: Included setupTracing.sh for configuring tracing group.
- Removed manual installation of ros-rolling-ros2trace: Automated installation in setupTracing.sh.
- Created alternative ManualTracing: Developed alternative ManualTracing.

Changed
-------
- ros2 launch sm_three_some sm_three_some: Changed launch command to include file extension.
- First ensure you have the necessary package installed: Added instructions for package installation.
- Use manual deployment for now: Implemented manual deployment temporarily.
- Use docs/ as source folder for documentation: Specified docs/ as source folder for documentation.
- Use docs/ as output directory: Set docs/ as output directory.
- Change extension of imports: Modified extension of imports.
- Update ci-build-source.yml: Updated ci-build-source.yml.
- Change extension: Altered file extension.
- This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61: Reverted a specific commit.
```

*pabloinigoblasco*

Section_9
----------

### Added
- New SM markdowns.
- Dockerfile for Rolling and Galactic.
- README tutorial for Dockerfile.
- New SM from sm_respira_1.
- Feature/core and navigation fixes.
- Base for the sm_aws_aarehouse navigation.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

### Changed
- Wording "smacc application" to "SMACC2 library".
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Update smacc2_rta command across readmes.

### Fixed
- Bug in smacc2 component.
- Reverted markdowns to HTML.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Clean up of sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Correct all linters and formatters.

### Removed
- Galactic builds from master and kept only rolling. Removed submodules and use .repos file.

### Miscellaneous
- Additional cleanup.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- More on performance and other issues.
- More changes on performance tests.
- More on navigation.
- Attempting pre-commit fixes.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_10
==========

Added
-----
- Introduce new feature `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for nav2, allowing waiting for nav2 nodes subscribing to the /bond topic and ensuring they are operational. Users can select specific nodes to wait for.

Changed
-------
- Enhance navigation parameters on sm_dance_bot.
- Improve formatting across various sections.
- Update gazebo settings to display the robot and lidar for sm_dance_bot visualizations.

Fixed
-----
- Resolve compile warnings (#96).
- Fix formatting issues in various parts of the codebase.

Removed
-------
- Eliminate neo_simulation2 package, adjusting source build packages for CI testing.

Other
-----
- Progress in AWS navigation demo.
- Merge and progress in development.
- Precommit cleanup and updates.
- Enable source build on PR for testing purposes.
- Various core improvements during navigation testing.
- Work on sm_multi_stage_1 functionality.
- Diverse enhancements in navigation and performance.
- Continuous development on sm_dance_bot and sm_aws_warehouse features.
- Collaboration with Ubuntu 20-04-02-amd64 (Brett) on multiple commits.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

# Section 11

## Added
- Feature/diverse improvements in navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122)
- First working version of sm template and template generator (#127)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)

## Changed
- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Minor navigation improvements (#141)

## Fixed
- Fix CI: format fix python version (#148)
- Fixing broken master build (#167)
- Fixing broken build in aws navigation (#174)

## Removed
- Remove merge markers from a python file (#119)
- Remove node creation and create only a logger (#149)
- Remove parameters smacc (#147)
- Remove test from main moveit cmake (#151)
- Remove sm_dance_bot_msgs (#144)

## Miscellaneous
- Minor linting and formatting improvements
- Minor format issues resolved
- Noticed launch command was incorrect in README.md, fixed
- Update package list (#142)
- Update READMEs with added SVGs
- Warehouse2 progress (#179)
- Format adjustments and merge operations
- Default values set

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, Denis Štogl <denis@stogl.de>, pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_12
==========

Added
-----
- Brettpac branch (#184)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185): warehouse2 changes
- Feature/sm warehouse 2 13 dec 2 (#186): format changes and headless mode
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188, #189): format changes, headless mode, and default values
- Pure spinning behavior missing files
- Feature/planner changes 16 12 (#191): minor changes and fixes
- Feature/replanning 16 dec (#193): replanning for all examples and fixes
- Several fixes (#194)
- Minor changes (#195)
- Feature/undo motion 20 12 (#196, #198): minor changes, replanning, improving undo motion navigation
- Tuning warehouse3 (#197)
- Feature/sync 21 12 (#199): minor changes, replanning, and format fixes
- Feature/warehouse2 22 12 (#200): minor changes, replanning, and format fixes
- Finishing warehouse2 (#201)
- Tuning and fixes (#202)
- Feature/minor tune (#203): tuning and fixes
- Fixing warehouse 3 problems (#204): removing deadlocks and improving core, added missing file from warehouse2
- Fix code generators (#221): resolved build issues, updated SM template, and improved example code visibility
- Feature/retry behavior warehouse 1 (#226): minor changes, replanning, backport to foxy, and format fixes
- Foxy backport (#206): formatting fixes, trailing spaces, codespell correction, linters warnings, CI build updates, dependency additions, and workflow improvements

Changed
-------
- Updated branching example
- Renamed header files and corrected format
- Added workflow for checking doc build
- Updated doxygen-check-build.yml
- Created doxygen-deploy.yml
- Created workflow for testing prerelease builds
- Renamed to smacc2 and smacc2_msgs
- Corrected GitHub branch reference
- Updated package name and package.xml
- Reset all versions to 0.0.0
- Ignored all packages except smacc2 and smacc2_msgs
- Updated changelogs
- Version 0.1.0

Removed
-------
- Removed use of node in the sm performance template
- Disabled ament_cpplint, cpplint, cppcheck linters, and some packages
- Ignored further packages
- Disabled disabled packages
- Removed some packages and updated workflows
- Disabled ament_lint_cmake
- Disabled some packages and updated workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Ignore further packages
- Disable disabled packages
- Ignore all packages except smacc2 and smacc2_msgs
``` 

*pabloinigoblasco*

```rst
Section_13
==========

Added
-----

- Added setupTracing.sh script to automate installation of necessary packages and configure tracing group.
- Added README tutorial for Dockerfile.
- Added new features:
  - SMACC2 library.
  - Dockerfile for Rolling and Galactic.
  - cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files.
- Added Navigation2 for semi-binary build.
- Added missing rolling repositories build.
- Added sm_multi_stage_1 feature.

Changed
-------

- Changed wording from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- Updated smacc2_rta command across readmes.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Reformatted sm_reference_library.
- Corrected trailing spaces.
- Edited tracing.md to reflect new tracing event names.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.

Fixed
-----

- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed formatting issues.
- Fixed pre-commit issues.
- Fixed several core improvements during navigation testing.
- Fixed sm_atomic_24hr format.
- Fixed sm_advanced_recovery_1 issues.
- Fixed sm_atomic_performance_test_a_2.
- Fixed sm_atomic_performance_test_c_1.
- Fixed sm_multi_stage_1 issues.
- Fixed wait topic message client behavior.

Removed
-------

- Removed manual installation of ros-rolling-ros2trace.
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Deleted tracing directory.
- Deleted tracing.md file.
- Deleted smacc2_sm_reference_library/sm_atomic/README.md file.
- Deleted sm_respira_1 format cleanup pre-commit.
- Deleted sm_respira_test_2.
- Deleted sm_atomic_performance_trace_1.
- Deleted sm_atomic_performance_test_a_1.
- Deleted sm_atomic_performance_test_a_2.
- Deleted sm_atomic_performance_test_c_1.
- Deleted sm_advanced_recovery_1 round 4.
- Deleted sm_advanced_recovery_1 reworked.
- Deleted sm_advanced_recovery_1 work.
- Deleted sm_multi_stage_1.
- Deleted sm_atomic_performance_test_c_1.
- Deleted sm_atomic_performance_test_a_1.
- Deleted sm_atomic_performance_test_a_2.
- Deleted sm_atomic_performance_trace_1.
- Deleted sm_atomic_24hr.
- Deleted sm_respira_1 format cleanup.
- Deleted sm_respira_test_2.
```

```rst
Section_14
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection.
- Base for the `sm_aws_aarehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_pause_slam`.
- `sm_dance_bot_lite` gazebo fixes.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.

Changed
-------
- Corrected all linters and formatters.
- Minor format improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Precommit cleanup run.
- Hotfix for `doxygen` deployment workflow.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- Cleaning and lidar show/hide option in `sm_dance_bot`.
- Gazebo fixes to show the robot and the lidar.
- Doubling in `sm_multi_stage_1`.

Fixed
----
- Removed some compile warnings.

Removed
-------
- Unused code.

Contributors
------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_15
==========

Added
-----

- Added multistage modes and sequences for sm_multi_stage_1 (#172)
- Added AWS navigation for sm_dance_bot (#174)
- Added QOS durability to SmaccPublisherClient (#163)

Changed
-------

- Renamed sm_advanced_recovery_1 (#171)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Updated package list (#142)
- Updated READMEs with SVGs for atomic, dance_bot, and others (#140)
- Refactored sm dance bot strikes back (#152)
- Improved Docker environment for local testing (#154)
- Updated README (#164)
- Improved navigation and performance (#116)
- Reworked sm_multi_stage_1 for better sequences (#172)
- Updated source build packages for CI testing
- Adjusted build packages for source CI
- Improved waypoint navigation for course completion (#155)
- Added reliability QOS config to SmaccPublisherClient
- Updated format for moveit migration testing (#151)
- Added .reps dependencies and fixed build errors
- Added dependency to ur5 client
- Updated format for move_it PR
- Improved Dockerfile for local tests
- Fixed compiling issues

Fixed
-----

- Removed neo_simulation2 package (#112)
- Fixed formatting issues
- Removed merge markers from a Python file (#119)
- Fixed CI format for Python version (#148)
- Fixed launch command in README.md
- Fixed broken master build

Removed
-------

- Removed neo_simulation2 package

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

Section_16
==========

Added
-----
- Update dependencies for husky in rolling and galactic.
- Progress on AWS navigation and refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress.
- Waypoint Inputs.
- Warehouse2 progress.
- Format.
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
- Tuning and fixes.
- Feature/minor tune.
- Fixing warehouse 3 problems and other core improvements.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in SMACC core.
- Autoware demo.
- Retry behavior warehouse 1.
- Update file for fake hardware simulation and add file for gazebo simulation.
- Add ignition file and update repos files.
- Foxy backport.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licenses.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Branching example.
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.

Changed
-------
- Fix formatting.
- Minor changes.
- Merge.
- Headless and other fixes.
- Default values.
- Format issues.
- Minor tune.
- Format.
- More changes and headless.
- Replanning for all our examples.
- Tuning and fixes.
- Fix broken source build.
- Only rolling version should be pre-released on master.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.
- Minor formatting fixes.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Disable disabled packages.
- Change extension.

Removed
-------
- Fixing broken build.
- Pure spinning behavior missing files.
- Missing SM.
- Minor broken build.
- Some reordering fixes.
- Docker files for different revisions, warnings removal, and more testing on navigation.
- Fixing docker for foxy and galactic.
- Minor format fix.
- Other minor changes.

Co-Authored-By
--------------
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- Reelrbtx <brett2@reelrobotics.com>
- Brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- Pablo Iñigo Blasco <pabloinigoblasco@ibrobotics.com>.

Section 17
===========

Added:
------
- First ensure you have the necessary package installed by running:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```

Changed:
---------
- Corrected formatting of python files.
- Edited Threesome launch command from:
  ```
  ros2 launch sm_three_some sm_three_some
  ```
  to:
  ```
  ros2 launch sm_three_some sm_three_some.launch
  ```
- Renamed header files and corrected format.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Renamed to smacc2 and smacc2_msgs.
- Corrected GitHub branch reference.
- Updated package name and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" commit.
- Updated description table.
- Updated table.
- Copied initial docs.
- Dockerfile now accepts ROS distro as an argument using:
  ```
  sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/
  ```
- Opened new folder for additional tracing contents.
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Added README tutorial for Dockerfile.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling, removing submodules and using .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Format cleanup for sm_respira_1.
- Format cleanup for sm_respira_1 pre-commit.
- Added sm_respira_test_2.
- More changes on performance tests.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Reformatted sm_reference_library.
- Corrected trailing spaces.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More Readme Updates.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Base for the sm_aws_aarehouse navigation.
- Progressing in AWS navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- More format improvements.
- More on navigation.
- Reworked sm_advanced_recovery_1.
- Fix pre-commit for sm_advanced_recovery_1.
- More work on sm_advanced_recovery_1.
- Round 4 of sm_advanced_recovery_1.
- Brettpac branch.
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1.
- Added sm_atomic_performance_test_c_1.
- Modified sm_atomic_performance_test_a_2.
- Added sm_multi_stage_1.
- Fixing precommit for sm_multi_stage_1.
- Update README.md with updated launch command.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>, Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```rst
Section_18
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success (#81)
- New feature: cb_wait_nav2_nodes, a client behavior for nav2 that subscribes to the /bond topic and waits for nodes to be alive, with optional node selection (#82)
- New feature: cb_pause_slam, a client behavior for pausing SLAM operations (#98)
- New feature: sm_dance_bot_lite, a lightweight version of sm_dance_bot (#99)
- New feature: sm_dance_bot_visualizing_turtlebot3, visualizes TurtleBot3 movements (#101)
- New feature: dance_bot_launch_gz_lidar_choice, adds lidar show/hide option to sm_dance_bot (#102)
- New feature: sm_dance_bot_lite_gazebo_fixes, fixes gazebo visualization for sm_dance_bot_lite (#104)

Changed
-------
- Corrected all linters and formatters for code quality (#81)
- Fixed navigation parameters on sm_dance_bot (#93, #95)
- Merged and progressed codebase (#94)
- Hotfix for minor issues (#100)

Removed
-------
- Removed some compile warnings (#96)
``` 

*pabloinigoblasco*

```rst
Section_19
==========

Added
-----
- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Visualizing turtlebot3
  - Cleaning and lidar show/hide option
  - Format fixes
  - Gazebo fixes to show the robot and the lidar
- Precommit cleanup run (#106)
- AWS demo (#108)
- Brettpac branch (#110)
  - Gaining traction sm_multi_stage_1
  - More stages
- A3 (#113)
- More sm_multi_stage_1 (#114)
- MM (#115)
- Diverse improvements navigation and performance (#116)
- Feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting
  - Remove merge markers from a python file (#119)
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Move method after the method it calls to prevent recursion (#126)
- First working version of sm template and template generator (#127)
- Minor tweaks (#130)
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
  - Some more progress on markers cleanup
- Minor format issues (#134)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140)
  - Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Removing parameters smacc (#147)
  - Workflows update
- Fix CI: format fix python version (#148)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Add QOS durability to SmaccPublisherClient (#163)
  - Add reliability QoS config

Changed
-------
- Progressing in the moveit migration testing (#151)
  - Initial migration to smacc2
  - Fixing errors introduced on formatting
  - Fixing linting warnings
  - Updating format
  - Adding .reps dependencies and fixing build errors
  - Adding dependency to ur5 client
  - Improving dockerfile for building local tests
  - Fixing compiling issues

Removed
-------
- Remove neo_simulation2 package (#112)
  - Correct formatting
  - Enable source build on PR for testing
  - Adjust build packages of source CI

Fixed
-----
- Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed typo
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor
  - Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
```

```rst
Section_20
==========

Added
-----
- Added more testing on MoveIt.
- Added progress on MoveIt behaviors.
- Added minor configuration changes.
- Added sm_pubsub_1 (#169).
- Added sm_pubsub_1 part 2 (#170).
- Added sm_advanced_recovery_1 renaming (#171).
- Added sm_multi_stage_1 reworking (#172).
- Added multistage modes.
- Added sm_multi_stage sequences.
- Added sm_multi_state_1 steps.
- Added sm_multi_stage_1 sequence d.
- Added sm_multi_stage_1 c sequence.
- Added mode_5_sequence_b.
- Added mode_4_sequence_b.
- Added sm_multi_stage_1 most.
- Added finishing touches 1.
- Added readme.
- Added Feature/aws navigation sm dance bot (#174).
- Added repo dependency.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added more on AWS demo.
- Added warehouse2 (#177).
- Added Waypoint Inputs (#178).
- Added wharehouse2 progress (#179).
- Added format (#180).
- Added sm_dance_bot_warehouse_3 (#181).
- Added Feature/sm warehouse 2 13 dec 2 (#182).
- Added more changes and headless.
- Added merge.
- Added headless and other fixes.
- Added default values.
- Added Brettpac branch (#184).
- Added Redoing sm_dance_bot_warehouse_3 waypoints.
- Added More Waypoints.
- Added SrConditional fixes and formatting (#168).
- Added fix: some formatting and templating on SrConditional.
- Added fix: move trigger logic into headers.
- Added fix: lint.
- Added Feature/wharehouse2 dec 14 (#185).
- Added warehouse2.
- Added Feature/sm warehouse 2 13 dec 2 (#186).
- Added finetuning waypoints (#187).
- Added Feature/cb pure spinning (#188).
- Added Feature/cb pure spinning (#189).
- Added pure spinning behavior missing files.
- Added minor changes (#190).
- Added Feature/planner changes 16 12 (#191).
- Added more fixes.
- Added Feature/replanning 16 dec (#193).
- Added replanning for all our examples.
- Added several fixes (#194).
- Added minor changes (#195).
- Added Feature/undo motion 20 12 (#196).
- Added improving undo motion navigation warehouse2.
- Added tuning warehouse3 (#197).
- Added Feature/undo motion 20 12 (#198).
- Added undo tuning and errors.
- Added format.
- Added Feature/sync 21 12 (#199).
- Added format issues.
- Added Feature/warehouse2 22 12 (#200).
- Added format issues.
- Added finishing warehouse2.
- Added Feature/warehouse2 23 12 (#201).
- Added tuning and fixes.
- Added Feature/minor tune (#203).
- Added fixing warehouse 3 problems, and other core improvements (#204).
- Added fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green.
- Added weird MoveIt not downloaded repo.
- Added added missing file from warehouse2 (#205).
- Added backport to foxy.
- Added minor format.
- Added minor linking errors foxy.
- Added updating subscriber publisher components.
- Added progress in autoware machine.
- Added refining cp subscriber cp publisher.
- Added improvements in smacc core adding more components mostly developed for autoware demo.
- Added autoware demo.
- Added foxy ci.
- Added fix.
- Added some reordering fixes.
- Added docker files for different revisions, warnings removal, and more testing on navigation.
- Added fixing docker for foxy and galactic.
- Added docker build files for all versions.
- Added barrel demo.
- Added barrel search build fix and warehouse3.
- Added fixing startup problems in warehouse 3.
- Added fixing format and minor.
- Added progress in barrel husky.
- Added barrel demo.
- Added progress.
- Added fixing broken build.
- Added Feature/barrel - do not merge yet (#233).
- Added backport to foxy.
- Added minor formatting fixes.

Changed
-------
- Changed progress on AWS navigation and some other refactorings on navigation clients and behaviors.

Fixed
-----
- Fixed pipeline error.
- Fixed broken master build.
- Fixed broken build.
- Fixed warehouse 3 problems, and other core improvements to remove dead lock.
- Fixed missing sm.
- Fixed minor broken build.
- Fixed some reordering fixes.
- Fixed fixing broken build.
```

```rst
Section_21
==========

Added
-----
- Add galactic CI build due to Navigation2 issues in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency for finding includes.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Use manual deployment temporarily.
- Use docs/ as source folder and output directory for documentation.
- Added setupTracing.sh for configuring tracing group.
- Added a dockerfile for Rolling and Galactic.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint linters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Enable cppcheck.
- Enable cppcheck and disable cpplint

```rst
Section_22
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection.
- New client behavior: cb pause slam for pausing SLAM operations.
- New client behavior: cb pause slam for pausing SLAM operations.

Changed
-------

- Updated launch command in README.md.
- Corrected all linters and formatters.

Fixed
-----

- Resolved compile warnings.

Removed
-------

- Removed some compile warnings.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_23
==========

Added
-----
- Rename doxygen deployment workflow (#100)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- aws demo (#108)
- Brettpac branch (#110, #111)
- a3 (#113)
- diverse improvements navigation and performance (#116)
- Feature/diverse improvements navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122)
- Feature/dance bot s pattern (#128, #129)
- First working version of sm template and template generator. (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)

Changed
-------
- Move method after the method it calls. Otherwise recursion could happen. (#126)
- Update package list. (#142)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)

Fixed
-----
- Remove neo_simulation2 package. (#112)
- minor hotfix
- gazebo fixes, to show the robot and the lidar
- format fixes
- gazebo fixes for sm_dance_bot_strikes_back
- precommit cleanup run (#106)
- got sm_multi_stage_1 working (barely) (#109)
- minor format
- minor tweaks (#130)
- minor navigation improvements (#141)
- using local action msgs
- removing sm_dance_bot_msgs
- pending references
- removing parameters smacc
- workflows update
- Noticed launch command was incorrect in README.md
- fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
- formatting
- minor

Removed
-------
- Remove neo_simulation2 package.
- removing parameters smacc
- removing test from main moveit cmake
- test ur5
- repos dependency
- adding dependency to ur5 client
- docker refactoring
- minor dockerfile test workaround

Contributors
------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pablo@ibrobotics.com>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_24
==========

Added
-----
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
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
- Refactor: remove line
- Feat: add reliability qos config
- Finetuning waypoints (#187)
- Tuning and fixes (#202)
- Minor tune
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green

Fixed
-----
- Fix: add a missing colon
- Fix: some formatting and templating on SrConditional
- Fix: move trigger logic into headers
- Fix: lint
- Several fixes (#194)

Removed
-------
- Minor broken build
- Some reordering fixes
- Warnings removal

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: reelrbtx <brett2@reelrobotics.com>
- Co-authored-by: brettpac <brett@robosoft.ai>
- Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
```

```rst
Section_25
==========

Added
-----
- Feature/docker improvements march 2022 (#235)
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Use manual deployment for now.
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing
- Added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Changed wording "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Fixed
-----
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Correct formatters.
- Correct formatting of python file.
- Enable cppcheck
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library
- Minor formatting

Removed
-------
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
- Deleted tracing directory

Other Changes
-------------
- Progress in husky demo
- Improving navigation behaviors
- Branching example
- Minor changes
- Replanning for all our examples
- Backport to foxy
- Foxy backport (#206)
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licences.
- Disable cpplint and cppcheck linters.
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Included necessary package and edited Threesome launch
- Execute on master update
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- Moved tracing.md to tracing directory
- Added smacc2_performance_tools
- Performance tests improvements
- More on performance and other issues
- SM respira 1 format cleanup
- SM respira test 2
- Do not execute clang-format on smacc2_sm_reference_library package
- SM reference library reformatting
- SM atomic 24hr
- SM atomic performance trace 1
- Update smacc2_rta command across readmes
- Clean up of SM atomic 24hr
- More SM atomic 24hr cleanup
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Some progress on navigation rolling
- Renamed folders, deleted tracing.md, edited README.md
- Bug in SMACC2 component
- Reverted markdowns to html
- Added README tutorial for Dockerfile
- Additional cleanup
- Cleanup
- Edited tracing.md to reflect new tracing event names
- Enable galactic CI setup and rename rolling files. (#58)
- Fix source CI and correct README overview. (#62)
- Changed launch command to ros2 launch SM respira 1 sm respira 1.launch (#69)
  also noticed a note I had made while producing these that was not removed
- Update doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)
- Created new SM from SM respira 1 (#76)
- Feature/core and navigation fixes (#78)
- Base for the SM AWS warehouse navigation
- Progressing in AWS navigation
- Several core improvements during navigation testing
- Formatting improvements

Co-authored-by
--------------
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_26
==========

Added
-----
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait

Changed
-------
- Navigation parameters fixes on sm_dance_bot
- Merge and progress
- Fix format
- Correct all linters and formaters

Fixed
----
- Fix pre-commit
- Trying to fix Pre-Commit
- Modifying sm_atomic_performance_test_a_2
- Remove some compile warnings

Removed
-------
- Several core improvements during navigation testing

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
``` 

*pabloinigoblasco*

```rst
Section_27
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting `nav2` nodes subscribing to the `/bond` topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- Progress in testing `sm_dance_bot` introducing `slam` pausing/resuming functionality.
- Polishing `sm_dance_bot` and `s-pattern`.
- Minor tuning to mitigate overshot issue cases in `waypoints` navigator.
- Minor navigation improvements.
- Using local action messages.
- Navigation 2 stack renaming.

Fixed
-----
- Move method after the method it calls to prevent recursion.
- Fix CI: format fix python version.

Removed
-------
- Removed `neo_simulation2` package.
- Removed parameters in `smacc`.

Other
-----
- Several core improvements during navigation testing.
- Formatting improvements.
- Precommit cleanup.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others.
- Noticed launch command was incorrect in `README.md`, fixed launch command for `sm_dance_bot_strikes_back`, and removed some comments.
- Workflow updates.
- Pending references.
```

```rst
Section_28
==========

Added
-----

- Rolling Docker environment to be executed from any environment (#154)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing errors introduced on formatting
  - Adding missing dependency
  - Fixing linting warnings
  - Removing test from main moveit cmake
  - Progressing in the moveit migration testing
  - Updating format
  - Adding .reps dependencies and fixing build errors
  - Adding dependency to ur5 client
- Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and refactorings on navigation clients and behaviors
  - More on aws demo
- Feature/warehouse2 22 12 (#200)
  - Replanning for all examples
  - Fixing format issues
  - Finishing warehouse2

Changed
-------

- Moved reference library SMs to smacc2_performance_tools (#166)
- Add QOS durability to SmaccPublisherClient (#163)
  - Added QOS durability to SmaccPublisherClient

Fixed
-----

- Update readme (#164)
  - More readme updates
- SrConditional fixes and formatting (#168)
  - Fixing formatting and templating on SrConditional
  - Moving trigger logic into headers
  - Lint fixes
- Several fixes (#194)

Removed
-------

- Pure spinning behavior missing files (#189)

Co-Authored-By
--------------

- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
```

```rst
Section_29
==========

Added
-----
- Use correct upstream .repos files for source builds (#243)
- Correct mergify branch names (#246)
- Correct name of source-build job and bump version of action (#242) (#247)
- Update galactic source build job name (#250)
- Galactic source build: update .repos file, bump action version and use correct version of upstream packages (backport #241) (#248)
- restoring workflow files (#252)
- restoring files (#253)
- Fix checkout branches for scheduled builds (#254)
- Update foxy-source-build.yml
- Feature/fixing husky build rolling (#257)
- Feature/fixing husky build rolling (#258)
- Update README.md (#262)
- Feature/fixing ur demos (#261)
- Feature/fixing type string walker (#263)
- Update README.md (#266)
- Update README.md (#267)
- Update README.md (#268)
- Significant update in Getting Started Instructions (#269)
- Fix urls to index.ros.org (#284)
- Fix foxy source build config to use repos file from foxy branch. (#285)
- urdf for ur to support namespaces
- Ignore packages which should not be released.
- Added changelogs.
- 0.4.0
- Revert "Ignore packages which should not be released."
- FakeSystem is working.
- Update CI setup (#305)
- Feature/testing smacc rta inter context routes (#306)
- Packml example (#300)
- initial commit of packml
- progressing substate for start and execute
- completing state
- finishing state machine
- fixing break in packml

Changed
-------
- fixing rolling build (#239)
- fixing to focal by the moment
- fixing building issue
- fixing broken build
- build fix
- fixing ur demo (#273)
- fix: initialise conditionFlag as false (#274)
- progress on the multi arm moveit
- more on multi-ur
- testing ur5 2
- progressing, two arms with roscontrol and moveit
- precommit fix (#280)
- more progress on fake controllers
- ros2 control gazebo repo dependency

Fixed
-----
- minor broken build
- some reordering fixes
- fixing docker for foxy and galactic
- fixing startup problems in warehouse 3
- fixing format and minor
- fixing husky project build on rolling
- fixing type string walker threesome demo
- removing other distribution files
- more formatting fix
- more progress
- more progress on fake controllers
- more merge
- more fixing rolling build
- more fixes
- more progress
- more progress in barrel husky
- more progress in husky demo
- more progress in husky demo
- more progress
- more progress
- more formatting fix
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress
- more progress

Removed
-------
- missing
- missing sm
- missing
- missing
- missing
- missing repo
- missing deps
- missing repo
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing
- missing

```rst
Section_30
==========

Author: Pablo Iñigo Blasco <pablo@ibrobotics.com>
Date:   Fri May 27 00:09:15 2022 +0200

Version 0.1.0
-------------

Added
-----

- Packml example with state machine progression and completion.
- Squashed commit for packml fixes and merging from galactic.

Changed
-------

- Renamed tracing events and folders.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Progress on navigation rolling.

Fixed
----

- Build issues on galactic.
- FakeSystem functionality.
- Bug in smacc2 component.

Removed
-------

- Manual installation steps for ros-rolling-ros2trace.
- Tracing.md file.

Author: brettpac <brettpac@users.noreply.github.com>
Date:   Thu May 19 00:05:46 2022 -0700

Version 0.1.0
-------------

Changed
-------

- Update README.md.

Author: Denis Štogl <denis@stogl.de>
Date:   Wed May 18 23:57:44 2022 +0200

Version 0.1.0
-------------

Changed
-------

- Update CI setup.

Author: Denis Štogl <denis@stogl.de>
Date:   Mon May 16 15:08:36 2022 +0200

Version 0.1.0
-------------

Fixed
----

- CI adjustments due to ros2_control API changes.
```

```rst
Section_31
==========

Added
-----
- Added galactic CI setup and renamed rolling files. (#58)
- Added more Readme Updates (#72), (#74)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
- Added navigation parameters fixes on sm_dance_bot

Changed
-------
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Changed event generator library name
- Updated smacc2_rta command across readmes
- Updated c_cpp_properties.json
- Updated README.md launch command
- Updated doxygen links

Fixed
----
- Fixed source CI and corrected README overview. (#62)
- Fixed trailing spaces
- Corrected all linters and formaters

Removed
-------
- Do not execute clang-format on smacc2_sm_reference_library package

Other
-----
- Optimized dependencies in move_base_z_planners_common
- Cleaned up sm_respira_1 format
- Cleaned up sm_respira_1 format pre-commit
- Cleaned up sm_reference_library format
- Cleaned up sm_atomic_24hr
- Cleaned up sm_atomic_performance_trace_1
- Cleaned up sm_atomic_24hr
- Cleaned up sm_advanced_recovery_1
- Cleaned up sm_atomic_performance_test_a_2
- Cleaned up sm_atomic_performance_test_a_1
- Cleaned up sm_atomic_performance_test_c_1
- Cleaned up sm_multi_stage_1
- Progressed in aws navigation
- Made several core improvements during navigation testing
- Made progress in aws navigation demo
- Made minor formatting improvements
- Attempted pre-commit fixes
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit
- Fixed pre-commit

```rst
Section_32
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- Gazebo fixes to show the robot and lidar.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.
- Source build enabled on PR for testing.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components. Introduces `smacc2::deep_history` syntax.
- First working version of `sm` template and template generator.
- Minor tweaks.

Changed
-------
- Formatting improvements throughout.
- Minor format adjustments.
- Navigation parameters fixes on `sm_dance_bot`.
- Cleaning and lidar show/hide option in `sm_dance_bot`.
- Polishing `sm_dance_bot` and `s-pattern`.
- Refinement in `sm_dance_bot`.
- Corrected "Finnaly" to "Finally".

Fixed
-----
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.

Removed
-------
- `neo_simulation2` package.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

Section_33
==========

Added
-----
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
- Progress in the sm_dance_bot tests (#135)
- Feature/nav2z renaming (#144)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)

Changed
-------
- Minor tuning to mitigate overshot issue cases
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Resolve compile warnings (#137)
- Using local action msgs
- Removing sm_dance_bot_msgs
- Format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Initial migration to smacc2
- Fixing some errors introduced on formatting
- Missing dependency
- Fixing some more linting warnings
- Removing test from main moveit cmake
- Test ur5
- Progressing in the moveit migration testing
- Updating format
- Adding .reps dependencies and also fixing some build errors
- Repos dependency
- Adding dependency to ur5 client
- Docker refactoring
- Progress on move_it PR
- Minor dockerfile test workaround
- Improving dockerfile for building local tests
- Fixing compiling issues
- Moved reference library SMs to smacc2_performance_tools (#166)
- Pre-commit cleanup
- Finetuning waypoints (#187)
- Several fixes (#194)

Fixed
-----
- Noticed launch command was incorrect in README.md
- Fix CI: format fix python version (#148)
- Fixing pipeline error
- Fixing broken master build
- Fixing broken build

Removed
-------
- Removing parameters smacc (#147)
- Removing parameters smacc
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Removing parameters smacc

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_34
==========

Added
-----
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Added missing file from warehouse2 (#205)
- Added galactic CI build because Navigation2 is broken in rolling
- Added partial changes for ament_cpplint
- Added tf2_ros as dependency to find include
- Added workflow for checking doc build
- Created doxygen-deploy.yml
- Created workflow for testing prerelease builds
- Use manual deployment for now
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Reset all versions to 0.0.0
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Added setupTracing.sh
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling
- Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Added smacc2_performance_tools
- Added README tutorial for Dockerfile
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling
- Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Added smacc2_performance_tools
- Added README tutorial for Dockerfile

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- Changed wording "smacc application" to "SMACC2 library"
- Updated ci-build-source.yml
- Updated doxygen-check-build.yml
- Updated smacc2_rta command across readmes
- Cleaned up sm_atomic_24hr
- Optimized deps in move_base_z_planners_common
- Renamed event generator library
- Minor formatting changes
- Renamed folders, deleted tracing.md, edited README.md
- More on performance and other issues
- More changes on performance tests
- Updated doxygen links
- Updated c_cpp_properties.json
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Corrected README overview

Fixed
-----
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Fixed warehouse 3 problems and other core improvements to remove dead lock, also making continuous integration green
- Fixed bug in smacc2 component

Removed
-------
- Deleted tracing directory
- Removed manual installation of ros-rolling-ros2trace
- Disabled ament_cpplint
- Disabled some packages and update workflows
- Bumped ccache version
- Ignored further packages
- Satisfied ament_lint_cmake
- Added missing licenses
- Disabled cpplint and cppcheck linters
- Corrected formatters
- Disabled disabled packages
- Corrected formatting of python file
- Included necessary package and edited Threesome launch
- Do not execute clang-format on smacc2_sm_reference_library package
- Removed tracing events after
- Reverted markdowns to html
- Deleted tracing.md
- Cleanup

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_35
==========

Added
-----

- More Readme (#74)
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Sm_multi_stage_1 (#90)
- Update README.md
- Wait topic message client behavior (#81)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Changed
-------

- Several core improvements during navigation testing
- Progress in AWS navigation
- Formatting improvements
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait
- Corrected all linters and formatters
- Navigation parameters fixes on sm_dance_bot
- Merge and progress
- Fix format

Removed
-------

- Minor format
```

*pabloinigoblasco*

```rst
Section_36
==========

Added
-----
- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are alive. Optional node selection available.
- New feature: cb_wait_topic_message. Asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Gazebo fixes to show the robot and lidar.

Fixed
----
- Removed some compile warnings (#96).
- Minor hotfix.
- Minor navigation improvements.
- Minor tuning to mitigate overshot issue cases.
- Minor format issues.

Removed
-------
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs.
- Removed parameters smacc.

Other
-----
- Several core improvements during navigation testing.
- Base for the sm_aws_warehouse navigation.
- Progress in AWS navigation.
- Formatting improvements.
- More on navigation.
- Precommit cleanup run.
- Updates yaml.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Remove merge markers from a python file.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Update package list.
``` 

*pabloinigoblasco*

```rst
Section_37
==========

Added
-----

- Add SM Atomic SM generator. (#143)
- Add SM Atomic SM generator to create only a logger. (#149)
- Add QOS durability to SmaccPublisherClient (#163)
- Add dependencies for husky simulation in AWS navigation. (#174)
- Add warehouse2 progress. (#179)
- Add Waypoint Inputs. (#178)
- Add SrConditional fixes and formatting. (#168)
- Add missing file from warehouse2. (#205)

Changed
-------

- Update launch command in README.md for sm_dance_bot_strikes_back. (#148)
- Refactor SM dance bot strikes back. (#152)
- Move reference library SMs to smacc2_performance_tools. (#166)
- Rework sm_multi_stage_1 with multistage modes and sequences. (#172)
- Redo sm_dance_bot_warehouse_3 waypoints. (#184)
- Finetune waypoints. (#187)
- Improve undo motion navigation in warehouse2. (#198)
- Tune and fix warehouse3 problems. (#204)

Fixed
-----

- Fix launch command for sm_dance_bot_strikes_back and remove past comments. (#148)
- Fix CI formatting for Python version. (#148)
- Fix node creation to create only a logger. (#149)
- Fix compiling issues. (#164)
- Fix broken master build. (#174)
- Fix pipeline error. (#174)
- Fix formatting in husky dependencies. (#174)
- Fix broken build in AWS navigation. (#174)
- Fix some formatting and templating on SrConditional. (#168)
- Fix move trigger logic into headers. (#168)
- Fix linting issues. (#168)
- Fix pure spinning behavior missing files. (#189)
- Fix several issues in replanning examples. (#194)
- Fix errors in undo motion navigation. (#198)
- Fix format issues in sync. (#199)
- Fix format issues in warehouse2. (#200)
- Fix tuning and fixes in warehouse2. (#202)
- Fix minor tune issues. (#203)
- Fix warehouse3 problems and core improvements to remove deadlock. (#204)
- Fix weird moveit not downloaded repo. (#204)

Removed
-------

- Remove test from main moveit CMake. (#164)
- Remove some comments from past in launch command for sm_dance_bot_strikes_back. (#148)
``` 

*pabloinigoblasco*

```rst
Section_38
==========

Added
-----
- Add mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Dockerfile w/ ROS distro as argument
- First ensure you have the necessary package installed.
- Include necessary package and edit Threesome launch
- Open new folder for additional tracing contents
- Rename header files and correct format.
- Rename to smacc2 and smacc2_msgs
- Try fixing CI for rolling. (#209)

Changed
-------
- Change extension of imports.
- Change wording "smacc application" to "SMACC2 library"
- Update name of package and package.xml to pass liter.
- Update description table.
- Update table
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Update smacc2_rta command across readmes

Fixed
-----
- Fix rolling builds (#222)
- Fix trailing spaces.
- Minor broken build

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages
- Ignore further packages
- Ignore all packages except smacc2 and smacc2_msgs
- Remove example things from Foxy CI setup. (#214)
- Remove manual installation of ros-rolling-ros2trace
- Revert "Ignore all packages except smacc2 and smacc2_msgs"

Other
-----
- Branching example
- Cleanup
- Correct codespell.
- Correct formatters.
- Correct formatting of python file.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Enable build of missing rolling repositories.
- Enable cppcheck
- Enable Navigation2 for semi-binary build.
- More changes on performance tests
- More on performance and other issues
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Reactivating smacc2 nav clients for rolling via submodules
- Refining cp subscriber cp publisher
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Replanning for all our examples
- Reset all versions to 0.0.0
- Satisfy ament_lint_cmake
- Some progress on navigation rolling
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Updating subscriber publisher components
- Updated tracing.md to reflect new tracing event names
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components
- Updating subscriber publisher components

```rst
Section_39
==========

Added
-----
- Add galactic CI setup and rename rolling files. (#58)
- More Readme Updates (#72)
- More Readme (#74)
- created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Wait topic message client behavior (#81)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)

Changed
-------
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library
- changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- update doxygen links (#70)
- Update README.md
- navigation parameters fixes on sm_dance_bot

Fixed
----
- Fix source CI and correct README overview. (#62)
- sm_atomic_performance_test_c_1 (#88)
- modifying sm_atomic_performance_test_a_2 (#89)
- fixing precommit
- Correct all linters and formaters.

Removed
-------
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- minor formatting
- several core improvements during navigation testing
- formatting improvements
- more on navigation
- format improvements
- progress in aws navigation demo
- base for the sm_aws_aarehouse navigation
- progressing in aws navigation
- minor

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_40
==========

Added
-----

- New client behavior for nav2: Now nodes subscribe to the /bond topic and wait until they are alive. You can optionally select the nodes to wait for.
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` gazebo fixes.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS navigation demo progress.
- Progress in AWS navigation.
- Base for the `sm_aws_warehouse` navigation.
- `sm_multi_stage_1` doubling.
- `smacc2::deep_history` syntax introduced.
- `dance_bot_s_pattern` improvements.
- First working version of `sm` template and template generator.
- Minor tuning to mitigate overshot issue cases in the waypoints navigator.
- Progress in the `sm_dance_bot` tests.
- More progress on markers cleanup.

Changed
-------

- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Cleaning and lidar show/hide option in `sm_dance_bot`.
- Polishing `sm_dance_bot` and `s-pattern`.
- Fixing format issues.

Fixed
-----

- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Removed merge markers from a Python file.
- Fixed recursion possibility by moving a method after the method it calls.
- Fixed minor format issues.

Removed
-------

- `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_41
==========

Added
-----

- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)

Changed
-------

- Renamed Feature/nav2z to navigation 2 stack (#144)
- Updated package list (#142)
- Refactored Feature/sm dance bot strikes back (#152)
- Refactored Feature/migration moveit client (#151)
- Refactored Feature/aws navigation sm dance bot (#174)
- Refactored Feature/wharehouse2 dec 14 (#185)
- Refactored Feature/cb pure spinning (#188)
- Refactored Feature/planner changes 16 12 (#191)
- Refactored Feature/replanning 16 dec (#193)
- Refactored Feature/undo motion 20 12 (#196)
- Refactored Feature/sync 21 12 (#199)

Fixed
-----

- Resolved compile warnings (#137)
- Fixed launch command in README.md (#148)
- Fixed CI format for Python version (#148)
- Fixed node creation in SM Atomic SM generator (#149)
- Fixed compiling issues in moveit migration (#164)
- Fixed broken master build in moveit testing (#167)
- Fixed pipeline error in moveit testing (#167)
- Fixed broken build in aws navigation (#174)
- Fixed formatting in warehouse2 (#180)
- Fixed formatting and templating in SrConditional (#168)
- Fixed several errors in undo motion navigation (#198)

Removed
-------

- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed test from main moveit CMake
- Removed unnecessary comments in launch command

Other
-----

- Co-authored changes with Brett <brett@robosoft.ai>, DecDury <declandury@gmail.com>, and Denis Štogl <destogl@users.noreply.github.com>
- Pre-commit cleanup in various features
- Updated Docker environment for better execution
- Moved reference library SMs to smacc2_performance_tools (#166)
- Progressed in moveit migration testing
- Added dependencies and fixed build errors in moveit migration
- Improved Dockerfile for building local tests
- Tuned waypoints in various features
```

Section 42
-----------

Added:
------
- Feature/warehouse2 22 12 (#200): Added missing file from warehouse2.
- Feature/warehouse2 23 12 (#201): Tuning and fixes.
- Feature/minor tune (#203): Minor tune and fixing warehouse 3 problems, along with other core improvements.
- dockerfiles (#225): Added docker files for different revisions, warnings removal, and more testing on navigation.
- Fix code generators (#221): Fixed other build issues, updated SM template, and made example code clearly visible.
- Feature/retry behavior warehouse 1 (#226): Foxy backport with minor formatting fixes, trailing spaces correction, codespell correction, python linters warnings correction, addition of galactic CI build, partial changes for ament_cpplint, addition of tf2_ros as dependency, and various workflow updates.
- Update sm_name.hpp: Updated templated to use Blackboard storage, resolve the global data correctly, and update sm_name.hpp.
- Update SM template and make example code clearly visible: Removed use of node in the sm performance template.
- Update template to resolve the global data correctly: Updated template to use Blackboard storage.
- Update ci-build-source.yml: Updated ci-build-source.yml.
- Update doxygen-check-build.yml: Created doxygen-check-build.yml.
- Create doxygen-deploy.yml: Created doxygen-deploy.yml.
- Create workflow for testing prerelease builds: Created workflow for testing prerelease builds.
- Update description table: Updated description table.
- Update table: Updated table.
- Copy initial docs: Copied initial docs.
- Dockerfile w/ ROS distro as argument: Added Dockerfile with ROS distro as argument.
- Opened new folder for additional tracing contents: Opened new folder for additional tracing contents.
- added setupTracing.sh: Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing: Created alternative ManualTracing.
- added new sm markdowns: Added new sm markdowns.
- added a dockerfile for Rolling and Galactic: Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh: Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md: Updated tracing/ManualTracing.md.
- Update smacc_sm_reference_library/sm_atomic/README.md: Updated smacc_sm_reference_library/sm_atomic/README.md.

Changed:
--------
- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch".
- Changed wording "smacc application" to "SMACC2 library".

Fixed:
------
- Fixing docker for foxy and galactic: Fixed docker for foxy and galactic.
- Fix other build issues: Fixed other build issues.
- Correct codespell: Corrected codespell.
- Correct python linters warnings: Corrected python linters warnings.
- Fix trailing spaces: Fixed trailing spaces.
- Update SM template and make example code clearly visible: Updated SM template and made example code clearly visible.
- Remove use of node in the sm performance template: Removed use of node in the sm performance template.
- Updated templated to use Blackboard storage: Updated templated to use Blackboard storage.
- Update template to resolve the global data correctly: Updated template to resolve the global data correctly.

Removed:
--------
- Removed manual installation of ros-rolling-ros2trace: This is now automated in setupTracing.sh.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>

```rst
Section_43
==========

Added
-----
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Renaming of event generator library.
- Update smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md launch command.

Fixed
-----
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Correct all linters and formatters.

Removed
-------
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Other
-----
- Some progress on navigation rolling.
- Performance tests improvements.
- More on performance and other issues.
- Minor formatting.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progressing in AWS navigation.
- More on navigation.
- Format improvements.
- Attempting pre-commit fixes.
- Fixing precommit.
- Progress in AWS navigation demo.
- Minor format.

Contributors
------------
- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_44
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the `/bond` topic and ensure they are alive. Nodes to wait for can be optionally selected.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.

Changed
-------

- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- Going forward in testing `sm_dance_bot` introducing slam pausing/resuming functionality.
- Move method after the method it calls to prevent recursion.

Fixed
-----

- Remove some compile warnings.
- Correct formatting.
- Additional linting and formatting.
- Remove merge markers from a Python file.

Removed
-------

- Remove `neo_simulation2` package.

Collaborators
-------------

- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pablo@ibrobotics.com>
```

Section_45
==========

Added
-----
- First working version of sm template and template generator. (#127)
- Feature/dance bot s pattern (#129)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Feature/cb pure spinning (#188)

Changed
-------
- Minor tweaks (#130)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Update package list. (#142)
- Update readme (#164)
- Initial migration to smacc2
- Progress on moveit migration testing
- Finetuning waypoints (#187)

Fixed
-----
- Waypoints navigator bug (#133)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing compiling issues
- Fixing broken master build
- SrConditional fixes and formatting (#168)

Removed
-------
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_46
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
- Fix broken source build (#227)
- Only rolling version should be pre-released on on master. (#230)
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
- Update file for fake hardware simulation and add file for gazebo simulation. (#224)
- Feature/improvements warehouse3 (#228)
- Foxy backport (#206)

Changed
-------
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Improving undo motion navigation warehouse2
- Tuning warehouse3
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Retry behavior warehouse 1
- Update file for fake hardware simulation and add file for gazebo simulation.
- Rename header files and correct format
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Reset all versions to 0.0.0
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh

Fixed
-----
- Several fixes (#194)
- Fix trailing spaces
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
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Execute on master update
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs". This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61

Removed
-------
- Weird moveit not downloaded repo
- Added missing file from warehouse2
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- First ensure you have the necessary package installed
- Use manual deployment for now
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Correct GitHub branch reference
- Ignore all packages except smacc2 and smacc2_msgs
- Ignore all packages except smacc2 and smacc2_msgs
``` 

*pabloinigoblasco*

```rst
Section_47
==========

Added
-----

- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------

- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events after.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.

Fixed
-----

- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_atomic_24hr.
- Fixed source CI and corrected README overview.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.

Removed
-------

- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other
-----

- Reactivated smacc2 nav clients for rolling via submodules.
- Edited tracing.md to reflect new tracing event names.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Some progress on navigation rolling.
- Performance tests improvements.
- More on performance and other issues.
- Format cleanup in sm_respira_1.
- Format cleanup in sm_respira_1 pre-commit.
- More changes on performance tests.
- Reformatted sm_reference_library.
- Minor formatting improvements.
- More readme updates.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- More on navigation.
- Attempting pre-commit fixes.
```

```rst
Section_48
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Optional selection of nodes to wait.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- `dance_bot_launch_gz_lidar_choice`: cleaning and lidar show/hide option.
- `sm_multi_stage_1` doubling.

Fixed
-----
- Remove some compile warnings.
- Minor hotfix.
- Format fixes for Gazebo to show the robot and lidar.
- Format fixes for `sm_dance_bot_strikes_back`.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Progress in AWS navigation demo.
- Several core improvements during navigation testing.
- Precommit cleanup run.
- Updates YAML.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Merge and progress.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section 49
-----------

Added
-----

- Added feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting
- Added Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing smacc2::deep_history syntax
  - Testing sm_dance_bot with slam pausing/resuming functionality
- Added Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern
  - Fixed typo "Finnaly" to "Finally"
- Added First working version of sm template and template generator (#127)
- Added Feature/sm dance bot refine (#131)
- Added Feature/sm dance bot refine 2 (#132)
  - Build fix
- Added waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Added progress in the sm_dance_bot tests (#135)
  - Some more progress on markers cleanup
- Added Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140)
  - Added remaining SVGs to READMEs (#145)
- Added Update package list (#142)
- Added Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes so robot can complete the course (#155)
- Added Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Progressing in the moveit migration testing
- Added Update readme (#164)
- Added initial state machine transition timestamp (#165)
- Added Add QOS durability to SmaccPublisherClient (#163)
- Added Feature/testing moveit behaviors (#167)
  - More testing on moveit
  - Progress on moveit
- Added sm_pubsub_1 (#169)
- Added sm_pubsub_1 part 2 (#170)
- Added sm_advanced_recovery_1 renaming (#171)
- Added sm_multi_stage_1 reworking (#172)
  - Multistage modes
  - Sequences and steps
- Added Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and refactorings on navigation clients and behaviors
  - More on aws demo
- Added warehouse2 (#177)
- Added Waypoint Inputs (#178)
- Added sm_dance_bot_warehouse_3 (#181)

Changed
-------

- Moved reference library SMs to smacc2_performance_tools (#166)

Fixed
-----

- Remove merge markers from a python file (#119)
- Move method after the method it calls to prevent recursion (#126)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing pipeline error
- Fixing broken master build
- Fixing compiling issues

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc

Collaborators
-------------

- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_50
==========

Added
-----
- Feature/sm warehouse 2 13 dec 2 (#182): Implemented warehouse feature with default values.
- Feature/Brettpac branch (#184): Added new branch with improvements on waypoints.
- Feature/wharehouse2 dec 14 (#185): Introduced minor changes to warehouse2.
- Feature/cb pure spinning (#188, #189): Implemented pure spinning behavior with default values.
- Feature/planner changes 16 12 (#191): Made minor changes and fixes to planner.
- Feature/replanning 16 dec (#193): Improved replanning for all examples.
- Feature/undo motion 20 12 (#196, #198): Enhanced undo motion navigation for warehouse2.
- Feature/sync 21 12 (#199): Addressed format issues in sync feature.
- Feature/warehouse2 22 12 (#200): Fixed format issues and completed warehouse2.
- Feature/warehouse2 23 12 (#201): Tuned and fixed warehouse2.
- Feature/minor tune (#203): Made minor tune-ups and fixes.
- Feature/undo motion 20 12 (#198): Improved undo motion navigation for warehouse2.
- Feature/sync 21 12 (#199): Addressed format issues in sync feature.
- Feature/warehouse2 22 12 (#200): Fixed format issues and completed warehouse2.
- Feature/warehouse2 23 12 (#201): Tuned and fixed warehouse2.
- Feature/minor tune (#203): Made minor tune-ups and fixes.
- Feature/barrel - do not merge yet (#233): Implemented changes for barrel feature.

Changed
-------
- SrConditional fixes and formatting (#168): Improved formatting and logic on SrConditional.
- Fix trailing spaces (#206): Corrected trailing spaces and codespell issues.
- Foxy backport (#206): Updated CI build for Foxy and Galactic versions.

Fixed
-----
- Several fixes (#194): Addressed various issues in the code.
- Tuning and fixes (#202): Tuned and fixed issues in the code.
- Fixing warehouse 3 problems, and other core improvements (#204): Resolved warehouse 3 problems and made core improvements.
- Fixing broken build (#206): Fixed issues causing build failures.

Removed
-------
- Removed redundant entries.

Collaborators
-------------
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
Section_51
==========

Added
-----

- Added setupTracing.sh script for installing necessary packages and configuring tracing group.
- Added README tutorial for Dockerfile.
- Added new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added sm_multi_stage_1 state machine.

Changed
-------

- Changed wording from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Reformatted sm_reference_library.
- Updated description table.
- Updated table.
- Edited tracing.md to reflect new tracing event names.

Fixed
-----

- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Fixed pre-commit issues in various sm_atomic_performance_test states.
- Fixed formatting in sm_advanced_recovery_1.
- Fixed formatting in sm_multi_stage_1.

Removed
-------

- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed galactic builds from master, keeping only rolling. Removed submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Other
-----

- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Reverted markdowns to html.
- Reactivated smacc2 nav clients for rolling via submodules.
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Created alternative ManualTracing.
- Opened new folder for additional tracing contents.
- Dockerfile now accepts ROS distro as argument.
- Several cleanup operations performed.
- Various improvements and progress made in navigation testing and performance tests.
- Various minor formatting and naming adjustments.
- Various commits and branches related to specific features and improvements.

Authors
-------

- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_52
==========

Added
-----
- Introducing new feature, cb_wait_topic_message: an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Added new client behavior for nav2, which waits for nav2 nodes to subscribe to the /bond topic and ensures they are operational. Users can select specific nodes to wait for.
- Implemented base for the sm_aws_aarehouse navigation.
- Added navigation parameters fixes for sm_dance_bot.
- Introduced cb pause slam client behavior.
- Added sm_dance_bot_lite for visualizing turtlebot3.
- Added choice for launching gz lidar in dance bot.
- Implemented gazebo fixes for showing the robot and lidar in dance bot scenarios.
- Doubled sm_multi_stage_1 functionality.
- Implemented gazebo fixes for sm_dance_bot_strikes_back.

Changed
-------
- Corrected all linters and formatters.
- Various formatting improvements.

Fixed
----
- Resolved compile warnings.

Removed
-------
- Removed some compile warnings.

Collaborators
-------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_53
==========

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
- Added README for `sm_multi_stage_1`.
- Added husky launch file in `sm_dance_bot`.
- Added dependencies for husky simulation.

Changed
-------
- Renamed `sm_advanced_recovery_1`.
- Reworked `sm_multi_stage_1` with more sequences and steps.

Fixed
-----
- Fixed compilation warnings.
- Fixed formatting in various files.
- Fixed launch command in README.md.
- Fixed CI format for Python version.
- Fixed pipeline errors.
- Fixed broken master build.

Removed
-------
- Removed `neo_simulation2` package.
- Removed merge markers from a Python file.
- Removed `sm_dance_bot_msgs`.
- Removed parameters from `smacc`.
- Removed node creation and created only a logger.

Other
-----
- Made diverse improvements in navigation and performance.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- Added SM core test.
- Added QOS durability to `SmaccPublisherClient`.
- Added reliability QOS config.
- Updated package list.
- Added SM Atomic SM generator.
- Rolled Docker environment to be executed from any environment.
- Updated README files with SVGs.
- Updated format in various files.
- Updated dependencies for moveit migration.
- Updated Dockerfile for building local tests.
- Updated README with more information.
- Progressed in navigation, slam toggle client behaviors, and slam_toolbox components.
- Progressed in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Made progress on moveit migration testing.
- Mitigated overshot issue cases in navigation.
- Made minor navigation improvements.
- Used local action messages.
- Made progress on markers cleanup.
- Added remaining SVGs to READMEs.
- Cleaned up pre-commit hooks.
- Noted typo and corrected it.
- Noted and fixed minor format issues.
- Noted and fixed minor tweaks.
- Noted and fixed minor issues.
- Noted and fixed minor configuration.
- Noted and fixed minor dockerfile issues.
- Noted and fixed compiling issues.
- Noted and fixed minor tuning issues.
- Noted and fixed minor linting warnings.
- Noted and fixed minor format issues.
- Noted and fixed minor build errors.
- Noted and fixed minor workflow issues.
- Noted and fixed minor CI issues.
- Noted and fixed minor test issues.
- Noted and fixed minor progress issues.
- Noted and fixed minor refinement issues.
- Noted and fixed minor changes in various components.
- Noted and fixed minor issues in `sm_dance_bot`.
- Noted and fixed minor issues in `sm_dance_bot` with s-pattern.
- Noted and fixed minor issues in `sm_dance_bot_lite`.
- Noted and fixed minor issues in `sm_pubsub_1`.
- Noted and fixed minor issues in `sm_advanced_recovery_1`.
- Noted and fixed minor issues in `sm_multi_stage_1`.
- Noted and fixed minor issues in `sm_multi_stage_1` with multistage modes.
- Noted and fixed minor issues in `sm_multi_stage_1` with sequences.
- Noted and fixed minor issues in `sm_multi_stage_1` with steps.
- Noted and fixed minor issues in `sm_multi_stage_1` with sequence d.
- Noted and fixed minor issues in `sm_multi_stage_1` with sequence c.
- Noted and fixed minor issues in `sm_multi_stage_1` with mode_5_sequence_b.
- Noted and fixed minor issues in `sm_multi_stage_1` with mode_4_sequence_b.
- Noted and fixed minor issues in `sm_multi_stage_1` with finishing touches.
- Noted and fixed minor issues in `sm_multi_stage_1` with README updates.
- Noted and fixed minor issues in `sm_multi_stage_1` with dependencies.
- Noted and fixed minor issues in `sm_multi_stage_1` with Docker refactoring.
- Noted and fixed minor issues in `sm_multi_stage_1` with repos dependencies.
- Noted and fixed minor issues in `sm_multi_stage_1` with progress updates.
- Noted and fixed minor issues in `sm_multi_stage_1` with mode_5_sequence_b.
- Noted and fixed minor issues in `sm_multi_stage_1` with mode_4_sequence_b.
- Noted and fixed minor issues in `sm_multi_stage_1` with finishing touches.
- Noted and fixed minor issues in `sm_multi_stage_1` with README updates.
- Noted and fixed minor issues in `sm_multi_stage_1` with dependencies.
- Noted and fixed minor issues in `sm_multi_stage_1` with Docker refactoring.
- Noted and fixed minor issues in `sm_multi_stage_1` with repos dependencies.
- Noted and fixed minor issues in `sm_multi_stage_1` with progress updates.

```rst
Section_54
==========

Added
-----
- Update dependencies for husky in rolling and galactic.
- Progress on AWS navigation and refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress.
- Waypoint Inputs.
- Warehouse2 progress.
- Format.
- Sm_dance_bot_warehouse_3.
- Feature/sm warehouse 2 13 dec 2.
- Warehouse2.
- Finetuning waypoints.
- Feature/cb pure spinning.
- Feature/planner changes 16 12.
- Feature/replanning 16 dec.
- Feature/undo motion 20 12.
- Tuning warehouse3.
- Feature/sync 21 12.
- Feature/warehouse2 22 12.
- Feature/warehouse2 23 12.
- Feature/minor tune.
- Fixing warehouse 3 problems and other core improvements.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core.
- Autoware demo.
- Foxy CI.
- Docker files for different revisions.
- Fixing docker for foxy and galactic.
- Barrel demo.
- Barrel search build fix and warehouse3.
- Fixing startup problems in warehouse 3.
- Progress in barrel husky.
- Barrel search updates.
- Making models local.
- Red picuup.
- Multiple controllable leds plugin.
- Progress in husky demo.
- Improving navigation behaviors.
- More merge.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licences.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Disable disabled packages.
- Enable cppcheck.
- Included necessary package and edited Threesome launch.

Changed
-------
- Fix formatting.
- Minor changes.
- Merge.
- Headless and other fixes.
- Default values.
- Replanning for all our examples.
- Improving undo motion navigation warehouse2.
- Tuning and fixes.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Change extension of imports.
- Correct formatting of python file.
- Branching example.
- Update ci-build-source.yml.

Removed
-------
- Disable disabled packages.
```

```rst
Section_55
==========

Version 0.1.0 (Unreleased)
---------------------------

Added
-----

- Ensure ros-rolling-ros2trace package is installed before running.
- Renamed header files and corrected format.
- Added workflows for doc build checking.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Implemented manual deployment.
- Added workflow for testing prerelease builds.
- Used 'docs/' as source and output directory for documentation.
- Renamed packages to smacc2 and smacc2_msgs.
- Updated GitHub branch reference.
- Updated package name and package.xml.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Updated description table.
- Updated table.
- Copied initial docs.
- Added Dockerfile with ROS distro as argument.
- Opened new folder for tracing contents.
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh for installing necessary packages and configuring tracing group.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added Dockerfile for Rolling and Galactic.
- Co-authored by Denis Štogl.
- Changed wording from "smacc application" to "SMACC2 library".
- Edited README from html to markdown syntax.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Reverted markdowns to html.
- Added README tutorial for Dockerfile.
- Cleaned up tracing.md to reflect new tracing event names.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds, kept only rolling, removed submodules and used .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Made progress on navigation rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Added smacc2_performance_tools.
- Made performance tests improvements.
- Made more performance and other issues changes.
- Cleaned up sm_respira_1 format.
- Cleaned up sm_respira_1 format pre-commit.
- Added sm_respira_test_2.
- Made more changes on performance tests.
- Did not execute clang-format on smacc2_sm_reference_library package.
- Reformatted sm_reference_library.
- Corrected trailing spaces.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Changed launch command to 'ros2 launch sm_respira_1 sm_respira_1.launch'.
- Updated doxygen links.
- Made more README updates.
- Created new sm from sm_respira_1.
- Made core and navigation fixes.
- Made progress in aws navigation.
- Made several core improvements during navigation testing.
- Made formatting improvements.
- Made progress in aws navigation demo.
- Made more format improvements.
- Made more navigation progress.
- Reworked sm_advanced_recovery_1.
- Fixed pre-commit.
- Made more sm_advanced_recovery_1 changes.
- Made more sm_advanced_recovery_1 work.
- Continued sm_advanced_recovery_1 work.
- Created sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1.
- Created sm_atomic_performance_test_c_1.
- Modified sm_atomic_performance_test_a_2.
- Created sm_multi_stage_1.
- Fixed precommit for sm_multi_stage_1.
- Made more sm_multi_stage_1 changes.
- Updated README.md with launch command changes.
- Fixed topic message client behavior.
```

*pabloinigoblasco*

```rst
Section_56
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `wait nav2 nodes` subscribes to the `/bond` topic and waits for them to be alive, with optional node selection.
- `cb_pause_slam` client behavior.

Changed
-------
- Corrected all linters and formatters.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor hotfixes.
- Cleaning and lidar show/hide option for `sm_dance_bot` visualizing `turtlebot3`.
- Gazebo fixes to show the robot and lidar.
- Doubling in `sm_multi_stage_1`.

Fixed
-----
- Removed some compile warnings.

Removed
-------
- Merge and progress.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

Section_57
==========

Added
-----

- Added visualization for turtlebot3 in sm_dance_bot.
- Added show/hide option for cleaning and lidar.
- Added AWS demo (#108, #110).
- Added Brettpac branch (#111).
- Added progress in sm_multi_stage_1 (#109, #114).
- Added diverse improvements in navigation and performance (#116).
- Added slam toggle and smacc deep history feature (#122).
- Added sm_dance_bot s-pattern feature (#128, #129).
- Added first working version of sm template and template generator (#127).
- Added SM core test (#138).
- Added local action messages usage (#139).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added SM Atomic SM generator (#143).
- Added rolling Docker environment execution from any environment (#154).
- Added initial state machine transition timestamp (#165).
- Added QOS durability to SmaccPublisherClient (#163).
- Added reliability QOS configuration.

Changed
-------

- Changed format in various files.
- Changed gazebo settings to show robot and lidar.
- Changed method order to prevent recursion (#126).
- Changed launch command in README.md for sm_dance_bot_strikes_back.
- Changed navigation 2 stack naming.
- Changed parameters in smacc (#147).
- Changed node creation to logger only (#149).
- Changed Dockerfile for building local tests.

Fixed
-----

- Fixed minor format issues (#134).
- Fixed compile warnings (#137).
- Fixed CI format for Python version (#148).
- Fixed overshot issue cases in waypoints navigator (#133).
- Fixed pipeline error in testing moveit behaviors (#167).

Removed
-------

- Removed neo_simulation2 package (#112).
- Removed merge markers from a Python file (#119).
- Removed sm_dance_bot_msgs.
- Removed parameters in smacc.
- Removed test from main moveit CMake.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> (multiple commits).
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com> (#116).
- Co-authored-by: DecDury <declandury@gmail.com> (#152).
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com> (#152).

```rst
Section_58
==========

Added
-----
- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency and husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Updated dependencies for husky in rolling and galactic.
  - Implemented progress on aws navigation and refactorings on navigation clients and behaviors.
  - Added more on aws demo.

- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2 progress.

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added more changes and headless merge.
  - Set default values.

- Feature/cb pure spinning (#188)
  - Implemented pure spinning behavior and fixed missing files.

- Feature/planner changes 16 12 (#191)
  - Made minor changes and more fixes.

- Feature/replanning 16 dec (#193)
  - Implemented replanning for all examples.

- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation in warehouse2.
  - Tuned warehouse3.

- Feature/sync 21 12 (#199)
  - Fixed format issues.

- Feature/warehouse2 22 12 (#200)
  - Finished warehouse2 and fixed format issues.

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed issues.

- Feature/minor tune (#203)
  - Tuned and fixed minor issues.

- Use correct upstream .repos files for source builds (#243)
- Corrected mergify branch names (#246)

Changed
-------
- Renamed sm_advanced_recovery_1 (#171).
- Reworked sm_multi_stage_1 (#172).
- Updated multistage modes and sequences.
- Modified sm_multi_state_1 steps.
- Adjusted sm_multi_stage_1 sequences and modes.
- Fine-tuned waypoints (#187).
- Tuned and fixed warehouse 3 problems, and other core improvements (#204).

Fixed
-----
- Fixed broken master build.
- Fixed broken builds.
- Fixed formatting on SrConditional.
- Moved trigger logic into headers.
- Linted code.
- Fixed errors in undo motion and warehouse3.
- Fixed format and minor issues.
- Fixed startup problems in warehouse 3.
- Fixed format and minor errors.
- Fixed warnings and tested navigation.
- Fixed docker for foxy and galactic.
- Fixed barrel search build and warehouse3.
- Fixed models localization.
- Fixed multiple controllable leds plugin.
- Fixed navigation behaviors.
- Fixed docker improvements.

Removed
-------
- Removed redundant entries.
```

Section_59
==========

Added
-----
- Added spawn entity delays.

Changed
-------
- Updated galactic source build job name.
- Updated .repos file, bumped action version, and used correct version of upstream packages for galactic source build (backport #241).
- Fixed checkout branches for scheduled builds.
- Corrected checkout branch on scheduled build.
- Significant update in Getting Started Instructions.
- Removed trailing spaces.
- Fixed URLs to index.ros.org.
- Fixed foxy source build config to use repos file from foxy branch.
- Fixed husky project build on rolling.
- Fixed type string walker threesome demo.

Fixed
-----
- Corrected name of source-build job and bumped version of action (#242) (#247).
- Fixed rolling build.
- Fixed dependencies for rolling build.
- Fixed missing repo and dependencies.
- Fixed building issue.
- Fixed broken build.
- Fixed build issue.
- Fixed husky build on rolling.
- Fixed UR demos.
- Initialized conditionFlag as false.
- Fixed precommit issue.

Removed
-------
- Ignored packages which should not be released.

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco

0.3.0 (2022-04-04)
------------------

### Added
- Improved navigation client behaviors and husky barrel demo (#311)
  - Many enhancements in action client and cb sequence for husky barrel search
  - Enhanced navigation behaviors on husky barrel search demo
  - Functionality improvements in navigation and warehouse 3
  - Format enhancements in navigation and warehouse 3 and husky

### Fixed
- Initial fixing of single UR sim (#302)
  - Updated repos files and README.md for gazebo simulation of UR5
  - Fixed formatting and Python flake formatting issues
  Co-authored-by: Manuel M <mamueluth@gmail.com>

### Changed
- Feature/multi UR5 sim (#290)
  - Removed galactic builds from master and kept only rolling
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling and performance tests
  - Renamed folders, deleted tracing.md, and edited README.md
  - Added smacc2_performance_tools
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Minor formatting changes
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Added galactic CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Cleaned up sm_atomic_24hr
  - More changes on performance tests
  - Do not execute clang-format on smacc2_sm_reference_library package
  - Corrected trailing spaces
  - Updated smacc2_rta command across readmes

### Removed
- Note that was not removed during production

### Added
- More Readme Updates (#72, #74)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

### Changed
- Feature/core and navigation fixes (#78)
  - Base for the sm_aws_warehouse navigation
  - Progress in AWS navigation demo and core improvements
  - Formatting enhancements

### Added
- Feature/aws demo progress (#80, #92)
  - Progress in AWS navigation demo and core improvements
  - Formatting enhancements

### Changed
- sm_advanced_recovery_1 reworked (#83, #84, #85, #86)
  - Reworked sm_advanced_recovery_1 and fixed pre-commit issues
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

### Added
- Brettpac branch (#87)
  - Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

### Changed
- sm_atomic_performance_test_c_1 (#88)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

### Added
- Modifying sm_atomic_performance_test_a_2 (#89)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

### Changed
- sm_multi_stage_1 (#90, #91)
  - Fixed precommit issues
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

### Added
- Wait topic message client behavior (#81)
  - New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Attempted precommit fixes
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

### Changed
- Feature/wait nav2 nodes client behavior (#82)
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive
  - Corrected all linters and formatters
  Co-authored-by: Denis Štogl <denis@stogl.de>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_61
==========

Added
-----
- New client behavior `cb_wait_topic_message` for Nav2: Waits for Nav2 nodes to subscribe to the `/bond` topic and confirms they are active. Optional node selection available.
- New client behavior for Nav2: Waits for Nav2 nodes to subscribe to the `/bond` topic and confirms they are active. Optional node selection available.
- New feature `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature added.
- Visualizing Turtlebot3 in `sm_dance_bot` and `sm_dance_bot_lite`.
- Choice to show/hide lidar in `sm_dance_bot`.
- Gazebo fixes to display the robot and lidar in `sm_dance_bot`, `sm_dance_bot_strikes_back`, and `sm_multi_stage_1`.
- AWS demo progress.
- `sm_multi_stage_1` enhancements.
- Various core improvements during navigation testing.
- `neo_simulation2` package removed.
- Source build enabled for testing.
- Build package adjustments for source CI.
- Diverse improvements in navigation and performance.

Changed
-------
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Merge and progress updates.
- Hotfix for minor issues.

Fixed
-----
- Compile warnings removed.

Removed
-------
- Unused `neo_simulation2` package.

Contributors
------------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_62
==========

Added
-----
- Additional linting and formatting.
- Feature/slam toggle and smacc deep history (#122): Progress in navigation, slam toggle client behaviors, slam_toolbox components, and smacc2::deep_history syntax.
- Feature/more_sm_dance_bot_fixes: Minor format improvements.
- Feature/dance bot s pattern (#128): Polishing sm_dance_bot and s-pattern.
- First working version of sm template and template generator (#127).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Update package list (#142).
- Add SM Atomic SM generator (#143).
- Rolling Docker environment to be executed from any environment (#154).
- Add QOS durability to SmaccPublisherClient (#163).
- Feature/aws navigation sm dance bot (#174): Progress on aws navigation, refactorings on navigation clients and behaviors.
- Waypoint Inputs (#178).
- Brettpac branch (#184): Redoing sm_dance_bot_warehouse_3 waypoints.

Changed
-------
- Move method after the method it calls (#126): Prevent recursion.
- Resolve compile warnings (#137).
- Minor navigation improvements (#141).
- Using local action messages (#139).
- Fix CI: format fix python version (#148).
- Fixing some errors introduced on formatting in Feature/migration moveit client (#151).
- Update readme (#164).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Minor changes (#175).

Fixed
-----
- Waypoints navigator bug (#133): Minor tuning to mitigate overshot issue cases.
- Noticed launch command was incorrect in README.md: Fixed launch command for sm_dance_bot_strikes_back and removed some comments.
- Fixing broken master build in Feature/testing moveit behaviors (#167).
- Fixing broken build in Feature/aws navigation sm dance bot (#174).

Removed
-------
- Removing sm_dance_bot_msgs: Pending references.
- Removing parameters smacc: Workflows update.
- Removing test from main moveit cmake: Test ur5.
- Removing node creation and create only a logger (#149).

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_63
==========

Added
-----
- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2 feature with minor changes.

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Implemented format changes, headless mode, and default values for warehouse2.

- Feature/cb pure spinning (#188)
  - Introduced pure spinning behavior with format changes and headless mode.

- Feature/replanning 16 dec (#193)
  - Implemented replanning for all examples with several fixes.

- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation for warehouse2 with minor changes.

- Feature/sync 21 12 (#199)
  - Addressed format issues for synchronization.

- Feature/warehouse2 22 12 (#200)
  - Resolved format issues and completed warehouse2.

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2.

- Feature/minor tune (#203)
  - Tuned and fixed minor issues.

- Feature/undo motion 20 12 (#198)
  - Improved undo motion navigation for warehouse2 with minor changes.

Changed
-------
- Finetuning waypoints (#187)
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  - Minor changes for waypoint finetuning.

- Fixing warehouse 3 problems, and other core improvements (#204)
  - Fixed warehouse 3 issues, removed deadlocks, and improved core functionality.

- Foxy backport (#206)
  - Backported changes to Foxy, addressing formatting, trailing spaces, and linting issues.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
  - Automation now handled by setupTracing.sh script.

- Deleted tracing directory.
  - Tracing contents now organized in a new folder.

- Removed disabled packages and updated workflows.
  - Updated CI build configurations and dependencies.

- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
  - Reverted commit to reset package versions.

- Removed galactic builds from master branch.
  - Kept only rolling builds and updated repository structure.

- Updated mentions of SMACC/ROS to SMACC2/ROS2.
  - Aligned references to the latest versions.

Fixed
-----
- SrConditional fixes and formatting (#168)
  - Addressed formatting and templating issues in SrConditional.

- Fixed trigger logic placement in headers for SrConditional.

- Linted codebase for SrConditional.

- Fixed pure spinning behavior missing files (#190)
  - Resolved missing files issue for pure spinning behavior.

- Corrected errors in undo tuning and formatting.

- Fixed linking errors for Foxy backport.

- Corrected formatting of python files.

- Fixed bug in smacc2 component.

- Resolved markdown to HTML conversion issues.

- Cleaned up tracing events and naming conventions.

- Enabled build of missing rolling repositories.

- Enabled Navigation2 for semi-binary build.

- Updated README tutorial for Dockerfile.

- Performed additional cleanup and bug fixes.

- Made progress on navigation for rolling version.
``` 

*pabloinigoblasco*

```rst
Section_64
==========

Added
-----
- Added `smacc2_performance_tools`.
- Added `galactic CI setup` and renamed rolling files (#58).
- Added new feature, `cb_wait_topic_message`: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior for nav2, `wait nav2 nodes subscribing to the /bond topic and waiting they are alive`. You optionally can select the nodes to wait.

Changed
-------
- Renamed folders.
- Updated `smacc2_rta` command across readmes.
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69).
- Corrected trailing spaces.
- Optimized dependencies in `move_base_z_planners_common`.
- Renamed event generator library.
- Updated `c_cpp_properties.json`.
- Corrected all linters and formatters.

Fixed
-----
- Fixed source CI and corrected README overview (#62).
- Fixed pre-commit issues in various packages.

Removed
-------
- Do not execute `clang-format` on `smacc2_sm_reference_library` package.

Other
-----
- Minor formatting improvements.
- Noticed a note that was not removed.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.

Author
------
- Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_65
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive, with optional node selection
- Gazebo fixes for `sm_dance_bot_strikes_back`

Changed
-------
- Progress in AWS navigation demo
- Formatting improvements
- Navigation parameters fixes on `sm_dance_bot`
- `cb_pause_slam` client behavior

Fixed
-----
- Minor format fixes
- Remove some compile warnings
- Correct formatting in `neo_simulation2` package removal
- Move method after the method it calls to prevent recursion

Removed
-------
- `neo_simulation2` package

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_66
==========

Added
-----

- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
- Progress in the sm_dance_bot tests (#135)
- Sm_dance_bot_lite (#136)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Feature/nav2z renaming (#144)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Initial migration to smacc2 (#151)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Sm_pubsub_1 (#169)
- Sm_pubsub_1 part 2 (#170)
- Sm_advanced_recovery_1 renaming (#171)
- Sm_multi_stage_1 reworking (#172)
- Feature/aws navigation sm dance bot (#174)
- Minor changes (#175)
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Warehouse2 progress (#179)
- Format (#180)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Pure spinning behavior missing files (#189)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)

Changed
-------

- Minor tuning to mitigate overshot issue cases
- Minor format issues (#134)
- Format fix python version (#148)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing some compiling issues
- Fixing broken master build
- Fixing broken build

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing parameters smacc
- Removing some comments in the past

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_67
===========

Added
-----
- Feature/undo motion 20 12 (#196)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Fix rolling builds (#222)
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
- Create doxygen-deploy.yml
- Use manual deployment for now.
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update
- Reset all versions to 0.0.0
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table.
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh

Changed
-------
- Several fixes (#194)
- Minor changes (#195)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Feature/minor tune (#203)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml
- Change wording "smacc application" to "SMACC2 library"

Fixed
-----
- Fix
- Minor broken build

Removed
-------
- Remove example things from Foxy CI setup.

Co-authored-by
-------------
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>

```rst
Section_68
==========

Added
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally, you can select the nodes to wait.

Changed
-------
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Renamed event generator library.

Fixed
----
- Bug in smacc2 component.
- Reverted markdowns to html.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_atomic_24hr.
- Fixed source CI and corrected README overview.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Other Changes
-------------
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Edited tracing.md to reflect new tracing event names.
- Edited sm_respira_1 format cleanup.
- Edited sm_atomic_24hr cleanup.
- Edited sm_reference_library reformatting.
- Edited sm_advanced_recovery_1 reworked.
- Edited sm_multi_stage_1.
- Edited sm_atomic_performance_test_a_2.
- Edited sm_atomic_performance_test_c_1.
- Edited sm_atomic_performance_test_a_2.
- Edited sm_multi_stage_1.
- Edited README.md.
- Attempted precommit fixes.
- Corrected all linters and formatters.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_69
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive, with optional node selection
- Base for the `sm_aws_warehouse` navigation
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` visualizing TurtleBot3
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`
- Minor formatting improvements
- Cleaning and lidar show/hide option
- Gazebo fixes to show the robot and the lidar
- Progress in navigation, `slam_toggle` client behaviors, and `slam_toolbox` components
- Introducing slam pausing/resuming functionality in testing `sm_dance_bot`
- `smacc2::deep_history` syntax

Fixed
----
- Remove some compile warnings
- Remove `neo_simulation2` package
- Correct formatting
- Enable source build on PR for testing
- Adjust build packages of source CI
- Additional linting and formatting
- Remove merge markers from a Python file

Removed
-------
- `neo_simulation2` package

Contributors
------------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_70
==========

Added
-----

- First working version of sm template and template generator. (#127)
- Add SM core test (#138)
- Add SM Atomic SM generator. (#143)
- Add QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Rolling Docker environment to be executed from any environment (#154)
- Added SM Atomic SM generator. (#143)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)

Changed
-------

- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Minor navigation improvements (#141)
- Using local action messages
- Update package list. (#142)
- Fix CI: format fix python version (#148)
- Update readme (#164)
- Finetuning waypoints (#187)

Fixed
-----

- Noticed typo: Finnaly > Finally
- Fix launch command in README.md for sm_dance_bot_strikes_back
- Fix compiling issues

Removed
-------

- Removed node creation and create only a logger. (#149)
- Removed parameters smacc (#147)
- Removed sm_dance_bot_msgs
- Removed test from main moveit cmake

Other
-----

- More changes in sm_dance_bot (#125, #128, #129, #131, #132)
- Polishing sm_dance_bot and s-pattern
- Minor tweaks (#130)
- Minor format issues (#134)
- Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
- Some more progress on markers cleanup
- Pending references
- Navigation 2 stack renaming
- Formatting
- Precommit cleanup
- Repos dependency
- Docker refactoring
- Progress on move_it PR
- Progress on moveit migration testing
- Progress on moveit
- Progress on aws demo
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- Progress on warehouse2
- More on aws demo
- More on moveit testing
- More on moveit behaviors testing
- More on sm_dance_bot_lite
- More refinement in sm_dance_bot
- More testing on moveit
- More testing on moveit behaviors
- More changes and headless in sm warehouse 2 13 dec 2
- More readme updates
- More
- Merge
- Headless and other fixes
- Default values
- Brettpac branch
- Redoing sm_dance_bot_warehouse_3 waypoints
- More waypoints
- Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Minor changes (#175)
- Warehouse2 (#177)
- Wharehouse2 progress (#179)
- Format (#180)
- Sm_dance_bot_warehouse_3 (#181)
- SrConditional fixes and formatting (#168)
```

## Section_71

### Added
- **Feature/cb pure spinning (#188)**
  - Implemented pure spinning behavior.
- **Feature/cb pure spinning (#189)**
  - Continued development on pure spinning behavior.
- **Feature/planner changes 16 12 (#191)**
  - Introduced planner changes.
- **Feature/replanning 16 dec (#193)**
  - Improved replanning functionality.
- **Feature/undo motion 20 12 (#196)**
  - Added undo motion navigation for warehouse2.
- **Feature/sync 21 12 (#199)**
  - Implemented synchronization feature.
- **Feature/warehouse2 22 12 (#200)**
  - Completed warehouse2 functionality.
- **Feature/warehouse2 23 12 (#201)**
  - Tuned and fixed warehouse2 operations.
- **Feature/minor tune (#203)**
  - Made minor tuning adjustments.
- **Feature/retry behavior warehouse 1 (#226)**
  - Added retry behavior for warehouse 1.
- **Foxy backport (#206)**
  - Backported changes to Foxy distribution.

### Changed
- **Fix code generators (#221)**
  - Resolved issues with code generation.
- **Fix other build issues**
  - Addressed various build problems.
- **Update SM template and make example code clearly visible**
  - Improved SM template visibility.
- **Update template to resolve the global data correctly**
  - Corrected global data handling in templates.
- **Update sm_name.hpp**
  - Updated sm_name.hpp file.
- **Fix trailing spaces**
  - Removed trailing spaces in code.
- **Correct codespell**
  - Fixed codespell errors.
- **Correct python linters warnings**
  - Resolved Python linter warnings.
- **Add galactic CI build because Navigation2 is broken in rolling**
  - Added Galactic CI build due to issues in Navigation2.
- **Add partial changes for ament_cpplint**
  - Included partial changes for ament_cpplint.
- **Add tf2_ros as dependency to find include**
  - Added tf2_ros as a dependency.
- **Disable ament_cpplint**
  - Deactivated ament_cpplint.
- **Disable some packages and update workflows**
  - Disabled certain packages and updated workflows.
- **Bump ccache version**
  - Updated ccache version.
- **Ignore further packages**
  - Ignored additional packages.
- **Satisfy ament_lint_cmake**
  - Met ament_lint_cmake requirements.
- **Add missing licences**
  - Included missing licenses.
- **Disable cpplint and cppcheck linters**
  - Turned off cpplint and cppcheck linters.
- **Correct formatters**
  - Adjusted code formatters.
- **Disable disabled packages**
  - Deactivated disabled packages.
- **Change extension of imports**
  - Modified import extensions.
- **Enable cppcheck**
  - Enabled cppcheck.
- **Correct formatting of python file**
  - Fixed Python file formatting.
- **Included necessary package and edited Threesome launch**
  - Added required package and edited Threesome launch.

### Fixed
- **Fixing docker for foxy and galactic**
  - Resolved Docker issues for Foxy and Galactic distributions.
- **Minor broken build**
  - Fixed minor issues causing build failures.
- **Some reordering fixes**
  - Made some reordering corrections.

### Removed
- **Removed manual installation of ros-rolling-ros2trace**
  - Eliminated manual installation step for ros-rolling-ros2trace.

```rst
Section_72
==========

Added
-----

- Automated setupTracing.sh location assumption following README.md instructions under "Getting started"
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Reactivated smacc2 nav clients for Rolling via submodules
- Added README tutorial for Dockerfile
- Added smacc2_performance_tools
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait

Changed
-------

- Changed wording "smacc application" to "SMACC2 library"
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Renamed tracing events
- Renamed folders
- Deleted tracing.md
- Edited README.md
- Renamed event generator library
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
- Updated smacc2_rta command across readmes
- Cleaned up sm_atomic_24hr
- Reformatted sm_reference_library
- Optimized dependencies in move_base_z_planners_common
- Minor formatting improvements

Fixed
-----

- Bug in smacc2 component
- Reverted markdowns to html
- Fixed source CI and corrected README overview
- Fixed trailing spaces
- Do not execute clang-format on smacc2_sm_reference_library package
- Fixed pre-commit issues

Removed
-------

- Removed galactic builds from master and kept only rolling
- Removed submodules and use .repos file

Collaborators
-------------

- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_73
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive, with optional node selection
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` for visualizing TurtleBot3
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`
- Minor hotfixes
- Cleaning and lidar show/hide option for `sm_dance_bot`
- Gazebo fixes to show the robot and lidar

Fixed
-----
- Remove some compile warnings
- Remove `neo_simulation2` package
- Correct formatting
- Enable source build on PR for testing
- Adjust build packages of source CI

Removed
-------
- `neo_simulation2` package

Other
-----
- Several core improvements during navigation testing
- Formatting improvements
- Progress in AWS navigation demo
- Merge and progress
- Precommit cleanup run
- Updates YAML
- Precommit
- More on navigation
- Progressing in AWS navigation
- Base for the `sm_aws_warehouse` navigation
- Progress in AWS navigation demo
- Minor format adjustments
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

```rst
Section_74
==========

Version 0.1.0 (2022-01-01)
---------------------------

Added
-----

- Feature/diverse improvements navigation performance (#117)
- Additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Testing sm_dance_bot with slam pausing/resuming functionality
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern
- First working version of sm template and template generator (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
  - Build fix
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
  - Some more progress on markers cleanup
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140)
  - Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Progressing in the moveit migration testing
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
  - More testing on moveit behaviors
- Sm_pubsub_1 (#169)
  - Part 2 (#170)
- Sm_advanced_recovery_1 renaming (#171)
- Sm_multi_stage_1 reworking (#172)
  - Multistage modes
  - Sm_multi_stage sequences
  - Sm_multi_state_1 steps
  - Sm_multi_stage_1 sequence d
  - Sm_multi_stage_1 c sequence
  - Mode_5_sequence_b
  - Mode_4_sequence_b
  - Sm_multi_stage_1 most
  - Finishing touches 1
  - Readme
- Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and some other refactorings on navigation clients and behaviors
  - More on aws demo
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Wharehouse2 progress (#179)
- Sm_dance_bot_warehouse_3 (#181)

Changed
-------

- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Remove node creation and create only a logger (#149)
- Fix CI: format fix python version (#148)

Fixed
-----

- Noticed typo: Finnaly > Finally

Removed
-------

- Remove merge markers from a python file (#119)
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing some comments in the past from README.md

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_75
==========

Added
-----
- Feature/sm warehouse 2 13 dec 2 (#182)
  - Implemented warehouse 2 feature
- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse 2 functionality
- Feature/cb pure spinning (#188)
  - Introduced pure spinning behavior
- Feature/planner changes 16 12 (#191)
  - Implemented planner changes
- Feature/replanning 16 dec (#193)
  - Added replanning feature
- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation
- Feature/sync 21 12 (#199)
  - Added synchronization feature
- Feature/warehouse2 22 12 (#200)
  - Completed warehouse 2 development
- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse 2
- Feature/minor tune (#203)
  - Made minor tuning and fixes
- Feature/improvements warehouse3 (#228)
  - Implemented improvements for warehouse 3

Changed
-------
- Corrected Focal-Rolling builds (#234)
  - Fixed version of rosdep yaml
- Foxy backport (#206)
  - Fixed formatting issues
  - Corrected trailing spaces and codespell
  - Resolved python linters warnings
  - Added galactic CI build
  - Made partial changes for ament_cpplint
  - Added tf2_ros as dependency
  - Disabled ament_cpplint and some packages
  - Updated workflows and bumped ccache version
  - Satisfied ament_lint_cmake
  - Added missing licenses
  - Corrected formatters and disabled cpplint and cppcheck linters
  - Enabled cppcheck and corrected formatting of python file
  - Included necessary package and edited Threesome launch

Fixed
-----
- Fix broken source build (#227)
  - Ensured only rolling version is pre-released on master
- Update file for fake hardware simulation and add file for gazebo simulation. (#224)
  - Updated simulation files
- Retry behavior warehouse 1
  - Fixed missing file and minor format issues
- Other minor changes
  - Made various minor fixes and improvements

Removed
-------
- Removed unnecessary docker files and warnings
- Removed some disabled packages and updated workflows
- Removed disabled cpplint and cppcheck linters
- Removed disabled packages
- Removed unnecessary extensions and packages
```

*pabloinigoblasco*

```rst
Section_76
==========

Added
-----
- Created workflow for testing prerelease builds.
- Renamed to smacc2 and smacc2_msgs.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added smacc2_performance_tools for performance tests improvements.
- Added new feature cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.

Changed
-------
- Updated name of package and package.xml to pass liter.
- Renamed "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Renamed event generator library.

Fixed
-----
- Corrected GitHub branch reference.
- Fixed bug in smacc2 component.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Reformatted sm_reference_library.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Fixed source CI and corrected README overview.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Removed
-------
- Ignored all packages except smacc2 and smacc2_msgs.
- Removed manual installation of ros-rolling-ros2trace; now automated in setupTracing.sh.

Contributors
------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

pabloinigoblasco
```

```rst
Section_77
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success. It allows selecting nodes to wait for in nav2.
- New client behavior for nav2: `wait nav2 nodes`, which subscribes to the `/bond` topic and waits for the nodes to become alive.
- New feature: `cb_pause_slam` client behavior.

Changed
-------

- Corrected all linters and formatters.
- Fixed navigation parameters on `sm_dance_bot`.
- Minor formatting improvements.
- Merge and progress in development.
- Hotfix for doxygen deployment workflow.
- Cleaning and lidar show/hide option in `sm_dance_bot` visualizing TurtleBot3.
- Gazebo fixes to show the robot and lidar in various components.

Removed
-------

- Removed some compile warnings.

Contributors
------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```

*pabloinigoblasco*

Section_78
-----------

Added
-----
- Implemented sm_multi_stage_1 functionality (#109, #110, #111, #114, #172) with initial stages and progress.
- Introduced Brettpac branch for development.
- Diverse improvements in navigation and performance (#116) with minor enhancements.
- Added slam toggle and smacc deep history feature (#122) for client behaviors.
- Implemented dance bot s pattern refinement (#128, #129) and sm template generator (#127).
- Added SM Atomic SM generator (#143) for streamlined state machine creation.
- Added QOS durability to SmaccPublisherClient (#163) for enhanced message reliability.

Changed
-------
- Moved reference library SMs to smacc2_performance_tools (#166) for better organization.
- Renamed sm_advanced_recovery_1 to sm_pubsub_1 (#171) for clarity.

Fixed
----
- Removed neo_simulation2 package (#112) and unnecessary node creation for logger (#149).
- Fixed formatting issues, compile warnings, and CI configurations.
- Resolved waypoint and iteration adjustments for course completion (#155).
- Corrected launch command in README.md and fixed CI formatting for Python version (#148).
- Updated package list and READMEs with SVGs (#140, #145) for better documentation.
- Fixed pipeline errors and broken master builds for stability.
- Mitigated overshot issues in navigation and improved waypoint tuning.
- Fixed compilation issues and linting warnings for smoother development process.

Removed
-------
- Removed redundant parameters in smacc (#147) and unnecessary test from main moveit cmake.
- Removed merge markers from a Python file (#119) for cleaner code.
- Removed sm_dance_bot_msgs package for simplification.
- Removed unnecessary comments and dependencies for cleaner builds.
- Removed test from main moveit cmake and fixed formatting errors.
- Removed some comments in the past for clarity.

Authors
-------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Brett <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>

## Section_79

### Added
- Introduce multistage modes and sequences:
  - `sm_multi_stage` sequences
  - `sm_multi_state_1` steps
  - `sm_multi_stage_1` sequence d
  - `sm_multi_stage_1` c sequence
  - `mode_5_sequence_b`
  - `mode_4_sequence_b`
  - `sm_multi_stage_1` most
  - Finishing touches 1
  - Readme

### Changed
- Enhance AWS navigation for `sm_dance_bot` (#174):
  - Add repository dependency
  - Include launch file for Husky in `sm_dance_bot`
  - Update dependencies for Husky in rolling and galactic
  - Progress on AWS navigation and refactorings on navigation clients and behaviors
  - More on AWS demo
  - Fix broken build

### Fixed
- Resolve minor issues in warehouse2 (#177)
- Implement waypoint inputs (#178)
- Improve warehouse3 waypoints (#181)
- Fine-tune waypoints (#187)
- Fix formatting issues in various features

### Removed
- Remove redundant files in pure spinning behavior (#189)
- Eliminate unnecessary files in warehouse2 (#205)

### Miscellaneous
- Collaborators: Brett, Denis Štogl, Declan Dury, reelrbtx, David Revay
- Various improvements and fixes across different features and components
- Docker files enhancements for different revisions and versions
- Address startup problems in warehouse 3
- Progress in barrel husky demo
- Backport changes to Foxy
- Update CI builds for Foxy and Galactic
- Branching example

Author: Pablo Iñigo Blasco

```rst
Section_80
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
- Added setupTracing.sh:
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Update smacc_sm_reference_library/sm_atomic/README.md.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added README tutorial for Dockerfile.
- Opened new folder for additional tracing contents.

Changed
-------
- Disable disabled packages.
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.
- Correct formatting of python file.
- Changed `ros2 launch sm_three_some sm_three_some` to `ros2 launch sm_three_some sm_three_some.launch`.
- Rename header files and correct format.
- Update doxygen-check-build.yml.
- Use manual deployment for now.
- Rename to smacc2 and smacc2_msgs.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Ignore all packages except smacc2 and smacc2_msgs.
- Update changelogs.
- Revert "Ignore all packages except smacc2 and smacc2_msgs".
- Update description table.
- Update table.
- Copy initial docs.
- Dockerfile w/ ROS distro as argument.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Removed manual installation of ros-rolling-ros2trace.
- Location of sh file assumed if user follows README.md under "Getting started".
- Renamed tracing events after.
- Bug in smacc2 component.
- Reverted markdowns to html.
- Changed wording "smacc application" to "SMACC2 library".
- Reactivating smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Sm_reference_library reformatting.
- Correct trailing spaces.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Minor formatting.
- Add galactic CI setup and rename rolling files.
- Fix source CI and correct README overview.
- Update c_cpp_properties.json.
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.
- Update doxygen links.
- More Readme Updates.
- More Readme.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Feature/aws demo progress.
- Sm_advanced_recovery_1 reworked.
- Fix pre-commit.
- Trying to fix Pre-Commit.
- More sm_advanced_recovery_1 work.
- Sm_atomic_performance_test_a_2.
- Sm_atomic_performance_test_a_1.
- Sm_atomic_performance_test_c_1.
- Modifying sm_atomic_performance_test_a_2.
- Sm_multi_stage_1.
- Fixing precommit.
```

```rst
Section_81
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92, #93, #94, #95, #98)
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- New client behavior: cb_pause_slam (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: cb_pause_slam client behavior

```rst
Section_82
==========

Added
-----

- Feature/sm dance bot strikes back gazebo fixes (#105)
- aws demo (#108)
- Brettpac branch (#110)
- a3 (#113)
- diverse improvements navigation and performance (#116)
- Feature/slam toggle and smacc deep history (#122)
- Move method after the method it calls. Otherwise recursion could happen. (#126)
- Feature/dance bot s pattern (#128)
- First working version of sm template and template generator. (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
- initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)

Changed
-------

- More changes in sm_dance_bot (#125)
- More changes in sm_dance_bot (#129)
- More changes in sm_dance_bot (#135)
- More changes in sm_dance_bot (#152)
- Progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
- Going forward in testing sm_dance_bot introducing slam pausing/resuming functionality
- Progressing in the moveit migration testing
- Progress on move_it PR

Fixed
-----

- Remove neo_simulation2 package. (#112)
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI
- Remove merge markers from a python file. (#119)
- Remove node creation and create only a logger. (#149)
- Fix CI: format fix python version (#148)
- Update package list. (#142)
- Noticed launch command was incorrect in README.md
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
- Update readme (#164)
- More readme updates

Removed
-------

- Remove neo_simulation2 package.
- removing sm_dance_bot_msgs
- removing parameters smacc
- workflows update
- workflow
- removing test from main moveit cmake
- test ur5
- repos dependency
- adding dependency to ur5 client
- docker refactoring
- fixing compiling issues
- more readme updates

Co-Authored-By
--------------

- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_83
==========

Added
-----
- Added QoS durability to SmaccPublisherClient.
- Added reliability QoS configuration.
- Added multistage modes, sequences, steps, and sequences for sm_multi_stage_1.
- Added warehouse2 progress.
- Added Waypoint Inputs.
- Added finetuning waypoints.
- Added pure spinning behavior.
- Added planner changes.
- Added replanning for all examples.
- Added undo motion improvements for navigation in warehouse2.
- Added warehouse2 finishing touches.
- Added minor tune.
- Added fixing warehouse 3 problems and other core improvements.
- Added missing files from warehouse2.
- Added improvements in SMACC core for Autoware demo.
- Added progress in Autoware machine.
- Added refining CP subscriber and CP publisher.
- Added improvements in navigation behaviors.
- Added multiple controllable LEDs plugin.
- Added progress in Husky demo.
- Added progress in barrel Husky.
- Added progress in barrel search.
- Added making models local.
- Added red pickup.
- Added barrel demo.
- Added barrel search updates.
- Added barrel search build fix.
- Added fixing startup problems in warehouse 3.
- Added fixing format issues.
- Added fixing Docker for Foxy and Galactic.
- Added Docker build files for all versions.
- Added barrel demo warnings removal.
- Added more testing on navigation.
- Added more merge.

Changed
-------
- Changed pipeline error.
- Changed broken master build.
- Changed repo dependencies.
- Changed Husky launch file in sm_dance_bot.
- Changed dependencies for Husky in rolling and galactic.
- Changed formatting.
- Changed default values.
- Changed several fixes.
- Changed format issues.
- Changed tuning and fixes.
- Changed backport to Foxy.
- Changed minor linking errors in Foxy.
- Changed minor broken build.
- Changed some reordering fixes.

Fixed
-----
- Fixed a missing colon.
- Fixed broken builds.
- Fixed linting.
- Fixed move trigger logic into headers.
- Fixed missing files in pure spinning behavior.
- Fixed weird MoveIt not downloaded repo.
- Fixed deadlocks in warehouse 3.
- Fixed warnings removal in barrel demo.

Removed
-------
- Removed a line.
- Removed minor configuration.
- Removed weird MoveIt not downloaded repo.
```

```rst
Section_84
==========

Added
-----
- Feature/docker improvements march 2022 (#235)
- Added workflow for checking doc build.
- Added doxygen-deploy.yml for manual deployment.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh for automated installation.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added sm_atomic_performance_trace_1.
- Added galactic CI setup and renamed rolling files. (#58)
- Fixed source CI and corrected README overview. (#62)
- More Readme Updates (#72)
- More Readme (#74)
- Created new sm from sm_respira_1 (#76)
- Base for the sm_aws_aarehouse navigation.
- Progressing in aws navigation.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- More on navigation.
- Reworked sm_advanced_recovery_1.

Changed
-------
- Renamed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Changed wording "smacc application" to "SMACC2 library".
- Updated name of package and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Renamed to smacc2 and smacc2_msgs.
- Updated GitHub branch reference.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Optimized deps in move_base_z_planners_common.
- Renamed event generator library.

Fixed
-----
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Correct formatters.
- Correct formatting of python file.
- Fixed trailing spaces.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
- Deleted tracing directory.
- Removed tracing.md.
- Disabled ament_cpplint.
- Disabled some packages and updated workflows.
- Ignored further packages.
- Ignored all packages except smacc2 and smacc2_msgs.
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Removed disabled packages.

Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_85
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Added new client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98).

Changed
-------
- Updated launch command in README.md.
- Corrected all linters and formatters (#82).

Fixed
----
- Fixed pre-commit issues (#81, #85, #86, #87, #88, #89, #90, #91).
- Resolved compile warnings (#96).
- Fixed formatting in various sections.

Removed
-------
- Removed some compile warnings (#96).
``` 

*pabloinigoblasco*

```rst
Section_86
==========

Added
-----
- New client behavior for nav2: Now waits for nav2 nodes subscribing to the /bond topic and ensures they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.

Changed
-------
- Minor formatting improvements.
- Navigation parameters fixes on sm_dance_bot.
- CB pause slam client behavior.
- Updates yaml.
- Rename doxygen deployment workflow.
- Sm_multi_stage_1 doubling.
- Gazebo fixes for sm_dance_bot_strikes_back.
- Precommit cleanup run.
- Diverse improvements in navigation and performance.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Waypoints navigator bug: Minor tuning to mitigate overshot issue cases.
- Minor navigation improvements.
- Rolling Docker environment to be executed from any environment.

Fixed
-----
- Move method after the method it calls to prevent recursion.
- Fix CI: Format fix python version.

Removed
-------
- Remove neo_simulation2 package.
- Remove node creation and create only a logger.
- Removing parameters smacc.
- Removing sm_dance_bot_msgs.
- Pending references.

Authors
-------
- Pablo Iñigo Blasco (@pabloinigoblasco)
```

```rst
Section_87
==========

Added
-----
- Added .reps dependencies and fixed build errors.
- Added dependency to ur5 client.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added Waypoint Inputs.
- Added default values.
- Added more Waypoints.
- Added pure spinning behavior missing files.
- Added replanning for all examples.
- Added format issues.
- Added tuning and fixes.
- Added improvements in undo motion navigation warehouse2.
- Added fixing warehouse 3 problems and other core improvements.
- Added missing file from warehouse2.
- Added backport to foxy.

Changed
-------
- Updated format.
- Improved dockerfile for building local tests.
- Progressed in the moveit migration testing.
- Progressed on move_it PR.
- Progressed on moveit.
- Progressed on aws navigation and refactorings on navigation clients and behaviors.
- Fine-tuned waypoints.
- Tuned warehouse3.
- Tuned and fixed warehouse2.
- Tuned and fixed minor issues.
- Reworked sm_multi_stage_1.
- Renamed sm_advanced_recovery_1.
- Redid sm_dance_bot_warehouse_3 waypoints.
- Refactored docker.
- Refactored pre-commit cleanup.
- Refactored SrConditional.
- Refactored warehouse2 progress.
- Refactored sm_dance_bot_warehouse_3.
- Refactored smacc core adding more components.
- Refactored autoware machine.
- Refactored cp subscriber and cp publisher.
- Refactored subscriber publisher components.
- Refactored move trigger logic into headers.

Fixed
-----
- Fixed compiling issues.
- Fixed pipeline error.
- Fixed broken master build.
- Fixed broken build.
- Fixed formatting.
- Fixed linting.
- Fixed some formatting and templating on SrConditional.
- Fixed linking errors in foxy CI.
- Fixed minor broken build.
- Fixed some reordering issues.

Removed
-------
- Removed test from main moveit cmake.
- Removed some linting warnings.
- Removed test ur5.
- Removed weird moveit not downloaded repo.
- Removed missing sm.
- Removed some reordering fixes.

Authors
-------
- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
```

```rst
Section_88
==========

Added
-----

- Added docker build files for all versions.
- Added barrel demo.
- Added barrel search build fix and warehouse3.
- Added progress in barrel husky.
- Added multiple controllable leds plugin.
- Added progress in husky demo.
- Added improving navigation behaviors.
- Added more merge in docker improvements.
- Added significant update in Getting Started Instructions.
- Added urdf for ur to support namespaces.
- Added changelogs.

Changed
-------

- Changed format and minor issues in various places.
- Changed to use correct upstream .repos files for source builds.
- Changed mergify branch names.
- Changed name of source-build job and bumped version of action.
- Changed galactic source build job name.
- Changed galactic source build to update .repos file, bump action version, and use correct version of upstream packages.
- Changed rolling build to focal by the moment.
- Changed cache matrix rolling and source build package.
- Changed to ignore packages which should not be released.
- Changed fix urls to index.ros.org.
- Changed foxy source build config to use repos file from foxy branch.
- Changed ros2 control gazebo repo dependency.
- Changed FakeSystem to be working.

Fixed
-----

- Fixed docker for foxy and galactic.
- Fixed startup problems in warehouse 3.
- Fixed missing repo and dependencies.
- Fixed building issues, broken builds, and dependencies.
- Fixed checkout branches for scheduled builds.
- Fixed initialise conditionFlag as false.
- Fixed precommit issues.
- Fixed format issues in multi-ur and more fixes.
- Fixed progress on the multi arm moveit.
- Fixed husky build rolling and type string walker threesome demo.
- Fixed restoring workflow files.
- Fixed restoring files.
- Fixed restoring files for husky project build on rolling.
- Fixed restoring files for ur demos.
- Fixed restoring files for packml example.
- Fixed fixing ur demo.
- Fixed merge in red for focal-rolling.
- Fixed more progress on fake controllers.
- Fixed more progress on multi-ur.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
- Fixed more progress on fake controllers.
-

```rst
Section_89
==========

Added
-----

- Added Packml example (#300) by Pablo Iñigo Blasco
  - Initial commit of packml
  - Progressing substate for start and execute
  - Completing state
  - Finishing state machine
  - Minor changes
  - Fixing break in packml
  - Merging galactic

Changed
-------

- Updated husky_improvements (#299) by Pablo Iñigo Blasco
  - Husky improvements
  - Different planners profiles for navigation
  - Getting changes from galactic
  - Planner switcher
  - Using galactic branch files
  - Fixing breaking changes
  - Minor fix
  - Removing nav from source files
  - Merge

Fixed
-----

- Fixed build on galactic (#297)

- FakeSystem is working (#294)

- Feature/galactic rolling merge (#288)
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs"
  - Updated description table
  - Updated table
  - Copied initial docs
  - Dockerfile with ROS distro as argument
  - Opened new folder for additional tracing contents
  - Deleted tracing directory
  - Moved tracing.md to tracing directory
  - Added setupTracing.sh
    Installs necessary packages and configures tracing group
  - Removed manual installation of ros-rolling-ros2trace
    Now automated in setupTracing.sh
  - Created alternative ManualTracing
  - Added new sm markdowns
  - Added a dockerfile for Rolling and Galactic
  - Changed wording "smacc application" to "SMACC2 library"
  - Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  - Updated tracing/ManualTracing.md
  - Reactivated smacc2 nav clients for rolling via submodules
  - Renamed tracing events
  - Bug fixes in smacc2 component
  - Reverted markdowns to html
  - Added README tutorial for Dockerfile
  - Cleanup and performance improvements
  - Edited tracing.md to reflect new tracing event names
  - Enabled build of missing rolling repositories
  - Enabled Navigation2 for semi-binary build
  - Removed galactic builds from master and kept only rolling
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - More on performance and other issues
  - Format cleanup for sm_respira_1 and sm_respira_test_2
  - Do not execute clang-format on smacc2_sm_reference_library package
  - Reformatting of sm_reference_library
  - Corrected trailing spaces
  - Added sm_atomic_24hr and sm_atomic_performance_trace_1
```

```rst
Section_90
==========

Added
-----
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.

Changed
-------
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links.
- Updated README.md launch command.
- Corrected all linters and formatters.

Fixed
----
- Fixed source CI and corrected README overview. (#62)
- Fixed pre-commit issues.

Removed
-------
- Removed note that was not removed from previous changes.

Other
-----
- Cleaned up sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Navigation parameters fixes on sm_dance_bot.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_91
==========

Added
-----
- New client behavior for nav2: Now waits for nav2 nodes to subscribe to the /bond topic and ensures they are alive. Optional node selection available.
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally checks its contents for success.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Gazebo fixes for sm_dance_bot visualizing turtlebot3.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also smacc2::deep_history syntax.
- Introducing slam pausing/resuming functionality in sm_dance_bot.
- Polishing sm_dance_bot and s-pattern.
- Refinement in sm_dance_bot.
- First working version of sm template and template generator.

Fixed
----
- Minor format fixes.
- Remove some compile warnings.
- Remove neo_simulation2 package.
- Correct formatting.
- Additional linting and formatting.
- Move method after the method it calls to prevent recursion.
- Minor tuning to mitigate overshot issue cases.
- Minor format issues.

Removed
-------
- Removed neo_simulation2 package.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_92
==========

Added
-----

- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added Waypoint Inputs (#178)
- Added more Waypoints to sm_dance_bot_warehouse_3 (#184)
- Added SrConditional fixes and formatting (#168)
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files (#189)
- Added several fixes (#194)

Changed
-------

- Renamed Feature/nav2z to navigation 2 stack (#144)
- Renamed sm_dance_bot_strikes_back launch command in README.md (#148)
- Renamed sm_advanced_recovery_1 to sm_advanced_recovery_1 renaming (#171)
- Reworked sm_multi_stage_1 (#172)
- Refactored Docker environment to be executed from any environment (#154)
- Refactored husky launch file in sm_dance_bot (#174)
- Refactored warehouse2 progress (#179)
- Refactored sm_dance_bot_warehouse_3 (#181)
- Refactored SrConditional (#168)
- Refactored undo motion navigation warehouse2 (#198)

Fixed
-----

- Resolved compile warnings (#137)
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148)
- Fixed CI format for Python version (#148)
- Fixed node creation and created only a logger (#149)
- Fixed compiling issues (#164)
- Fixed broken master build (#167, #174)
- Fixed pipeline error (#167)
- Fixed broken build in AWS navigation (#174)
- Fixed formatting in warehouse2 (#180)
- Fixed minor issues in various features

Removed
-------

- Removed sm_dance_bot_msgs and parameters smacc (#147)
- Removed test from main moveit cmake (#151)
- Removed some comments in the past in README.md (#148)
- Removed line in reliability qos config (#163)
- Removed mode_5_sequence_b, mode_4_sequence_b, and other sequences in sm_multi_stage_1 (#172)
- Removed test workaround in minor dockerfile (#164)
- Removed some linting warnings in moveit migration (#151)
- Removed dependencies in various features

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Brett <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_93
==========

Added
-----

- Feature/warehouse2 22 12 (#200): Implemented minor changes, replanned examples, and fixed format issues.
- Feature/warehouse2 23 12 (#201): Completed warehouse2 functionality.
- tuning and fixes (#202): Made tuning adjustments and fixes.
- Feature/minor tune (#203): Implemented minor tuning changes.
- fixing warehouse 3 problems, and other core improvements (#204): Fixed warehouse 3 issues, improved core functionality, and resolved deadlocks.
- added missing file from warehouse2 (#205): Included a missing file in warehouse2.
- Foxy backport (#206): Backported changes to Foxy, addressing minor formatting issues.
- Enable cppcheck: Enabled cppcheck for code analysis.
- Update ci-build-source.yml: Updated CI build source configuration.
- Update doxygen-check-build.yml: Improved Doxygen build checking workflow.
- Create doxygen-deploy.yml: Established Doxygen deployment process.
- Create workflow for testing prerelease builds: Implemented workflow for testing prerelease versions.
- Update changelogs: Updated changelogs for version 0.1.0.
- Update description table: Enhanced description table.
- Copy initial docs: Duplicated initial documentation.
- Dockerfile w/ ROS distro as argument: Added Dockerfile with ROS distro as an argument for building.
- Opened new folder for additional tracing contents: Created a new folder for tracing contents.
- added setupTracing.sh: Added setup script for configuring tracing.
- Created alternative ManualTracing: Developed an alternative manual tracing method.
- added new sm markdowns: Included new markdown files for SM.
- added a dockerfile for Rolling and Galactic: Added Dockerfile for Rolling and Galactic versions.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh: Updated Docker build script.
- Update tracing/ManualTracing.md: Updated manual tracing instructions.
- changed wording "smacc application" to "SMACC2 library": Updated terminology from "smacc application" to "SMACC2 library".
- Update smacc_sm_reference_library/sm_atomic/README.md: Edited README file for SMACC2 library.
- Enable build of missing rolling repositories: Enabled building missing repositories for Rolling.
- Enable Navigation2 for semi-binary build: Enabled Navigation2 for semi-binary builds.
- Remove galactic builds from master and keep only rolling: Removed Galactic builds from master branch, keeping only Rolling.
- updated mentions of SMACC/ROS to SMACC2/ROS2: Updated references from SMACC/ROS to SMACC2/ROS2.
- some progress on navigation rolling: Made progress on Rolling navigation.
- added smacc2_performance_tools: Added performance tools for SMACC2.
- performance tests improvements: Improved performance testing.
- more on performance and other issues: Addressed additional performance and other issues.
- sm_respira_1 format cleanup: Cleaned up formatting for sm_respira_1.
- sm_respira_test_2: Implemented sm_respira_test_2.
- more changes on performance tests: Made further changes to performance tests.
- sm_reference_library reformatting: Reformatted sm_reference_library.
- sm_atomic_24hr: Implemented sm_atomic_24hr functionality.
- sm_atomic_performance_trace_1: Added sm_atomic_performance_trace_1.
- Update smacc2_rta command across readmes: Updated smacc2_rta command across README files.
- Clean up of sm_atomic_24hr: Conducted cleanup of sm_atomic_24hr.
- Optimized deps in move_base_z_planners_common: Optimized dependencies in move_base_z_planners_common.
- Renaming of event generator library: Renamed the event generator library.
- Add galactic CI setup and rename rolling files. (#58): Added Galactic CI setup and renamed Rolling files.
- Fix source CI and correct README overview. (#62): Fixed source CI and corrected README overview.
- changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69): Updated launch command for sm_respira_1.
- update doxygen links (#70): Updated Doxygen links.
- More Readme Updates (#72): Made additional updates to README files.
- More Readme (#74): Added more content to README files.
- created new sm from sm_respira_1 (#76): Created a new SM from sm_respira_1.
- Feature/core and navigation fixes (#78): Implemented core features and navigation fixes.
- base for the sm_aws_aarehouse navigation: Established the base for AWS warehouse navigation.
- progressing in aws navigation: Made progress in AWS navigation.

Changed
-------

- ros2 launch sm_three_some sm_three_some: Updated launch command for sm_three_some to improve clarity.

Fixed
-----

- Fix trailing spaces: Resolved trailing spaces issue.
- Correct codespell: Fixed codespell errors.
- Correct python linters warnings: Addressed Python linters warnings.
- Add galactic CI build because Navigation2 is broken in rolling: Added Galactic CI build due to Navigation2 issues in Rolling.
- Add partial changes for ament_cpplint: Included partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include: Added tf2_ros as a dependency for include.
- Disable ament_cpplint: Disabled ament_cpplint.
- Disable some packages and update workflows: Disabled certain packages and updated workflows.
- Bump ccache version: Updated ccache version.
- Ignore further packages: Ignored additional packages.
- Satisfy ament_lint_cmake: Ensured compliance with ament_lint_cmake.
- Add missing licences: Included missing licenses.
- Disable cpplint and cppcheck linters: Disabled cpplint and cppcheck linters.
- Correct formatters: Fixed formatting issues.
- Disable disabled packages: Deactivated disabled packages.
- Change extension: Modified file extension.
- Change extension of imports: Adjusted import file extensions.
- Enable cppcheck: Enabled cppcheck for code analysis.
- Correct formatting of python file: Fixed formatting of Python file.
- Included necessary package and edited Threesome launch: Added necessary package and edited Threesome launch command.
- Revert "Ignore all packages except smacc2 and smacc2_msgs": Reverted commit to ignore all packages except smacc2 and smacc2_msgs.

Removed
-------

- Delete tracing directory: Removed tracing directory.
- Moved tracing.md to tracing directory: Transferred tracing.md to tracing directory.
- Removed manual installation of ros-rolling-ros2trace: Automated installation of ros-rolling-ros2trace.
- This is now automated in setupTracing.sh: Tracing installation is now automated in setupTracing.sh.
- location of sh file assumed if user follows README.md under "Getting started": Location of sh file assumed if following README.md instructions.
- reverted markdowns to html: Reverted markdowns to HTML.
- additional cleanup: Performed additional cleanup.
- cleanup: Conducted cleanup operations.
- edited tracing.md to reflect new tracing event names: Updated tracing.md to reflect new event names.
```

```rst
Section_94
==========

Added
-----
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success
- Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait

Changed
-------
- Several core improvements during navigation testing
- Formatting improvements
- Progress in aws navigation demo
- Format improvements
- Navigation parameters fixes on sm_dance_bot

Fixed
----
- Fix pre-commit
- Correct all linters and formatters

Removed
-------
- Trying to fix Pre-Commit

Other
-----
- Merge and progress
- Minor format

Commits
-------
- Feature/aws demo progress (#80)
- sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 work (#85)
- sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- sm_multi_stage_1 (#90)
- More sm_multi_stage_1 (#91)
- Wait topic message client behavior (#81)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_95
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success. (#98)
- New client behavior for nav2: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available. (#98)
- Gazebo fixes for `sm_dance_bot_lite` and `sm_dance_bot_strikes_back`. (#101, #104, #105)
- New feature: `sm_multi_stage_1` doubling. (#103)
- AWS demo improvements. (#108)
- Source build enabled on PR for testing. Adjusted build packages for source CI. (#112)
- Diverse improvements in navigation and performance. (#116, #117)
- Progress in navigation, slam toggle client behaviors, and `smacc2::deep_history` syntax. Introducing slam pausing/resuming functionality. (#122)
- First working version of `sm` template and template generator. (#127)
- Waypoints navigator bug minor tuning to mitigate overshot issue cases. (#133)
- Added SVGs to READMEs of atomic, dance_bot, and others. (#140, #145)
- Added remaining SVGs to READMEs. (#145)
- Added SM Atomic SM generator. (#143)

Changed
-------
- Minor format improvements during navigation testing. (#98)
- Formatting improvements in various sections. (#98)
- Cleaning and lidar show/hide option in `sm_dance_bot_lite` and `sm_dance_bot_strikes_back`. (#102, #104, #105)
- More fixes and refinements in `sm_dance_bot` and `s-pattern`. Typo correction. (#128, #129)
- Method moved after the method it calls to prevent recursion. (#126)
- Minor tweaks and build fixes. (#130, #132)
- Minor navigation improvements. (#141)

Fixed
-----
- Compile warnings removed. (#96)
- Format fixes in various sections. (#98, #104, #105, #134, #148)
- Minor format issues resolved. (#134)
- CI fixed: format fix for Python version. (#148)

Removed
-------
- Removed `neo_simulation2` package. (#112)
- Removed `sm_dance_bot_msgs` package. (#141, #144)
- Removed parameters from `smacc`. (#147)

Collaborators
-------------
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai> in multiple commits.
- Co-authored by pabloinigoblasco <pablo@ibrobotics.com> in diverse improvements commit.
```

Section_96
==========

Added
-----
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability QOS configuration to SmaccPublisherClient

Changed
-------
- Refactored Feature/sm dance bot strikes back (#152)
- Refactored Feature/migration moveit client (#151)
- Refactored Docker environment for execution from any environment (#154)
- Refactored initial migration to smacc2 for fixing errors and missing dependencies
- Refactored move_it PR progress and Dockerfile for building local tests
- Refactored moveit behaviors testing and configurations
- Refactored AWS navigation and navigation clients and behaviors
- Refactored SrConditional for formatting and trigger logic
- Refactored warehouse2 progress and Waypoint Inputs
- Refactored sm_dance_bot_warehouse_3 waypoints and Redoing sm_dance_bot_warehouse_3 waypoints
- Refactored pure spinning behavior for missing files
- Refactored planner changes and replanning for all examples
- Refactored undo motion navigation for warehouse2
- Refactored tuning warehouse3 and undo motion navigation
- Refactored sync and warehouse2 features for format issues and finishing touches
- Refactored minor tune and fixing warehouse 3 problems for continuous integration

Fixed
-----
- Fixed compiling issues and linting warnings
- Fixed broken master build and pipeline errors
- Fixed broken builds and formatting issues
- Fixed errors in warehouse3 and added missing files
- Fixed linking errors for foxy and removed deadlocks
- Fixed weird moveit not downloaded repo issue

Removed
-------
- Removed node creation and created only a logger

Authors
-------
- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- reelrbtx <brett2@reelrobotics.com>

```rst
Section_97
==========

Added
-----
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
- Remove example things from Foxy CI setup. (#214)
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Fix rolling builds (#222)
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
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Use manual deployment for now.
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Update description table.
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- changed wording "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax

Changed
-------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- renamed tracing events after
- bug in smacc2 component
- reverted markdowns to html
- some progress on navigation rolling
- renamed folders, deleted tracing.md, edited README.md
- sm_respira_1 format cleanup
- sm_respira_1 format cleanup pre-commit
- sm_respira_test_2
- sm_respira_test_2
- Do not execute clang-format on smacc2_sm_reference_library package.
- sm_reference_library reformatting
- Correct trailing spaces.
- sm_atomic_24hr
- sm_atomic_performance_trace_1
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library
- minor formatting

Fixed
-----
- minor broken build
- removing warnings (#213)
- minor changes
- replanning for all our examples
- backport to foxy
- minor format
- minor linking errors foxy
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
- updated mentions of SMACC/ROS to SMACC2/ROS2
- more changes on performance tests

Removed
-------
- missing
- missing sm
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- missing
- foxy ci
- fix
- minor broken build

Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```rst
Section_98
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success. Also, added a new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for. (#81, #82, #92, #93, #94)
- Progress in aws navigation demo. (#80, #81, #82, #92, #93, #94)
- Merge and progress in sm_dance_bot. (#94)
- Navigation parameters fixes on sm_dance_bot. (#93)

Changed
-------
- Updated launch command to 'ros2 launch sm_respira_1 sm_respira_1.launch'. (#69)
- Updated doxygen links. (#70)
- Minor formatting improvements. (#78, #80, #81, #82, #92, #93, #94)
- Several core improvements during navigation testing. (#78, #80, #81, #82, #92, #93, #94)
- Progress in aws navigation. (#78, #80, #81, #82, #92, #93, #94)
- Reworked sm_advanced_recovery_1. (#83, #84, #85, #86)
- Modified sm_atomic_performance_test_a_2. (#89)
- Fixed pre-commit issues. (#83, #84, #85, #86, #89)
- Corrected all linters and formatters. (#82)

Removed
-------
- Note that was not removed while producing changes. (#69)
```

```rst
Section_99
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- Gazebo fixes for showing the robot and the lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- AWS demo progress.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also `smacc2::deep_history` syntax.
- First working version of `sm_template` and template generator.
- Minor tuning to mitigate overshot issue cases.
- Minor navigation improvements.
- Added SM core test.
- Using local action messages.
- Pending references.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Formatting improvements.
- Minor format adjustments.
- Cleaning and lidar show/hide option.
- More fixes in various areas.
- Polishing `sm_dance_bot` and `s-pattern`.
- Noticed typo correction.
- Minor tweaks.
- Build fix.
- Waypoints navigator bug addressed.
- Progress in `sm_dance_bot` tests.
- Some more progress on markers cleanup.
- Minor format issues.

Fixed
-----
- Remove some compile warnings (#96).
- Remove `neo_simulation2` package (#112).
- Remove merge markers from a Python file (#119).
- Move method after the method it calls to prevent recursion (#126).

Removed
-------
- Removed `sm_dance_bot_msgs`.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

# Section_100

## Added
- Renamed navigation 2 stack.
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient (#163).
- Added durability to SmaccPublisherClient.
- Added reliability QoS config.

## Changed
- Updated package list (#142).
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Refactored feature "sm dance bot strikes back" (#152).
- Refactored AWS navigation and navigation clients and behaviors.
- Refactored warehouse2.
- Redone sm_dance_bot_warehouse_3 waypoints.
- Finetuned waypoints (#187).

## Fixed
- Fixed CI: format fix python version (#148).
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed broken build.

## Removed
- Removed parameters smacc.
- Removed test from main moveit CMake.

## Miscellaneous
- Precommit cleanup.
- Updated format.
- Updated format in README (#164).
- Updated README.
- Updated README with more information.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated format

```rst
Section_101
===========

Added
-----

- Added missing file from warehouse2 (#205)
- Added dockerfiles (#225)
- Added Feature/retry behavior warehouse 1 (#226)
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Added setupTracing.sh to install necessary packages and configure tracing group
- Added alternative ManualTracing
- Added README tutorial for Dockerfile
- Added smacc2_performance_tools

Changed
-------

- Changed wording "smacc application" to "SMACC2 library"
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- Changed extension of imports
- Changed extension of imports
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
- Changed wording "smacc application" to "SMACC2 library"
-

```rst
Section_102
===========

Added
-----

- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.
- Added navigation parameters fixes on sm_dance_bot.

Changed
-------

- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated README.md with launch command.
- Updated smacc2_rta command across readmes.
- Renamed event generator library.

Fixed
-----

- Fixed source CI and corrected README overview. (#62).
- Corrected trailing spaces.
- Corrected all linters and formatters.

Removed
-------

- Do not execute clang-format on smacc2_sm_reference_library package.

Other
-----

- Performance tests improvements.
- More changes on performance tests.
- Optimized dependencies in move_base_z_planners_common.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Progressing in aws navigation.
- Format improvements.
- More on navigation.
- More on performance and other issues.
- Noticed a note that was not removed while producing these changes.
- Attempting pre-commit fixes.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

pabloinigoblasco
```

```rst
Section_103
===========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for Nav2: `add` for waiting Nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Nodes to wait can be optionally selected.
- New gazebo fixes for showing the robot and the lidar in `sm_dance_bot_strikes_back`.

Changed
-------
- Progress in AWS navigation demo.
- Minor format improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Cleaning and lidar show/hide option in `sm_dance_bot visualizing turtlebot3`.

Fixed
-----
- Remove some compile warnings.
- Correct formatting in removing `neo_simulation2` package.
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Several core improvements during navigation testing.
- Formatting improvements.
- Merge and progress.
- Base for the `sm_aws_aarehouse` navigation.
- Precommit cleanup run.
- Updates yaml.
- Enable source build on PR for testing.
- Additional linting and formatting.
- Remove merge markers from a Python file.
- First working version of `sm` template and template generator.
- Noticed typo correction.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- Going forward in testing `sm_dance_bot` introducing slam pausing/resuming functionality.
- Polishing `sm_dance_bot` and `s-pattern`.
- More refinement in `sm_dance_bot`.
- Finally > Finally.
- Minor tweaks.
- More on navigation.
- Keep hammering.
- Two stages.
- 3 part.
- 4th stage.
- 5th stage.
- Gaining traction `sm_multi_stage_1`.
- More.
- Don't remember.
- Making progress.
- Diverse improvements in navigation and performance.
``` 

*pabloinigoblasco*

Section_104
------------

Added
-----
- Feature/sm dance bot refine (#131)
  - More changes in sm_dance_bot
  - Minor
- Feature/sm dance bot refine 2 (#132)
  - More changes in sm_dance_bot
  - Minor
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
  - Some more progress on markers cleanup
- Sm_dance_bot_lite (#136)
  - Resolve compile warnings (#137)
  - Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
  - Removing sm_dance_bot_msgs
- Feature/nav2z renaming (#144)
  - Using local action messages
  - Removing sm_dance_bot_msgs
  - Navigation 2 stack renaming
  - Formatting
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Precommit cleanup
- Update package list (#142)
- Removing parameters smacc (#147)
  - Workflows update
- Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Fix CI: format fix python version (#148)
- Add SM Atomic SM generator (#143)
- Remove node creation and create only a logger (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing some errors introduced on formatting
  - Missing dependency
  - Fixing some more linting warnings
  - Removing test from main moveit cmake
  - Test ur5
  - Progressing in the moveit migration testing
  - Updating format
  - Adding .reps dependencies and also fixing some build errors
  - Repos dependency
  - Adding dependency to ur5 client
  - Docker refactoring
  - Progress on move_it PR
  - Minor dockerfile test workaround
  - Improving dockerfile for building local tests
  - Fixing compiling issues
- Update readme (#164)
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Add QOS durability to SmaccPublisherClient (#163)
  - Feat: add qos durability to SmaccPublisherClient
  - Fix: add a missing colon
  - Refactor: remove line
  - Feat: add reliability qos config
- Feature/testing moveit behaviors (#167)
  - More testing on moveit
  - Progress on moveit
  - More testing on moveit behaviors
  - Minor configuration
  - Fixing pipeline error
  - Fixing broken master build
- Sm_pubsub_1 (#169)
- Sm_pubsub_1 part 2 (#170)
- Sm_advanced_recovery_1 renaming (#171)
- Sm_multi_stage_1 reworking (#172)
  - Multistage modes
  - Sm_multi_stage sequences
  - Sm_multi_state_1 steps
  - Sm_multi_stage_1 sequence d
  - Sm_multi_stage_1 c sequence
  - Mode_5_sequence_b
  - Mode_4_sequence_b
  - Sm_multi_stage_1 most
  - Finishing touches 1
  - Readme
- Feature/aws navigation sm dance bot (#174)
  - Repo dependency
  - Husky launch file in sm_dance_bot
  - Add dependencies for husky simulation
  - Fix formatting
  - Progress on aws navigation and some other refactorings on navigation clients and behaviors
  - More on aws demo
  - Fixing broken build
- Minor changes (#175)
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Warehouse2 progress (#179)
- Format (#180)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
  - Format
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
- Brettpac branch (#184)
  - Sm_dance_bot_warehouse_3
  - Redoing sm_dance_bot_warehouse_3 waypoints
  - More waypoints
- SrConditional fixes and formatting (#168)
  - Fix: some formatting and templating on SrConditional
  - Fix: move trigger logic into headers
  - Fix: lint
- Feature/wharehouse2 dec 14 (#185)
  - Warehouse2
- Feature/sm warehouse 2 13 dec 2 (#186)
  - Format
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
  - Format
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
- Feature/cb pure spinning (#189)
  - Format
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
  - Pure spinning behavior missing files
- Minor changes (#190)
- Feature/planner changes 16 12 (#191)
  - Minor changes
  - More fixes
- Feature/replanning 16 dec (#193)
  - Minor changes
  - Replanning for all our examples
- Several fixes (#194)
- Minor changes (#195)

Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_105
===========

Added
-----

- Feature/undo motion 20 12 (#196)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/improvements warehouse3 (#228)
- Foxy backport (#206)
- Added missing file from warehouse2 (#205)
- Update file for fake hardware simulation and add file for gazebo simulation. (#224)
- Add ignition file and update repos files.
- Added docker files for different revisions, warnings removal, and more testing on navigation.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------

- Minor changes in various features.
- Replanning for all examples.
- Improvements in undo motion navigation warehouse2.
- Tuning warehouse3 (#197).
- Tuning and fixes (#202).
- Fix broken source build (#227).
- Only rolling version should be pre-released on master. (#230).
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234).
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licenses.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.
- Correct formatting of python file.
- Included necessary package and edited Threesome launch.
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Update name of package and package.xml to pass linter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Ignore all packages except smacc2 and smacc2_msgs.
- Update changelogs.
- Revert "Ignore all packages except smacc2 and smacc2_msgs".
- Update description table.
- Update table.
- Copy initial docs.
- Dockerfile w/ ROS distro as argument.
- Opened new folder for additional tracing contents.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.

Fixed
-----

- Fixing warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green.
- Fix minor broken build.
- Some reordering fixes.
- Minor format fix.
- Minor linking errors in foxy.
- Minor formatting fixes.
- Fix broken source build.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Fix minor format.
- Fix broken source build.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Fix minor format.

Removed
-------

- Weird moveit not downloaded repo.
- Missing files.
- Missing sm.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
- Missing file.
-

```rst
Section_106
===========

Added
-----
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.
- Optimized dependencies in move_base_z_planners_common.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Fixed formatting issues.
- Fixed pre-commit issues.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.

Other
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Made some progress on navigation rolling.
- Made progress in aws navigation.
- Made several core improvements during navigation testing.
- Cleaned up smacc2_sm_reference_library package.
- Reformatted sm_reference_library.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_atomic_performance_trace_1.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_advanced_recovery_1.
- Made more changes on performance tests.
- Made more sm_atomic_24hr cleanup.
- Made more sm_advanced_recovery_1 work.
- Modified sm_atomic_performance_test_a_2.
- Modified sm_atomic_performance_test_a_2.
- Modified sm_atomic_performance_test_c_1.
- Modified sm_multi_stage_1.
- Modified sm_multi_stage_1.
- Modified sm_atomic_performance_test_a_2.
- Modified sm_atomic_performance_test_a_2.
- Modified sm_multi_stage_1.
- Modified README.md.
- Attempted pre-commit fixes.
- Corrected README overview.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.
- Corrected README formatting.

```rst
Section_107
===========

Added
-----

- New client behavior for nav2: now waits for nav2 nodes subscribing to the /bond topic to be alive, with optional node selection.
- Progress in AWS navigation demo.
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Merge and progress.
- New client behavior `cb_pause_slam`.
- Visualizing TurtleBot3 in `sm_dance_bot_lite`.
- Choice to launch Gazebo with lidar in `sm_dance_bot`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- AWS demo improvements.
- Removed `neo_simulation2` package.

Changed
-------

- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Format fixes for Gazebo to show the robot and lidar.

Fixed
-----

- Compile warnings removed.
- `sm_multi_stage_1` improvements.

Removed
-------

- Unused `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_108

### Added
- Additional linting and formatting.
- Introduced feature for slam toggle and smacc deep history (#122).
- Implemented progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also included smacc2::deep_history syntax.
- Added functionality for testing sm_dance_bot with slam pausing/resuming.
- First working version of sm template and template generator (#127).
- Added SM Atomic SM generator (#143).
- Rolling Docker environment to be executed from any environment (#154).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added QOS durability to SmaccPublisherClient (#163).
- Introduced feature for testing moveit behaviors (#167).
- Added husky launch file in sm_dance_bot for AWS navigation (#174).
- Implemented waypoint inputs (#178).

### Changed
- Moved method after the method it calls to prevent recursion (#126).
- Renamed state machine transition timestamp to smacc2_performance_tools (#166).
- Refactored SM dance bot strikes back (#152).
- Renamed sm_advanced_recovery_1 (#171).
- Reworked sm_multi_stage_1 with multistage modes and sequences (#172).
- Updated dependencies for husky in rolling and galactic for AWS navigation (#174).

### Fixed
- Resolved compile warnings (#137).
- Fixed CI format for Python version (#148).
- Fixed launch command in README.md.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.

### Removed
- Removed merge markers from a Python file (#119).
- Removed node creation and created only a logger (#149).
- Removed parameters smacc (#147).
- Removed test from main moveit CMake.

### Authors
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_109
===========

Added
-----
- Feature/wharehouse2 dec 14 (#185)
  - Implemented warehouse2 feature with minor changes.

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added formatting improvements and headless mode to warehouse2.

- Feature/replanning 16 dec (#193)
  - Improved replanning for all examples with several fixes.

- Feature/undo motion 20 12 (#196)
  - Enhanced undo motion navigation for warehouse2.

- Feature/sync 21 12 (#199)
  - Addressed format issues for better synchronization.

- Feature/warehouse2 22 12 (#200)
  - Resolved format issues and finalized warehouse2.

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2 for better performance.

- Feature/minor tune (#203)
  - Fine-tuned and fixed minor issues.

- Feature/barrel - do not merge yet (#233)
  - Made minor changes and backported to foxy.

Changed
-------
- Fixed trailing spaces, codespell, and Python linters warnings in Foxy backport (#206).
- Updated CI build for galactic due to Navigation2 issues in rolling.
- Made various adjustments to satisfy linting requirements and improve code quality.

Removed
-------
- Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61 to reset all versions to 0.0.0.

Co-Authored-By
--------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

Section 110
===========

Added
-----
- Opened new folder for additional tracing contents.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Moved tracing.md to tracing directory.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.

Fixed
-----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Minor formatting fixes.
- Several core improvements during navigation testing.
- Formatting improvements.
- More on performance tests.
- Format improvements.
- Pre-commit fixes.

Removed
-------
- Manual installation of ros-rolling-ros2trace (now automated in setupTracing.sh).
- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Commits
-------
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh (#58).
- Fix source CI and correct README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Feature/aws demo progress (#80).
- Wait topic message client behavior (#81).
- Feature/wait nav2 nodes client behavior (#82).
- sm_advanced_recovery_1 reworked (#83).
- More sm_advanced_recovery_1 work (#85).
- sm_atomic_performance_test_c_1 (#88).
- modifying sm_atomic_performance_test_a_2 (#89).
- sm_multi_stage_1 (#90).
- Update README.md.

```rst
Section_111
===========

Added
-----

- Feature/aws demo progress (#92)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Added new feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for Nav2: `wait nav2 nodes subscribing to the /bond topic and waiting they are alive`.
  - Continued progress in AWS navigation demo.

- Feature/sm dance bot fixes (#93)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Fixed navigation parameters on sm_dance_bot.

- Feature/sm aws warehouse (#94)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Continued progress in AWS navigation demo.
  - Merged and made progress.

- Feature/sm dance bot fixes (#95)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Fixed navigation parameters on sm_dance_bot.

- Feature/cb pause slam (#98)
  - Implemented base for the sm_aws_aarehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Added new feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for Nav2: `wait nav2 nodes subscribing to the /bond topic and waiting they are alive`.
  - Continued progress in AWS navigation demo.
  - Fixed navigation parameters on sm_dance_bot.
  - Implemented `cb pause slam` client behavior.

- Feature/sm dance bot lite gazebo fixes (#104)
  - Improved visualization of turtlebot3.
  - Added cleaning and lidar show/hide option.
  - Made formatting improvements.
  - Fixed gazebo issues to show the robot and the lidar.

- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Improved visualization of turtlebot3.
  - Added cleaning and lidar show/hide option.
  - Made formatting improvements.
  - Fixed gazebo issues for sm_dance_bot_strikes_back.

- AWS demo (#108)
  - Implemented AWS demo.

- Brettpac branch (#110)
  - Made progress on sm_multi_stage_1.
  - Gained traction on sm_multi_stage_1.
  - Continued development on multiple stages.

- Brettpac branch (#111)
  - Made progress on sm_multi_stage_1.
  - Gained traction on sm_multi_stage_1.
  - Continued development on multiple stages.

- a3 (#113)

Changed
-------

- Minor hotfixes.

Fixed
-----

- Corrected all linters and formatters.
- Removed some compile warnings.

Removed
-------

- Unused code.

Collaborators
-------------

- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_112

### Added
- Enable source build on PR for testing. (#112)
- Added SM core test. (#138)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient. (#163)
- Added AWS navigation SM Dance Bot. (#174)

### Changed
- Adjusted build packages of source CI. (#114)
- Progress in navigation, SLAM toggle client behaviors, and SLAM toolbox components. Also SMACC2::deep_history syntax. (#122)
- Progress in testing SM Dance Bot introducing SLAM pausing/resuming functionality. (#122)
- Progress in the MoveIt migration testing. (#151)
- Moved reference library SMs to smacc2_performance_tools. (#166)
- Reworked SM Multi-Stage 1. (#172)

### Fixed
- Corrected formatting. (#112)
- Removed merge markers from a Python file. (#119)
- Fixed CI: format fix Python version. (#148)
- Fixed launch command for SM Dance Bot Strikes Back and removed some comments. (#149)
- Fixed compile warnings. (#137)
- Fixed minor navigation improvements. (#141)
- Fixed minor format issues. (#134)
- Fixed minor tuning to mitigate overshot issue cases. (#133)
- Fixed broken master build. (#167)
- Fixed broken build in AWS navigation. (#174)

### Removed
- Removed Neo Simulation2 package. (#112)
- Removed parameters SMACC. (#147)
- Removed node creation and create only a logger. (#149)
- Removed SM Dance Bot messages. (#144)

### Miscellaneous
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, pabloinigoblasco <pablo@ibrobotics.com>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, Denis Štogl <denis@stogl.de>.

```rst
Section_113
===========

Added
-----

- Added "minor changes" (#175).
- Added "warehouse2" (#177).
- Added "Waypoint Inputs" (#178).
- Added "wharehouse2 progress" (#179).
- Added "format" (#180).
- Added "sm_dance_bot_warehouse_3" (#181).
- Added "Feature/sm warehouse 2 13 dec 2" (#182).
- Added "more changes and headless", "merge", "headless and other fixes", "default values", and "Brettpac branch" (#184).
- Added "Redoing sm_dance_bot_warehouse_3 waypoints" and "More Waypoints" (#186).
- Added "finetuning waypoints" (#187).
- Added "Feature/cb pure spinning" (#188).
- Added "pure spinning behavior missing files" and "minor changes" (#190).
- Added "Feature/planner changes 16 12" (#191).
- Added "Feature/replanning 16 dec" (#193).
- Added "several fixes" (#194).
- Added "Feature/undo motion 20 12" (#196).
- Added "improving undo motion navigation warehouse2", "minor", and "tuning warehouse3" (#197).
- Added "undo tuning and errors", "format", and "Feature/sync 21 12" (#198).
- Added "Feature/warehouse2 22 12" (#200).
- Added "finishing warehouse2" and "Feature/warehouse2 23 12" (#201).
- Added "tuning and fixes" (#202).
- Added "Feature/minor tune" (#203).
- Added "fixing warehouse 3 problems, and other core improvements" (#204).
- Added "added missing file from warehouse2" (#205).
- Added "Foxt backport" (#206).
- Added various fixes and improvements related to CI and linting.

Changed
-------

- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch".
- Renamed header files and corrected format.
- Added workflow for checking doc build.

Removed
-------

- Removed trailing spaces.
- Removed codespell errors.
- Removed python linters warnings.
- Removed ament_cpplint.
- Removed disabled packages and updated workflows.
- Removed cpplint and cppcheck linters.
- Removed further packages.
- Removed disabled packages.
- Removed unnecessary extensions.
- Removed unnecessary imports.

Co-Authored-By
--------------

- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```rst
Section_114
===========

Added
-----

- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Update description table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Added smacc2_performance_tools
- Performance tests improvements
- Optimized deps in move_base_z_planners_common
- Renaming of event generator library
- Add galactic CI setup and rename rolling files
- Fix source CI and correct README overview
- Update c_cpp_properties.json
- Update README.md
- Update launch command to ros2 launch sm_respira_1 sm_respira_1.launch
- Update doxygen links
- More Readme Updates
- Created new sm from sm_respira_1
- Feature/core and navigation fixes
- Feature/aws demo progress
- More on navigation
- Sm_advanced_recovery_1 reworked
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1
- Sm_multi_stage_1
- Wait topic message client behavior

Changed
-------

- Use manual deployment for now
- Correct GitHub branch reference
- Execute on master update
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Changed wording "smacc application" to "SMACC2 library"
- Reactivating smacc2 nav clients for rolling via submodules
- Reverted markdowns to html
- Edited tracing.md to reflect new tracing event names
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- Minor formatting
- Several core improvements during navigation testing
- Progress in aws navigation demo
- More on performance and other issues
- Do not execute clang-format on smacc2_sm_reference_library package
- Sm_reference_library reformatting
- Correct trailing spaces
- More sm_atomic_24hr cleanup
- More sm_advanced_recovery_1 work
- Fixing precommit

Removed
-------

- Ignore all packages except smacc2 and smacc2_msgs
- Removed manual installation of ros-rolling-ros2trace
- Delete tracing directory
- Removed tracing.md
- Removed submodules and use .repos file
- Deleted tracing.md
- Deleted tracing directory

Authors
-------

- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_115
===========

Added
-----
- Introduce new feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for nav2: `wait nav2 nodes` subscribes to the `/bond` topic and waits for them to be alive, with optional node selection.

Changed
-------
- Correct all linters and formatters.

Fixed
-----
- Resolve navigation parameters issues on `sm_dance_bot`.
- Address minor format fixes.
- Remove some compile warnings.

Removed
-------
- No items removed.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

Details
-------
- Progress in AWS navigation demo.
- Several core improvements during navigation testing.
- Formatting enhancements.
- Merge and progress.
- Precommit fixes.
- Updates yaml.
- Minor hotfix.
- Cleaning and lidar show/hide option.
- Cleaning files and making formatting work.
- More fixes.
- Gazebo fixes to show the robot and the lidar.
- Doubling in `sm_multi_stage_1`.
```

```rst
Section_116
===========

Added
-----
- Added AWS demo (#108, #110).
- Added Brettpac branch (#110, #111).
- Added a3 (#113).
- Added diverse improvements in navigation and performance (#116).
- Added more sm_multi_stage_1 improvements (#114).
- Added Feature/slam toggle and smacc deep history (#122).
- Added Feature/dance bot s pattern (#128, #129).
- Added First working version of sm template and template generator (#127).
- Added Feature/sm dance bot refine (#131, #132).
- Added waypoints navigator bug (#133).
- Added SM core test (#138).
- Added minor navigation improvements (#141).
- Added using local action messages (#139).
- Added Resolve compile warnings (#137).
- Added Add SM Atomic SM generator (#143).
- Added Rolling Docker environment to be executed from any environment (#154).
- Added initial state machine transition timestamp (#165).
- Added Add QOS durability to SmaccPublisherClient (#163).
- Added Feature/testing moveit behaviors (#167).
- Added sm_pubsub_1 (#169).
- Added sm_pubsub_1 part 2 (#170).

Changed
-------
- Improved gazebo fixes for sm_dance_bot_strikes_back.
- Improved sm_multi_stage_1 functionality (#109, #111).
- Improved navigation, slam toggle client behaviors, and slam_toolbox components (#122).
- Improved dance bot s pattern and sm_dance_bot (#128, #129).
- Improved sm_dance_bot_lite (#136).
- Improved minor tweaks (#130).
- Improved minor format issues (#134).
- Improved minor tuning to mitigate overshot issue cases (#133).
- Improved progress in the sm_dance_bot tests (#135).
- Improved minor format fix python version (#148).
- Improved removing node creation and create only a logger (#149).
- Improved progress on moveit migration testing (#151).
- Improved progress on move_it PR (#151).
- Improved updating format (#151).
- Improved adding .reps dependencies and fixing build errors (#151).
- Improved adding dependency to ur5 client (#151).
- Improved docker refactoring (#151).
- Improved progress on moveit (#151).
- Improved fixing compiling issues (#151).
- Improved update readme (#164).
- Improved moving reference library SMs to smacc2_performance_tools (#166).
- Improved feat: add qos durability to SmaccPublisherClient (#163).
- Improved fix: add a missing colon (#163).
- Improved refactor: remove line (#163).
- Improved feat: add reliability qos config (#163).

Removed
-------
- Removed neo_simulation2 package (#112).
- Removed sm_dance_bot_msgs (#144).
- Removed parameters smacc (#147).
- Removed test from main moveit cmake (#151).

Fixed
-----
- Fixed recursion issue by moving method after the method it calls (#126).
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed CI format issues (#148).
- Fixed broken master build (#167).

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> in multiple commits.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com> in diverse improvements navigation and performance (#116).
- Co-authored-by: DecDury <declandury@gmail.com> in Feature/sm dance bot strikes back refactoring (#152).
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com> in Feature/sm dance bot strikes back refactoring (#152).
```

```rst
Section_117
===========

Added
-----

- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency
  - Added husky launch file in sm_dance_bot
  - Added dependencies for husky simulation
  - Fixed formatting
  - Updated dependencies for husky in rolling and galactic
  - Improved progress on aws navigation and refactorings on navigation clients and behaviors
  - Added more on aws demo
  - Fixed broken build

- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2
  - Minor changes

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added more changes and headless
  - Merged changes
  - Added headless and other fixes
  - Set default values
  - Fine-tuned waypoints

Changed
-------

- SrConditional fixes and formatting (#168)
  - Improved formatting and templating on SrConditional
  - Moved trigger logic into headers
  - Linted code

- Feature/undo motion 20 12 (#196)
  - Made minor changes
  - Improved undo motion navigation in warehouse2
  - Tuned warehouse3

- Feature/sync 21 12 (#199)
  - Made minor changes
  - Fixed format issues

- Feature/warehouse2 22 12 (#200)
  - Made minor changes
  - Fixed format issues
  - Finished warehouse2

- Feature/warehouse2 23 12 (#201)
  - Made minor changes
  - Tuned and fixed issues

- Feature/minor tune (#203)
  - Tuned and fixed minor issues

- Update galactic source build job name (#250)

Fixed
-----

- Use correct upstream .repos files for source builds (#243)
- Correct mergify branch names (#246)
- Correct name of source-build job and bump version of action (#242) (#247)

Removed
-------

- Nothing

Authors
-------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
```

# Changelog

## [Unreleased]

### Added
- Galactic source build: updated .repos file, bumped action version, and used correct version of upstream packages (backport #241) (#248) [Denis Štogl]
- Cache matrix rolling and source build package
- Added spawn entity delays
- Added changelogs
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Added smacc2_performance_tools
- Performance tests improvements
- More changes on performance tests
- Do not execute clang-format on smacc2_sm_reference_library package
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library
- Added galactic CI setup and renamed rolling files (#58)
- Fix source CI and correct README overview (#62)
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Updated doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)
- Created new sm from sm_respira_1 (#76)
- Base for the sm_aws_aarehouse navigation
- Progressing in AWS navigation
- Several core improvements during navigation testing
- Formatting improvements
- Progress in AWS navigation demo
- More on navigation
- Reworked sm_advanced_recovery_1 (#83)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1 (#88)
- Sm_multi_stage_1
- Fixing precommit

### Changed
- Updated foxy-source-build.yml
- Feature/fixing husky build rolling (#257)
- Feature/fixing husky build rolling (#258)
- Updated smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- More sm_atomic_24hr cleanup
- Sm_reference_library reformatting
- Minor formatting

### Fixed
- Restoring workflow files (#252)
- Restoring files (#253)
- Fix checkout branches for scheduled builds (#254)
- Fix: initialise conditionFlag as false (#274)
- Precommit fix (#280)
- Fix urls to index.ros.org (#284)
- Fix foxy source build config to use repos file from foxy branch (#285)
- Fixing sm_dance_bot examples
- Working on fix of image messages for husky_barrel demo

### Removed
- Remove trailing spaces

### Miscellaneous
- Significant update in Getting Started Instructions (#269)
- Revert "Ignore packages which should not be released" [Commit: ee2cc86db3c0a24f9eb0a9e33217de3f7a691a1c]
- Revert "Ignore packages which should not be released" [Commit: dec14a936a877b2ef722a6a32f1bf3df09312542]

## [0.4.0] - YYYY-MM-DD

- Initial release

[pabloinigoblasco]

```rst
Section_119
===========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92, #93, #94, #95, #98)
- New client behavior for nav2: `wait nav2 nodes subscribing to the /bond topic and waiting they are alive`, with optional node selection (#82, #92, #93, #94, #95, #98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `cb pause slam` for pausing SLAM operations (#98)
- New client behavior: `

```rst
Section_120
===========

Added
-----

- Added lidar show/hide option for cleaning.
- Added gazebo fixes to display the robot and lidar.
- Added AWS demo (#108).
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added smacc2::deep_history syntax for state machines.
- Added waypoints navigator bug fix (#133).
- Added SM core test (#138).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient (#163).

Changed
-------

- Improved formatting for cleaning files.
- Improved gazebo fixes for sm_dance_bot_strikes_back.
- Improved navigation and performance.
- Refactored sm_dance_bot strikes back (#105) and (#152).
- Refactored migration to smacc2 (#151).
- Updated package list (#142).
- Renamed navigation 2 stack.
- Updated READMEs.
- Updated Docker environment for execution from any environment.
- Updated format for CI.
- Updated README (#164).

Fixed
-----

- Fixed formatting issues.
- Fixed recursion possibility in method calls.
- Fixed overshot issue cases in navigation.
- Fixed compile warnings.
- Fixed launch command in README.md.
- Fixed CI format for Python version.
- Fixed node creation in SMs.
- Fixed waypoint 4 and iterations for robot course completion.
- Fixed errors introduced in migration formatting.
- Fixed linting warnings.
- Fixed compiling issues.

Removed
-------

- Removed neo_simulation2 package.
- Removed merge markers from a Python file.
- Removed parameters from smacc.
- Removed test from main moveit CMake.
- Removed test from main moveit CMake.
- Removed node creation and created only a logger.

Authors
-------

- Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_121
===========

Added
-----
- Added qos durability to SmaccPublisherClient.
- Added reliability qos config.
- Added multistage modes, sequences, steps, and most finishing touches.
- Added Waypoint Inputs.
- Added dependencies for husky simulation.
- Added warehouse2 progress.
- Added format issues.
- Added tuning and fixes.
- Added minor tune.
- Added backport to foxy.
- Added galactic CI build.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency.
- Added workflow for checking doc build.
- Added workflow for testing prerelease builds.
- Added manual deployment for now.
- Added docs/ as source and output directory.
- Added branching example.
- Added necessary package and edited Threesome launch.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added doxygen-check-build.yml.
- Added ros-rolling-ros2trace installation command.

Changed
-------
- Removed a missing colon.
- Removed a line.
- Refactored moveit behaviors.
- Refactored moveit behaviors pipeline.
- Refactored SrConditional formatting.
- Refactored move trigger logic into headers.
- Refactored lint.
- Refactored repo dependency formatting.
- Refactored husky launch file in sm_dance_bot.
- Refactored dependencies for husky in rolling and galactic.
- Refactored aws navigation and some other refactorings.
- Refactored aws demo.
- Refactored warehouse3 tuning.
- Refactored undo motion navigation warehouse2.
- Refactored warehouse2 finishing.
- Refactored warehouse3 problems and core improvements.
- Refactored warehouse2 format issues.
- Refactored warehouse2 tuning and fixes.
- Refactored warehouse2 minor tune.
- Refactored warehouse2 continuous integration.
- Refactored warehouse2 dead lock removal.
- Refactored warehouse2 weird moveit not downloaded repo.
- Refactored warehouse2 missing file addition.
- Refactored warehouse2 minor format.
- Refactored warehouse2 minor linking errors.
- Refactored warehouse2 extension changes.
- Refactored warehouse2 cppcheck enablement.
- Refactored warehouse2 python file formatting.
- Refactored warehouse2 necessary package inclusion.
- Refactored warehouse2 package and package.xml name update.
- Refactored warehouse2 master update execution.
- Refactored warehouse2 version reset to 0.0.0.
- Refactored warehouse2 package ignore.
- Refactored warehouse2 branch reference correction.

Fixed
-----
- Fixed pipeline error.
- Fixed broken master build.
- Fixed broken build.
- Fixed formatting.
- Fixed several issues.
- Fixed pure spinning behavior missing files.
- Fixed warehouse3 problems.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed formatters.
- Fixed warehouse3 problems and core improvements to remove dead lock.
- Fixed warehouse2 missing file addition.
- Fixed warehouse2 minor format.
- Fixed warehouse2 minor linking errors.
- Fixed warehouse2 extension changes.
- Fixed warehouse2 cppcheck enablement.
- Fixed warehouse2 python file formatting.
- Fixed warehouse2 necessary package inclusion.
- Fixed warehouse2 package and package.xml name update.
- Fixed warehouse2 version reset to 0.0.0.
- Fixed warehouse2 package ignore.
- Fixed warehouse2 branch reference correction.

Removed
-------
- Removed minor configuration.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
- Removed minor changes.
- Removed minor.
-

```rst
Section_122
===========

0.1.0 (2022-01-01)
------------------

Added
-----
- Initial documentation setup with description table and basic docs.
- Dockerfile now accepts ROS distro as an argument for flexibility.
- New folder created for tracing contents.
- Script `setupTracing.sh` added for automated installation of necessary packages.
- Alternative manual tracing method `ManualTracing` created.
- New markdown files added for SMACC2 library.
- Performance tools and tests improvements.
- Navigation improvements for rolling release.
- New feature `cb_wait_topic_message` for asynchronous client behavior.

Changed
-------
- Renamed `smacc application` to `SMACC2 library`.
- Updated references from SMACC/ROS to SMACC2/ROS2.
- Various wording improvements in documentation.

Fixed
----
- Bug fixes in `smacc2 component`.
- Trailing spaces corrected.
- Formatting issues in various files resolved.

Removed
-------
- Manual installation of `ros-rolling-ros2trace` now automated in `setupTracing.sh`.
- Galactic builds removed, keeping only rolling release.
- Unused `tracing.md` file deleted.

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_123
===========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` behavior waits for `nav2` nodes to subscribe to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- `cb_pause_slam` client behavior.

Changed
-------
- Corrected all linters and formatters.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor format improvements.
- Progress in AWS navigation demo.
- Formatting improvements.

Fixed
-----
- Removed some compile warnings.

Removed
-------
- Redundant format improvements entries.

Collaborators
-------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Features
--------
- Progress in AWS navigation.
- Base for the `sm_aws_warehouse` navigation.
- Progressing in AWS navigation.
- Several core improvements during navigation testing.
- `sm_dance_bot_lite` feature.
- Visualizing `turtlebot3`.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and lidar.
- Doubling `sm_multi_stage_1`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Other
-----
- Precommit cleanup run.
- Updates yaml.
- Rename doxygen deployment workflow.
- AWS demo.
- Got `sm_multi_stage_1` working (barely).
- Gaining traction with `sm_multi_stage_1`.
- Brettpac branch progress.
```

*pabloinigoblasco*

Section_124
===========

Added
-----

- Implemented sm_multi_stage_1 with initial functionality.
- Added diverse improvements to navigation and performance.
- Introduced slam toggle client behaviors and slam_toolbox components.
- Added sm_dance_bot refinement and s-pattern polishing.
- Implemented First working version of sm template and template generator.
- Added SM core test.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Added AWS navigation to sm_dance_bot.

Changed
-------

- Renamed sm_advanced_recovery_1.
- Moved reference library SMs to smacc2_performance_tools.
- Improved local action message usage.
- Updated package list.
- Refactored Docker environment for cross-environment execution.

Fixed
-----

- Resolved compile warnings.
- Fixed CI formatting for Python version.
- Fixed launch command in README.md.
- Fixed minor navigation issues.
- Mitigated overshot issue cases in waypoints navigator.
- Fixed waypoint 4 and iterations for course completion.
- Fixed errors and linting warnings in moveit migration.
- Fixed compiling issues.
- Improved Dockerfile for local test building.
- Updated README files.

Removed
-------

- Removed neo_simulation2 package.
- Removed merge markers from a Python file.
- Removed node creation in favor of logger creation.
- Removed parameters from smacc.
- Removed test from main moveit CMake.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_125
===========

Added
-----

- Dependencies added for husky simulation.
- Waypoint Inputs (#178).
- Warehouse2 progress (#179).
- Sm_dance_bot_warehouse_3 (#181).
- Finetuning waypoints (#187).
- Feature/cb pure spinning (#188).
- Feature/planner changes 16 12 (#191).
- Feature/replanning 16 dec (#193).
- Feature/undo motion 20 12 (#196).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200

```rst
Section_126
===========

Added
-----

- Initial documentation copied.
- Dockerfile now accepts ROS distro as argument. Use ``sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/``.
- New folder created for additional tracing contents.
- Added ``setupTracing.sh`` script to install necessary packages and configure tracing group.
- Created alternative ``ManualTracing``.
- Added new markdown files for SM library.
- Added Dockerfile for Rolling and Galactic.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Added ``smacc2_performance_tools``.
- Performance tests improvements.
- Optimized dependencies in ``move_base_z_planners_common``.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files. (#58)
- Fixed source CI and corrected README overview. (#62)
- Updated ``c_cpp_properties.json``.
- Changed launch command to ``ros2 launch sm_respira_1 sm_respira_1.launch`` (#69).
- Updated doxygen links (#70).
- More README updates (#72).
- Created new SM from ``sm_respira_1`` (#76).
- Feature/core and navigation fixes (#78).
- Progress in AWS navigation.
- Several core improvements during navigation testing.
- More on navigation.
- New feature: ``cb_wait_topic_message`` asynchronous client behavior.
- Attempted pre-commit fixes.

Changed
-------

- Wording changed from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated ``smacc2`` nav clients for rolling via submodules.
- Renamed tracing events.
- Bug fixed in ``smacc2`` component.
- Reverted markdowns to HTML.
- Edited ``tracing.md`` to reflect new tracing event names.
- Cleaned up formatting in various files.
- Updated ``smacc2_rta`` command across readmes.
- Cleaned up ``sm_atomic_24hr``.
- Reformatted ``sm_reference_library``.
- Corrected trailing spaces.
- Renamed folders, deleted ``tracing.md``, edited ``README.md``.
- More work on ``sm_advanced_recovery_1``.
- Fixed pre-commit issues.
- More work on ``sm_atomic_performance_test_a_2``.
- More work on ``sm_multi_stage_1``.

Removed
-------

- Manual installation of ``ros-rolling-ros2trace``. Now automated in ``setupTracing.sh``.
- Galactic builds removed from master, keeping only rolling. Submodules removed, using ``.repos`` file instead.
``` 

*pabloinigoblasco*

```rst
Section_127
===========

Added
-----
- New client behavior for nav2: Waits for nav2 nodes to subscribe to the /bond topic and confirms they are active. Optional selection of nodes to wait for.
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior.

Changed
-------
- Corrected all linters and formatters.
- Several core improvements made during navigation testing.
- Formatting enhancements.

Fixed
-----
- Removed some compile warnings.

Removed
-------
- Minor hotfix.

Collaborators
-------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

# Section_128

## Added
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation sm dance bot (#174)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)

## Changed
- Moved reference library SMs to smacc2_performance_tools (#166)
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)
- Refactored to remove merge markers from a python file (#119)
- Refactored to resolve compile warnings (#137)
- Refactored to fix CI format for Python version (#148)
- Refactored to update package list (#142)
- Refactored to update README (#164)
- Refactored to update dependencies for husky in rolling and galactic (#174)

## Fixed
- Fixed formatting issues (#134)
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#147)
- Fixed minor format issues (#134)
- Fixed waypoint 4 and iterations for robot course completion (#155)

## Removed
- Removed neo_simulation2 package (#112)
- Removed node creation and created only a logger (#149)
- Removed parameters smacc (#147)
- Removed sm_dance_bot_msgs (#144)

## Miscellaneous
- Adjusted build packages of source CI
- Adjusted navigation and performance improvements
- Adjusted waypoints navigator bug (#133)
- Adjusted navigation 2 stack renaming
- Adjusted Docker environment to be executed from any environment (#154)
- Adjusted progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Adjusted progress in testing moveit behaviors
- Adjusted progress in sm_dance_bot tests (#135)
- Adjusted progress on moveit migration testing
- Adjusted progress on markers cleanup
- Adjusted progress on sm_dance_bot_lite (#136)
- Adjusted progress on sm_dance_bot strikes back refactoring (#152)
- Adjusted progress on testing moveit behaviors (#167)
- Adjusted progress on move_it PR
- Adjusted progress on moveit
- Adjusted progress on moveit behaviors
- Adjusted progress on moveit migration
- Adjusted progress on moveit PR
- Adjusted progress on moveit testing
- Adjusted progress on sm_dance_bot and s-pattern polishing
- Adjusted progress on sm_dance_bot fixes
- Adjusted progress on sm_multi_stage_1 (#114)
- Adjusted progress on sm_multi_stage_1 (#115)
- Adjusted progress on sm_multi_stage_1 (#172)
- Adjusted progress on sm_pubsub_1 (#169)
- Adjusted progress on sm_pubsub_1 part 2 (#170)
- Adjusted progress on sm_multi_stage_1 most
- Adjusted progress on sm_multi_stage_1 finishing touches 1
- Adjusted progress on sm_multi_stage_1 readme
- Adjusted progress on sm_multi_stage_1 multistage modes
- Adjusted progress on sm_multi_stage_1 sm_multi_stage sequences
- Adjusted progress on sm_multi_stage_1 sm_multi_state_1 steps
- Adjusted progress on sm_multi_stage_1 sequence d
- Adjusted progress on sm_multi_stage_1 c sequence
- Adjusted progress on sm_multi_stage_1 mode_5_sequence_b
- Adjusted progress on sm_multi_stage_1 mode_4_sequence_b

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Pablo Iñigo Blasco

```rst
Section_129
===========

Added
-----
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress (#177).
- Waypoint Inputs (#178).
- Warehouse2 progress (#179).
- Format (#180).
- sm_dance_bot_warehouse_3 (#181).
- Feature/sm warehouse 2 13 dec 2 (#182).
- Feature/wharehouse2 dec 14 (#185).
- Feature/cb pure spinning (#188).
- Feature/planner changes 16 12 (#191).
- Feature/replanning 16 dec (#193).
- Feature/undo motion 20 12 (#196).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/cb pure spinning (#189).
- Feature/undo motion 20 12 (#196).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/cb pure spinning (#189).
- Feature/planner changes 16 12 (#191).
- Feature/replanning 16 dec (#193).
- Feature/undo motion 20 12 (#196).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204).
- Added missing file from warehouse2 (#205).
- Add mergify rules file.
- Try fixing CI for rolling (#209).
- Add Autoware Auto Msgs into not-released dependencies (#220).
- Fix rolling builds (#222).
- Odom tracker improvements and adding forward behavior retry functionality (#223).
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licenses.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Branching example.
- Disable disabled packages.
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.
- Correct formatting of python file.
- Included necessary package and edited Threesome launch.

Changed
-------
- Fixing broken build.
- Minor changes (#175).
- Minor changes.
- More changes and headless.
- Merge.
- Headless and other fixes.
- Default values.
- Finetuning waypoints (#187).
- Minor changes.
- Replanning for all our examples.
- Several fixes (#194).
- Minor changes (#195).
- Tuning and fixes (#202).
- Some reordering fixes.
- Remove example things from Foxy CI setup (#214).
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Fixing docker for Foxy and Galactic.
- Removing warnings (#213).
- Minor formatting fixes.
- Correcting codespell.
- Correcting python linters warnings.

Removed
-------
- Pure spinning behavior missing files.
- Weird moveit not downloaded repo.
- Minor broken build.
- Missing.
- Missing sm.
- Updating subscriber publisher components.
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine.
- Refining cp subscriber cp publisher.
- Improvements in SMACC core adding more components mostly developed for Autoware demo.
- Autoware demo.
- Foxy CI.
- Fix.
- Minor linking errors Foxy.
- Minor format.
- Minor linking errors Foxy.
- Foxy backport (#206).
```

```rst
Section_130
===========

Added
-----
- Renamed header files and corrected format.
- Added workflow for checking doc build.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Used docs/ as source folder and output directory for documentation.
- Renamed to smacc2 and smacc2_msgs.
- Updated GitHub branch reference.
- Updated package name and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Copied initial docs.
- Created Dockerfile with ROS distro as argument.
- Opened new folder for additional tracing contents.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Added smacc2_performance_tools.
- Improved performance tests.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More readme updates.
- Created new sm from sm_respira_1.
- Progressed in AWS navigation.
- Reworked sm_advanced_recovery_1.
- Fixed pre-commit for sm_advanced_recovery_1.
- Added sm_atomic_performance_test_a_1 and sm_atomic_performance_test_a_2.
- Added sm_atomic_performance_test_c_1.
- Added sm_multi_stage_1.
- Updated README.md with launch command.
- Improved Wait topic message client behavior.

Changed
-------
- Used manual deployment for now.
- Ignored all packages except smacc2 and smacc2_msgs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Changed wording from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for Rolling via submodules.
- Renamed tracing events.
- Bug fix in smacc2 component.
- Reverted markdowns to HTML.
- Edited tracing.md to reflect new tracing event names.
- Enabled build of missing Rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed Galactic builds from master and kept only Rolling.
- Removed submodules and used .repos file.
- Renamed folders, deleted tracing.md, and edited README.md.
- Formatted improvements.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace (now automated in setupTracing.sh).
- Deleted tracing directory.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_131
===========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `wait nav2 nodes` subscribing to the `/bond` topic and waiting for them to be alive, with optional node selection
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` gazebo fixes to show the robot and lidar
- `sm_multi_stage_1` doubling

Changed
-------
- Progress in AWS navigation demo
- Minor formatting improvements
- Navigation parameters fixes on `sm_dance_bot`
- Cleaning and lidar show/hide option for `sm_dance_bot` visualizing TurtleBot3

Fixed
-----
- Corrected all linters and formatters
- Removed some compile warnings

Removed
-------
- Attempting precommit fixes

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

Commits
-------
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)
- Remove some compile warnings (#96)
- Feature/cb pause slam (#98)
- Rename doxygen deployment workflow (#100)
- sm_dance_bot visualizing TurtleBot3 (#101)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- Feature/sm dance bot strikes back gazebo fixes (#105)
``` 

*pabloinigoblasco*

```rst
Section_132
===========

Added
-----

- Added AWS demo (#108, #110)
- Added progress in sm_multi_stage_1 (#109, #111)
- Added diverse improvements in navigation and performance (#116)
- Added slam toggle and smacc deep history feature (#122)
- Added dance bot s pattern feature (#128, #129)
- Added first working version of sm template and template generator (#127)
- Added SM core test (#138)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added more testing on moveit behaviors (#167)

Changed
-------

- Updated package list (#142)
- Renamed navigation 2 stack (#144)
- Refactored sm dance bot strikes back (#152)
- Moved reference library SMs to smacc2_performance_tools (#166)

Fixed
-----

- Fixed gazebo issues to show the robot and the lidar
- Fixed format issues
- Fixed compilation warnings (#137, #148)
- Fixed minor navigation improvements (#141)
- Fixed launch command in README.md
- Fixed CI format for Python version
- Fixed node creation in SM Atomic SM generator (#149)
- Fixed Docker environment to be executed from any environment (#154)
- Fixed slight waypoint 4 and iterations changes for robot completion (#155)

Removed
-------

- Removed neo_simulation2 package (#112)
- Removed sm_dance_bot_msgs
- Removed parameters in smacc (#147)
- Removed node creation and create only a logger (#149)

Authors
-------

- Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_133
===========

Added
-----

- Added `sm_pubsub_1 part 2` feature.
- Added `sm_advanced_recovery_1` renaming (#171).
- Added `sm_multi_stage_1` reworking (#172).
- Added `multistage modes`.
- Added `sm_multi_stage sequences`.
- Added `sm_multi_state_1 steps`.
- Added `sm_multi_stage_1 sequence d`.
- Added `sm_multi_stage_1 c sequence`.
- Added `mode_5_sequence_b`.
- Added `mode_4_sequence_b`.
- Added `sm_multi_stage_1 most`.
- Added `finishing touches 1`.
- Added `readme`.
- Added `Feature/aws navigation sm dance bot (#174)`.
- Added `repo dependency`.
- Added `husky launch file in sm_dance_bot`.
- Added dependencies for `husky simulation`.
- Added fixes for formatting.
- Added updates for `husky` in `rolling` and `galactic`.
- Added progress on `aws navigation` and refactorings on navigation clients and behaviors.
- Added more on `aws demo`.
- Added fixes for broken build.
- Added `warehouse2 (#177)`.
- Added `Waypoint Inputs (#178)`.
- Added progress for `wharehouse2 (#179)`.
- Added `format (#180)`.
- Added `sm_dance_bot_warehouse_3 (#181)`.
- Added `Feature/sm warehouse 2 13 dec 2 (#182)`.
- Added more changes and `headless`.
- Added `merge`.
- Added `default values`.
- Added `Brettpac branch (#184)`.
- Added `redoing sm_dance_bot_warehouse_3 waypoints`.
- Added more `Waypoints`.
- Added fixes and formatting for `SrConditional (#168)`.
- Added fixes for `move trigger logic` into headers.
- Added fixes for `lint`.
- Added `Feature/wharehouse2 dec 14 (#185)`.
- Added fixes for `warehouse2`.
- Added `Feature/sm warehouse 2 13 dec 2 (#186)`.
- Added more changes and `headless`.
- Added `merge`.
- Added `default values`.
- Added `finetuning waypoints (#187)`.
- Added `Feature/cb pure spinning (#188)`.
- Added more changes and `headless`.
- Added `merge`.
- Added `default values`.
- Added `pure spinning behavior missing files`.
- Added `Feature/planner changes 16 12 (#191)`.
- Added more fixes.
- Added `Feature/replanning 16 dec (#193)`.
- Added replanning for all examples.
- Added several fixes (#194).
- Added `Feature/undo motion 20 12 (#196)`.
- Added improving undo motion navigation for `warehouse2`.
- Added `tuning warehouse3 (#197)`.
- Added `Feature/undo motion 20 12 (#198)`.
- Added improving undo motion navigation for `warehouse2`.
- Added `undo tuning and errors`.
- Added `format`.
- Added `Feature/sync 21 12 (#199)`.
- Added format issues.
- Added `Feature/warehouse2 22 12 (#200)`.
- Added format issues.
- Added finishing for `warehouse2`.
- Added `Feature/warehouse2 23 12 (#201)`.
- Added tuning and fixes (#202).
- Added `Feature/minor tune (#203)`.
- Added tuning and fixes.
- Added `minor tune`.
- Added fixing for `warehouse 3 problems` and other core improvements (#204).
- Added fixing for `warehouse 3 problems` and other core improvements to remove deadlocks, also making continuous integration green.
- Added `weird moveit not downloaded repo`.
- Added missing file from `warehouse2 (#205)`.
- Added backport to `foxy`.
- Added minor format.
- Added minor linking errors for `foxy`.
- Added `docker files for different revisions`, warnings removal, and more testing on navigation.
- Added fixing docker for `foxy` and `galactic`.
- Added fixes for `code generators (#221)`.
- Added fixes for other build issues.
- Added updates for `SM template` and clear example code visibility.
- Added removal of `node` in the `sm performance template`.
- Added updates for template to use `Blackboard storage`.
- Added updates for template to resolve the global data correctly.
- Added updates for `sm_name.hpp`.
- Added `Feature/retry behavior warehouse 1 (#226)`.
- Added backport to `foxy`.
- Added minor format.
- Added minor linking errors for `foxy`.
- Added `Foxy backport (#206)`.
- Added minor formatting fixes.
- Added fixes for trailing spaces.
- Added corrections for codespell.
- Added corrections for Python linters warnings.

Changed
-------

- Renamed `sm_advanced_recovery_1` (#171) to improve clarity.

Fixed
-----

- Fixed formatting issues.
- Fixed broken build issues.
- Fixed errors in `warehouse3`.
- Fixed format issues.
- Fixed minor linking errors.
- Fixed trailing spaces.
- Corrected codespell errors.
- Corrected Python linters warnings.

Removed
--------

- Removed redundant entries.
```

```rst
Section_134
===========

Added
-----
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as a dependency to locate includes.
- Added missing licenses.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh for automated installation.
- Added alternative ManualTracing.
- Added new sm markdowns.
- Added Dockerfile for Rolling and Galactic.
- Added smacc2_performance_tools.
- Added README tutorial for Dockerfile.
- Added sm_atomic_performance_test_a_2.

Changed
-------
- Changed extension of imports.
- Changed wording from "smacc application" to "SMACC2 library".
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Changed mentions of SMACC/ROS to SMACC2/ROS2.
- Changed name of package and package.xml for liter compliance.
- Changed GitHub branch reference.
- Changed formatters.
- Changed extension of imports.
- Changed wording in README overview.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed formatting of python file.
- Fixed trailing spaces.
- Fixed source CI and corrected README overview.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
- Removed tracing directory.
- Removed galactic builds from master, keeping only rolling.
- Removed submodules and use .repos file.
- Removed manual installation of ros-rolling-ros2trace.

Other
-----
- Updated ci-build-source.yml.
- Updated doxygen-check-build.yml.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated description table.
- Updated table.
- Updated name of package and package.xml to pass liter.
- Updated changelogs.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated smacc2_sm_reference_library/sm_atomic/README.md.
- Updated sm_three_some launch command.
- Updated sm_atomic_24hr.
- Updated sm_atomic_performance_trace_1.
- Updated sm_atomic_24hr cleanup.
- Updated sm_advanced_recovery_1 reworked.
- Updated sm_advanced_recovery_1 round 4.
- Updated sm_advanced_recovery_1 work.
- Updated sm_advanced_recovery_1.
- Updated sm_respira_1 format cleanup.
- Updated sm_respira_test_2.
- Updated sm_aws_aarehouse navigation.
- Updated sm_reference_library reformatting.
- Updated sm_atomic_performance_test_a_2.
- Updated sm_atomic_performance_trace_1.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_performance_test_a_2.
- Updated sm_respira_1 format cleanup pre-commit.
- Updated sm_respira_1 format cleanup.
- Updated sm_respira_test_2.
- Updated sm_three_some to launch sm_three_some.launch.
- Updated to use manual deployment for now.
- Updated to satisfy ament_lint_cmake.
- Updated to correct formatters.
- Updated to enable cppcheck.
- Updated to enable build of missing rolling repositories.
- Updated to enable Navigation2 for semi-binary build.
- Updated to reactivate smacc2 nav clients for rolling via submodules.
- Updated to optimize deps in move_base_z_planners_common.
- Updated to rename header files and correct format.
- Updated to rename to smacc2 and smacc2_msgs.
- Updated to reset all versions to 0.0.0.
- Updated to ignore all packages except smacc2 and smacc2_msgs.
- Updated to ignore further packages.
- Updated to disable ament_cpplint.
- Updated to disable cpplint and cppcheck linters.
- Updated to disable some packages and update workflows.
- Updated to disable disabled packages.
- Updated to bump ccache version.
- Updated to ignore further packages.
- Updated to change extension.
- Updated to change extension of imports.
- Updated to execute on master update.
- Updated to copy initial docs.
- Updated to use docs/ as source folder for documentation.
- Updated to use docs/ as output directory.
- Updated to create workflow for testing prerelease builds.
- Updated to use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/".
- Updated to open new folder for additional tracing contents.
- Updated to moved tracing.md to tracing directory.
- Updated to added setupTracing.sh.
- Updated to created new sm from sm_respira_1.
- Updated to progress in aws navigation demo.
- Updated to format improvements.
- Updated to more on navigation.
- Updated to more on performance tests.
- Updated to more changes on performance tests.
- Updated to more sm_atomic_24hr cleanup.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more Readme Updates.
- Updated to more Readme.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1 work.
- Updated to more sm_advanced_recovery_1.
- Updated to more sm_advanced_recovery_1

```rst
Section_135
===========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Nodes to wait can be optionally selected.
- New feature: cb pause slam client behavior.

Changed
-------

- Updated launch command in README.md.
- Corrected all linters and formatters.

Fixed
-----

- Fixed compile warnings.

Removed
-------

- Removed some compile warnings.

Other
-----

- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Minor formatting improvements.
- Merge and progress.
- Precommit updates in sm_dance_bot_lite.
```

*pabloinigoblasco*

## Section_136

### Added
- Added `sm_dance_bot` visualizing `turtlebot3` (#101)
- Added `Feature/dance bot launch gz lidar choice` (#102)
- Added `Feature/sm dance bot lite gazebo fixes` (#104)
- Added `Feature/sm dance bot strikes back gazebo fixes` (#105)
- Added `aws demo` (#108)
- Added `Brettpac branch` (#110)
- Added `a3` (#113)
- Added `mm` (#115)
- Added `Feature/diverse improvements navigation performance` (#117)
- Added `Feature/slam toggle and smacc deep history` (#122)
- Added `Feature/dance bot s pattern` (#128)
- Added `Feature/sm dance bot refine` (#131)
- Added `Feature/sm dance bot refine 2` (#132)
- Added `Add SM core test` (#138)
- Added `Feature/nav2z renaming` (#144)
- Added `Add SM Atomic SM generator` (#143)
- Added `Rolling Docker environment to be executed from any environment` (#154)
- Added `Feature/sm dance bot strikes back refactoring` (#152)
- Added `Feature/migration moveit client` (#151)

### Changed
- Renamed `doxygen deployment workflow` (#100)
- Minor hotfix
- Various cleaning and formatting improvements
- Gazebo fixes to show the robot and the lidar
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Progress in testing `sm_dance_bot` introducing slam pausing/resuming functionality
- Minor tweaks and improvements in navigation
- Using local action messages
- Navigation 2 stack renaming
- Minor tuning to mitigate overshot issue cases
- Progress in `sm_dance_bot` tests
- Minor format issues

### Fixed
- Fixed recursion issue by moving method after the method it calls (#126)
- Fixed compile warnings (#137)
- Fixed CI format for Python version (#148)
- Removed `neo_simulation2` package (#112)
- Removed merge markers from a Python file (#119)
- Removed parameters from `smacc` (#147)
- Removed node creation and create only a logger (#149)
- Fixed launch command for `sm_dance_bot_strikes_back` in README.md

### Removed
- Removed `neo_simulation2` package
- Removed `sm_dance_bot_msgs`
- Removed parameters from `smacc`

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_137
===========

Added
-----

- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
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
- Finetuning waypoints (#187)
- Tuning warehouse3 (#197)
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green (#204)

Fixed
-----

- Fix: add a missing colon
- Fix: some formatting and templating on SrConditional
- Fix: move trigger logic into headers
- Fix: lint
- Several fixes (#194)

Removed
-------

- Retry behavior warehouse 1

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: reelrbtx <brett2@reelrobotics.com>
- Co-authored-by: brettpac <brett@robosoft.ai>
- Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>

pabloinigoblasco
```

```rst
Section_138
===========

Added
-----
- First ensure you have the necessary package installed:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml.
- Create doxygen-deploy.yml.
- Use manual deployment for now.
- Create workflow for testing prerelease builds.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Rename to smacc2 and smacc2_msgs.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Ignore all packages except smacc2 and smacc2_msgs.
- Update changelogs.
- Revert "Ignore all packages except smacc2 and smacc2_msgs".
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Update description table.
- Update table.
- Copy initial docs.
- Dockerfile w/ ROS distro as argument:
  Use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/".
- Opened new folder for additional tracing contents.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh:
  Installs necessary packages and configures tracing group.
- Removed manual installation of ros-rolling-ros2trace:
  This is now automated in setupTracing.sh;
  location of sh file assumed if user follows README.md under "Getting started".
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Changed wording "smacc application" to "SMACC2 library".
- Update smacc_sm_reference_library/sm_atomic/README.md:
  Edit from html to markdown syntax.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling.
  Remove submodules and use .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- sm_respira_1 format cleanup.
- sm_respira_1 format cleanup pre-commit.
- sm_respira_test_2.
- More changes on performance tests.
- Do not execute clang-format on smacc2_sm_reference_library package.
- sm_reference_library reformatting.
- Correct trailing spaces.
- sm_atomic_24hr.
- sm_atomic_performance_trace_1.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- More sm_atomic_24hr cleanup.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Minor formatting.
- Add galactic CI setup and rename rolling files (#58).
- Fix source CI and correct README overview (#62).
- Update c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
  Also noticed a note I had made while producing these that was not removed.

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Fixed
-----
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234).

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licences.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Branching example.
- Disable disabled packages.
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.
- Correct formatting of python file.
- Included necessary package and edited Threesome launch.
``` 

*pabloinigoblasco*

```rst
Section_139
===========

Added
-----
- Update doxygen links (#70) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More Readme Updates (#72) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More Readme (#74) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Wait topic message client behavior (#81) by Denis Štogl <destogl@users.noreply.github.com>
- Feature/wait nav2 nodes client behavior (#82) by Denis Štogl <denis@stogl.de>
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Changed
-------
- Several core improvements during navigation testing
- Progress in aws navigation demo
- Formatting improvements
- Progress in aws navigation demo
- Minor format improvements
- Navigation parameters fixes on sm_dance_bot
- Merge and progress
- Fix format

Fixed
-----
- Fix pre-commit
- Trying to fix Pre-Commit
- Correct all linters and formaters

Removed
-------
- Modifying sm_atomic_performance_test_a_2 (#89)
```

```rst
Section_140
===========

Added
-----

- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2, wait for nav2 nodes subscribing to the /bond topic and wait for them to be alive. Optionally select the nodes to wait.
- Base for the sm_aws_warehouse navigation.
- Gazebo fixes to show the robot and the lidar.
- First working version of sm template and template generator.
- More refinement in sm_dance_bot.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Progress in testing sm_dance_bot introducing slam pausing/resuming functionality.
- Several core improvements during navigation testing.
- Using local action messages.
- Minor navigation improvements.
- Minor tuning to mitigate overshot issue cases.
- Added SVGs to READMEs of atomic, dance_bot, and others.

Changed
-------

- Formatting improvements.
- Minor format adjustments.
- Minor tweaks.
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.
- Correct formatting.
- Enable source build on PR for testing.
- Additional linting and formatting.

Fixed
-----

- Navigation parameters fixes on sm_dance_bot.
- Remove some compile warnings.
- Remove neo_simulation2 package.
- Remove merge markers from a Python file.
- Minor format issues.

Removed
-------

- Remove neo_simulation2 package.
- Removing sm_dance_bot_msgs.
- Pending references.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_141

### Added
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation (#174)

### Changed
- Updated package list (#142)
- Fixed launch command in README.md
- Refactored SM Dance Bot Strikes Back (#152)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Reworked SM Multi-Stage 1 (#172)
- Redoing SM Dance Bot Warehouse 3 waypoints (#184)
- Finetuned waypoints (#187)
- Improved undo motion navigation in Warehouse2 (#198)

### Fixed
- Fixed CI: format fix python version (#148)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed broken build in AWS navigation
- Fixed formatting and templating on SrConditional (#168)
- Fixed several issues in replanning and undo motion examples
- Fixed warehouse 3 problems and other core improvements (#204)

### Removed
- Removed parameters smacc (#147)
- Removed node creation and created only a logger (#149)
- Removed test from main MoveIt CMake

### Miscellaneous
- Precommit cleanup
- Workflows update
- Docker refactoring
- Minor configuration changes
- More testing on MoveIt behaviors
- Progress on MoveIt migration testing
- Progress on AWS navigation and other refactorings
- More on AWS demo
- Warehouse2 progress
- Finishing Warehouse2
- Tuning and fixes
- Default values set
- Merge changes
- Headless mode adjustments
- Added missing dependencies
- Updated formats in various files
- Added .reps dependencies
- Added dependency to UR5 client
- Improved Dockerfile for building local tests
- Added reliability QoS config
- Added SM PubSub 1 (#169, #170)
- Added SM Advanced Recovery 1 renaming (#171)
- Added CB Pure Spinning behaviors (#188, #189)
- Added Planner changes (#191)
- Added Replanning for all examples (#193)
- Added Sync feature (#199)
- Added Warehouse2 features (#200, #201)
- Added Minor Tune feature (#203)

```rst
Section_142
===========

Added
-----
- Added missing file from warehouse2 (#205).
- Added backport to foxy.
- Added docker build files for all versions.
- Added barrel search build fix and warehouse3.
- Added progress in barrel husky.
- Added testing dance bot demos.
- Added runtime dependency.
- Added restoring ur dependency.

Changed
-------
- Updated subscriber publisher components.
- Improved smacc core by adding more components mostly developed for autoware demo.
- Improved docker files for different revisions, warnings removal, and more testing on navigation.
- Improved warehouse 3 startup problems.
- Improved format and minor issues.
- Improved docker for foxy and galactic.
- Improved master rolling to galactic backport.
- Improved build fixing.

Fixed
-----
- Fixed warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green.
- Fixed weird moveit not downloaded repo.
- Fixed minor linking errors in foxy.
- Fixed minor broken build.
- Fixed some reordering fixes.
- Fixed broken build.

Removed
-------
- Removed some redundant entries.

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco
```
