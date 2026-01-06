# Changelog for package undo_path_global_planner

## 2.3.16 (2023-07-16)
- Merged branch 'humble' from https://github.com/robosoft-ai/SMACC2 into humble
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted to fix a weird issue with ros buildfarm
  - Further work on the buildfarm issue
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

## 2.3.6 (2023-03-12)

## 1.22.1 (2022-11-09)
- Pre-release
- Contributors: pabloinigoblasco

## 0.3.0 (2022-04-04)

## 0.0.0 (2022-11-09)
- Humble check
- Publisher progress in migration to humble
- Reverted "Ignore packages which should not be released."
  - This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Ignore packages which should not be released
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - More on performance and other issues
  - Format cleanup for sm_respira_1
  - Format cleanup pre-commit for sm_respira_1
  - Added sm_respira_test_2
  - More changes on performance tests
  - Do not execute clang-format on smacc2_sm_reference_library package
  - Reformatting for sm_reference_library
  - Corrected trailing spaces
  - Added sm_atomic_24hr
  - Added sm_atomic_performance_trace_1
  - Updated smacc2_rta command across readmes
  - Cleaned up sm_atomic_24hr
  - More cleanup for sm_atomic_24hr
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Minor formatting
  - Added galactic CI setup and renamed rolling files (#58)
  - Fixed source CI and corrected README overview (#62)
  - Updated c_cpp_properties.json
  - Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  - Updated doxygen links (#70)
  - More Readme Updates (#72)
  - More Readme (#74)
  - Created new sm from sm_respira_1 (#76)
  - Feature/core and navigation fixes (#78)
  - Base for the sm_aws_aarehouse navigation
  - Progressing in aws navigation
  - Several core improvements during navigation testing
  - Formatting improvements
  - Progress in aws navigation demo
  - Feature/aws demo progress (#80)
  - More on navigation
  - Reworked sm_advanced_recovery_1 (#83)
  - Fix pre-commit for sm_advanced_recovery_1
  - More work on sm_advanced_recovery_1 (#85)
  - Round 4 for sm_advanced_recovery_1 (#86)
  - Brettpac branch (#87)
  - Added sm_atomic_performance_test_a_2
  - Added sm_atomic_performance_test_a_1
  - Added sm_atomic_performance_test_c_1 (#88)
  - Modified sm_atomic_performance_test_a_2 (#89)
  - Added sm_multi_stage_1
  - Fixing precommit for sm_multi_stage_1
  - More work on sm_multi_stage_1 (#91)
  - Updated README.md
  - Wait topic message client behavior (#81)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Attempting precommit fixes
  - Feature/wait nav2 nodes client behavior (#82)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait
  - Corrected all linters and formatters
  - Co-authored-by: Denis Štogl <denis@stogl.de>
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  - Feature/aws demo progress (#92)
  - Base for the sm_aws_aarehouse navigation

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive. Optional node selection available.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.
- Minor hotfix.
- Cleaning and lidar show/hide option for `sm_dance_bot`.
- Gazebo fixes to show the robot and the lidar.
- Format fixes.

Fixed
-----

- Remove some compile warnings.
- Correct formatting for removing `neo_simulation2` package.
- Enable source build on PR for testing.
- Adjust build packages of source CI.

Removed
-------

- Removed `neo_simulation2` package.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_3
==========

Version 0.1.0 (2022-01-01)
---------------------------

### Added
- Feature/diverse improvements navigation performance (#117)
- Additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Introducing slam pausing/resuming functionality in testing sm_dance_bot
- Feature/dance bot s pattern (#128)
- First working version of sm template and template generator (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
- Minor navigation improvements (#141)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Slight waypoint 4 and iterations changes so the robot can complete the course (#155)
- Feature/migration moveit client (#151)
- Initial migration to smacc2
- Progressing in the moveit migration testing
- Adding .reps dependencies and fixing build errors
- Adding dependency to ur5 client
- Improving dockerfile for building local tests
- Update readme (#164)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- More testing on moveit behaviors
- Sm_pubsub_1 (#169)
- Sm_pubsub_1 part 2 (#170)
- Sm_advanced_recovery_1 renaming (#171)
- Sm_multi_stage_1 reworking (#172)
- Feature/aws navigation sm dance bot (#174)
- Husky launch file in sm_dance_bot
- Add dependencies for husky simulation
- Update dependencies for husky in rolling and galactic
- Waypoint Inputs (#178)

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Resolving compile warnings (#137)
- Removing node creation and creating only a logger (#149)
- Moved reference library SMs to smacc2_performance_tools (#166)

### Fixed
- Fix CI: format fix python version (#148)

### Removed
- Remove merge markers from a python file (#119)
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake

### Miscellaneous
- Minor format adjustments
- Noticed typo correction
- Finnaly > Finally correction
- Precommit cleanup
- Pending references cleanup
- Workflow updates
- Docker refactoring
- Repos dependency updates
- Minor tweaks and improvements
- Readme updates
- Warehouse2 (#177)
- Minor changes (#175)

```rst
Section_4
=========

Added
-----
- Implement wharehouse2 progress (#179)
- Add feature for sm_dance_bot_warehouse_3 (#181)
- Introduce Feature/sm warehouse 2 13 dec 2 (#182)
- Include Brettpac branch (#184)
- Add finetuning waypoints (#187)
- Implement Feature/cb pure spinning (#188)
- Introduce Feature/planner changes 16 12 (#191)
- Add Feature/replanning 16 dec (#193)
- Include Feature/undo motion 20 12 (#196)
- Implement Feature/sync 21 12 (#199)
- Introduce Feature/warehouse2 22 12 (#200)
- Add Feature/warehouse2 23 12 (#201)
- Include Feature/minor tune (#203)
- Implement Feature/warehouse2 23 12 (#201)
- Add Feature/minor tune (#203)
- Include fixing warehouse 3 problems and other core improvements (#204)
- Add Foxy backport (#206)
- Include various fixes and improvements

Changed
-------
- Update ros2 launch sm_three_some to sm_three_some.launch
- Rename header files and correct format
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Update package names and references
- Reset all versions to 0.0.0
- Update description table
- Update table
- Copy initial docs
- Update Dockerfile with ROS distro as argument
- Open new folder for tracing contents
- Move tracing.md to tracing directory
- Add setupTracing.sh for automated installation
- Update wording from "smacc application" to "SMACC2 library"
- Edit smacc_sm_reference_library/sm_atomic/README.md
- Reactivate smacc2 nav clients for rolling via submodules
- Rename tracing events
- Fix bug in smacc2 component
- Revert markdowns to html
- Add README tutorial for Dockerfile
- Perform additional cleanup

Removed
-------
- Delete tracing directory
- Remove manual installation of ros-rolling-ros2trace

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section 5
=========

Added
-----
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.

Fixed
-----
- Fixed source CI and corrected README overview.
- Fixed build of missing rolling repositories.
- Fixed navigation2 for semi-binary build.
- Fixed trailing spaces.
- Fixed galactic CI setup and renamed rolling files.
- Fixed formatting in sm_atomic_24hr.
- Fixed formatting in sm_advanced_recovery_1.
- Fixed formatting in sm_multi_stage_1.
- Fixed formatting in cb_wait_topic_message.
- Fixed pre-commit in sm_advanced_recovery_1.
- Fixed pre-commit in sm_multi_stage_1.
- Fixed pre-commit in attempting pre-commit fixes.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Removed execution of clang-format on smacc2_sm_reference_library package.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_6
=========

Added
-----

- New client behavior `cb_wait_topic_message` added for nav2, allowing nodes to subscribe to the `/bond` topic and wait until they are active. Nodes to wait for can be optionally selected.
- New client behavior `cb_pause_slam` added.
- New client behavior `sm_dance_bot_lite` added for visualizing TurtleBot3 in Gazebo.
- New feature `dance_bot_launch_gz_lidar_choice` added for showing/hiding the lidar in Gazebo.
- New feature `sm_dance_bot_strikes_back` added for visualizing TurtleBot3 in Gazebo with lidar show/hide option.
- New feature `sm_multi_stage_1` added.
- New feature `slam_toggle_and_smacc_deep_history` added for navigation and slam toggle client behaviors, as well as `smacc2::deep_history` syntax.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.
- Formatting improvements in various areas.
- Minor hotfixes and format adjustments.
- Cleanup and format fixes for Gazebo visualization.
- Adjustments to source build packages for CI testing.
- Method reordering to prevent recursion in `sm_dance_bot`.
- Progress in AWS navigation demo.
- Progress in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Various core improvements during navigation testing.

Fixed
-----

- Removed some compile warnings.
- Removed `neo_simulation2` package.

Removed
-------

- `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_7
=========

Added
-----

- First working version of sm template and template generator. (#127)
- Feature/dance bot s pattern (#129)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- sm_dance_bot_lite (#136)
- Add SM core test (#138)
- using local action msgs (#139)
- Feature/nav2z renaming (#144)
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- initial migration to smacc2
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)
- Feature/aws navigation sm dance bot (#174)
- warehouse2 (#177)
- Waypoint Inputs (#178)
- sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)

Changed
-------

- Noticed typo "Finnaly" corrected to "Finally"
- minor tweaks (#130)
- minor navigation improvements (#141)
- Resolve compile warnings (#137)
- fixing some errors introduced on formatting in migration moveit client (#151)
- fixing compiling issues in migration moveit client (#151)
- update readme (#164)
- moved reference library SMs to smacc2_performance_tools (#166)
- finetuning waypoints (#187)

Fixed
-----

- Fix CI: format fix python version (#148)
- Fix launch command in README.md for sm_dance_bot_strikes_back
- fixing broken master build in testing moveit behaviors (#167)
- fixing pipeline error in testing moveit behaviors (#167)
- fixing broken build in aws navigation (#174)

Removed
-------

- removing sm_dance_bot_msgs
- removing parameters smacc
- removing test from main moveit cmake

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section 8
=========

Added
-----
- Added Feature/planner changes 16 12 (#191).
- Added Feature/replanning 16 dec (#193).
- Added Feature/undo motion 20 12 (#196).
- Added Feature/undo motion 20 12 (#198).
- Added Feature/sync 21 12 (#199).
- Added Feature/warehouse2 22 12 (#200).
- Added Feature/warehouse2 23 12 (#201).
- Added Feature/minor tune (#203).
- Added Merging code from backport foxy and updates about autoware (#208).
- Added Foxy backport (#206).
- Added galactic CI build because Navigation2 is broken in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added workflow for checking doc build.
- Added workflow for testing prerelease builds.
- Added Dockerfile w/ ROS distro as argument.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Added alternative ManualTracing.
- Added sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Changed wording "smacc application" to "SMACC2 library".

Fixed
-----
- Fixed several fixes (#194).
- Fixed tuning warehouse3 (#197).
- Fixed warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green (#204).
- Fixed minor broken build.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed formatting of python file.
- Fixed bug in smacc2 component.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
```

Por favor, házmelo saber si necesitas más cambios o información.

Section_9
==========

Added
-----
- Added `smacc2_performance_tools`.
- Added new feature, `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for Nav2, `wait nav2 nodes`, subscribing to the `/bond` topic and waiting for them to be alive. You can optionally select the nodes to wait.

Changed
-------
- Updated mentions of `SMACC/ROS` to `SMACC2/ROS2`.
- Renamed folders, deleted `tracing.md`, and edited `README.md`.
- Renamed `sm_respira_1` to `sm_respira_test_2`.
- Updated launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.
- Updated `smacc2_rta` command across readmes.
- Renamed event generator library.
- Optimized dependencies in `move_base_z_planners_common`.
- Corrected trailing spaces.
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.

Fixed
-----
- Fixed source CI and corrected README overview.
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used `.repos` file.
- Do not execute `clang-format` on `smacc2_sm_reference_library` package.

Miscellaneous
-------------
- Some progress on navigation rolling.
- Performance tests improvements.
- More changes on performance tests.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progress in AWS navigation.
- Progressing in AWS navigation.
- More on navigation.
- Format improvements.
- Attempting pre-commit fixes.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_10
==========

Added
-----

- New client behavior `cb_wait_topic_message` added for nav2. This behavior waits for nav2 nodes to subscribe to the `/bond` topic and ensures they are alive. Users can optionally select the nodes to wait for.
- New feature `cb_pause_slam` added for pausing SLAM functionality.
- New feature `sm_dance_bot_lite` introduced for visualizing TurtleBot3 in Gazebo.
- New feature `sm_multi_stage_1` with improvements and doubling functionality.
- New feature `sm_dance_bot_strikes_back` with fixes for Gazebo visualization.
- New feature `aws_demo` added.

Changed
-------

- Progress made in AWS navigation demo.
- Navigation parameters fixed for `sm_dance_bot`.
- Formatting improvements across various features.
- Gazebo fixes implemented to show the robot and lidar in different scenarios.
- Source build enabled on PR for testing purposes.
- Adjustments made to build packages for source CI.
- Various improvements made in navigation and performance.
- Method reordering to prevent recursion in `sm_dance_bot`.

Fixed
-----

- Compilation warnings removed.
- Merge markers removed from a Python file.

Removed
-------

- `neo_simulation2` package removed.

Collaborators
-------------

- Brett <brett@robosoft.ai>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
```

# Section_11

## Added
- First working version of sm template and template generator. (#127)
- Minor tweaks (#130)
- Feature/dance bot s pattern (#129)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Add SM Atomic SM generator. (#143)
- Add SM core test (#138)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/migration moveit client (#151)
- Feature/nav2z renaming (#144)
- Feature/aws navigation sm dance bot (#174)
- Feature/cb pure spinning (#188)
- Feature/wharehouse2 dec 14 (#185)

## Changed
- Renamed reference library SMs to smacc2_performance_tools (#166)
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#172)
- Reworked sm_multi_stage_1 (#

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
- Foxy backport (#206)
- Add mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies. (#220)
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
- Enable cppcheck
- Included necessary package and edited Threesome launch
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
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

Changed
-------

- some reordering fixes
- Remove example things from Foxy CI setup. (#214)
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- fixing docker for foxy and galactic
- removing warnings (#213)
- branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Correct formatting of python file.

Fixed
-----

- minor broken build

Removed
-------

- merge
- headless and other fixes
- default values
- minor
- pure spinning behavior missing files
- more fixes
- several fixes (#194)
- tuning warehouse3 (#197)
- fixing warehouse 3 problems, and other core improvements (#204)
- weird moveit not downloaded repo
- added missing file from warehouse2 (#205)
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- foxy ci
- fix
- minor format
- minor linking errors foxy
- minor format
- minor linking errors foxy
``` 

*pabloinigoblasco*

```rst
Section_13
==========

Added
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Minor formatting improvements.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Updated doxygen links.
- Updated c_cpp_properties.json.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed pre-commit issues.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other
-----
- Made progress on navigation rolling.
- Made progress in aws navigation.
- Made several core improvements during navigation testing.
- Made more changes on performance tests.
- Made more sm_atomic_24hr cleanup.
- Made more sm_advanced_recovery_1 work.
- Corrected all linters and formatters.

Collaborators
-------------
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_14
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, which waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.

Changed
-------
- Minor formatting improvements.

Fixed
-----
- Navigation parameters fixes on `sm_dance_bot`.
- Removed some compile warnings.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Base for the `sm_aws_warehouse` navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Merge and progress.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite`.
- Updates `yaml`.
- `sm_dance_bot` visualizing `turtlebot3`.
- `dance bot launch gz lidar choice`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- Precommit cleanup run.
- AWS demo.
- Got `sm_multi_stage_1` working (barely).
- `Brettpac` branch.
- `a3`.
- `mm`.
- Diverse improvements in navigation and performance.
``` 

*pabloinigoblasco*

Section_15
=========

Added
-----

- Feature/diverse improvements in navigation and performance (#117)
- Feature/slam toggle and smacc deep history (#122)
- Feature/dance bot s pattern (#128)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/migration moveit client (#151)
- Feature/aws navigation sm dance bot (#174)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Initial state machine transition timestamp (#165)
- Waypoint Inputs (#178)

Changed
-------

- Move method after the method it calls to prevent recursion (#126)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Minor changes in sm_dance_bot (#125, #131, #132, #136)
- Minor navigation improvements (#141)
- Using local action messages instead of sm_dance_bot_msgs (#139, #144)
- Removed node creation and create only a logger (#149)
- Updated package list (#142)
- Removed parameters smacc (#147)
- Fix CI: format fix python version (#148)
- Fixed launch command for sm_dance_bot_strikes_back in README.md
- Minor changes in sm_multi_stage_1 (#172)
- Warehouse2 progress (#179)
- Format improvements (#180)
- sm_dance_bot_warehouse_3 (#181)

Fixed
-----

- Minor tuning to mitigate overshot issue cases in waypoints navigator bug (#133)
- Progress in the sm_dance_bot tests (#135)
- Fixed compiling issues in sm_pubsub_1 (#169)
- Fixed broken master build in Feature/testing moveit behaviors (#167)
- Fixed pipeline error in Feature/testing moveit behaviors (#167)
- Fixed broken build in Feature/aws navigation sm dance bot (#174)
- Minor changes (#175)

Removed
-------

- Removed merge markers from a python file (#119)
- Removed sm_dance_bot_msgs dependency
- Removed test from main moveit cmake
- Removed some comments in the past from launch command in README.md

Other
-----

- Noticed typo "Finnaly" corrected to "Finally"
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Precommit cleanup
- Pending references
- Repos dependency
- Added dependency to ur5 client
- Docker refactoring
- Added .reps dependencies and fixed some build errors
- Improving dockerfile for building local tests
- More readme updates
- Default values added
- Merge and headless fixes in Feature/sm warehouse 2 13 dec 2 (#182)

Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_16
==========

Added
-----
- Brettpac branch (#184)
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188, #189)
- Pure spinning behavior missing files
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Several fixes (#194)
- Feature/undo motion 20 12 (#196, #198)
- Improving undo motion navigation warehouse2
- Tuning warehouse3 (#197)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Finishing warehouse2
- Feature/warehouse2 23 12 (#201)
- Tuning and fixes (#202)
- Feature/minor tune (#203)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Added missing file from warehouse2 (#205)
- Docker build files for all versions (#225)
- Feature/retry behavior warehouse 1 (#226)
- Foxy backport (#206)
- Fix code generators (#221)
- Fix other build issues
- Update SM template and make example code clearly visible
- Remove use of node in the SM performance template
- Updated template to use Blackboard storage
- Update template to resolve the global data correctly
- Update sm_name.hpp
- Fix trailing spaces
- Correct codespell
- Correct Python linters warnings
- Add Galactic CI build
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency
- Disable ament_cpplint
- Disable some packages and update workflows
- Bump ccache version
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licenses
- Disable cpplint and cppcheck linters
- Correct formatters
- Branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension of imports
- Enable cppcheck
- Correct formatting of Python file
- Included necessary package and edited Threesome launch
- Rename header files and correct format
- Add workflow for checking doc build
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Use manual deployment for now
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference
- Update name of package and package.xml to pass liter
- Execute on master update
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- 0.1.0

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Removed
-------
- First ensure you have the necessary package installed
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command
```

```rst
Section_17
==========

Added
-----

- Added setupTracing.sh script to automate installation of necessary packages and configuration of tracing group.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added galactic CI setup and renamed rolling files. (#58)
- Added source CI fix and corrected README overview. (#62)
- Added galactic builds and removed submodules, using .repos file.
- Added Navigation2 for semi-binary build.
- Added alternative ManualTracing method.
- Added sm_multi_stage_1.
- Added sm_atomic_performance_test_c_1 (#88).
- Added sm_atomic_performance_test_a_1 and sm_atomic_performance_test_a_2.
- Added sm_atomic_performance_test_a_2 modification (#89).
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1.
- Added sm_atomic_performance_trace_1 and sm_atomic_24hr.
- Added sm_advanced_recovery_1 reworked and fixed pre-commit.
- Added sm_advanced_recovery_1 round 4.
- Added sm_aws_aarehouse navigation base and progress.
- Added sm_respira_test_2 and sm_respira_1 format cleanup.
- Added sm_reference_library reformatting.
- Added Dockerfile with ROS distro as argument.
- Added tracing event renaming.
- Added tracing event name cleanup.
- Added tracing event name edits in tracing.md.
- Added tracing event name edits in README.md.
- Added tracing event name edits in tracing/ManualTracing.md.
- Added tracing event name edits in smacc_sm_reference_library/sm_atomic/README.md.
- Added tracing event name edits in smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Added tracing event name edits in smacc2_rta command across readmes.
- Added tracing event name edits in sm_atomic_24hr.
- Added tracing event name edits in sm_atomic_24hr cleanup.
- Added tracing event name edits in move_base_z_planners_common dependencies optimization.
- Added tracing event name edits in event generator library renaming.
- Added tracing event name edits in sm_atomic_24hr formatting cleanup.
- Added tracing event name edits in sm_atomic_24hr more cleanup.
- Added tracing event name edits in sm_respira_1 format cleanup.
- Added tracing event name edits in sm_respira_1 format cleanup pre-commit.
- Added tracing event name edits in sm_respira_1 launch command change to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Added tracing event name edits in doxygen links update (#70).
- Added tracing event name edits in README.md update.
- Added tracing event name edits in README.md update launch command.
- Added tracing event name edits in README.md update doxygen links.
- Added tracing event name edits in README.md update formatting.
- Added tracing event name edits in README.md update navigation rolling progress.
- Added tracing event name edits in README.md update navigation aws demo progress.
- Added tracing event name edits in README.md update navigation aws demo format improvements.
- Added tracing event name edits in README.md update navigation aws demo more progress.
- Added tracing event name edits in README.md update navigation aws demo format improvements.
- Added tracing event name edits in README.md update navigation aws demo format improvements.
- Added tracing event name edits in README.md update navigation aws demo more on navigation.
- Added tracing event name edits in README.md update navigation aws demo new feature.
- Added tracing event name edits in README.md update navigation aws demo formatting.
- Added tracing event name edits in README.md update navigation aws demo precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo source CI fix.
- Added tracing event name edits in README.md update navigation aws demo core improvements.
- Added tracing event name edits in README.md update navigation aws demo progress.
- Added tracing event name edits in README.md update navigation aws demo format improvements.
- Added tracing event name edits in README.md update navigation aws demo progress.
- Added tracing event name edits in README.md update navigation aws demo format improvements.
- Added tracing event name edits in README.md update navigation aws demo format improvements.
- Added tracing event name edits in README.md update navigation aws demo more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait topic message client behavior.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior formatting.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior more on navigation.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior precommit fixes.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior source CI fix.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior core improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior progress.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior format improvements.
- Added tracing event name edits in README.md update navigation aws demo wait nav2 nodes client behavior

```rst
Section_18
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_aarehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` updates.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo progress.

Changed
-------
- Corrected all linters and formatters.
- Several core improvements during navigation testing.
- Formatting improvements.

Fixed
----
- Removed some compile warnings.

Removed
-------
- Minor hotfix.

Contributors
------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_19

### Added
- Added multistage modes to `sm_multi_stage_1`.
- Added sequences to `sm_multi_stage_1`.
- Added steps to `sm_multi_stage_1`.
- Added sequence d to `sm_multi_stage_1`.
- Added sequence c to `sm_multi_stage_1`.
- Added mode_5_sequence_b.
- Added mode_4_sequence_b.
- Added finishing touches 1 to `sm_multi_stage_1`.
- Added AWS navigation to `sm_dance_bot`.

### Changed
- Reworked `sm_multi_stage_1` with new multistage modes, sequences, and steps.

### Fixed
- Fixed a recursion issue by moving a method after the one it calls.
- Fixed a waypoint issue to prevent overshot cases.
- Fixed format issues in the code.
- Fixed launch command in README.md for `sm_dance_bot_strikes_back`.
- Fixed CI formatting for Python version.

### Removed
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs` package.
- Removed parameters from `smacc`.

### Miscellaneous
- Improved navigation and performance.
- Added local action messages.
- Renamed navigation 2 stack.
- Added SVGs to READMEs.
- Updated package list.
- Added SM Atomic SM generator.
- Rolled Docker environment for universal execution.
- Added QOS durability to `SmaccPublisherClient`.
- Moved reference library SMs to `smacc2_performance_tools`.
- Added reliability QOS configuration.
- Added dependencies for Husky simulation.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

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
- Format.
- Sm_dance_bot_warehouse_3.
- Feature/sm warehouse 2 13 dec 2.
- More changes and headless.
- Merge.
- Headless and other fixes.
- Default values.
- Brettpac branch.
- Redoing sm_dance_bot_warehouse_3 waypoints.
- More Waypoints.
- SrConditional fixes and formatting.
- Move trigger logic into headers.
- Lint.
- Feature/wharehouse2 dec 14.
- Feature/sm warehouse 2 13 dec 2.
- Finetuning waypoints.
- Feature/cb pure spinning.
- Pure spinning behavior missing files.
- Feature/planner changes 16 12.
- Feature/replanning 16 dec.
- Replanning for all examples.
- Several fixes.
- Feature/undo motion 20 12.
- Improving undo motion navigation warehouse2.
- Tuning warehouse3.
- Undo tuning and errors.
- Feature/sync 21 12.
- Format issues.
- Feature/warehouse2 22 12.
- Finishing warehouse2.
- Feature/warehouse2 23 12.
- Tuning and fixes.
- Feature/minor tune.
- Fixing warehouse 3 problems and other core improvements.
- Added missing file from warehouse2.
- Backport to foxy.
- Minor format.
- Minor linking errors foxy.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components.
- Autoware demo.
- Foxy ci.
- Docker files for different revisions, warnings removal, and more testing on navigation.
- Fixing docker for foxy and galactic.
- Update file for fake hardware simulation and add file for gazebo simulation.
- Docker build files for all versions.
- Retry behavior warehouse 1.
- Multiple controllable leds plugin.
- Progress in husky demo.
- Add ignition file and update repos files.
- Progressing in husky demo.
- Improving navigation behaviors.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.

Changed
-------

- Fix formatting.
- Only rolling version should be pre-released on master.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.
- Barrel search build fix and warehouse3.
- Fixing startup problems in warehouse 3.
- Fix broken source build.
- Making models local.
- Correct codespell.
- Correct python linters warnings.
- Fix trailing spaces.

Removed
-------

- Minor broken build.
- Some reordering fixes.
- Weird moveit not downloaded repo.
```

```rst
Section_21
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
- Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Changed wording "smacc application" to "SMACC2 library".
- Update smacc_sm_reference_library/sm_atomic/README.md: edit from html to markdown syntax.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Update mentions of SMACC/ROS to SMACC2/ROS2.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Update smacc2_rta command across readmes.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files (#58).
- Fix source CI and correct README overview (#62).
- Update c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Base for the sm_aws_aarehouse navigation.
- Progressing in aws navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in aws navigation demo.
- More on navigation.
- sm_advanced_recovery_1 reworked (#83).
- Fix pre-commit.
- Trying to fix Pre-Commit.
- More sm_advanced_recovery_1 (#84).
- More sm_advanced_recovery_1 work (#85).
- sm_advanced_recovery_1 round 4 (#86).
- Brettpac branch (#87).
- sm_atomic_performance_test_a_2.
- sm_atomic_performance_test_a_1.
- sm_atomic_performance_test_c_1 (#88).

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Rename header files and correct format.
- Change extension of imports.
- Correct formatting of python file.
- Update name of package and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Update description table.
- Update table.
- Copy initial docs.
- Dockerfile w/ ROS distro as argument: use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/".
- Opened new folder for additional tracing contents.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Removed manual installation of ros-rolling-ros2trace: this is now automated in setupTracing.sh, location of sh file assumed if user follows README.md under "Getting started".
- Revert "Ignore all packages except smacc2 and smacc2_msgs": this reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Correct trailing spaces.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- More sm_atomic_24hr cleanup.
- Renaming of event generator library.
- Minor formatting.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Correct formatters.
- Branching example.
- Change extension.
- Enable cppcheck.
- Included necessary package and edited Threesome launch.
- Ignore all packages except smacc2 and smacc2_msgs.
- Update changelogs.
- Revert "Ignore all packages except smacc2 and smacc2_msgs".

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
```

```rst
Section_22
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92, #93, #94, #95, #98)
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- New feature: cb pause slam client behavior (#98)
- New feature: sm_dance_bot_lite (#99)
- New feature: sm_dance_bot visualizing turtlebot3 (#101)
- New feature: dance bot launch gz lidar choice (#102)

Changed
-------

- Updated launch command
- Corrected all linters and formatters
- Merge and progress in navigation
- Minor format improvements
- Navigation parameters fixes on sm_dance_bot
- Formatting improvements

Fixed
-----

- Fixing precommit
- Remove some compile warnings (#96)
- Minor hotfix

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_23
==========

Added
-----
- Added visualization of turtlebot3 for sm_dance_bot.
- Added lidar show/hide option for cleaning.
- Added gazebo fixes to show the robot and lidar.
- Added AWS demo functionality.
- Added support for five stages in sm_multi_stage_1.
- Added support for deep history in slam_toolbox components.
- Added support for s-pattern in sm_dance_bot.
- Added SM core test.
- Added SM Atomic SM generator.

Changed
-------
- Improved navigation and performance.
- Refactored sm_dance_bot_strikes_back.
- Refactored sm_dance_bot strikes back.
- Refactored moveit client migration.
- Updated READMEs with SVGs.
- Updated package list.
- Updated Docker environment for cross-environment execution.

Fixed
-----
- Fixed formatting issues.
- Fixed recursion possibility in method calls.
- Fixed overshot issue cases in waypoints navigator.
- Fixed compile warnings.
- Fixed CI format for Python version.
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed node creation in SM Atomic SM generator.
- Fixed dependencies and linting warnings in moveit migration.
- Fixed compiling issues in Docker environment.

Removed
-------
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs.
- Removed unused parameters in smacc.
- Removed test from main moveit cmake.

Authors
-------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Brett <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>

Section_24
-----------

### Added
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability QoS config
- Added feature for testing moveit behaviors (#167)
- Added multistage modes, sequences, and steps for sm_multi_stage_1 (#172)
- Added AWS navigation for sm dance bot (#174)
- Added warehouse2 progress (#179)
- Added Waypoint Inputs (#178)
- Added more waypoints for sm_dance_bot_warehouse_3 (#184)
- Added finetuning waypoints (#187)
- Added pure spinning behavior (#188, #189)
- Added planner changes (#191)
- Added replanning for all examples (#193, #194)
- Added undo motion improvements for warehouse2 (#196, #198)
- Added sync feature (#199)
- Added warehouse2 features (#200, #201)
- Added minor tune feature (#203)

### Changed
- Moved reference library SMs to smacc2_performance_tools
- Refactored SmaccPublisherClient to include QOS durability

### Fixed
- Fixed missing colon in SmaccPublisherClient
- Fixed pipeline error
- Fixed broken master build
- Fixed broken build issues in various features
- Fixed formatting issues
- Fixed linting issues
- Fixed errors in undo motion and warehouse3
- Fixed deadlocks in warehouse3
- Fixed missing files in warehouse2
- Fixed linking errors for Foxy
- Fixed broken build issues in barrel demo
- Fixed startup problems in warehouse 3

### Removed
- Removed redundant pre-commit cleanup

### Miscellaneous
- Progress made on moveit testing
- Progress on AWS navigation and related refactorings
- Progress on autoware machine and refining CP subscriber and publisher
- Progress on smacc core improvements for autoware demo
- Docker files created for different revisions
- Docker build files for Foxy and Galactic versions
- Barrel demo progress and search build fixes
- More testing on navigation components

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>

*pabloinigoblasco*

```rst
Section_25
==========

Added
-----
- Docker improvements.
- Runtime dependency.
- Restoring UR dependency.

Changed
-------
- Master rolling to galactic backport.
- Updating galactic repositories.

Fixed
-----
- Fixing build.

Other
-----
- More merge.
- Testing dance bot demos.

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco
- pabloinigoblasco

Co-authored-by
--------------
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- DecDury <declandury@gmail.com>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
``` 

*Autoría de Pablo Iñigo Blasco (pabloinigoblasco)*
