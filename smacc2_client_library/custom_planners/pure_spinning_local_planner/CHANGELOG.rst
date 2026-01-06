Changelog for package pure_spinning_local_planner
==================================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2 <https://github.com/robosoft-ai/SMACC2>`_
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Fixed weird issue with ros buildfarm

### Contributors
- brettpac
- pabloinigoblasco

2.3.6 (2023-03-12)
------------------

1.22.1 (2022-11-09)
-------------------
### Added
- Pre-release

### Contributors
- pabloinigoblasco

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
### Added
- Humble check
- Progress in migration to humble
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Ignored packages which should not be released
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup for sm_respira_1
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Added galactic CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Updated doxygen links
  - More readme updates
  - Created new sm from sm_respira_1
  - Several core improvements during navigation testing
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive

### Contributors
- Denis Štogl
- brettpac
- pabloinigoblasco

### Co-authored-by
- brettpac
- Denis Štogl

```rst
Section_2
=========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes to subscribe to the `/bond` topic and ensures they are alive.

Changed
-------
- Improved core functionality during navigation testing.
- Progress in AWS navigation demo.
- Minor formatting enhancements.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.

Fixed
-----
- Removed some compile warnings.
- Fixed format issues.
- Gazebo fixes for `sm_dance_bot` and `sm_dance_bot_strikes_back`.

Removed
-------
- `neo_simulation2` package removed.

Other
-----
- Various improvements and fixes in AWS navigation.
- `sm_multi_stage_1` enhancements.
- `sm_dance_bot_lite` and `sm_dance_bot_strikes_back` visualizations.
- Precommit cleanup and workflow updates.
- `sm_multi_stage_1` progress and development.
- `a3` and `mm` updates.

Commits
-------
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/cb pause slam (#98)
- Rename doxygen deployment workflow (#100)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- aws demo (#108)
- Brettpac branch (#110)
- Brettpac branch (#111)
- Remove neo_simulation2 package. (#112)
- mm (#115)

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

Section_3
=========

Version 0.1.0 (2022-01-01)
--------------------------

Added
-----

- Diverse improvements in navigation and performance (#116)
- Feature: diverse improvements in navigation performance (#117)
- Additional linting and formatting
- Introduced slam toggle and smacc deep history (#122)
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Testing progress for sm_dance_bot with slam pausing/resuming functionality
- More fixes for sm_dance_bot (#125)
- Added dance bot s pattern feature (#128)
- Polishing for sm_dance_bot and s-pattern
- Fixed a typo
- First working version of sm template and template generator (#127)
- Minor tweaks (#130)
- Resolve compile warnings (#137)
- Added SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Removed sm_dance_bot_msgs
- Pending references
- Renamed navigation 2 stack
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Updated package list (#142)
- Removed parameters smacc
- Updated workflows
- Fixed launch command in README.md for sm_dance_bot_strikes_back
- Fixed CI: format fix python version (#148)
- Added SM Atomic SM generator (#143)
- Removed node creation and created only a logger (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Refactored sm dance bot strikes back (#152)
- Slight waypoint 4 and iterations changes for robot course completion (#155)
- Feature: migration moveit client (#151)
- Initial migration to smacc2
- Fixed errors in formatting
- Added missing dependency
- Fixed linting warnings
- Progress in moveit migration testing
- Added .reps dependencies and fixed build errors
- Added dependency to ur5 client
- Docker refactoring
- Progress on move_it PR
- Improved dockerfile for building local tests
- Fixed compiling issues
- Updated readme (#164)
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability qos config
- Feature: testing moveit behaviors (#167)
- More testing on moveit behaviors
- Fixed pipeline error
- Fixed broken master build
- Added sm_pubsub_1 (#169)
- Added sm_pubsub_1 part 2 (#170)
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)
- Added multistage modes, sequences, steps, and finishing touches
- Added feature: aws navigation sm dance bot (#174)
- Added husky launch file in sm_dance_bot
- Added dependencies for husky simulation
- Updated dependencies for husky in rolling and galactic
- More progress on aws navigation and refactorings on navigation clients and behaviors
- More on aws demo
- Fixed broken build
- Minor changes (#175)
- Added warehouse2 (#177)
- Added waypoint inputs (#178)
- Progress on warehouse2 (#179)
- Format improvements (#180)
- Added sm_dance_bot_warehouse_3 (#181)

Changed
-------

- Moved method after the method it calls to prevent recursion (#126)
- Moved reference library SMs to smacc2_performance_tools (#166)

Removed
-------

- Merge markers from a python file (#119)
- Removed test from main moveit cmake

Fixed
-----

- Minor format issues (#134)
- Fixed compiling issues
- Fixed broken build

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Author: Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_4
=========

Added
-----

- Feature/sm warehouse 2 13 dec 2 (#182)
  - Added new feature for warehouse 2 on December 13th.
- Feature/wharehouse2 dec 14 (#185)
  - Implemented changes for warehouse 2 on December 14th.
- Feature/cb pure spinning (#188)
  - Introduced pure spinning behavior enhancements.
- Feature/replanning 16 dec (#193)
  - Added replanning functionality on December 16th.
- Feature/undo motion 20 12 (#196) and Feature/undo motion 20 12 (#198)
  - Improved undo motion navigation for warehouse 2.
- Feature/sync 21 12 (#199)
  - Addressed format issues for synchronization.
- Feature/warehouse2 22 12 (#200) and Feature/warehouse2 23 12 (#201)
  - Finalized warehouse 2 improvements.
- Feature/minor tune (#203)
  - Tuned and fixed minor issues.
- Feature/planner changes 16 12 (#191)
  - Made planner changes on December 16th.
- Foxy backport (#206)
  - Backported changes to Foxy version.

Changed
-------

- Updated formatting and fixed issues in SrConditional (#168).
- Adjusted trigger logic in SrConditional.
- Reworked waypoints for sm_dance_bot_warehouse_3.
- Fine-tuned waypoints for sm_dance_bot_warehouse_3.
- Improved warehouse 3 navigation and core functionality (#204).

Fixed
-----

- Resolved linting issues.
- Fixed various bugs and errors.
- Corrected formatting in multiple files.
- Addressed missing files and dependencies.
- Fixed trailing spaces and codespell errors.
- Solved Python linting warnings.
- Fixed formatting of Python files.
- Corrected formatters and linters.
- Fixed linking errors and licenses.
- Resolved build issues and updated workflows.
- Fixed extension and imports in code files.
- Enabled cppcheck and updated CI workflows.
- Fixed missing repositories for Rolling.
- Fixed bugs in smacc2 components.

Removed
-------

- Removed manual installation of ros-rolling-ros2trace.
  - Installation is now automated in setupTracing.sh.
- Deleted tracing directory and manual tracing instructions.
- Removed unnecessary packages and disabled linters.
- Ignored certain packages for linting and checking.
- Reverted changes in commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_5
=========

Added
-----

- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.

Changed
-------

- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update doxygen links (#70).
- Update c_cpp_properties.json.
- Correct all linters and formatters.

Fixed
-----

- Correct trailing spaces.

Removed
-------

- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.

Other
-----

- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

```rst
Section_6
=========

Added
-----

- New client behavior `cb_wait_topic_message` for asynchronous waiting of topic messages with optional content check for success.
- New client behavior for `nav2` to wait for nodes subscribing to the `/bond` topic to become active, with optional node selection.
- Base for `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature with gazebo lidar visualization.
- `sm_multi_stage_1` feature enhancements.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo improvements.

Changed
-------

- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in AWS navigation demo.
- Cleaning and lidar show/hide options for `sm_dance_bot`.
- Format fixes for gazebo visualization.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components.
- Introducing slam pausing/resuming functionality for `sm_dance_bot`.

Fixed
-----

- Removed some compile warnings.
- Corrected formatting issues.
- Adjusted build packages for source CI.
- Removed `neo_simulation2` package.
- Fixed recursion possibility by moving method calls after the methods they invoke.

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

- Added `Feature/dance bot s pattern (#129)` for sm_dance_bot template.
- Added `First working version of sm template and template generator. (#127)`.
- Added `minor tweaks (#130)` to improve functionality.
- Added `waypoints navigator bug (#133)` for bug fixing.
- Added `Add SM core test (#138)` for testing purposes.
- Added `using local action msgs (#139)` for local messaging.
- Added `Feature/nav2z renaming (#144)` for renaming navigation stack.
- Added `added SVGs to READMEs of atomic, dance_bot, and others (#140)` for visual enhancements.
- Added `Update package list. (#142)` for package management.
- Added `Add SM Atomic SM generator. (#143)` for generating SM Atomic.
- Added `Rolling Docker environment to be executed from any environment (#154)` for Docker environment flexibility.
- Added `Add QOS durability to SmaccPublisherClient (#163)` for durability enhancement.
- Added `Feature/testing moveit behaviors (#167)` for testing moveit behaviors.
- Added `sm_pubsub_1 (#169)` for pubsub functionality.
- Added `sm_pubsub_1 part 2 (#170)` for continuation of pubsub functionality.
- Added `sm_advanced_recovery_1 renaming (#171)` for renaming purposes.
- Added `sm_multi_stage_1 reworking (#172)` for reworking multi-stage functionality.
- Added `Feature/aws navigation sm dance bot (#174)` for AWS navigation integration.
- Added `warehouse2 (#177)` for warehouse functionality.
- Added `Waypoint Inputs (#178)` for input management.
- Added `wharehouse2 progress (#179)` for progress tracking.
- Added `sm_dance_bot_warehouse_3 (#181)` for warehouse management.
- Added `Feature/sm warehouse 2 13 dec 2 (#182)` for warehouse management.
- Added `Brettpac branch (#184)` for branch management.
- Added `SrConditional fixes and formatting (#168)` for conditional fixes.
- Added `Feature/wharehouse2 dec 14 (#185)` for warehouse management.
- Added `Feature/cb pure spinning (#188)` for spinning functionality.

Changed
-------

- Changed `minor navigation improvements (#141)` for improved navigation.
- Changed `progress on aws navigation and some other refactorings on navigation clients and behaviors` for AWS navigation improvements.
- Changed `finetuning waypoints (#187)` for improved waypoint management.

Fixed
-----

- Fixed `Fix CI: format fix python version (#148)` for CI fixing.
- Fixed `Fix formatting.` for formatting issues.
- Fixed `fixing broken master build` for build fixing.
- Fixed `fix: some formatting and templating on SrConditional` for SrConditional formatting.
- Fixed `fix: move trigger logic into headers` for trigger logic fixing.
- Fixed `fix: lint` for linting issues.

Removed
-------

- Removed `removing sm_dance_bot_msgs` for cleanup purposes.
- Removed `removing parameters smacc` for parameter cleanup.
- Removed `removing test from main moveit cmake` for test removal.
- Removed `repos dependency` for dependency removal.
- Removed `docker refactoring` for Docker refactoring removal.

Authors
-------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>

```rst
Section 8
=========

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
- Merging code from backport foxy and updates about autoware (#208)
- Foxy backport (#206)
- Add galactic CI build because Navigation2 is broken in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency to find include
- Create workflow for testing prerelease builds
- Use manual deployment for now
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
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable Navigation2 for semi-binary build

Changed
-------
- Renamed header files and corrected format
- Changed wording "smacc application" to "SMACC2 library"

Fixed
-----
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Disable ament_cpplint
- Disable cpplint and cppcheck linters
- Correct formatters
- Enable cppcheck
- Correct formatting of python file
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build

Removed
-------
- Minor broken build
- Disable disabled packages
- Ignore further packages
- Ignore all packages except smacc2 and smacc2_msgs
```

*pabloinigoblasco*

```rst
Section_9
=========

Added
-----

- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------

- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated smacc2_rta command across readmes.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md.

Fixed
-----

- Fixed source CI and corrected README overview. (#62).
- Corrected all linters and formatters.

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
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.

```rst
Section_10
==========

Added
-----
- New client behavior `cb_wait_topic_message` added for nav2. It allows waiting for nodes subscribing to the `/bond` topic to become active, with optional node selection.
- New feature `cb_pause_slam` added for pausing SLAM functionality.

Changed
-------
- Progress made in AWS navigation demo.
- Navigation parameters fixed for `sm_dance_bot`.
- Formatting improvements in various areas.
- Gazebo fixes implemented for visualizing Turtlebot3 and lidar options.

Fixed
-----
- Compilation warnings removed.
- Recursion issue fixed by moving a method to prevent potential recursion.

Removed
-------
- `neo_simulation2` package removed.

Other
-----
- Various core improvements during navigation testing.
- Precommit cleanup run performed.
- Source build enabled on PR for testing.
- `sm_multi_stage_1` functionality enhanced and tested.
- `smacc2::deep_history` syntax introduced for deep history tracking.
- Merge markers removed from a Python file.
- Additional linting and formatting applied.
- `sm_dance_bot` functionality expanded with new fixes and patterns.
- Collaboration with Ubuntu 20-04-02-amd64 (Brett) and pabloinigoblasco.
```

# Section 11

## Added
- First working version of sm template and template generator. (#127)
- Minor tweaks (#130)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
- Sm_dance_bot_lite (#136)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Initial migration to smacc2 (#151)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Sm_pubsub_1 (#169)
- Sm_pubsub_1 part 2 (#170)
- Sm_advanced_recovery_1 renaming (#171)
- Sm_multi_stage_1 reworking (#172)
- Feature/aws navigation sm dance bot (#174)
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)

## Changed
- Minor format issues (#134)
- Update readme (#164)
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

## Fixed
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing some errors introduced on formatting in migration to smacc2 (#151)
- Fixing pipeline error in testing moveit behaviors (#167)
- Fixing broken master build in testing moveit behaviors (#167)

## Removed
- Removing sm_dance_bot_msgs (#144)
- Removing parameters smacc (#147)
- Removing test from main moveit cmake in migration to smacc2 (#151)
- Removing some comments in the past in README.md (#154)

## Authors
- Pablo Iñigo Blasco
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
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
- Remove example things from Foxy CI setup. (#214)
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

Changed
-------

- Improving undo motion navigation warehouse2
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Updating subscriber publisher components
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Odom tracker improvements
- Adding forward behavior retry funcionality
- Docker files for different revisions, warnings removal and more testing on navigation
- Fixing docker for foxy and galactic
- Removing warnings (#213)
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Reset all versions to 0.0.0
- Update description table.
- Update table
- Copy initial docs
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- This is now automated in setupTracing.sh
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic

Fixed
-----

- Several fixes (#194)
- Fix
- Minor broken build
- Minor linking errors foxy
- Minor format
- Minor linking errors foxy
- Foxy backport (#206)
``` 

*pabloinigoblasco*

```rst
Section_13
==========

Added
-----
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders.
- Renamed event generator library.

Fixed
----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Minor formatting improvements.
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Deleted tracing.md.

Other
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Reverted markdowns to html.
- Edited tracing.md to reflect new tracing event names.
- Edited README.md.
- More on performance and other issues.
- More changes on performance tests.
- More on navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- More on navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Attempting precommit fixes.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Minor progress.
- Progressing in aws navigation.
- Minor progress.
- Progress in aws navigation demo.
- More on navigation.

Commits
-------
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Update tracing/ManualTracing.md (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Update smacc_sm_reference_library/sm_atomic/README.md (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Add galactic CI setup and rename rolling files. (#58).
- Fix source CI and correct README overview. (#62).
- Update c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update doxygen links (#70) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- More Readme Updates (#72) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- More Readme (#74) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Feature/aws demo progress (#80).
- Wait topic message client behavior (#81).
- Feature/wait nav2 nodes client behavior (#82).
- Feature/aws demo progress (#92).
- Brettpac branch (#87).
```
*pabloinigoblasco*

```rst
Section_14
==========

Added
-----
- Introducing new feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Added new client behavior for `nav2`, `add` for waiting for nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Optional selection of nodes to wait for.
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
- Merge and progress.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite`.
- Updates YAML.
- `sm_dance_bot` visualizing `turtlebot3`.
- Feature to launch `gz_lidar` choice for `dance_bot`.
- Cleaning and lidar show/hide option.
- More fixes.
- Gazebo fixes to show the robot and the lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Precommit cleanup run.
- AWS demo.
- Got `sm_multi_stage_1` working (barely).
- Brettpac branch.
- Gaining traction in `sm_multi_stage_1`.
- Keep hammering.
- Two stages.
- 3 part.
- 4th stage.
- 5th stage.
- A3.
- MM.
- Diverse improvements in navigation and performance.
``` 

*pabloinigoblasco*

Section_15
===========

Added
-----

- Feature/diverse improvements in navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122): Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Introducing smacc2::deep_history syntax and testing sm_dance_bot with slam pausing/resuming functionality.
- Feature/dance bot s pattern (#128): Polishing sm_dance_bot and s-pattern, fixing a typo.
- First working version of sm template and template generator (#127).
- Added SM Atomic SM generator (#143).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Rolling Docker environment to be executed from any environment (#154).
- Initial state machine transition timestamp (#165).
- Added QOS durability to SmaccPublisherClient (#163).
- Feature/migration moveit client (#151): Initial migration to smacc2, fixing formatting errors, adding missing dependencies, and progressing in moveit migration testing.
- Feature/aws navigation sm dance bot (#174): Progress on aws navigation, husky simulation dependencies, and updates for husky in rolling and galactic environments.

Changed
-------

- Move method after the method it calls to prevent recursion (#126).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Minor navigation improvements (#141).
- Using local action msgs instead of sm_dance_bot_msgs (#139).
- Removed node creation and created only a logger (#149).
- Fix CI: format fix python version (#148).
- Fixing some errors in moveit behaviors testing, linting warnings, and build errors.
- Update package list (#142).
- Removed parameters smacc (#147).
- Updated READMEs (#164).
- Added .reps dependencies and fixed build errors in move_it PR.
- Improved Dockerfile for building local tests.
- Minor changes in formatting and progress on moveit.
- Warehouse2 progress (#179).
- Waypoint Inputs (#178).

Fixed
-----

- Minor tuning to mitigate overshot issue cases in waypoints navigator bug (#133).
- Fixed launch command in README.md for sm_dance_bot_strikes_back and removed outdated comments.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.

Removed
-------

- Removed merge markers from a python file (#119).
- Removed sm_dance_bot_msgs dependencies.
- Removed test from main moveit cmake.
- Removed test from main moveit cmake.

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
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- finetuning waypoints (#187)
- Feature/cb pure spinning (#188, #189)
- pure spinning behavior missing files
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- several fixes (#194)
- Feature/undo motion 20 12 (#196, #198)
- tuning warehouse3 (#197)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- finishing warehouse2
- Feature/warehouse2 23 12 (#201)
- tuning and fixes (#202)
- Feature/minor tune (#203)
- fixing warehouse 3 problems, and other core improvements (#204)
- added missing file from warehouse2 (#205)
- docker build files for all versions (#225)
- Feature/retry behavior warehouse 1 (#226)
- Foxy backport (#206)
- Fix code generators (#221)
- Fix other build issues
- Update SM template and make example code clearly visible
- Remove use of node in the sm performance template
- Updated templated to use Blackboard storage
- Update template to resolve the global data correctly
- Update sm_name.hpp
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
- branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- ros2 launch sm_three_some sm_three_some
- First ensure you have the necessary package installed
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
```

```rst
Section_17
==========

Added
-----
- Added Dockerfile with ROS distro as argument for easy setup.
- Added setupTracing.sh script for automated installation of necessary packages and configuration of tracing group.
- Added alternative ManualTracing method.
- Added new SM markdowns.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools for performance testing improvements.
- Added new feature cb_wait_topic_message for asynchronous client behavior.

Changed
-------
- Updated wording from "smacc application" to "SMACC2 library" for clarity.
- Updated references of SMACC/ROS to SMACC2/ROS2 for consistency.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Renamed event generator library for better organization.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_reference_library formatting.
- Corrected trailing spaces in code.
- Reformatted sm_atomic_24hr for better readability.
- Updated smacc2_rta command across READMEs.
- Renamed folders, deleted tracing.md, and edited README.md for clarity.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed issue with smacc2_sm_reference_library package formatting.
- Fixed source CI setup and corrected README overview.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Collaborators
-------------
- Co-authored by Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
```

```rst
Section_18
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive; optional node selection
- `cb_pause_slam` client behavior

Changed
-------
- Corrected all linters and formatters
- Navigation parameters fixes on `sm_dance_bot`
- Minor formatting improvements
- Progress in AWS navigation demo
- Formatting improvements in AWS navigation demo
- Gazebo fixes for showing the robot and lidar in `sm_dance_bot` and `sm_dance_bot_strikes_back`

Fixed
----
- Removed some compile warnings

Removed
-------
- Redundant entries for `cb_wait_topic_message` and client behavior addition for nav2

Collaborators
-------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Commits
-------
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)
- Remove some compile warnings. (#96)
- Feature/cb pause slam (#98)
- Rename doxygen deployment workflow (#100)
- sm_dance_bot visualizing turtlebot3 (#101)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- sm_multi_stage_1 doubling (#103)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- Precommit cleanup run (#106)
- aws demo (#108)
- Brettpac branch (#110)
- Brettpac branch (#111)
```

Section 19
-----------

Added
-----

- Added multistage modes to sm_multi_stage_1 (#172)
- Added sequences and steps to sm_multi_stage_1 reworking (#172)
- Added finishing touches and readme to sm_multi_stage_1 reworking (#172)
- Added AWS navigation to sm_dance_bot (#174)
- Added dependencies for husky simulation in sm_dance_bot (#174)
- Added QOS durability to SmaccPublisherClient (#163)

Changed
-------

- Changed method order to prevent recursion in sm_dance_bot (#125)
- Changed launch command in README.md for sm_dance_bot_strikes_back (#147)
- Changed references library SMs to smacc2_performance_tools (#166)
- Changed node creation to logger in SM Atomic SM generator (#149)
- Changed format in CI for Python version (#148)
- Changed Docker environment to be executed from any environment (#154)
- Changed moveit client to smacc2 (#151)

Fixed
----

- Fixed source build on PR for testing (#113)
- Fixed formatting in neo_simulation2 package removal (#112)
- Fixed compile warnings (#137)
- Fixed minor navigation issues (#141)
- Fixed launch command in README.md (#147)
- Fixed CI format for Python version (#148)
- Fixed missing dependency in moveit migration (#151)
- Fixed compiling issues in Dockerfile (#164)
- Fixed broken master build (#167)

Removed
-------

- Removed neo_simulation2 package (#112)
- Removed merge markers from a Python file (#119)
- Removed parameters in smacc (#147)
- Removed test from main moveit CMake (#151)
- Removed test from main moveit CMake (#151)
- Removed sm_dance_bot_msgs package (#144)

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com)

```rst
Section_20
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
- More changes and headless.
- Merge.
- Headless and other fixes.
- Default values.
- Pure spinning behavior missing files.
- Feature/planner changes 16 12.
- More fixes.
- Feature/replanning 16 dec.
- Replanning for all examples.
- Several fixes.
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
- Fixing warehouse 3 problems and core improvements.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core.
- Autoware demo.
- Fixing docker for foxy and galactic.
- Update file for fake hardware simulation and add file for gazebo simulation.
- Retry behavior warehouse 1.
- Multiple controllable leds plugin.
- Progress in husky demo.
- Add ignition file and update repos files.
- Improving navigation behaviors.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.

Changed
-------

- Fix formatting.
- Minor changes.
- Format.
- Minor tune.
- Minor format fix.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.

Fixed
-----

- Fixing broken build.
- Fixing broken source build.
- Fix broken source build.
- Fixing format and minor.
- Foxy backport.
- Minor formatting fixes.

Removed
-------

- Only rolling version should be pre-released on master.
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
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- Enable Navigation2 for semi-binary build.
- Enable build of missing rolling repositories.
- Enable cppcheck.
- Enable clang-format.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files.
- Fix source CI and correct README overview.
- Update c_cpp_properties.json.

Changed
-------
- Rename header files and correct format.
- Change extension of imports.
- Update name of package and package.xml to pass liter.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Correct trailing spaces.
- Update description table.
- Update table.
- Update changelogs.
- Update ci-build-source.yml.
- Update doxygen-check-build.yml.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Update smacc_sm_reference_library/sm_atomic/README.md.
- Update sm_three_some to launch sm_three_some.launch.
- Update smacc2 and smacc2_msgs.
- Update GitHub branch reference.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links.
- Update doxygen links

```rst
Section_22
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection.
- New feature: cb pause slam client behavior.
- New feature: sm_dance_bot_lite.

Changed
-------
- Updated launch command.
- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor hotfixes.

Fixed
----
- Fixed precommit issues.
- Removed some compile warnings.

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
- Added `sm_dance_bot` visualizing `turtlebot3`.
- Added lidar show/hide option for cleaning.
- Added formatting improvements.
- Added gazebo fixes for `sm_dance_bot_lite` (#104).
- Added gazebo fixes to display the robot and lidar.
- Added `sm_multi_stage_1` doubling (#103).
- Added gazebo fixes for `sm_dance_bot_strikes_back`.
- Added AWS demo (#108).
- Added progress in `sm_multi_stage_1` (#114).
- Added diverse improvements in navigation and performance (#116).
- Added `slam_toolbox` components and `smacc2::deep_history` syntax (#122).
- Added progress in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Added `dance bot s pattern` feature (#128).
- Added first working version of `sm template` and template generator (#127).
- Added `waypoints navigator` bug fix (#133).
- Added `SM core` test (#138).
- Added minor navigation improvements (#141).
- Added `navigation 2` stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added rolling Docker environment to be executed from any environment (#154).
- Added slight waypoint 4 and iterations changes for robot course completion (#155).
- Added initial migration to `smacc2` in `moveit` client (#151).
- Added missing dependencies and fixed linting warnings.
- Added `repos` dependency and fixed build errors.
- Added dependency to `ur5` client.
- Added progress on `move_it` PR.
- Added improvements in Dockerfile for building local tests.
- Added updates to README (#164).

Changed
-------
- Changed "Finnaly" to "Finally" for correction.

Fixed
-----
- Fixed format issues (#134).
- Fixed compilation warnings (#137).
- Fixed CI format for Python version (#148).
- Fixed launch command in README.md for `sm_dance_bot_strikes_back`.

Removed
-------
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs` package.
- Removed parameters in `smacc` package.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>.
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.

Author
------
Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_24
==========

Added
-----
- Initial state machine transition timestamp (#165)
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability QOS config
- Added feature for testing moveit behaviors (#167)
- Added feature for AWS navigation sm dance bot (#174)
- Added dependencies for husky simulation
- Added Waypoint Inputs (#178)
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files
- Added several fixes (#194)
- Added undo motion navigation warehouse2
- Added tuning warehouse3 (#197)
- Added fixing warehouse 3 problems and other core improvements (#204)
- Added backport to foxy
- Added missing file from warehouse2 (#205)
- Added progress in autoware machine
- Added improvements in smacc core for autoware demo
- Added docker files for different revisions
- Added barrel demo
- Added progress in barrel husky

Changed
-------
- Moved reference library SMs to smacc2_performance_tools
- Pre-commit cleanup
- Refactored to remove line
- Minor configuration
- Refactored SrConditional fixes and formatting (#168)
- Refactored move trigger logic into headers
- Refactored lint
- Refactored default values
- Refactored formatting
- Refactored more changes and headless
- Refactored merge
- Refactored headless and other fixes
- Refactored minor changes
- Refactored replanning for all examples
- Refactored tuning and fixes
- Refactored minor tune
- Refactored fixing warehouse 3 problems and other core improvements to remove dead lock
- Refactored weird moveit not downloaded repo
- Refactored fixing docker for foxy and galactic
- Refactored fixing startup problems in warehouse 3
- Refactored fixing format

Fixed
-----
- Fixed missing colon
- Fixed pipeline error
- Fixed broken master build
- Fixed pipeline error
- Fixed broken build
- Fixed format issues
- Fixed minor linking errors for foxy
- Fixed broken build
- Fixed some reordering fixes
- Fixed minor broken build
- Fixed warnings removal
- Fixed broken build

Removed
-------
- Removed line

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
```

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
