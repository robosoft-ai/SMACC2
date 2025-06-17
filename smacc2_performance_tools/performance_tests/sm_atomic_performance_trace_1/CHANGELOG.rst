Changelog for package sm_atomic_performance_trace_1
======================================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue

### Contributors
- brettpac
- pabloinigoblasco

2.3.6 (2023-03-12)
------------------
No changes.

1.22.1 (2022-11-09)
-------------------
### Added
- Pre-release

### Contributors
- pabloinigoblasco

0.3.0 (2022-04-04)
------------------
No changes.

0.0.0 (2022-11-09)
------------------
### Added
- Ignored packages not meant for release
- Feature/master rolling to galactic backport (#236)
  - Updated references from SMACC/ROS to SMACC2/ROS2
  - Various improvements in performance testing
  - Cleaned up formatting in several packages
  - Added new performance tools
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Updated CI setup and renamed rolling files
  - Corrected README overview
  - Updated launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`
  - Added new feature for asynchronous client behavior
  - Added new client behavior for nav2

### Contributors
- brettpac
- Denis Štogl
- Ubuntu 20-04-02-amd64

### Co-authored
- brettpac
- Denis Štogl

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` behavior waits for `nav2` nodes to subscribe to the `/bond` topic and ensures they are alive. Optional node selection available.
- Progress in AWS navigation demo.
- Base for the `sm_aws_aarehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `dance bot launch gz lidar choice`: cleaning and lidar show/hide option.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `aws demo`.
- `sm_multi_stage_1` doubling.
- Diverse improvements in navigation and performance.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.

Fixed
-----

- Remove some compile warnings.
- Minor hotfix.
- Format fixes.

Removed
-------

- `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_3
==========

Added
-----
- Diverse improvements in navigation and performance.
- Additional linting and formatting.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introducing slam pausing/resuming functionality in sm_dance_bot.
- First working version of sm template and template generator.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Rolling Docker environment to be executed from any environment.
- Initial migration to smacc2 in moveit client.
- Added QOS durability to SmaccPublisherClient.
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors.
- Waypoint Inputs.

Changed
-------
- Move method after the method it calls to prevent recursion.
- Renamed reference library SMs to smacc2_performance_tools.
- Minor configuration changes.
- Husky launch file in sm_dance_bot.
- Update dependencies for husky in rolling and galactic.

Fixed
-----
- Resolved compile warnings.
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fix CI: format fix python version.
- Fixed compiling issues.

Removed
-------
- Removed merge markers from a python file.
- Removed parameters smacc.
- Removed test from main moveit cmake.
- Removed node creation and create only a logger.

Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section 4
=========

Added
-----
- Added Brettpac branch (#184) with improvements on sm_dance_bot_warehouse_3 waypoints.
- Added Feature/wharehouse2 dec 14 (#185) for warehouse2 enhancements.
- Added Feature/sm warehouse 2 13 dec 2 (#186) with format improvements.
- Added Feature/cb pure spinning (#188) for pure spinning behavior enhancements.
- Added Feature/replanning 16 dec (#193) with replanning improvements.
- Added Feature/undo motion 20 12 (#196) for undo motion navigation enhancements.
- Added Feature/sync 21 12 (#199) with format fixes.
- Added Feature/warehouse2 22 12 (#200) with format fixes and warehouse2 completion.
- Added Feature/warehouse2 23 12 (#201) with tuning and fixes.
- Added Feature/minor tune (#203) with tuning and fixes.
- Added fixes (#204) for warehouse 3 problems and core improvements.
- Added Foxy backport (#206) with minor formatting fixes.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Changed wording "smacc application" to "SMACC2 library".

Fixed
-----
- Fixed SrConditional (#168) with formatting and logic improvements.
- Fixed several linting issues.
- Fixed minor changes (#190).
- Fixed minor changes (#195).
- Fixed trailing spaces, codespell, and python linters warnings.
- Fixed formatters and corrected formatting of python files.
- Fixed missing files in warehouse2 (#205).
- Fixed minor linking errors in Foxy backport (#206).
- Fixed bug in smacc2 component.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> for improvements in waypoints.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com> for Dockerfile and tracing enhancements.
```

*pabloinigoblasco*

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
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md.
- Corrected all linters and formatters.

Fixed
-----
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview. (#62).
- Attempted pre-commit fixes.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.

Other
-----
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting.
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

```rst
Section_6
=========

Added
-----

- New client behavior `cb_wait_topic_message` added for nav2, allowing nodes to subscribe to the `/bond` topic and wait for them to be alive. Optional node selection available.
- New feature `sm_aws_warehouse` (#94) introduced as a base for AWS navigation.
- Asynchronous client behavior `cb_pause_slam` added.
- `sm_dance_bot_lite` (#99) and `sm_dance_bot_strikes_back` (#105) gazebo fixes implemented for visualizing TurtleBot3 and lidar show/hide options.
- Progress made in AWS navigation demo.
- `sm_multi_stage_1` (#103) improvements and fixes.
- `smacc2::deep_history` syntax introduced for navigating and slam toggle client behaviors.
- `sm_dance_bot_s_pattern` (#128) feature added.

Changed
-------

- Navigation parameters fixed for `sm_dance_bot`.
- Formatting and core improvements made during navigation testing.
- Minor format changes and cleanup.
- `sm_multi_stage_1` (#114) enhancements.

Fixed
-----

- Compile warnings removed (#96).
- `neo_simulation2` package removed (#112).
- Recursion issue fixed by moving method after the one it calls (#126).
- Merge markers removed from a Python file (#119).

Removed
-------

- `neo_simulation2` package removed.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

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
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)

Changed
-------

- Minor tweaks (#130)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Update package list. (#142)
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----

- Waypoints navigator bug (#133)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Fixing pipeline error
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
```

```rst
Section_8
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
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable Navigation2 for semi-binary build

Changed
-------
- Renamed "smacc application" to "SMACC2 library"

Fixed
-----
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
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
- Reactivating smacc2 nav clients for rolling via submodules
- Bug in smacc2 component
- Enable build of missing rolling repositories

Removed
-------
- Delete tracing directory
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
- Added galactic CI setup and renamed rolling files. (#58).
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated c_cpp_properties.json.
- Updated README.md.

Fixed
-----
- Corrected trailing spaces.
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
- Performance tests improvements.
- More changes on performance tests.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Minor formatting improvements.
- Noticed a note that was not removed.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progress in AWS navigation.
- Progressing in AWS navigation.
- Base for the sm_aws_aarehouse navigation.
- Formatting improvements.
- More on navigation.
- Format improvements.
- Attempting pre-commit fixes.
- Fixing pre-commit.
- Minor format fixes.
``` 

*pabloinigoblasco*

```rst
Section_10
==========

Added
~~~~~
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and ensure they are alive, with the option to select nodes to wait for.
- Base for the sm_aws_warehouse navigation.
- cb_pause_slam client behavior.
- sm_dance_bot_lite.
- sm_dance_bot visualizing turtlebot3.
- Feature: dance bot launch gz lidar choice, with cleaning and lidar show/hide options.
- gazebo fixes to show the robot and the lidar for sm_dance_bot, sm_dance_bot_lite, and sm_dance_bot_strikes_back.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- smacc2::deep_history syntax introduced.
- Feature: dance bot s pattern.

Changed
~~~~~~~
- Navigation parameters fixes on sm_dance_bot.
- Minor formatting improvements.
- Remove some compile warnings.
- Minor hotfix.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Move method after the method it calls to prevent recursion.

Fixed
~~~~
- Several core improvements during navigation testing.

Removed
~~~~~~~
- Removed neo_simulation2 package.

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
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
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)

## Changed
- Minor format issues (#134)
- Update readme (#164)
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

## Fixed
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing pipeline error
- Fixing broken master build

## Removed
- Removing sm_dance_bot_msgs
- Removing parameters smacc

## Authors
- Pablo Iñigo Blasco (pabloinigoblasco)
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
- Foxy backport (#206)
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
- branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- Rename header files and correct format.
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

- several fixes (#194)
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- weird moveit not downloaded repo
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- minor broken build
- some reordering fixes
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- minor formatting fixes

Removed
-------

- default values
- pure spinning behavior missing files
- more fixes
- undo tuning and errors
- format issues
- finishing warehouse2
- tuning and fixes
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- removing warnings (#213)
- backport to foxy
- minor format
- minor linking errors foxy
- minor broken build
- minor format
- minor linking errors foxy

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
```

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
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Cleaned up sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Reformatted sm_reference_library.
- Minor formatting improvements.

Fixed
~~~~
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed several core improvements during navigation testing.
- Fixed formatting issues.
- Corrected all linters and formatters.

Removed
~~~~~~~
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Deleted tracing.md.

Authors
~~~~~~~
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_14
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: wait for nav2 nodes subscribing to the `/bond` topic and ensure they are alive (optional node selection)
- Base for the `sm_aws_warehouse` navigation
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` visualizing TurtleBot3
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo

Changed
-------
- Progress in AWS navigation demo
- Navigation parameters fixes on `sm_dance_bot`
- Minor format improvements
- Format fixes for gazebo to show the robot and the lidar
- Format fixes for `sm_dance_bot_strikes_back`

Fixed
-----
- Remove some compile warnings
- Correct formatting for `neo_simulation2` package removal
- Enable source build on PR for testing
- Adjust build packages of source CI
- Diverse improvements in navigation and performance

Removed
-------
- Remove `neo_simulation2` package

Contributors
------------
- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

# Changelog

## [Unreleased]

### Added
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added dependencies for husky in rolling and galactic in AWS navigation sm dance bot (#174)
- Added dependencies for husky simulation in AWS navigation sm dance bot (#174)
- Added warehouse2 (#177)
- Added waypoint inputs (#178)

### Changed
- Moved reference library SMs to smacc2_performance_tools (#166)
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)
- Renamed sm_multi_stage_1 (#171)
- Renamed sm_multi_stage_1 to sm_multi_stage sequences (#172)
- Renamed sm_multi_stage_1 to sm_multi_state_1 steps (#172)
- Renamed sm_multi_stage_1 sequence d (#172)
- Renamed sm_multi_stage_1 c sequence (#172)
- Renamed mode_5_sequence_b (#172)
- Renamed mode_4_sequence_b (#172)
- Renamed sm_multi_stage_1 most (#172)
- Renamed finishing touches 1 (#172)

### Fixed
- Fix CI: format fix python version (#148)
- Fix compiling issues (#164)
- Fix broken master build in testing moveit behaviors (#167)
- Fix broken build in AWS navigation sm dance bot (#174)
- Fix formatting in AWS navigation sm dance bot (#174)
- Fix launch command in README.md for sm_dance_bot_strikes_back (#147)
- Fix warehouse2 progress (#179)

### Removed
- Removed node creation and create only a logger (#149)
- Removed parameters smacc (#147)
- Removed sm_dance_bot_msgs in nav2z renaming (#144)
- Removed test from main moveit cmake in migration moveit client (#151)
- Removed some comments in the past in launch command for sm_dance_bot_strikes_back (#147)

### Miscellaneous
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

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
- Fix code generators (#221)
- Feature/retry behavior warehouse 1 (#226)
- Foxy backport (#206)

Changed
-------
- Update SM template and make example code clearly visible.
- Remove use of node in the sm performance template.
- Updated templated to use Blackboard storage.
- Update template to resolve the global data correctly.
- Update sm_name.hpp
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
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- Rename header files and correct format.
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
- 0.1.0

Removed
-------
- Some reordering fixes
- Minor broken build
- Minor linking errors foxy
- Disable disabled packages
- Change extension
- Disable some packages
```

```rst
Section_17
==========

Added
-----
- Added setupTracing.sh script for installing necessary packages and configuring tracing group.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added galactic CI setup and renamed rolling files. (#58)
- Added source CI and corrected README overview. (#62)
- Added more Readme Updates (#72) and More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Added base for the sm_aws_aarehouse navigation.
- Added sm_atomic_performance_test_a_2, sm_atomic_performance_test_a_1, sm_atomic_performance_test_c_1, sm_atomic_performance_test_a_2, sm_multi_stage_1.
- Added sm_atomic_24hr, sm_atomic_performance_trace_1, sm_advanced_recovery_1.
- Added sm_advanced_recovery_1 round 4, sm_advanced_recovery_1 work, sm_advanced_recovery_1 reworked.
- Added sm_multi_stage_1, sm_atomic_performance_test_c_1, sm_atomic_performance_test_a_2, sm_atomic_performance_test_a_1.
- Added sm_multi_stage_1, sm_atomic_performance_test_c_1, sm_atomic_performance_test_a_2, sm_atomic_performance_test_a_1.

Changed
-------
- Updated description table.
- Updated table.
- Changed wording "smacc application" to "SMACC2 library".
- Renamed tracing events.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated smacc2_rta command across readmes.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated c_cpp_properties.json.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Corrected trailing spaces.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Reactivated smacc2 nav clients for rolling via submodules.
- Reverted markdowns to html.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax.
- Edited tracing/ManualTracing.md.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.
- Edited tracing.md to reflect new tracing event names.

```rst
Section_18
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection.
- Base for the `sm_aws_aarehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_pause_slam`.
- Visualizing `turtlebot3` in `sm_dance_bot_lite`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.

Changed
~~~~~~~
- Corrected all linters and formatters.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor formatting improvements.
- Merge and progress.
- Minor hotfix.
- Cleaning and lidar show/hide option.
- Cleaning files and formatting work.
- Gazebo fixes to show the robot and the lidar.
- Precommit cleanup run.

Fixed
~~~~
- Removed some compile warnings.

Removed
~~~~~~~
- Compile warnings.

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section 19
-----------

Added
~~~~~
- Added multistage modes to sm_multi_stage_1 (#172).
- Added sequences and steps to sm_multi_stage_1.
- Added finishing touches to sm_multi_stage_1.
- Added AWS navigation to sm_dance_bot (#174).

Changed
~~~~~~~
- Renamed sm_advanced_recovery_1 (#171).
- Reworked sm_multi_stage_1 for improved functionality.

Fixed
~~~~
- Fixed waypoint and iteration changes for course completion (#155).
- Fixed CI formatting for Python version (#148).

Removed
~~~~~~~
- Removed neo_simulation2 package (#112).
- Removed redundant sm_dance_bot_msgs package.
- Removed unnecessary parameters from smacc (#147).
- Removed node creation in favor of logger (#149).

Other
~~~~~
- Co-authored commits by Ubuntu 20-04-02-amd64, Brett (brett@robosoft.ai), DecDury (declandury@gmail.com), and Denis Štogl (destogl@users.noreply.github.com).
- Various minor improvements, linting, and formatting changes.
- Updated README files and added SVGs for better documentation.
- Docker environment now executable from any environment.
- Added QOS durability to SmaccPublisherClient.
- Moved reference library SMs to smacc2_performance_tools.
- Progress made in moveit migration testing.
- Husky launch file dependencies added for husky simulation.
- Workflows and launch commands updated for accuracy.
- Improved navigation and performance in various components.
- Ongoing progress in testing and refining sm_dance_bot functionality.

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
- Finetuning waypoints.
- Feature/cb pure spinning.
- Feature/planner changes 16 12.
- Feature/replanning 16 dec.
- Feature/undo motion 20 12.
- Tuning warehouse3.
- Feature/warehouse2 22 12.
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
- Barrel demo.
- Barrel search build fix and warehouse3.
- Progress in barrel husky.
- Barrel search updates.
- Making models local.
- Red picuup.
- Foxy backport.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.

Changed
-------
- Fix formatting.
- Minor changes.
- Merge.
- Headless and other fixes.
- Default values.
- Several fixes.
- Format issues.
- Tuning and fixes.

Fixed
-----
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
- Pure spinning behavior missing files.
- Weird moveit not downloaded repo.
- Missing file.
- Some reordering fixes.
- Warnings removal and more testing on navigation.
- Fixing docker for foxy and galactic.
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
- Use docs/ as source folder and output directory for documentation.
- Added setupTracing.sh:
  Installs necessary packages and configures tracing group.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added sm_respira_test_2.
- Added sm_atomic_performance_test_a_1 and sm_atomic_performance_test_a_2.
- Added sm_atomic_performance_test_c_1.

Changed
-------
- Change extension of imports.
- Rename header files and correct format.
- Update name of package and package.xml to pass liter.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Update c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Update doxygen links.
- Update description table.
- Update table.
- Update changelogs.
- Update ci-build-source.yml.
- Update doxygen-check-build.yml.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Update smacc_sm_reference_library/sm_atomic/README.md.
- Update smacc2_sm_reference_library/sm_atomic/README.md.
- Update sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Update smacc2 and smacc2_msgs references.
- Update GitHub branch reference.
- Update name of package and package.xml.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and smacc2_msgs references.
- Update smacc2 and

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
- Progress in aws navigation demo.
- Merge and progress.

Fixed
-----

- Fixed precommit issues.
- Removed some compile warnings.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

Section_23
==========

Added
-----
- Added visualization of turtlebot3 in sm_dance_bot.
- Added lidar show/hide option for cleaning.
- Added gazebo fixes for showing the robot and lidar in sm_dance_bot.
- Added AWS demo functionality.
- Added support for 5 stages in sm_multi_stage_1.
- Added source build enablement for testing.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components in Feature/slam toggle and smacc deep history.
- Added smacc2::deep_history syntax support.
- Added slam pausing/resuming functionality in sm_dance_bot.
- Added s-pattern refinement in sm_dance_bot.
- Added first working version of sm template and template generator.
- Added SM core test.
- Added local action messages usage.
- Added navigation 2 stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added rolling Docker environment execution from any environment.
- Added progress in moveit migration testing.

Changed
-------
- Improved navigation and performance.
- Updated format in Feature/slam toggle and smacc deep history.
- Updated launch command in README.md for sm_dance_bot_strikes_back.
- Updated format for CI in Fix CI: format fix python version.
- Updated Dockerfile for building local tests.
- Updated readme with more information.

Fixed
-----
- Fixed recursion issue in method calls.
- Fixed overshot issue cases in waypoints navigator.
- Fixed minor format issues.
- Fixed compile warnings.
- Fixed node creation in SM Atomic SM generator.
- Fixed format errors in migration to smacc2.
- Fixed linting warnings.
- Fixed build errors.
- Fixed compiling issues.

Removed
-------
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs.
- Removed parameters in smacc.
- Removed test from main moveit cmake.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

Section_24
-----------

Added
~~~~~

- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability QOS config
- Added feature for testing moveit behaviors (#167)
- Added feature for AWS navigation sm dance bot (#174)
- Added dependencies for husky simulation
- Added Waypoint Inputs (#178)
- Added progress on AWS navigation and refactorings on navigation clients and behaviors
- Added warehouse2 progress (#179)
- Added progress on sm_dance_bot_warehouse_3
- Added more waypoints to sm_dance_bot_warehouse_3
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files
- Added several fixes (#194)
- Added improving undo motion navigation warehouse2
- Added tuning warehouse3 (#197)
- Added tuning and fixes (#202)
- Added minor tune (#203)
- Added fixing warehouse 3 problems and other core improvements (#204)
- Added backport to foxy
- Added missing file from warehouse2 (#205)
- Added progress in autoware machine
- Added refining cp subscriber cp publisher
- Added improvements in smacc core for autoware demo
- Added autoware demo
- Added docker files for different revisions, warnings removal, and more testing on navigation
- Added fixing docker for foxy and galactic
- Added docker build files for all versions
- Added barrel demo
- Added barrel search build fix and warehouse3
- Added fixing startup problems in warehouse 3
- Added progress in barrel husky

Changed
~~~~~~~

- Changed pre-commit cleanup
- Changed reference library SMs to smacc2_performance_tools
- Changed adding a missing colon
- Changed removing a line
- Changed repo dependency
- Changed husky launch file in sm_dance_bot
- Changed update dependencies for husky in rolling and galactic
- Changed default values
- Changed merge
- Changed headless and other fixes
- Changed some formatting and templating on SrConditional
- Changed move trigger logic into headers
- Changed lint
- Changed format issues

Fixed
~~~~~

- Fixed fixing pipeline error
- Fixed fixing broken master build
- Fixed fixing broken build
- Fixed minor changes
- Fixed minor format
- Fixed minor linking errors for foxy
- Fixed minor broken build
- Fixed some reordering fixes
- Fixed fixing broken build

Removed
~~~~~~~

- Removed weird moveit not downloaded repo

```rst
Section_25 (Unreleased)
=======================

Added
-----
- Docker improvements.
- Testing dance bot demos.
- Updating galactic repos.
- Runtime dependency.
- Restoring UR dependency.

Changed
-------
- Master rolling to galactic backport.

Fixed
-----
- Fixing build.

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco

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
