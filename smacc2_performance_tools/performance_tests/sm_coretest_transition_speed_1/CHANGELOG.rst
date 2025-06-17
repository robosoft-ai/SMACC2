# Changelog for package sm_coretest_transition_speed_1

## 2.3.16 (2023-07-16)
- Merged branch 'humble' from `robosoft-ai/SMACC2`
- Addressed issue with ros buildfarm
- Co-authored by: brettpac
- Contributors: brettpac, pabloinigoblasco

## 2.3.6 (2023-03-12)

## 1.22.1 (2022-11-09)
- Pre-release
- Contributors: pabloinigoblasco

## 0.3.0 (2022-04-04)

## 0.0.0 (2022-11-09)
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Ignored packages not to be released
- Updated SMACC/ROS mentions to SMACC2/ROS2
- Added smacc2_performance_tools
- Improved performance tests
- Cleaned up sm_respira_1 format
- Updated smacc2_rta command across readmes
- Renamed event generator library
- Fixed various formatting issues
- Added new feature cb_wait_topic_message
- Added new client behavior for nav2
- Co-authored by: brettpac, Denis Štogl

```rst
Section_2
=========

Added
-----
- New feature: `cb_wait_topic_message` asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` behavior waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Progress in AWS navigation demo.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.
- `neo_simulation2` package removal.
- Source build enabled on PR for testing.
- Build package adjustments for source CI.
- Diverse improvements in navigation and performance.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor hotfixes.
- Cleaning and lidar show/hide option in `sm_dance_bot`.
- Format fixes.

Fixed
-----
- Remove some compile warnings.

Removed
-------
- `neo_simulation2` package.

Contributors
------------
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_3
==========

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
- Refactored to remove line and add reliability qos config.
- Reworked sm_multi_stage_1 with multistage modes, sequences, and steps.
- Updated dependencies for husky in rolling and galactic.

Fixed
-----
- Minor tuning to mitigate overshot issue cases.
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed CI format for python version.
- Removed node creation and created only a logger.
- Fixed compiling issues.

Removed
-------
- Removed merge markers from a python file.
- Removed parameters smacc.
- Removed sm_dance_bot_msgs.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section 4
=========

Added
-----
- Implemented Brettpac branch (#184) with warehouse2 improvements.
- Added SrConditional fixes and formatting (#168).
- Introduced Feature/wharehouse2 dec 14 (#185) with minor changes.
- Implemented Feature/sm warehouse 2 13 dec 2 (#186) with format adjustments.
- Added finetuning waypoints (#187) with co-authorship.
- Introduced Feature/cb pure spinning (#188) with format changes.
- Implemented Feature/replanning 16 dec (#193) with several fixes.
- Added Feature/undo motion 20 12 (#196) with improvements in navigation.
- Introduced Feature/sync 21 12 (#199) with format fixes.
- Implemented Feature/warehouse2 22 12 (#200) with format adjustments.
- Added Feature/warehouse2 23 12 (#201) with tuning and fixes.
- Introduced Feature/minor tune (#203) with tuning and fixes.
- Added Foxy backport (#206) with minor formatting fixes.

Changed
-------
- Updated changelogs and reset all versions to 0.0.0.
- Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Updated description table and table.
- Renamed "sm_three_some" to "sm_three_some.launch".
- Renamed header files and corrected format.
- Changed wording from "smacc application" to "SMACC2 library".
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Enabled Navigation2 for semi-binary build.

Removed
-------
- Deleted tracing directory and manual installation of ros-rolling-ros2trace.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```

*pabloinigoblasco*

```rst
Section_5
=========

Added
-----

- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files (#58).
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait.

Changed
-------

- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated README.md.

Fixed
-----

- Fixed source CI and corrected README overview (#62).
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
-

```rst
Section_6
=========

Added
-----

- New client behavior `cb_wait_topic_message` for asynchronous waiting and optional content check on a topic message.
- New client behavior for `nav2`: wait for nodes subscribing to `/bond` topic to become active, with optional node selection.
- Base for `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature with precommit updates.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- Lidar show/hide option in `sm_dance_bot`.
- Gazebo fixes for robot and lidar visualization in various dance bot features.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `sm_multi_stage_1` improvements.
- `smacc2::deep_history` syntax in `slam_toolbox` components.
- `slam` toggle client behaviors and testing improvements in navigation.
- `sm_dance_bot` pattern changes.

Changed
-------

- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Various core improvements during navigation testing.

Fixed
-----

- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting issues.
- Adjusted build packages for source CI.
- Removed merge markers from a Python file.
- Moved method to prevent recursion in `sm_dance_bot`.

Removed
-------

- `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_7
=========

Added
-----

- First working version of sm template and template generator (#127)
- Feature/dance bot s pattern (#129)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Feature/nav2z renaming (#144)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)

Changed
-------

- Minor tweaks (#130)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Removing sm_dance_bot_msgs
- Update package list (#142)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----

- Waypoints navigator bug (#133)
- Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
- Minor format issues (#134)
- Fixing compiling issues
- Fixing broken master build
- Warehouse2 progress (#179)
- SrConditional fixes and formatting (#168)

Removed
-------

- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references
- Removing test from main moveit cmake
- Removing parameters smacc

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
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
- Disable ament_cpplint
- Disable some packages and update workflows
- Bump ccache version
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licences
- Disable cpplint and cppcheck linters
- Correct formatters
- Enable cppcheck
- Include necessary package and edit Threesome launch
- Add workflow for checking doc build
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Reset all versions to 0.0.0
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Open new folder for additional tracing contents
- Delete tracing directory
- Move tracing.md to tracing directory
- Add setupTracing.sh
- Remove manual installation of ros-rolling-ros2trace
- Create alternative ManualTracing
- Add new sm markdowns
- Add a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Change wording "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build

Changed
-------
- Rename header files and correct format
- Correct codespell
- Correct python linters warnings
- Fix trailing spaces
- Update ci-build-source.yml
- Change extension of imports
- Correct formatting of python file
- Update doxygen-check-build.yml
- Update docs/ as source folder for documentation
- Update docs/ as output directory
- Update GitHub branch reference
- Execute on master update

Fixed
-----
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Minor broken build
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build

Removed
-------
- Merge
- Headless and other fixes
- Default values
- Pure spinning behavior missing files
- More fixes
- Replanning for all our examples
- Updating subscriber publisher components
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Foxy ci
- Minor format
- Minor linking errors foxy
- Weird moveit not downloaded repo
- Missing
- Missing sm
- Minor format issues
- Finishing warehouse2
- Tuning and fixes
- Tuning and fixes
- Minor tune
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Added missing file from warehouse2
- Minor linking errors foxy
- Bug in smacc2 component
- Reverted markdowns to html
- Additional cleanup
- Cleanup
- Cleanup
- Edited tracing.md to reflect new tracing event names
``` 

*pabloinigoblasco*

## Section_9

### Added
- Added `smacc2_performance_tools`.
- Added new feature, `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for Nav2, `wait nav2 nodes`, subscribing to the `/bond` topic and waiting for them to be alive. Optionally select the nodes to wait.

### Changed
- Updated mentions of `SMACC/ROS` to `SMACC2/ROS2`.
- Renamed folders, deleted `tracing.md`, and edited `README.md`.
- Renamed `sm_respira_1` to `sm_respira_test_2`.
- Renamed `sm_reference_library` to `event generator library`.
- Updated launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.
- Updated `smacc2_rta` command across READMEs.
- Optimized dependencies in `move_base_z_planners_common`.

### Fixed
- Corrected trailing spaces.
- Fixed source CI and corrected README overview.
- Corrected all linters and formatters.

### Removed
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used `.repos` file.
- Removed execution of `clang-format` on `smacc2_sm_reference_library` package.

### Miscellaneous
- Some progress on navigation rolling.
- Minor formatting improvements.
- Noticed a note that was not removed.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progress in AWS navigation.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress

```rst
Section_10
==========

Added
~~~~~
- New client behavior for nav2: Waits for nav2 nodes to subscribe to the /bond topic and confirm they are alive. Optional node selection available.
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.

Changed
~~~~~~~
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Core improvements during navigation testing.
- Formatting improvements.

Fixed
~~~~
- Removed some compile warnings.
- Minor hotfix.
- Fixed format issues.

Removed
~~~~~~~
- Removed `neo_simulation2` package.

Other
~~~~~
- Various improvements in navigation and performance.
- Precommit cleanup run.
- Source build enabled on PR for testing.
- Adjusted build packages of source CI.
- Moved method after the method it calls to prevent recursion.
- `smacc2::deep_history` syntax introduced.
- Introducing slam pausing/resuming functionality in `sm_dance_bot`.
- More changes in `sm_dance_bot`.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components.
- `smacc2::deep_history` syntax improvements.
- Introducing `dance_bot` 's pattern.
``` 

*pabloinigoblasco*

```rst
Section_11
==========

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
- Initial migration to smacc2
- Progress on moveit migration testing
- Finetuning waypoints (#187)

Fixed
-----
- Waypoints navigator bug (#133)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Noticed launch command was incorrect in README.md
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
```

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
- Try fixing CI for rolling. (#209)
- Add Autoware Auto Msgs into not-released dependencies. (#220)
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
- some reordering fixes
- default values
- tuning and fixes
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- branching example
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- First ensure you have the necessary package installed.
- Rename header files and correct format.
- use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Installs necessary packages and configures tracing group.
- location of sh file assumed if user follows README.md under "Getting started"

Fixed
-----
- several fixes (#194)
- fixing docker for foxy and galactic
- removing warnings (#213)
- minor broken build
- minor linking errors foxy
- minor format
- minor linking errors foxy

Removed
-------
- merge
- headless and other fixes
- pure spinning behavior missing files
- more fixes
- undo tuning and errors
- format
- weird moveit not downloaded repo
- missing
- missing sm
- foxy ci
- fix
- minor format
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Disable some packages and update workflows.
```

```rst
Section_13
==========

Added
~~~~~
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
~~~~~~~
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

Fixed
~~~~
- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fix source CI and correct README overview.
- Attempting precommit fixes.
- Correct all linters and formatters.

Removed
~~~~~~~
- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.
- Deleted tracing.md.

Other
~~~~~
- Reactivated smacc2 nav clients for rolling via submodules.
- Some progress on navigation rolling.
- More changes on performance tests.
- Performance tests improvements.
- More on performance and other issues.
- Format cleanup in sm_respira_1.
- Format cleanup pre-commit in sm_respira_1.
- Format cleanup in sm_atomic_24hr.
- Format cleanup in sm_atomic_performance_trace_1.
- Cleaned up sm_atomic_24hr.
- More cleanup in sm_atomic_24hr.
- Cleaned up sm_reference_library.
- Cleaned up sm_advanced_recovery_1.
- More cleanup in sm_advanced_recovery_1.
- Reworked sm_advanced_recovery_1.
- More work on sm_advanced_recovery_1.
- More on sm_multi_stage_1.
- More cleanup in sm_multi_stage_1.
- Base for the sm_aws_aarehouse navigation.
- Progressing in aws navigation.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- More on navigation.

Commits
~~~~~~~
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_14
==========

Added
~~~~~
- Introduce new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for nav2, waiting for nav2 nodes to subscribe to the /bond topic and ensuring they are operational. Users can select specific nodes to wait for.
- Progress in AWS navigation demo.

Changed
~~~~~~~
- Minor formatting improvements.

Fixed
~~~~
- Resolve navigation parameters issues on sm_dance_bot.

Removed
~~~~~~~
- Eliminate some compile warnings.

Other
~~~~
- Implement cb pause slam client behavior.
- Update yaml configuration.
- Rename doxygen deployment workflow.
- Add lidar show/hide option for sm_dance_bot.
- Fix gazebo visualization for sm_dance_bot.
- Enhance gazebo visualization for sm_dance_bot_strikes_back.
- Cleanup precommit tasks.
- Introduce AWS demo functionality.
- Improve sm_multi_stage_1 performance.
- Remove neo_simulation2 package.
- Enable source build on PR for testing.
- Adjust build packages for source CI.
- Implement diverse improvements in navigation and performance.
```

*pabloinigoblasco*

Section_15
===========

Added
-----
- Feature/diverse improvements in navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122): Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Introducing smacc2::deep_history syntax. Testing sm_dance_bot with slam pausing/resuming functionality.
- Feature/dance bot s pattern (#128): Polishing sm_dance_bot and s-pattern. Noticed typo.
- First working version of sm template and template generator (#127).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Add SM Atomic SM generator (#143).
- Rolling Docker environment to be executed from any environment (#154).
- Add QOS durability to SmaccPublisherClient (#163): Added QOS durability to SmaccPublisherClient. Fixed missing colon. Removed line. Added reliability QOS config.
- Feature/aws navigation sm dance bot (#174): Progress on aws navigation and refactorings on navigation clients and behaviors. Added dependencies for husky simulation. Updated dependencies for husky in rolling and galactic.

Changed
-------
- Move method after the method it calls (#126): Moved method after the method it calls to prevent recursion.
- Resolve compile warnings (#137).
- Minor navigation improvements (#141).
- Using local action msgs (#139): Now using local action msgs instead of sm_dance_bot_msgs.
- Update package list (#142).
- Update readme (#164): Updated readme with more information.
- Waypoint Inputs (#178).

Fixed
-----
- Fix CI: format fix python version (#148): Fixed CI format for python version.
- Fixing broken master build (#167): Fixed broken master build.
- Fixing broken build (#174): Fixed broken build.
- Warehouse2 (#177): Minor changes and progress.
- Format (#180): Minor formatting fixes.

Removed
-------
- Removing parameters smacc (#147): Removed parameters smacc.
- Removing node creation and create only a logger (#149).
- Removing test from main moveit cmake (#151): Removed test from main moveit cmake.

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
- Feature/wharehouse2 dec 14 (#185): warehouse2 changes
- Feature/sm warehouse 2 13 dec 2 (#186): format, headless changes
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188, #189): format, headless changes, default values
- Pure spinning behavior missing files, minor changes (#190)
- Feature/planner changes 16 12 (#191): minor changes, fixes
- Feature/replanning 16 dec (#193): replanning for examples, fixes
- Several fixes (#194)
- Minor changes (#195)
- Feature/undo motion 20 12 (#196, #198): minor changes, replanning, improving undo motion navigation
- Tuning warehouse3 (#197)
- Feature/sync 21 12 (#199): replanning, format fixes
- Feature/warehouse2 22 12 (#200, #201): replanning, format fixes, finishing warehouse2
- Feature/minor tune (#203): tuning, fixes
- Fixing warehouse 3 problems, core improvements (#204): removing dead lock, CI improvements
- Added missing file from warehouse2 (#205)
- Docker build files for all versions, dockerfiles (#225)
- Fix code generators (#221): resolving build issues, updating templates
- Feature/retry behavior warehouse 1 (#226): replanning, backport to foxy, format fixes
- Foxy backport (#206): formatting fixes, code corrections, CI updates

Changed
-------
- Updated SM template, example code visibility
- Removed use of node in SM performance template
- Updated template to use Blackboard storage
- Resolved global data in template
- Updated sm_name.hpp
- Changed ros2 launch command in sm_three_some
- Renamed header files, corrected format
- Updated workflow for doc build checking
- Created doxygen-deploy workflow
- Updated workflows for testing prerelease builds
- Renamed packages to smacc2 and smacc2_msgs
- Corrected GitHub branch reference
- Updated package name and package.xml
- Reset all versions to 0.0.0
- Ignored all packages except smacc2 and smacc2_msgs
- Updated changelogs to 0.1.0

Removed
-------
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Refining cp subscriber cp publisher
- Improvements in smacc core for autoware demo
- Autoware demo
- Foxy CI
- Minor broken build
- Some reordering fixes
- Docker files for different revisions, warnings removal, navigation testing
- Fixing docker for foxy and galactic
```

*pabloinigoblasco*

```rst
Section_17
==========

Added
-----
- Dockerfile now accepts ROS distro as an argument for easier setup.
- New setupTracing.sh script automates installation and configuration of tracing group.
- Added alternative ManualTracing method.
- Introduced new SM markdowns.
- Performance tests and improvements in smacc2_performance_tools.
- Added smacc2_sm_reference_library for SMACC2 library.
- New feature: cb_wait_topic_message for asynchronous client behavior.

Changed
-------
- Renamed "smacc application" to "SMACC2 library" for clarity.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Cleaned up sm_reference_library and sm_atomic_24hr.
- Updated launch commands and README files for consistency.

Fixed
-----
- Fixed bug in smacc2 component.
- Corrected trailing spaces in code.
- Resolved issue with missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Fixed source CI setup and corrected README overview.

Removed
-------
- Manual installation of ros-rolling-ros2trace is now automated in setupTracing.sh.
- Removed galactic builds, keeping only rolling. Submodules replaced with .repos file.

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
``` 

*pabloinigoblasco*

```rst
Section_18
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, which waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Base for the `sm_aws_aarehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_pause_slam`.
- `sm_dance_bot_lite` updates.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- Lidar show/hide option in `sm_dance_bot`.
- Gazebo fixes to show the robot and lidar in `sm_dance_bot`.
- Doubling in `sm_multi_stage_1`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Changed
~~~~~~~
- Corrected all linters and formatters.
- Formatting improvements.

Fixed
~~~~
- Navigation parameters fixes on `sm_dance_bot`.
- Minor fixes.
- Compile warnings removed.

Removed
~~~~~~
- Some compile warnings.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_19
===========

Added
-----
- Added multistage modes to sm_multi_stage_1.
- Added sequences to sm_multi_stage_1.
- Added steps to sm_multi_stage_1.
- Added sequence d to sm_multi_stage_1.
- Added sequence c to sm_multi_stage_1.
- Added mode_5_sequence_b.
- Added mode_4_sequence_b.
- Added finishing touches 1 to sm_multi_stage_1.
- Added husky launch file in sm_dance_bot for AWS navigation.

Changed
-------
- Reworked sm_multi_stage_1 for improved functionality.

Fixed
-----
- Fixed recursion issue by moving method after the method it calls.
- Fixed minor format issues.
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed CI format for Python version.
- Fixed compiling issues.

Removed
-------
- Removed neo_simulation2 package.
- Removed merge markers from a Python file.
- Removed node creation and created only a logger.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_20
==========

Added
~~~~~
- Update dependencies for husky in rolling and galactic.
- Progress on aws navigation and refactorings on navigation clients and behaviors.
- More on aws demo.
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
- Feature/undo motion 20 12.
- Improving undo motion navigation warehouse2.
- Tuning warehouse3.
- Feature/undo motion 20 12.
- Improving undo motion navigation warehouse2.
- Undo tuning and errors.
- Feature/sync 21 12.
- Format issues.
- Feature/warehouse2 22 12.
- Format issues.
- Finishing warehouse2.
- Feature/warehouse2 23 12.
- Tuning and fixes.
- Feature/minor tune.
- Fixing warehouse 3 problems and core improvements.
- Added missing file from warehouse2.
- Backport to foxy.
- Minor format.
- Minor linking errors foxy.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core.
- Autoware demo.
- Foxy CI.
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
~~~~~~~
- Fix formatting.
- Minor changes.
- Finetuning waypoints.
- Tuning and fixes.
- Minor tune.
- Fixing warehouse 3 problems and core improvements to remove dead lock.
- Weird moveit not downloaded repo.
- Fix broken source build.
- Making models local.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.
- Barrel search build fix and warehouse3.
- Fixing startup problems in warehouse 3.
- Only rolling version should be pre-released on master.
- Barrel search updates.
- Red picuup.
- Minor formatting fixes.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.

Removed
~~~~~~~
- Some reordering fixes.
- Warnings removal and more testing on navigation.
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
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Update mentions of SMACC/ROS to SMACC2/ROS2.
- Optimized dependencies in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files.
- Fix source CI and correct README overview.
- Update c_cpp_properties.json.
- Update doxygen links.
- More Readme Updates.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Progress in aws navigation demo.
- Feature/aws demo progress.
- More on navigation.
- Reworked sm_advanced_recovery_1.
- Fix pre-commit.
- Brettpac branch.

Changed
-------
- Change extension of imports.
- Correct formatting of python file.
- Rename header files and correct format.
- Update name of package and package.xml to pass liter.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Minor formatting.
- Several core improvements during navigation testing.
- Formatting improvements.

Fixed
-----
- Correct formatters.
- Correct GitHub branch reference.
- Correct trailing spaces.
- Bug in smacc2 component.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_22
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection.
- New client behavior: cb pause slam for pausing SLAM operations.
- New feature: dance bot launch gz lidar choice.

Changed
-------

- Updated launch command.
- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor formatting improvements.
- Merge and progress in development.

Fixed
-----

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
- Added visualization for `sm_dance_bot` using `turtlebot3`.
- Added option to show/hide cleaning and lidar.
- Added gazebo fixes for `sm_dance_bot` to display the robot and lidar.
- Added `sm_multi_stage_1` doubling functionality.
- Added AWS demo.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added `smacc2::deep_history` syntax.
- Added `dance bot s pattern` feature.
- Added first working version of `sm` template and template generator.
- Added `waypoints navigator` bug fix.
- Added local action messages usage.
- Added navigation 2 stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added rolling Docker environment to be executed from any environment.
- Added slight waypoint 4 and iterations changes for robot course completion.
- Added initial migration to `smacc2`.
- Added missing dependencies and fixed linting warnings for moveit migration.
- Added `.reps` dependencies and fixed build errors.
- Added dependency to `ur5` client.
- Added progress on moveit migration testing.
- Added improvements to Dockerfile for building local tests.
- Added update to README.

Changed
-------
- Improved formatting for `sm_dance_bot` and `s-pattern`.
- Corrected typo from `Finnaly` to `Finally`.
- Updated format for move_it PR.

Fixed
-----
- Fixed format issues.
- Fixed compile warnings.
- Fixed CI format for Python version.
- Fixed launch command in README.md for `sm_dance_bot_strikes_back`.
- Fixed waypoint 4 and iterations for robot course completion.
- Fixed errors introduced on formatting during migration to `smacc2`.
- Fixed compiling issues.

Removed
-------
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs`.
- Removed parameters from `smacc`.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

Section_24
===========

Added
-----
- Initial state machine transition timestamp (#165)
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability QoS config
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Added dependencies for husky simulation
- Added Waypoint Inputs (#178)
- Added repo dependency for husky launch file in sm_dance_bot
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files
- Added replanning for all our examples
- Added improving undo motion navigation warehouse2
- Added tuning and fixes (#202)
- Added minor tune (#203)
- Added fixing warehouse 3 problems, and other core improvements (#204)
- Added backport to foxy
- Added missing file from warehouse2 (#205)
- Added progress in autoware machine
- Added refining cp subscriber cp publisher
- Added improvements in smacc core for autoware demo
- Added foxy CI
- Added docker files for different revisions, warnings removal, and more testing on navigation
- Added fixing docker for foxy and galactic
- Added docker build files for all versions
- Added barrel demo
- Added barrel search build fix and warehouse3
- Added fixing startup problems in warehouse 3
- Added progress in barrel husky

Changed
-------
- Moved reference library SMs to smacc2_performance_tools
- Pre-commit cleanup
- Minor configuration
- Progress on moveit
- More testing on moveit behaviors
- Fixing pipeline error
- Fixing broken master build
- More on aws demo
- Fixing broken build
- More changes and headless
- Merge changes
- Default values
- Several fixes
- Format issues
- Finishing warehouse2
- Tuning and fixes
- Weird moveit not downloaded repo
- Minor format
- Minor linking errors foxy
- Fixing broken build
- Some reordering fixes
- Minor broken build
- Fixing format and minor
- Progress in barrel demo
- Progress in barrel demo
- Progress

Removed
-------
- Removed a missing colon
- Removed a line

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
- Testing dance bot demos.
- Updating Galactic repositories.

Changed
-------

- Master rolling to Galactic backport.
- Restoring UR dependency.
- Runtime dependency.

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
