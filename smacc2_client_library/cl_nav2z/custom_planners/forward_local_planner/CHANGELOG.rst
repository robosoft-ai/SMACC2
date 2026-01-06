Changelog for package forward_local_planner
==============================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for a weird issue with ros buildfarm.

### Contributors
- brettpac
- pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
No changes.

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release.

### Contributors
- pabloinigoblasco

Version 0.3.0 (2022-04-04)
--------------------------
No changes.

Version 0.0.0 (2022-11-09)
---------------------------
### Added
- Publisher progress in migration to humble.
- Reverted "Ignore packages which should not be released."
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
  - Fixed source CI and corrected README overview.
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
  - Added new feature, cb_wait_topic_message.
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic.
  - Corrected all linters and formatters.

### Contributors
- Denis Štogl
- pabloinigoblasco
- Ubuntu 20-04-02-amd64

### Co-Authored
- brettpac
- Denis Štogl

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Nodes to wait can be optionally selected.
- New client behavior: `cb_pause_slam` for pausing SLAM.
- New client behavior: `sm_dance_bot_lite` for visualizing TurtleBot3.
- New client behavior: `sm_multi_stage_1` doubling.
- New client behavior: `sm_dance_bot_strikes_back` gazebo fixes.

Changed
-------

- Improved core functionality during navigation testing.
- Formatting enhancements.
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Gazebo fixes to show the robot and lidar.
- Minor hotfixes.

Fixed
-----

- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.

Removed
-------

- Removed `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_3
=========

Version 0.1.0 (2022-01-01)
--------------------------

### Added
- Diverse improvements in navigation and performance (#116)
- Feature: diverse improvements in navigation performance (#117)
- Additional linting and formatting
- Added slam toggle and smacc deep history feature (#122)
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Introducing slam pausing/resuming functionality in testing sm_dance_bot
- More fixes for sm_dance_bot (#125)
- Added dance bot s pattern feature (#128)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability qos config to SmaccPublisherClient
- Added durability qos to SmaccPublisherClient
- Added warehouse2 (#177)
- Added waypoint inputs (#178)

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)
- Renamed sm_multi_stage_1
- Reworked multistage modes, sequences, steps, and finishing touches

### Fixed
- Minor format issues (#134)
- Fixed launch command in README.md for sm_dance_bot_strikes_back
- Fixed CI format for Python version (#148)

### Removed
- Removed merge markers from a Python file (#119)
- Removed node creation and created only a logger (#149)
- Removed parameters smacc (#147)
- Removed test from main moveit CMake

### Authors
- Pablo Iñigo Blasco
- Brett (brett@robosoft.ai)
- DecDury (declandury@gmail.com)
- Denis Štogl (denis@stogl.de, destogl@users.noreply.github.com)

```rst
Section_4
=========

Added
-----
- Feature/sm warehouse 2 13 dec 2 (#182)
  - Implemented new warehouse feature.
- Feature/wharehouse2 dec 14 (#185)
  - Added improvements to warehouse functionality.
- Feature/cb pure spinning (#188)
  - Introduced pure spinning behavior.
- Feature/replanning 16 dec (#193)
  - Enhanced replanning capabilities.
- Feature/undo motion 20 12 (#196, #198)
  - Improved undo motion navigation in warehouse2.
- Feature/sync 21 12 (#199)
  - Addressed synchronization issues.
- Feature/warehouse2 22 12 (#200)
  - Resolved warehouse2 format issues.
- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2.
- Feature/minor tune (#203)
  - Made minor adjustments for optimization.
- Fix trailing spaces, codespell, python linters warnings (#206)
  - Corrected various code issues and added Galactic CI build.

Changed
-------
- Updated sm_three_some launch command.
- Renamed header files and corrected formatting.
- Updated doxygen-check-build.yml and doxygen-deploy.yml workflows.
- Renamed to smacc2 and smacc2_msgs.
- Updated GitHub branch reference.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Updated changelogs.

Removed
-------
- Manual installation of ros-rolling-ros2trace.
  - Now automated in setupTracing.sh script.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_5
=========

Added
-----
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected trailing spaces.
- Cleaned up sm_reference_library.
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.

Fixed
-----
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview.
- Fixed pre-commit issues.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.

Other
-----
- Some progress on navigation rolling.
- Performance tests improvements.
- More on performance and other issues.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progressing in AWS navigation.
- Minor formatting improvements.
- More changes on performance tests.
- More on navigation.
- Noticed a note that was not removed.
``` 

*pabloinigoblasco*

```rst
Section_6
=========

Added
-----

- New client behavior `cb_wait_topic_message` added for nav2. It allows waiting for nav2 nodes to subscribe to the `/bond` topic and ensuring they are alive. Nodes to wait for can be optionally selected.
- New feature `cb_pause_slam` added.
- New feature `sm_dance_bot_lite` added.
- New feature `sm_dance_bot_strikes_back` added.
- New feature `sm_multi_stage_1` added.
- New feature `sm_multi_stage_1` doubling added.
- AWS demo feature added.
- Progress in AWS navigation demo.
- Progress in AWS navigation.
- Progress in navigation testing.
- Progress in `sm_aws_warehouse` navigation.
- Progress in `sm_dance_bot` fixes.
- Progress in `sm_dance_bot` visualizing TurtleBot3.
- Progress in `sm_dance_bot` launch Gazebo lidar choice.
- Progress in `sm_multi_stage_1` working.
- Progress in diverse improvements in navigation and performance.
- Progress in slam toggle client behaviors and `slam_toolbox` components.
- Progress in testing `sm_dance_bot` introducing slam pausing/resuming functionality.
- Progress in `sm_dance_bot` S pattern.

Changed
-------

- Minor format changes.
- Navigation parameters fixes on `sm_dance_bot`.
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.

Fixed
-----

- Format fixes.
- Gazebo fixes to show the robot and the lidar.
- Fix format.

Removed
-------

- Removed `neo_simulation2` package.

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
- Feature/sm dance bot strikes back refactoring (#152)
- Feature/migration moveit client (#151)
- Add SM core test (#138)
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
- Minor format issues (#134)
- Fix CI: format fix python version (#148)
- Update package list. (#142)
- Update readme (#164)
- Add dependencies for husky in rolling and galactic.
- Finetuning waypoints (#187)

Fixed
-----

- Waypoints navigator bug (#133)
- Resolve compile warnings (#137)
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
- Removing parameters smacc
- Removing references to sm_dance_bot_msgs
- Removing test from main moveit cmake

Other
-----

- Noticed typo "Finnaly" corrected to "Finally"
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Noticed some minor tuning to mitigate overshot issue cases
- Noticed some progress in the sm_dance_bot tests
- Noticed some progress on markers cleanup
- Noticed some progress on moveit migration testing
- Noticed some progress on moveit
- Noticed some more readme updates
- Noticed some progress on aws navigation and some other refactorings on navigation clients and behaviors
- Noticed some warehouse2 progress
- Noticed some more waypoints added
- Noticed some slight waypoint 4 and iterations changes so robot can complete course
- Noticed some progress on move_it PR
- Noticed some minor dockerfile test workaround
- Noticed some improving dockerfile for building local tests
- Noticed some repos dependency added
- Noticed some dependency to ur5 client added
- Noticed some docker refactoring
- Noticed some progress on moveit
- Noticed some more testing on moveit behaviors
- Noticed some minor configuration
- Noticed some fixing pipeline error
- Noticed some warehouse2
- Noticed some warehouse2 progress
- Noticed some format
- Noticed some more changes and headless
- Noticed some merge
- Noticed some headless and other fixes
- Noticed some default values
- Noticed some more waypoints finetuning
- Noticed some more changes and headless
- Noticed some default values
- Noticed some minor changes
- Noticed some format
- Noticed some more changes and headless
- Noticed some merge
- Noticed some headless and other fixes
- Noticed some default values
- Noticed some minor changes
```

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
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Execute on master update
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
- Added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Update smacc_sm_reference_library/sm_atomic/README.md

Changed
-------
- Rename header files and correct format
- Change extension of imports
- Correct wording from "smacc application" to "SMACC2 library"

Fixed
-----
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Fixing warehouse 3 problems, and other core improvements (#204)
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
- Several fixes
- Improving undo motion navigation warehouse2
- Undo tuning and errors
- Format issues
- Finishing warehouse2
- Tuning and fixes
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
- Disable disabled packages
- Change extension
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Use manual deployment for now
- Ignore all packages except smacc2 and smacc2_msgs
- Reverted markdowns to html
- Added README tutorial for Dockerfile
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
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links.
- Updated README.md with launch command.

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
- Minor formatting.
- Noticed a note that was not removed.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progressing in AWS navigation.
- More on navigation.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
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
- New client behavior `cb_wait_topic_message`: asynchronous behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: waits for nav2 nodes to subscribe to the /bond topic and confirms they are active. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- Gazebo fixes to show the robot and lidar.
- Progress in AWS navigation demo.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components.
- Introducing `smacc2::deep_history` syntax.
- Slam pausing/resuming functionality for `sm_dance_bot`.
- More fixes and improvements in `sm_dance_bot`.

Changed
-------
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Remove some compile warnings.
- Correct formatting for `neo_simulation2` package removal.
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.

Fixed
----
- Merge and progress.
- Minor hotfix.
- Precommit cleanup run.

Removed
-------
- Removed `neo_simulation2` package.

Contributors
------------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

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
- Add SM core test (#138)
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
- Removing sm_dance_bot_msgs
- Update package list. (#142)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----
- Waypoints navigator bug (#133)
- Resolve compile warnings (#137)
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing compiling issues
- Fixing broken master build
- SrConditional fixes and formatting (#168)

Removed
-------
- Removing parameters smacc
- Removing test from main moveit cmake

Other
-----
- Polishing sm_dance_bot and s-pattern
- Noticed typo "Finnaly" corrected to "Finally"
- Minor format issues (#134)
- Minor tuning to mitigate overshot issue cases
- Some more progress on markers cleanup
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Precommit cleanup
- Pending references
- Docker refactoring
- Repos dependency
- Progress on move_it PR
- Progress on moveit migration testing
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- Warehouse2 progress (#179)
- Format (#180)
- Format
- More changes and headless
- Merge
- Headless and other fixes
- Default values
- Brettpac branch (#184)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More waypoints
- Finishing touches 1
- Readme updates
- More readme updates
- Update readme (#164)
- Initial state machine transition timestamp (#165)
- Multistage modes
- Sm_multi_stage sequences
- Sm_multi_state_1 steps
- Sm_multi_stage_1 sequence d
- Sm_multi_stage_1 c sequence
- Mode_5_sequence_b
- Mode_4_sequence_b
- Sm_multi_stage_1 most
- Warehouse2
- Waypoint Inputs (#178)
```

*pabloinigoblasco*

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
- Foxy backport (#206)
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
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter.
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
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- branching example
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- Rename header files and correct format.
- Update doxygen-check-build.yml
- Use manual deployment for now.
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Correct GitHub branch reference.
- Execute on master update
- Reset all versions to 0.0.0
- Update ci-build-source.yml
- Change extension
- Change extension of imports.

Fixed
-----
- minor broken build
- fixing docker for foxy and galactic
- removing warnings (#213)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- weird moveit not downloaded repo
- added missing file from warehouse2 (#205)
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.

Removed
-------
- missing
- missing sm
- backport to foxy
- minor format
- minor linking errors foxy

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>

```

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
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------

- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.

Fixed
-----

- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Correct all linters and formatters.

Removed
-------

- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.
- Deleted tracing.md.

Other
-----

- Reactivated smacc2 nav clients for rolling via submodules.
- Reverted markdowns to html.
- Edited tracing.md to reflect new tracing event names.
- Edited README.md.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- More changes on performance tests.
- More on performance and other issues.
- Progress in aws navigation demo.
- More on navigation.
- Some progress on navigation rolling.
- Formatting improvements.
- Cleanup.
- Additional cleanup.
- Format cleanup.
- Format cleanup pre-commit.
- Reworked sm_advanced_recovery_1.
- Fix pre-commit.
- Trying to fix Pre-Commit.
- More sm_advanced_recovery_1 work.
- More on sm_advanced_recovery_1.
- More sm_multi_stage_1.
- Fixing precommit.
- More sm_multi_stage_1.
- More on sm_atomic_performance_test_a_2.
- Modifying sm_atomic_performance_test_a_2.
- More sm_atomic_performance_test_c_1.
- More sm_atomic_performance_test_a_1.
- More sm_atomic_performance_test_a_2.
- More sm_atomic_24hr cleanup.
- Clean up of sm_atomic_24hr.
- More Readme Updates.
- More Readme.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Feature/aws demo progress.
- Feature/wait nav2 nodes client behavior.
- Brettpac branch.

Commits
-------

- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Update tracing/ManualTracing.md (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Update smacc_sm_reference_library/sm_atomic/README.md (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Update c_cpp_properties.json.
- Update README.md.
- Update doxygen links (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
```
*pabloinigoblasco*

```rst
Section_14
==========

Added
-----
- Introducing new feature `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- New client behavior `add` for nav2: waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
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
- Updates yaml.
- `sm_dance_bot` visualizing `turtlebot3`.
- Cleaning and lidar show/hide option.
- Cleaning files and fixing formatting.
- More fixes.
- Gazebo fixes to show the robot and the lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Precommit cleanup run.
- AWS demo.
- Got `sm_multi_stage_1` working (barely).
- Brettpac branch.
- Gaining traction with `sm_multi_stage_1`.
- More progress on `sm_multi_stage_1`.
- Keep hammering.
- Two stages.
- 3 part.
- 4th stage.
- 5th stage.
- Diverse improvements in navigation and performance.
``` 

*pabloinigoblasco*

# Changelog

## [Unreleased]

### Added
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added dependencies for husky in rolling and galactic in AWS navigation (#174)

### Changed
- Moved reference library SMs to smacc2_performance_tools (#166)
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)
- Renamed sm_multi_stage_1 (#172)
- Renamed sm_multi_stage_1 sequence d (#172)
- Renamed sm_multi_stage_1 c sequence (#172)
- Renamed mode_5_sequence_b (#172)
- Renamed mode_4_sequence_b (#172)
- Updated navigation 2 stack renaming (#144)
- Updated package list (#142)
- Updated readme (#164)
- Updated readme (#164)
- Updated readme (#164)
- Updated dependencies for husky in rolling and galactic in AWS navigation (#174)

### Fixed
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#147)
- Fixed CI: format fix python version (#148)
- Fixed compiling issues (#154)
- Fixed broken master build in testing moveit behaviors (#167)
- Fixed pipeline error in testing moveit behaviors (#167)
- Fixed broken build in AWS navigation (#174)

### Removed
- Removed merge markers from a python file (#119)
- Removed parameters smacc (#147)
- Removed node creation and create only a logger (#149)
- Removed sm_dance_bot_msgs in nav2z renaming (#144)
- Removed test from main moveit cmake in migration moveit client (#151)

## [1.0.0] - 2022-01-01

### Added
- First working version of sm template and template generator. (#127)
- Added SM core test (#138)
- Added SM Atomic SM generator. (#143)

### Changed
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components in slam toggle and smacc deep history (#122)
- Progress in testing sm_dance_bot introducing slam pausing/resuming functionality in slam toggle and smacc deep history (#122)
- Progress in the sm_dance_bot tests (#135)
- Progress on moveit migration testing in migration moveit client (#151)

### Fixed
- Minor tuning to mitigate overshot issue cases in waypoints navigator bug (#133)
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Fixing some errors introduced on formatting in migration moveit client (#151)
- Fixing some more linting warnings in migration moveit client (#151)
- Fixing compiling issues in migration moveit client (#151)

### Removed
- Pending references in nav2z renaming (#144)

## [0.1.0] - 2021-12-01

### Added
- More changes in sm_dance_bot (#125)
- More changes in sm_dance_bot (#128)
- More changes in sm_dance_bot (#129)
- More changes in sm_dance_bot (#131)
- More changes in sm_dance_bot (#132)
- More changes in sm_dance_bot (#136)
- More testing on moveit behaviors (#167)
- More testing on moveit behaviors (#167)
- More testing on moveit behaviors (#167)
- More testing on moveit behaviors (#167)
- More testing on moveit behaviors (#167)
- More testing on moveit behaviors (#167)

### Changed
- Move method after the method it calls to prevent recursion in (#126)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#128)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in dance bot s pattern (#129)
- Polishing sm_dance_bot and s-pattern in

```rst
Section_16
==========

Added
-----
- Brettpac branch (#184)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- Feature/wharehouse2 dec 14 (#185)
- warehouse2
- Feature/sm warehouse 2 13 dec 2 (#186)
- format
- more changes and headless
- merge
- headless and other fixes
- default values
- finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- pure spinning behavior missing files
- minor changes (#190)
- Feature/planner changes 16 12 (#191)
- minor changes
- more fixes
- Feature/replanning 16 dec (#193)
- minor changes
- replanning for all our examples
- several fixes (#194)
- minor changes (#195)
- Feature/undo motion 20 12 (#196)
- minor changes
- replanning for all our examples
- improving undo motion navigation warehouse2
- tuning warehouse3 (#197)
- Feature/undo motion 20 12 (#198)
- minor changes
- replanning for all our examples
- improving undo motion navigation warehouse2
- undo tuning and errors
- Feature/sync 21 12 (#199)
- minor changes
- replanning for all our examples
- format issues
- Feature/warehouse2 22 12 (#200)
- minor changes
- replanning for all our examples
- format issues
- finishing warehouse2
- Feature/warehouse2 23 12 (#201)
- minor changes
- replanning for all our examples
- tuning and fixes (#202)
- Feature/minor tune (#203)
- tuning and fixes
- minor tune
- fixing warehouse 3 problems, and other core improvements (#204)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- added missing file from warehouse2 (#205)
- backport to foxy
- minor format
- minor linking errors foxy
- docker build files for all versions
- dockerfiles (#225)
- Feature/retry behavior warehouse 1 (#226)
- minor changes
- replanning for all our examples
- backport to foxy
- minor format
- minor linking errors foxy

Changed
-------
- Fix code generators (#221)
- Fix other build issues.
- Update SM template and make example code clearly visible.
- Remove use of node in the sm performance template.
- Updated templated to use Blackboard storage.
- Update template to resolve the global data correctly.
- Update sm_name.hpp
- Foxy backport (#206)
- minor formatting fixes
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
- branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- ros2 launch sm_three_some sm_three_some
- to
- ros2 launch sm_three_some sm_three_some.launch
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

Fixed
-----
- SrConditional fixes and formatting (#168)
- fix: some formatting and templating on SrConditional
- fix: move trigger logic into headers
- fix: lint
- fixing docker for foxy and galactic

Removed
-------
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- missing
- foxy ci
- fix
- minor broken build
- some reordering fixes
- minor
- docker files for different revisions, warnings removval and more testing on navigation
```

*pabloinigoblasco*

```rst
Section_17
==========

Added
-----

- Added setupTracing.sh script for installing necessary packages and configuring tracing group.
- Added alternative ManualTracing method.
- Added new SM markdowns.
- Added Dockerfile for Rolling and Galactic.
- Added galactic CI setup and renamed rolling files. (#58)
- Added source CI and corrected README overview. (#62)
- Added doxygen links update (#70)
- Added more Readme Updates (#72)
- Added more Readme (#74)
- Added new SM from sm_respira_1 (#76)
- Added base for the sm_aws_aarehouse navigation.
- Added feature/core and navigation fixes (#78).
- Added feature/aws demo progress (#80).
- Added feature/wait nav2 nodes client behavior (#82).
- Added new feature cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.

Changed
-------

- Updated description table.
- Updated table.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc2_rta command across readmes.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for rolling via submodules.
- Reverted markdowns to HTML.
- Edited tracing.md to reflect new tracing event names.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Reformatted sm_reference_library.
- Corrected trailing spaces.
- Minor formatting improvements.

Fixed
-----

- Fixed bug in smacc2 component.
- Fixed issue with sm_atomic_24hr.
- Fixed pre-commit issues.
- Fixed formatting in sm_advanced_recovery_1.
- Fixed issue with sm_atomic_performance_test_a_2.
- Fixed issue with sm_atomic_performance_test_c_1.
- Fixed issue with sm_multi_stage_1.
- Fixed behavior of wait topic message client.

Removed
-------

- Removed manual installation of ros-rolling-ros2trace (now automated in setupTracing.sh).
- Deleted tracing directory.
- Removed tracing events after renaming.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Removed unnecessary cleanup entries.

Collaborators
-------------

- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_18
-----------

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_pause_slam`.
- `sm_dance_bot_lite` visualization for TurtleBot3.
- Lidar show/hide option for `sm_dance_bot`.
- Gazebo fixes to show the robot and lidar for various dance bot versions.
- `sm_multi_stage_1` doubling.
- `sm_multi_stage_1` working (barely).
- `sm_multi_stage_1` progress in `Brettpac` branch.

Changed
-------
- Corrected all linters and formatters.
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.

Fixed
----
- Removed some compile warnings.

Removed
-------
- Redundant formatting entries.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

## Section_19

### Added
- Added multistage modes to `sm_multi_stage_1`.
- Added sequences and steps to `sm_multi_stage_1`.
- Added sequences `mode_5_sequence_b` and `mode_4_sequence_b` to `sm_multi_stage_1`.
- Added finishing touches and updated README to `sm_multi_stage_1`.
- Added dependencies for husky simulation in `sm_dance_bot`.

### Changed
- Reworked `sm_multi_stage_1` to include multistage modes, sequences, and steps.
- Renamed `sm_advanced_recovery_1`.
- Moved reference library SMs to `smacc2_performance_tools`.
- Added QOS durability to `SmaccPublisherClient`.

### Fixed
- Fixed formatting in `neo_simulation2` package removal.
- Corrected source build packages for testing.
- Adjusted build packages in source CI.
- Fixed recursion possibility by moving method after the one it calls.
- Fixed launch command in `README.md` for `sm_dance_bot_strikes_back`.
- Fixed CI format for Python version.
- Removed node creation and created only a logger.
- Fixed compiling issues in Docker environment.

### Removed
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs` and parameters in `smacc`.
- Removed test from main MoveIt CMake.

### Miscellaneous
- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, and pabloinigoblasco <pablo@ibrobotics.com>.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Updated package list.
- Updated README files.
- Added remaining SVGs to READMEs.
- Precommit cleanup.
- Added SM Atomic SM generator.
- Progressed in MoveIt migration testing.
- Updated format and dependencies for MoveIt migration.
- Improved Dockerfile for building local tests.
- Added .reps dependencies and fixed build errors.
- Updated README files.
- Added reliability QOS config.
- Progressed in testing MoveIt behaviors.
- Fixed pipeline and master build errors.

```rst
Section_20
==========

Added
-----

- Update dependencies for husky in rolling and galactic.
- Progress on aws navigation and some other refactorings on navigation clients and behaviors.
- More on aws demo.
- Warehouse2 progress.
- Waypoint Inputs.
- Wharehouse2 progress.
- Sm_dance_bot_warehouse_3.
- Feature/sm warehouse 2 13 dec 2.
- Finetuning waypoints.
- Feature/cb pure spinning.
- Pure spinning behavior missing files.
- Feature/planner changes 16 12.
- Feature/replanning 16 dec.
- Replanning for all our examples.
- Several fixes.
- Improving undo motion navigation warehouse2.
- Undo tuning and errors.
- Format issues.
- Finishing warehouse2.
- Tuning and fixes.
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components mostly developed for autoware demo.
- Autoware demo.
- Docker files for different revisions, warnings removal and more testing on navigation.
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

- Default values.

Fixed
-----

- Fix formatting.
- Fixing broken build.
- Fix: some formatting and templating on SrConditional.
- Fix: move trigger logic into headers.
- Fix: lint.
- Fixing format and minor.
- Fix broken source build.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.

Removed
-------

- Only rolling version should be pre-released on on master.

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
- Pablo Iñigo Blasco <pabloinigoblasco@ibrobotics.com>
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
- Added README tutorial for Dockerfile.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added sm_atomic_performance_trace_1.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files (#58).
- Fix source CI and correct README overview (#62).
- Update c_cpp_properties.json.
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Feature/aws demo progress (#80).
- More on navigation.
- More sm_advanced_recovery_1 work (#85).
- sm_atomic_performance_test_c_1 (#88).

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Rename header files and correct format.
- Change extension of imports.
- Update doxygen-check-build.yml.
- Use manual deployment for now.
- Rename to smacc2 and smacc2_msgs.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Update description table.
- Update table.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Minor formatting.
- Changed wording "smacc application" to "SMACC2 library".

Fixed
-----
- Correct formatters.
- Correct formatting of python file.
- Correct trailing spaces.
- Fixed bug in smacc2 component.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Ignore further packages.
- Disable ament_lint_cmake.
- Ignore all packages except smacc2 and smacc2_msgs.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh.

Co-Authored-By
--------------
- Denis Štogl <destogl@users.noreply.github.com>.
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
```

```rst
Section_22
==========

Added
-----
- Added new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection.
- Added new client behavior: cb pause slam for pausing SLAM operations.
- Added new client behavior: cb dance bot lite for lightweight dance bot operations.

Changed
-------
- Modified sm_atomic_performance_test_a_2 (#89).
- Updated launch command.
- Corrected all linters and formatters.
- Fixed navigation parameters on sm_dance_bot.
- Minor formatting improvements.
- Renamed doxygen deployment workflow (#100).
- Visualized TurtleBot3 in sm_dance_bot.

Removed
-------
- Removed some compile warnings (#96).

Fixed
----
- Fixed precommit issues.
- Fixed formatting.
- Fixed minor issues in navigation testing.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

Section_23
==========

Added
-----
- Implemented sm_dance_bot visualization for turtlebot3.
- Added option to show/hide cleaning and lidar.
- Introduced gazebo fixes for sm_dance_bot to display the robot and lidar.
- Implemented sm_multi_stage_1 doubling functionality.
- Added AWS demo feature.
- Enabled source build on PR for testing purposes.
- Added diverse improvements for navigation and performance.
- Progressed in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introduced smacc2::deep_history syntax.
- Added slam pausing/resuming functionality for sm_dance_bot.
- Implemented dance bot s pattern.
- Added first working version of sm template and template generator.
- Added SM core test.
- Added local action msgs usage.
- Renamed navigation 2 stack.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Rolled Docker environment to be executed from any environment.
- Refactored sm dance bot strikes back.
- Made slight changes to waypoint 4 and iterations for robot course completion.
- Initiated migration to smacc2 for moveit client.
- Added .reps dependencies and fixed build errors.
- Updated readme files.

Changed
-------
- Improved formatting in various files.
- Updated launch command in README.md for sm_dance_bot_strikes_back.
- Refactored Docker environment for building local tests.

Fixed
-----
- Fixed format issues in CI related to Python version.
- Removed node creation and created only a logger.
- Mitigated overshot issue cases in navigation.
- Resolved compile warnings.
- Removed parameters from smacc.
- Fixed linting warnings.
- Fixed compiling issues.

Removed
-------
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs package.

Collaborators
-------------
- Brett <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>

Section_24
==========

Added
-----
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- Feature/wharehouse2 dec 14 (#185)
- finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#201)
- Feature/minor tune (#203)
- Feature/warehouse2 23 12 (#201)

Changed
-------
- Moved reference library SMs to smacc2_performance_tools (#166)
- Added reliability qos config
- Redoing sm_dance_bot_warehouse_3 waypoints
- Improving undo motion navigation warehouse2
- Tuning and fixes (#202)
- Minor tune
- Improvements in smacc core adding more components mostly developed for autoware demo
- Refining cp subscriber cp publisher
- Several fixes (#194)
- Improving undo motion navigation warehouse2
- Tuning warehouse3 (#197)
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Added missing file from warehouse2 (#205)
- Updating subscriber publisher components
- Progress in autoware machine
- Progress in autoware demo
- Progress in barrel husky
- Progress in barrel demo
- Progress in barrel search build fix and warehouse3
- Progress in autoware demo
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- More testing on moveit
- More testing on moveit behaviors
- More waypoints
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless

Fixed
-----
- Add a missing colon
- Fixing pipeline error
- Fixing broken master build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken

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
- Master rolling to Galactic backport.
- Updating Galactic repositories.

Fixed
-----
- Fixing build.

Other
-----
- More merge.
- Testing Dance Bot demos.

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
