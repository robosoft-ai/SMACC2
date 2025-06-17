Changelog for package multirole_sensor_client
=============================================

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

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
### Added
- Progress in humble SMACC2 deb generation
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Ignored packages not to be released
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup for sm_respira_1
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - CI setup and file renaming
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Added new feature, cb_wait_topic_message
  - Corrected all linters and formatters
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, Denis Štogl <denis@stogl.de>
- Contributors: Denis Štogl, pabloinigoblasco

```rst
Section_2
=========

Added
-----

- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Adding new client behavior for nav2, waiting for nav2 nodes to subscribe to the /bond topic and ensuring they are alive. Nodes to wait for can be optionally selected.
- Base for the sm_aws_warehouse navigation.
- CB pause slam client behavior.
- Sm_dance_bot visualizing turtlebot3.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.
- Gazebo fixes for sm_dance_bot_strikes_back.
- AWS demo.
- Got sm_multi_stage_1 working (barely).
- Gaining traction sm_multi_stage_1.
- More stages added.

Changed
-------

- Navigation parameters fixes on sm_dance_bot.
- Minor hotfix.
- Format fixes.

Fixed
-----

- Remove some compile warnings.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Precommit cleanup run.

Removed
-------

- Remove neo_simulation2 package.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_3
=========

Added
-----

- Feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing slam pausing/resuming functionality for testing sm_dance_bot
- Feature/dance bot s pattern (#128, #129)
  - Polishing sm_dance_bot and s-pattern
  - First working version of sm template and template generator

Changed
-------

- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
  - Add reliability qos config

Fixed
-----

- Waypoint 4 and iterations changes for robot course completion (#155)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Update readme (#164)
- Fixing pipeline error and broken master build for testing moveit behaviors (#167)
- Fixing compiling issues for sm_pubsub_1 (#169)
- Fix formatting for aws navigation sm dance bot (#174)
- Fixing broken build for aws demo (#174)

Removed
-------

- Removing parameters smacc (#147)
- Removing sm_dance_bot_msgs for local action msgs (#144)

Co-authored-by: Brett <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

## Section_4

### Added
- Added `Feature/sm warehouse 2 13 dec 2 (#182)` with various changes and headless mode.
- Added `Feature/wharehouse2 dec 14 (#185)` for warehouse2 improvements.
- Added `Feature/cb pure spinning (#188)` for pure spinning behavior enhancements.
- Added `Feature/planner changes 16 12 (#191)` with replanning for examples.
- Added `Feature/replanning 16 dec (#193)` with replanning improvements.
- Added `Feature/undo motion 20 12 (#196)` for undo motion navigation enhancements.
- Added `Feature/sync 21 12 (#199)` with format fixes.
- Added `Feature/warehouse2 22 12 (#200)` with format fixes and finishing warehouse2.
- Added `Feature/warehouse2 23 12 (#201)` with tuning and fixes.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/undo motion 20 12 (#198)` with undo tuning and errors.
- Added `Feature/sync 21 12 (#199)` with format issues.

### Changed
- Changed `ros2 launch sm_three_some sm_three_some` to `ros2 launch sm_three_some sm_three_some.launch`.
- Updated header files and format.
- Updated GitHub branch reference.
- Updated package name and package.xml.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Updated changelogs.

### Fixed
- Fixed trailing spaces.
- Corrected codespell.
- Corrected python linters warnings.
- Fixed missing rolling repositories build.

### Removed
- Removed manual installation of ros-rolling-ros2trace, now automated in `setupTracing.sh`.

### Miscellaneous
- Various minor changes and fixes across different features.
- Branching example.
- Added necessary packages and edited Threesome launch.
- Created workflows for checking doc build, testing prerelease builds, and deploying doxygen.
- Renamed to `smacc2` and `smacc2_msgs`.
- Created alternative `ManualTracing`.
- Moved tracing files to a new folder.
- Added setup script for tracing.
- Added Dockerfile for Rolling and Galactic.
- Updated Dockerfile build script.
- Edited tracing files and events.
- Reactivated smacc2 nav clients for rolling.
- Reverted markdowns to HTML temporarily.
- Added README tutorial for Dockerfile.
- Enabled build of missing rolling repositories.

---

*Autor: Pablo Iñigo Blasco (pabloinigoblasco)*

```rst
Section_5
=========

Added
-----
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added galactic CI setup and rename rolling files. (#58)
- Add new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update doxygen links (#70).
- Update README.md with updated launch command.

Fixed
-----
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Correct trailing spaces.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Optimized dependencies in move_base_z_planners_common.
- Correct all linters and formatters.

Removed
-------
- Removed sm_respira_1 format cleanup pre-commit.
- Removed several repetitions of progress in aws navigation demo.
- Removed minor format improvements repetitions.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_6
=========

Added
-----
- New client behavior for nav2: now waits for nav2 nodes to subscribe to the /bond topic and confirm they are alive. Nodes to wait for can be optionally selected.
- Progress in AWS navigation demo.
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.

Changed
-------
- Navigation parameters fixes on sm_dance_bot.
- Core improvements during navigation testing.
- Formatting improvements.

Fixed
-----
- Removed some compile warnings.
- Minor hotfixes.
- Fixed format issues.
- Fixed gazebo issues to show the robot and lidar.

Removed
-------
- Removed neo_simulation2 package.

Other
-----
- Precommit cleanup run.
- Updates yaml.
- Corrected formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- Additional linting and formatting.
- Removed merge markers from a python file.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introduced slam pausing/resuming functionality in sm_dance_bot.
- Fixed recursion possibility by moving a method after the one it calls.
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
- Add SM Atomic SM generator. (#143)
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
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Update package list. (#142)
- Update readme (#164)
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
- Removing parameters smacc
- Removing references
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing references
- Removing parameters smacc
- Removing references

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

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
- Fixed fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green (#204).
- Fixed minor broken build.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed formatting of python file.
- Fixed bug in smacc2 component.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh.

Co-Authored-By
--------------
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
```

```rst
Section_9
=========

Added
-----
- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally, you can select the nodes to wait.

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
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview. (#62).
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.

Other
-----
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting improvements.
- Noticed a note that was not removed.
- Attempted pre-commit fixes.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_10
==========

Added
-----
- New client behavior `cb_wait_topic_message`: asynchronous behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: waits for nav2 nodes to subscribe to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature with precommit updates.
- `sm_dance_bot` visualizing TurtleBot3 with lidar show/hide option and formatting improvements.
- Gazebo fixes for showing the robot and lidar in various dance bot features.
- Progress in AWS navigation demo.
- Progress in navigation testing with core improvements.
- Progress in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Progress in `sm_multi_stage_1` functionality.

Changed
-------
- Minor formatting improvements.
- Minor navigation parameters fixes on `sm_dance_bot`.
- Adjusted build packages of source CI.
- Additional linting and formatting.
- Moved method after the method it calls to prevent recursion.

Fixed
-----
- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Removed merge markers from a Python file.

Removed
-------
- Nothing.

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pablo@ibrobotics.com>
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
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Moved reference library SMs to smacc2_performance_tools (#166)
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
- Removing test from main moveit cmake

Other
-----

- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Update readme (#164)
- Update readme
- More readme updates
- Noticed launch command was incorrect in README.md
  Fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
- Precommit cleanup
- Workflows update
- Docker refactoring
- Repos dependency
- Adding dependency to ur5 client
- Progress on move_it PR
- Improving dockerfile for building local tests
- Fixing compiling issues
- Warehouse2 progress (#179)
- Format (#180)
- Sm_dance_bot_warehouse_3 (#181)
- Brettpac branch (#184)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More waypoints
- Finishing touches 1
- Readme
- Default values
- Merge
- Headless and other fixes
- Minor changes (#175)
- Waypoint Inputs (#178)
- Warehouse2
- Husky launch file in sm_dance_bot
- Add dependencies for husky simulation.
- Fix formatting.
- Update dependencies for husky in rolling and galactic.
- More on aws demo
- Minor changes
- Mode_5_sequence_b
- Mode_4_sequence_b
- Sm_multi_stage_1 most
- Sm_multi_stage_1 sequence d
- Sm_multi_stage_1 c sequence
- Sm_multi_stage_1 reworking (#172)
- Multistage modes
- Sm_multi_stage sequences
- Sm_multi_state_1 steps
- Initial migration to smacc2
- Fixing some errors introduced on formatting
- Missing dependency
- Fixing some more linting warnings
- Progressing in the moveit migration testing
- Updating format
- Adding .reps dependencies and also fixing some build errors
- Docker refactoring
- Minor dockerfile test workaround
- Minor
- Progress on moveit
- More testing on moveit behaviors
- Minor configuration
- Pending references
- Navigation 2 stack renaming
- Formatting
- Minor format issues (#134)
- Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
- Some more progress on markers cleanup
- Minor
- Build fix
- Pre-commit cleanup
- Feat: add qos durability to SmaccPublisherClient
- Feat: add reliability qos config
- Refactor: remove line
- Fix: add a missing colon
- Fix: add a missing colon
- Fix: some formatting and templating on SrConditional
- Fix: move trigger logic into headers
- Fix: lint
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional

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
- several fixes (#194)
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements (#204)
- format issues
- minor formatting fixes

Fixed
-----
- headless and other fixes
- default values
- more fixes
- fixing docker for foxy and galactic
- removing warnings (#213)
- minor broken build

Removed
-------
- pure spinning behavior missing files
- weird moveit not downloaded repo
- minor linking errors foxy

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
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

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
- Corrected all linters and formatters.
- Minor formatting improvements.

Fixed
~~~~
- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Fix source CI and correct README overview.
- Fix pre-commit issues.
- Attempting pre-commit fixes.

Removed
~~~~~~~
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.
- Deleted tracing.md.

Other
~~~~~
- Some progress on navigation rolling.
- More changes on performance tests.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- More on performance and other issues.
- Format improvements.
```

```rst
Section_14
==========

Added
-----

- Introduce new feature `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Implement new client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Nodes to wait can be optionally selected.
- Progress in AWS navigation demo.

Changed
-------

- Minor formatting improvements.

Fixed
-----

- Navigation parameters fixes on `sm_dance_bot`.
- Remove some compile warnings.
- Fix format on `cb_pause_slam` client behavior.
- Minor hotfix.
- Fix gazebo to show the robot and the lidar.

Removed
-------

- Remove `neo_simulation2` package.

Other
-----

- Merge and progress.
- Precommit cleanup run.
- Updates yaml.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Various core improvements during navigation testing.
- More on navigation.
- Keep hammering on `sm_multi_stage_1`.
- Progress in `sm_dance_bot` visualizing turtlebot3.
- Cleaning and lidar show/hide option.
- Cleaning files and making formatting work.
- More fixes.
- Gaining traction on `sm_multi_stage_1`.
- Two stages.
- 3 part.
- 4th stage.
- 5th stage.
- Diverse improvements in navigation and performance.
``` 

*pabloinigoblasco*

## Section_15

### Added
- Feature/diverse improvements in navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122)
- Feature/dance bot s pattern (#128)
- First working version of sm template and template generator (#127)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Feature/migration moveit client (#151)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Remove node creation and create only a logger (#149)
- Moved reference library SMs to smacc2_performance_tools (#166)

### Fixed
- Minor tuning to mitigate overshot issue cases in waypoints navigator (#133)
- Fix CI: format fix python version (#148)
- Fixing broken master build (#167)
- Fixing broken build in aws navigation (#174)

### Removed
- Remove merge markers from a python file (#119)
- Removing parameters smacc (#147)
- Removing sm_dance_bot_msgs (#144)

### Miscellaneous
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back and removed some comments (#148)
- Update package list (#142)
- Update readme (#164)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Precommit cleanup
- Workflows update
- Pending references
- Repos dependency
- Docker refactoring
- More readme updates

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

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
- minor changes (#195)
- Feature/undo motion 20 12 (#196, #198)
- improving undo motion navigation warehouse2
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

Changed
-------
- Fix code generators (#221)
- Fix other build issues.
- Update SM template and make example code clearly visible.
- Remove use of node in the sm performance template.
- Updated templated to use Blackboard storage.
- Update template to resolve the global data correctly.
- Update sm_name.hpp

Fixed
-----
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

Co-authored-by
--------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```rst
Section_17
==========

Added
-----
- Dockerfile now accepts ROS distro as argument. Use ``sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/``.
- New folder created for additional tracing contents.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Alternative ManualTracing created.
- New sm markdowns added.
- README tutorial added for Dockerfile.
- smacc2_performance_tools added.
- Performance tests improvements made.
- Optimized dependencies in move_base_z_planners_common.
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.

Changed
-------
- Wording changed from "smacc application" to "SMACC2 library".
- Mentions of SMACC/ROS updated to SMACC2/ROS2.
- Event generator library renamed.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Trailing spaces corrected.
- Launch command updated to ``ros2 launch sm_respira_1 sm_respira_1.launch``.
- Several core improvements made during navigation testing.
- Pre-commit fixes attempted.

Fixed
-----
- Bug in smacc2 component resolved.
- Build of missing rolling repositories enabled.
- Navigation2 now available for semi-binary build.
- Galactic builds removed from master, keeping only rolling. Submodules removed and .repos file used.

Removed
-------
- Manual installation of ros-rolling-ros2trace. Now automated in setupTracing.sh.
- Clang-format execution on smacc2_sm_reference_library package.

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
-----
- Introducing new feature `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Added new client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to become active. Optionally select nodes to wait for.
- Implemented base for the sm_aws_warehouse navigation.
- Added navigation parameters fixes for sm_dance_bot.
- Introducing new feature `cb_pause_slam` client behavior.

Changed
-------
- Corrected all linters and formatters.
- Various formatting improvements.
- Progress made in AWS navigation demo.
- Minor format adjustments.
- Updated yaml files.
- Renamed doxygen deployment workflow.
- Added lidar show/hide option for dance bot launch in Gazebo.
- Gazebo fixes for visualizing the robot and lidar in sm_dance_bot.
- Doubled sm_multi_stage_1 functionality.
- Gazebo fixes for sm_dance_bot_strikes_back.

Fixed
-----
- Removed some compile warnings.

Removed
-------
- No items removed.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

--- 

**Autor:** Pablo Iñigo Blasco (pabloinigoblasco)

## Section_19

### Added
- Added multistage modes to `sm_multi_stage_1`.
- Added sequences and steps to `sm_multi_stage_1`.
- Added new sequences `mode_5_sequence_b` and `mode_4_sequence_b` to `sm_multi_stage_1`.
- Added AWS navigation to `sm_dance_bot`.

### Changed
- Renamed `sm_advanced_recovery_1`.
- Reworked `sm_multi_stage_1` with new sequences and steps.

### Fixed
- Fixed waypoint 4 and iterations for robot course completion.
- Fixed initial state machine transition timestamp.
- Fixed QOS durability for `SmaccPublisherClient`.

### Removed
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs` and `smacc` parameters.

### Miscellaneous
- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, and pabloinigoblasco <pablo@ibrobotics.com>.
- Various minor improvements, formatting fixes, and dependency updates.
- Added SVGs to READMEs.
- Docker environment now executable from any environment.
- Husky launch file added to `sm_dance_bot`.
- Added dependencies for Husky simulation.

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
- Feature/sync 21 12.
- Feature/warehouse2 22 12.
- Feature/warehouse2 23 12.
- Feature/minor tune.
- Fixing warehouse 3 problems and other core improvements.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components for autoware demo.
- Autoware demo.
- Docker files for different revisions, warnings removal, and more testing on navigation.
- Update file for fake hardware simulation and add file for gazebo simulation.
- Retry behavior warehouse 1.
- Progress in barrel husky.
- Only rolling version should be pre-released on master.
- Barrel search build fix and warehouse3.
- Fixing startup problems in warehouse 3.
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
- Merge.
- Headless and other fixes.
- Default values.
- Several fixes.
- Tuning and fixes.
- Minor tune.
- Format issues.
- Finishing warehouse2.
- Tuning and fixes.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.

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
- Weird moveit not downloaded repo.
- Pure spinning behavior missing files.
- Missing sm.
- Missing.
- Foxy CI.
- Fix.
- Some reordering fixes.
- Missing file.
- Minor format fix.
- Other minor changes.

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
Section_21
==========

Added
-----
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Added setupTracing.sh.
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Update tracing/ManualTracing.md.
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Update smacc_sm_reference_library/sm_atomic/README.md.
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added README tutorial for Dockerfile.
- Enable cppcheck.
- Added missing licenses.

Changed
-------
- Rename header files and correct format.
- Update doxygen-check-build.yml.
- Use manual deployment for now.
- Rename to smacc2 and smacc2_msgs.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Update changelogs.
- Changed wording "smacc application" to "SMACC2 library".
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Reactivating smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Bug in smacc2 component.
- Reverted markdowns to html.
- More changes on performance tests.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Sm_reference_library reformatting.
- Correct trailing spaces.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Minor formatting.
- Add galactic CI setup and rename rolling files (#58).
- Fix source CI and correct README overview (#62).
- Update c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
  Also noticed a note I had made while producing these that was not removed.
- Update doxygen links (#70).
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- More Readme Updates (#72).
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- More Readme (#74).
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Base for the sm_aws_aarehouse navigation.
- Progressing in AWS navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- More on navigation.
- Sm_advanced_recovery_1 reworked (#83).
- Fix pre-commit.
- Trying to fix Pre-Commit.
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- More sm_advanced_recovery_1 (#84).
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- More sm_advanced_recovery_1 work (#85).
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Sm_advanced_recovery_1 round 4 (#86).
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Brettpac branch (#87).
- Sm_atomic_performance_test_a_2.
- Sm_atomic_performance_test_a_1.
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Sm_atomic_performance_test_c_1 (#88).

Fixed
-----
- Correct formatters.
- Correct formatting of python file.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Ignore further packages.
- Ignore all packages except smacc2 and smacc2_msgs.
- Removed manual installation of ros-rolling-ros2trace.
  This is now automated in setupTracing.sh.
  Location of sh file assumed if user follows README.md under "Getting started".
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Removed galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Cleanup.
- Edited tracing.md to reflect new tracing event names.
```

```rst
Section_22
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2: `wait nav2 nodes` subscribing to the `/bond` topic and waiting for them to be alive. Nodes to wait can be optionally selected.
- New feature: `cb_pause_slam` client behavior.
- New feature: `sm_dance_bot_lite`.

Changed
-------
- Updated launch command.
- Corrected all linters and formatters.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor format improvements.
- Merge and progress in various features.
- Formatting improvements.

Fixed
-----
- Fixed precommit.
- Removed some compile warnings.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

Section_23
-----------

### Added
- Added `sm_dance_bot` visualizing `turtlebot3`.
- Added lidar show/hide option for cleaning.
- Added formatting improvements for cleaning files.

### Changed
- Improved `sm_dance_bot` Lite Gazebo functionality (#104).
- Enhanced Gazebo visualization for the robot and lidar.
- Doubled functionality for `sm_multi_stage_1` (#103).
- Improved Gazebo fixes for `sm_dance_bot_strikes_back`.

### Fixed
- Resolved formatting issues.
- Fixed issues with `sm_multi_stage_1` functionality (#109).
- Fixed issues with `sm_multi_stage_1` progress (#110, #111).
- Fixed issues with `neo_simulation2` package removal (#112).
- Fixed recursion possibility in method calls (#126).
- Fixed overshot issues in waypoints navigator (#133).
- Fixed minor format issues (#134).
- Fixed CI format for Python version (#148).

### Removed
- Removed `neo_simulation2` package.
- Removed unnecessary parameters from `smacc`.
- Removed `sm_dance_bot_msgs` references.

### Miscellaneous
- Co-authored commits with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored commits with DecDury <declandury@gmail.com> and Denis Štogl <destogl@users.noreply.github.com>.
- Updated READMEs with SVGs for atomic, dance_bot, and others.
- Updated package list.
- Updated launch command in README.md.
- Updated Docker environment for cross-environment execution.
- Progressed in migration to `smacc2` for MoveIt client.
- Added dependencies and fixed build errors for MoveIt migration.
- Improved Dockerfile for local test building.
- Updated README with additional information.

### Contributors
- Collaborated with pabloinigoblasco <pablo@ibrobotics.com>.

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
- Finishing touches 1
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- Improving undo motion navigation warehouse2
- Tuning and fixes
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green

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
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build
- Fixing broken build

```rst
Section_25

Version 0.1.0 (Date: TBD)

Added
-----
- Build-status table
- Detailed install instructions (adjusted from [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme))

Changed
-------
- Default build type set to `Release` for faster performance and smaller executables
- Updated examples section

Fixed
-----
- Resolved missing dependency in smacc_msgs and reorganized for better overview
- Fixed build issues
- Fixed bug in smacc2 component
- Cleaned up formatting in various packages
- Optimized dependencies in move_base_z_planners_common
- Corrected build-overview table
- Updated and unified CI configurations

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh

Other
-----
- Reverted changes in commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Restored all versions to 0.0.0
- Ignored all packages except smacc2 and smacc2_msgs
- Updated changelogs
- Reformatting the whole project
- Enabled all packages to compile
- Added getLogger functionality and refactoring
- Reactivated smacc2 nav clients for rolling via submodules
- Edited tracing.md to reflect new tracing event names
- Performance tests improvements
- Various performance and bug fixes
- Renamed event generator library
- Used tf_geometry_msgs.h in galactic
- Used galactic branches in .repos-file
- Do not execute clang-format on smacc2_sm_reference_library package
- Cleaned up trailing spaces
- Added README tutorial for Dockerfile
- Updated README.md
- Updated tracing/ManualTracing.md
- Added setupTracing.sh for automated installation of ros-rolling-ros2trace
- Location of sh file assumed if user follows README.md under "Getting started"

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco
- pabloinigoblasco
- reelrbtx
- Declan Dury
- brettpac
- David Revay
```

*pabloinigoblasco*
