Changelog for package sm_branching
======================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` into humble

### Changed
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted to fix weird issue with ros buildfarm
  - Made more adjustments related to the buildfarm issue

### Contributors
- brettpac, pabloinigoblasco

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
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Ignored packages which should not be released
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Made progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Improved performance tests
  - Cleaned up sm_respira_1 format
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Added galactic CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Updated doxygen links
  - Created new sm from sm_respira_1
  - Made core and navigation fixes
  - Reworked sm_advanced_recovery_1
  - Worked on sm_atomic_performance_test_a_2, sm_atomic_performance_test_a_1, sm_atomic_performance_test_c_1
  - Modified sm_atomic_performance_test_a_2
  - Worked on sm_multi_stage_1
  - Updated README.md
  - Implemented new feature, cb_wait_topic_message
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive
  - Corrected all linters and formatters

### Contributors
- brettpac, Ubuntu 20-04-02-amd64, Denis Štogl

```rst
Section_2
=========

Added
-----
- Introduce new feature `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and ensure they are operational. Optionally select nodes to wait for.
- Progress in AWS navigation demo.

Changed
-------
- Improve formatting throughout the changelog entries.

Fixed
-----
- Resolve navigation parameters issues on sm_dance_bot.
- Address minor formatting inconsistencies.
- Remove some compile warnings.

Removed
-------
- Eliminate neo_simulation2 package.

Other
-----
- Merge and progress in development.
- Implement `cb_pause_slam` client behavior.
- Visualize TurtleBot3 in sm_dance_bot.
- Add lidar show/hide option in sm_dance_bot.
- Fix gazebo issues for various bots.
- Work on sm_multi_stage_1 functionality.
- Collaborators: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
```

Section_3
=========

Added
-----

- Feature/diverse improvements navigation performance (#117)
  - Diverse improvements in navigation and performance.
  - Additional linting and formatting.
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
  - Introducing smacc2::deep_history syntax.
  - Testing sm_dance_bot with slam pausing/resuming functionality.
- More changes in sm_dance_bot (#125)
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern.
  - Typo correction.
- First working version of sm template and template generator. (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
  - Build fix.
- Waypoints navigator bug (#133)
  - Tuning to mitigate overshot issue cases.
- Progress in the sm_dance_bot tests (#135)
  - Progress on markers cleanup.
- Minor navigation improvements (#141)
  - Using local action messages.
  - Removing sm_dance_bot_msgs.
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming.
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140).
  - Added remaining SVGs to READMEs (#145).
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154).
- Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes so the robot can complete the course (#155).
- Feature/migration moveit client (#151)
  - Initial migration to smacc2.
  - Progressing in the moveit migration testing.
  - Adding .reps dependencies and fixing build errors.
  - Adding dependency to ur5 client.
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
  - Adding QOS durability to SmaccPublisherClient.
  - Fixing a missing colon.
  - Removing a line.
  - Adding reliability QOS config.
- Feature/testing moveit behaviors (#167)
  - More testing on moveit.
  - Minor configuration.
  - Fixing pipeline error.
  - Fixing broken master build.
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)
  - Multistage modes.
  - Sequences and steps.
  - Finishing touches and readme updates.
- Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and refactorings on navigation clients and behaviors.
  - More on aws demo.
  - Fixing broken build.
- Waypoint Inputs (#178)
- Warehouse2 (#177)
- Warehouse2 progress (#179)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
  - More changes and headless.
  - Merge headless and other fixes.

Changed
-------

- Move method after the method it calls. Otherwise, recursion could happen. (#126)
- Noticed launch command was incorrect in README.md.
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments.
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Update readme (#164)
  - More readme updates.

Removed
-------

- Removing parameters smacc (#147)
  - Workflows update.
  - Workflow.
- Removing test from main moveit cmake.

Fixed
-----

- Minor format issues (#134).
- Format (#180).

Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>

Section 4
----------

Added
~~~~~
- Added default values.
- Added BrettPac branch (#184).
- Added redoing sm_dance_bot_warehouse_3 waypoints.
- Added more waypoints.
- Added SrConditional fixes and formatting (#168).
- Added feature/warehouse2 dec 14 (#185).
- Added feature/sm warehouse 2 13 dec 2 (#186).
- Added finetuning waypoints (#187).
- Added feature/cb pure spinning (#188).
- Added pure spinning behavior missing files.
- Added minor changes (#190).
- Added feature/planner changes 16 12 (#191).
- Added replanning for all our examples.
- Added feature/replanning 16 dec (#193).
- Added several fixes (#194).
- Added minor changes (#195).
- Added feature/undo motion 20 12 (#196).
- Added improving undo motion navigation warehouse2.
- Added tuning warehouse3 (#197).
- Added feature/undo motion 20 12 (#198).
- Added undo tuning and errors.
- Added format issues.
- Added feature/sync 21 12 (#199).
- Added format issues.
- Added finishing warehouse2.
- Added feature/warehouse2 23 12 (#201).
- Added tuning and fixes (#202).
- Added feature/minor tune (#203).
- Added fixing warehouse 3 problems and other core improvements (#204).
- Added added missing file from warehouse2 (#205).
- Added backport to foxy.
- Added minor format.
- Added minor linking errors foxy.
- Added foxy backport (#206).

Changed
~~~~~~~
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Added
~~~~~
- Added instructions to ensure the necessary package is installed before running a command.

Fixed
~~~~~
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.

Removed
~~~~~~~
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.

Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_5
=========

Added
-----

- Added smacc2_performance_tools for performance monitoring.
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------

- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Updated c_cpp_properties.json.
- Corrected all linters and formatters.

Fixed
-----

- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview. (#62).
- Fixed pre-commit issues in various packages.

Removed
-------

- Removed redundant note that was not removed.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_6
=========

Added
-----

- Feature/sm aws warehouse (#94)
  - Implemented base for the sm_aws_warehouse navigation.
  - Made progress in AWS navigation.
  - Various core improvements during navigation testing.
  - Improved formatting.
  - Added new feature: cb_wait_topic_message for asynchronous client behavior.
  - Added new client behavior for nav2 to wait for nodes subscribing to the /bond topic.
  - Continued progress in AWS navigation demo.

Changed
-------

- Feature/sm dance bot fixes (#95)
  - Fixed navigation parameters on sm_dance_bot.

- Feature/cb pause slam (#98)
  - Added cb pause slam client behavior.

- sm_dance_bot_lite (#99)
  - Updated YAML configuration.

- Rename doxygen deployment workflow (#100)
  - Applied minor hotfix.

- sm_dance_bot visualizing turtlebot3 (#101)
  - Improved visualization of turtlebot3.

- Feature/dance bot launch gz lidar choice (#102)
  - Added option to show/hide lidar in dance bot launch.

- Feature/sm dance bot lite gazebo fixes (#104)
  - Fixed gazebo visualization for sm_dance_bot.

- sm_multi_stage_1 doubling (#103)
  - Collaborative work with Ubuntu 20-04-02-amd64 for doubling sm_multi_stage_1.

- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Fixed gazebo visualization for sm_dance_bot_strikes_back.

- Precommit cleanup run (#106)
  - Collaborative work with Ubuntu 20-04-02-amd64 for precommit cleanup.

- aws demo (#108)
  - Demonstrated AWS capabilities.

- Brettpac branch (#110)
  - Continued development on sm_multi_stage_1.

- Brettpac branch (#111)
  - Extended development on sm_multi_stage_1.

- a3 (#113)
  - Collaborative work with Ubuntu 20-04-02-amd64 on a3.

- Remove neo_simulation2 package. (#112)
  - Removed neo_simulation2 package.
  - Corrected formatting.
  - Enabled source build on PR for testing.
  - Adjusted build packages of source CI.

- mm (#115)
  - Collaborative work with Ubuntu 20-04-02-amd64 on mm.

- diverse improvements navigation and performance (#116)
  - Made diverse improvements in navigation and performance.

- Feature/diverse improvements navigation performance (#117)
  - Added linting and formatting improvements.

- Remove merge markers from a python file. (#119)
  - Removed merge markers from a Python file.

- Feature/slam toggle and smacc deep history (#122)
  - Progressed in navigation, slam toggle client behaviors, and slam_toolbox components.
  - Introduced smacc2::deep_history syntax.
  - Added slam pausing/resuming functionality to sm_dance_bot.

- Move method after the method it calls. Otherwise recursion could happen. (#126)

- Feature/dance bot s pattern (#128)
  - Polished sm_dance_bot and s-pattern.
  - Corrected typo "Finnaly" to "Finally".

- Feature/dance bot s pattern (#129)
  - Continued refinement in sm_dance_bot.
```

Section_7
==========

Added
-----
- First working version of sm template and template generator. (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- progress in the sm_dance_bot tests (#135)
- sm_dance_bot_lite (#136)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- minor navigation improvements (#141)
- using local action msgs (#139)
- Feature/nav2z renaming (#144)
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)
- Feature/aws navigation sm dance bot (#174)
- warehouse2 (#177)
- Waypoint Inputs (#178)
- wharehouse2 progress (#179)
- sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)
- Feature/planner changes 16 12 (#191)

Changed
-------
- minor tweaks (#130)
- minor format issues (#134)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)

Fixed
-----
- minor tuning to mitigate overshot issue cases
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- fixing some errors introduced on formatting
- fixing some more linting warnings
- fixing compiling issues
- fixing broken master build
- fixing broken build
- fix: add a missing colon
- fix: some formatting and templating on SrConditional
- fix: move trigger logic into headers
- fix: lint

Removed
-------
- removing sm_dance_bot_msgs
- removing parameters smacc
- removing test from main moveit cmake
- removing parameters smacc
- removing parameters smacc
- removing parameters smacc

Authors
-------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_8
=========

Version 0.1.0 (2021-12-23)
---------------------------

Added
~~~~~

- Feature/replanning 16 dec (#193): Replanning for all examples.
- Feature/undo motion 20 12 (#196): Improving undo motion navigation in warehouse2.
- Feature/warehouse2 22 12 (#200): Finishing warehouse2.
- Feature/warehouse2 23 12 (#201): Tuning and fixes (#202).
- Feature/minor tune (#203): Fixing warehouse 3 problems and core improvements (#204).
- Merging code from backport foxy and updates about autoware (#208).
- Foxy backport (#206): Various fixes and updates.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Added smacc2_performance_tools.
- Performance tests improvements.

Changed
~~~~~~~

- Corrected various formatting issues.
- Updated subscriber publisher components.
- Refactored cp subscriber cp publisher.
- Improved smacc core by adding more components developed for autoware demo.
- Renamed "sm_three_some" launch command.
- Updated extension of imports.
- Corrected formatting of python files.
- Updated ci-build-source.yml.
- Changed extension of imports.
- Corrected GitHub branch reference.
- Updated name of package and package.xml.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Copied initial docs.
- Changed wording from "smacc application" to "SMACC2 library".
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Bug fixes in smacc2 component.
- Reverted markdowns to html.
- Updated README tutorial for Dockerfile.
- Various cleanup actions.

Removed
~~~~~~~

- Deleted tracing directory.
- Removed manual installation of ros-rolling-ros2trace.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh (location assumed if user follows README.md under "Getting started").
- Created alternative ManualTracing.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
```

--- 

Este fragmento ha sido mejorado por Pablo Iñigo Blasco (pabloinigoblasco).

## Section_9

### Added

- Added galactic CI setup and renamed rolling files. (#58)
- Added more Readme updates (#72, #74)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92)
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for (#82, #92)
- Added navigation parameters fixes on sm_dance_bot (#93)

### Changed

- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69)
- Updated `smacc2_rta` command across readmes
- Updated `c_cpp_properties.json`
- Updated README.md with launch command

### Fixed

- Fixed source CI and corrected README overview. (#62)
- Fixed trailing spaces
- Corrected all linters and formatters

### Removed

- Removed execution of clang-format on `smacc2_sm_reference_library` package

### Miscellaneous

- Cleaned up `sm_respira_1` format
- Cleaned up `sm_respira_1` format pre-commit
- Cleaned up `sm_reference_library` formatting
- Cleaned up `sm_atomic_24hr`
- Cleaned up `sm_atomic_performance_trace_1`
- Cleaned up `sm_atomic_24hr` more
- Cleaned up `move_base_z_planners_common` optimized dependencies
- Renamed event generator library
- Minor formatting improvements
- Progressed in AWS navigation
- Made several core improvements during navigation testing
- Progressed in AWS navigation demo
- Progressed in AWS navigation demo more
- Progressed in AWS navigation demo further
- Reworked `sm_advanced_recovery_1`
- Fixed pre-commit in `sm_advanced_recovery_1`
- Tried to fix pre-commit in `sm_advanced_recovery_1`
- Modified `sm_atomic_performance_test_a_2`
- Fixed pre-commit in `sm_multi_stage_1`
- More work on `sm_multi_stage_1`
- Created new `sm` from `sm_respira_1`
- Worked on `sm_advanced_recovery_1` round 4
- Worked on `sm_atomic_performance_test_c_1`
- Modified `sm_atomic_performance_test_a_2`
- Worked on `sm_multi_stage_1` further
- Progressed in AWS navigation demo more
- Progressed in AWS navigation demo further
- Progressed in AWS navigation demo even more
- Progressed in AWS navigation demo yet more
- Progressed in AWS navigation demo additionally
- Progressed in AWS navigation demo even further
- Progressed in AWS navigation demo even additionally
- Progressed in AWS navigation demo even more further

```rst
Section_10
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive; optional node selection
- Base for the `sm_aws_warehouse` navigation
- Gazebo fixes to show the robot and lidar
- Gazebo fixes for `sm_dance_bot_strikes_back`
- AWS demo progress
- Source build enabled on PR for testing
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components
- Introducing slam pausing/resuming functionality in `sm_dance_bot`
- First working version of `sm` template and template generator

Changed
-------
- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Cleaning and lidar show/hide option in `sm_dance_bot`
- Polishing `sm_dance_bot` and `s-pattern`
- `Finally` corrected to `Finally`
- Minor tweaks

Fixed
-----
- Remove some compile warnings
- Remove `neo_simulation2` package
- Remove merge markers from a Python file
- Move method after the method it calls to prevent recursion

Removed
-------
- `neo_simulation2` package

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_11
==========

Added
-----

- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
- Progress in the sm_dance_bot tests (#135)
- Feature/nav2z renaming (#144)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/sm warehouse 2 13 dec 2 (#182)
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)

Changed
-------

- Minor tuning to mitigate overshot issue cases
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Format fix CI: format fix python version (#148)
- Fixing some errors introduced on formatting
- Progressing in the moveit migration testing
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----

- Resolve compile warnings (#137)
- Fix CI: format fix python version (#148)
- Fixing broken master build
- Fixing pipeline error
- Fixing compiling issues

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing node creation and create only a logger. (#149)

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
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Added missing file from warehouse2 (#205)
- Add mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Enable cppcheck
- Added workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use manual deployment for now.
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Opened new folder for additional tracing contents
- added setupTracing.sh
- Created alternative ManualTracing
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Improving undo motion navigation in warehouse2
- Undo tuning and errors
- Tuning and fixes (#202)
- Fixing warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green
- Odom tracker improvements
- Adding forward behavior retry functionality
- Minor formatting fixes
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Rename header files and correct format.
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Update description table.
- Update table
- Copy initial docs
- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch"
- Changed wording "smacc application" to "SMACC2 library"
- Edit from html to markdown syntax in smacc_sm_reference_library/sm_atomic/README.md

Removed
-------
- Remove example things from Foxy CI setup. (#214)
- Remove tracing directory
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  Location of sh file assumed if user follows README.md under "Getting started"
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages
- Ignore further packages

Fixed
-----
- Fix rolling builds (#222)
- Fixing docker for foxy and galactic
- Removing warnings (#213)
- Minor broken build
- Minor linking errors foxy
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Fixing warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green

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
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait.

Changed
~~~~~~~
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Renamed sm_atomic_24hr to sm_atomic_performance_trace_1.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Modified launch command for sm_atomic_performance_test_a_2.
- Updated README.md.

Fixed
~~~~
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Cleaned up sm_atomic_24hr.
- Fixed pre-commit issues in sm_advanced_recovery_1.
- Corrected all linters and formatters.

Removed
~~~~~~~
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

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
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive. Nodes to wait for can be optionally selected.
- New client behavior: `cb_pause_slam`.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor hotfixes and format improvements.
- Progress in AWS navigation demo.
- Progress in `sm_aws_warehouse` navigation.
- Progress in AWS navigation.
- Progress in `sm_dance_bot_lite`.
- Progress in `sm_dance_bot_strikes_back`.
- Progress in `sm_multi_stage_1`.
- Diverse improvements in navigation and performance.

Fixed
-----
- Removed some compile warnings.
- Removed `neo_simulation2` package.

Removed
-------
- Removed merge markers from a Python file.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_15
==========

Added
-----
- Introducing slam pausing/resuming functionality to sm_dance_bot testing (#124, #125, #126, #128, #129, #131, #132, #135)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Waypoint Inputs (#178)
- Redoing sm_dance_bot_warehouse_3 waypoints (#184)

Changed
-------
- Move method after the method it calls to prevent recursion (#126)
- Renamed state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Progress on moveit migration testing (#151, #167)
- Reworked sm_multi_stage_1 (#172)

Fixed
-----
- Minor format issues (#134, #141, #180)
- Fixed launch command in README.md (#147)
- Fix CI: format fix python version (#148)
- Fixed compiling issues (#164)
- Fixed broken master build (#167, #174)
- Fixed pipeline error (#167)
- Fixed formatting and templating on SrConditional (#168)
- Fixed linting issues (#151, #168)

Removed
-------
- Removed node creation and create only a logger (#149)
- Removed parameters smacc (#147)

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> (multiple contributions)
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_16
==========

Added
-----
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- dockerfiles (#225)
- Feature/retry behavior warehouse 1 (#226)

Changed
-------
- Updated SM template and made example code clearly visible.
- Removed use of node in the SM performance template.
- Updated template to use Blackboard storage.
- Updated template to resolve the global data correctly.
- Updated sm_name.hpp
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Fixed
-----
- Fix code generators (#221)
- Fix other build issues.
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

Removed
-------
- Delete tracing directory

Other
-----
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: reelrbtx <brett2@reelrobotics.com>
- Co-authored-by: brettpac <brett@robosoft.ai>
- Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>
```

```rst
Section_17
==========

Added
-----

- Moved tracing.md to tracing directory.
- Added setupTracing.sh script to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added sm_respira_1 format cleanup.
- Added sm_respira_test_2.
- Added sm_atomic_24hr.
- Added sm_atomic_performance_trace_1.
- Added sm_atomic_24hr cleanup.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More Readme Updates.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Feature/aws demo progress.
- More on navigation.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.
- Corrected all linters and formatters.

Changed
-------

- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Reactivated smacc2 nav clients for rolling via submodules.
- Reverted markdowns to HTML.
- Renamed folders, deleted tracing.md, edited README.md.
- Edited tracing.md to reflect new tracing event names.
- Cleaned up sm_reference_library.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_advanced_recovery_1.
- Fixed pre-commit issues.
- Attempted pre-commit fixes.

Removed
-------

- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.
```

*pabloinigoblasco*

```rst
Section_18
==========

Added
-----
- New feature: `cb_wait_topic_message` asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `cb_wait_topic_message` waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor format improvements.

Fixed
-----
- Remove some compile warnings.

Removed
-------
- Removed `neo_simulation2` package.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

Section_19
-----------

Added
~~~~~
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Added diverse improvements in navigation and performance.
- Added linting and formatting.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introduced slam pausing/resuming functionality in sm_dance_bot.
- Added first working version of sm template and template generator.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM core test.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added progress on aws navigation and some other refactorings on navigation clients and behaviors.
- Added more on aws demo.

Changed
~~~~~~~
- Moved method after the method it calls to prevent recursion.
- Renamed sm_advanced_recovery_1.
- Reworked sm_multi_stage_1 with multistage modes, sequences, steps, and finishing touches.
- Moved reference library SMs to smacc2_performance_tools.
- Updated package list.
- Rolled Docker environment to be executed from any environment.
- Refactored dockerfile for building local tests.
- Updated format in README.
- Updated format in README.
- Updated format in README.
- Updated dependencies for husky in rolling and galactic.

Fixed
~~~~
- Removed merge markers from a python file.
- Fixed launch command for sm_dance_bot_strikes_back.
- Fixed CI format for python version.
- Removed node creation and created only a logger.
- Mitigated overshot issue cases in minor tuning.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed broken build.

Removed
~~~~~~~
- Removed merge markers from a python file.
- Removed parameters smacc.
- Removed test from main moveit cmake.
- Removed sm_dance_bot_msgs.
- Removed pending references.
- Removed some comments in the past.

Authors
~~~~~~~
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_20
==========

Added
-----

- Waypoint Inputs (#178) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- wharehouse2 progress (#179)
- sm_dance_bot_warehouse_3 (#181) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- finetuning waypoints (#187) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Feature/cb pure spinning (#188)
- pure spinning behavior missing files
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- several fixes (#194)
- Feature/undo motion 20 12 (#196)
- tuning warehouse3 (#197)
- Feature/undo motion 20 12 (#198)
- undo tuning and errors
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- finishing warehouse2
- Feature/warehouse2 23 12 (#201)
- tuning and fixes (#202)
- Feature/minor tune (#203)
- fixing warehouse 3 problems, and other core improvements (#204)
- added missing file from warehouse2 (#205)
- Update file for fake hardware simulation and add file for gazebo simulation. (#224)
- Update file for fake hardware simulation and add file for gazebo simulation. (Co-authored-by: Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>, Declan Dury <44791484+DecDury@users.noreply.github.com>, DecDury <declandury@gmail.com>, reelrbtx <brett2@reelrobotics.com>, brettpac <brett@robosoft.ai>, David Revay <MrBlenny@users.noreply.github.com>, pabloinigoblasco <pabloinigoblasco@ibrobotics.com>)
- progress in barrel husky
- Only rolling version should be pre-released on on master. (#230)
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
- multiple controllable leds plugin
- progress in husky demo
- Feature/improvements warehouse3 (#228) (Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>)
- Foxy backport (#206)

Changed
-------

- fix: some formatting and templating on SrConditional
- fix: move trigger logic into headers
- fix: lint
- minor changes
- more fixes
- improving undo motion navigation warehouse2
- format issues
- fixing docker for foxy and galactic
- some reordering fixes
- retry behavior warehouse 1
- fixing format and minor
- making models local
- red picuup
- improving navigation behaviors
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

Removed
-------

- minor broken build
- minor linking errors foxy
- minor format fix
- other minor changes
```

```rst
Section_21
==========

Added
-----
- First ensure you have the necessary package installed by running:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```

Changed
-------
- Corrected formatting of Python files.
- Renamed header files and corrected format.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Renamed to smacc2 and smacc2_msgs.
- Corrected GitHub branch reference.
- Updated package name and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" (commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61).
- Updated description table.
- Updated table.
- Copied initial docs.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Updated tracing/ManualTracing.md (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Changed wording "smacc application" to "SMACC2 library" (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Updated smacc_sm_reference_library/sm_atomic/README.md from HTML to markdown syntax (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Reactivated smacc2 nav clients for Rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Added README tutorial for Dockerfile.
- Enabled build of missing Rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed Galactic builds from master, keeping only Rolling, and removed submodules using .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Made progress on navigation Rolling.
- Renamed folders, deleted tracing.md, and edited README.md.
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
- Added galactic CI setup and renamed Rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Made more Readme updates (#72) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Made more Readme updates (#74) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Created new sm from sm_respira_1 (#76).
- Made feature/core and navigation fixes (#78).
- Based for the sm_aws_aarehouse navigation.
- Made progress in AWS navigation.
- Made several core improvements during navigation testing.
- Made formatting improvements.
- Made progress in AWS navigation demo.
- Made format improvements.
- Made more on navigation.
- Reworked sm_advanced_recovery_1 (#83).
- Fixed pre-commit for sm_advanced_recovery_1.
- Made more sm_advanced_recovery_1 work (#85).
- Continued sm_advanced_recovery_1 round 4 (#86).
- Created Brettpac branch (#87).
- Added sm_atomic_performance_test_a_2.
- Added sm_atomic_performance_test_a_1.
- Added sm_atomic_performance_test_c_1 (#88).
- Modified sm_atomic_performance_test_a_2 (#89).
- Added sm_multi_stage_1.
- Fixed precommit for sm_multi_stage_1.
- Made more sm_multi_stage_1 changes (#91).
- Updated README.md with launch command.
```

```rst
Section_22
==========

Added
~~~~~
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success (#81, #82, #92, #93, #94, #95, #98)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Denis Štogl <denis@stogl.de>
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- Added navigation parameters fixes for sm_dance_bot (#93, #95)
- Added cb_pause_slam client behavior (#98)
- Added cb_pause_slam client behavior for sm_dance_bot_lite (#99)
- Added gazebo fixes for sm_dance_bot visualizing turtlebot3 (#104)
- Added choice for launching gazebo with lidar for dance bot (#102)
- Added sm_multi_stage_1 doubling (#103)

Changed
~~~~~~~
- Corrected all linters and formatters (#82)

Fixed
~~~~
- Removed some compile warnings (#96)
- Minor hotfix for doxygen deployment workflow (#100)
```

```rst
Section_23
==========

Added
-----
- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Visualizing turtlebot3
  - Cleaning and lidar show/hide option
  - Gazebo fixes to show the robot and the lidar
  - Format fixes
- Precommit cleanup run (#106)
- AWS demo (#108)
- Got sm_multi_stage_1 working (barely) (#109)
- Brettpac branch (#110)
- A3 (#113)
- Diverse improvements navigation and performance (#116)
- Feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing slam pausing/resuming functionality for testing sm_dance_bot
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern
- First working version of sm template and template generator (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
  - Some more progress on markers cleanup
- Minor navigation improvements (#141)
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Slight waypoint 4 and iterations changes so the robot can complete the course (#155)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Progressing in the moveit migration testing
  - Adding .reps dependencies and fixing some build errors
  - Adding dependency to ur5 client
  - Docker refactoring
  - Progress on move_it PR
- Update readme (#164)
  - More readme updates
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Pre-commit cleanup
- Add QOS durability to SmaccPublisherClient (#163)
  - Add qos durability to SmaccPublisherClient
  - Add reliability qos config
- Feature/testing moveit behaviors (#167)

Changed
-------
- Move method after the method it calls to prevent recursion (#126)
- Noticed typo corrected to "Finally"

Removed
-------
- Remove neo_simulation2 package (#112)
  - Correct formatting
  - Enable source build on PR for testing
  - Adjust build packages of source CI
- Remove node creation and create only a logger (#149)
- Removing parameters smacc (#147)
  - Workflows update
  - Noticed launch command was incorrect in README.md, fixed for sm_dance_bot_strikes_back

Authors
-------
- Pablo Iñigo Blasco (@pabloinigoblasco)
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_24
==========

Added
-----

- Added more testing on MoveIt.
- Added progress on MoveIt behaviors.
- Added minor configuration changes.
- Added repository dependency for AWS navigation in sm_dance_bot.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added updates to dependencies for husky in rolling and galactic.
- Added progress on AWS navigation and refactorings on navigation clients and behaviors.
- Added more on AWS demo.
- Added warehouse2 progress.
- Added waypoint inputs.
- Added progress on sm_dance_bot_warehouse_3.
- Added redoing waypoints for sm_dance_bot_warehouse_3.
- Added more waypoints for sm_dance_bot_warehouse_3.
- Added finetuning waypoints.
- Added pure spinning behavior missing files.
- Added several fixes.
- Added replanning for all examples.
- Added improvements in undo motion navigation for warehouse2.
- Added tuning for warehouse3.
- Added fixing warehouse3 problems and other core improvements.
- Added backport to foxy.
- Added missing file from warehouse2.
- Added updates to subscriber publisher components.
- Added progress in autoware machine.
- Added refining cp subscriber cp publisher.
- Added improvements in smacc core for autoware demo.
- Added updates to docker files for different revisions.
- Added warnings removal and more testing on navigation.
- Added fixing docker for foxy and galactic.
- Added docker build files for all versions.
- Added barrel demo.
- Added barrel search build fix and warehouse3.
- Added fixing startup problems in warehouse3.
- Added progress in barrel husky.
- Added progress in barrel demo.
- Added testing dance bot demos.
- Added updating galactic repositories.
- Added runtime dependency restoration.

Changed
-------

- Changed formatting in various features.
- Changed default values in multiple features.
- Changed some reordering fixes.
- Changed format issues in features.
- Changed tuning and fixes in various features.

Fixed
-----

- Fixed pipeline error.
- Fixed broken master build.
- Fixed broken build issues.
- Fixed linting errors.
- Fixed minor linking errors for foxy.
- Fixed weird moveit not downloaded repository.
- Fixed formatting and templating on SrConditional.
- Fixed move trigger logic into headers.
- Fixed dead lock issues to make continuous integration green.
- Fixed broken build issues.

Removed
-------

- Removed some redundant entries.
```

```rst
Section_25
==========

Version 0.1.0 (Date: Unreleased)
--------------------------------

### Added
- Build-status table.
- Detailed install instructions ([source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme)).
- `setupTracing.sh` script for installing necessary packages and configuring tracing group.

### Changed
- Default build type set to `Release` for faster performance and smaller executables.
- Updated examples section.

### Fixed
- Resolved missing dependency in `smacc_msgs` and reorganized for better overview.
- Fixed bug in `smacc2` component.
- Cleaned up formatting in `sm_respira_1` and `sm_atomic_24hr`.
- Optimized dependencies in `move_base_z_planners_common`.
- Renamed event generator library.
- Corrected build-overview table.
- Updated and unified CI configurations.
- Used `tf_geometry_msgs.h` in Galactic.
- Updated to use Galactic branches in `.repos-file`.

### Removed
- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`.

### Miscellaneous
- Reverted commit related to ignoring packages except `smacc2` and `smacc2_msgs`.
- Reorganized project structure.
- Added README tutorial for Dockerfile.
- Edited tracing.md to reflect new tracing event names.
- Improved performance tests.
- Various fixes and improvements.

Contributors: Denis Štogl, Pablo Iñigo Blasco, DecDury, reelrbtx, brettpac
```
