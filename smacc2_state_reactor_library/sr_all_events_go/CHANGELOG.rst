Changelog for package sr_all_events_go
=========================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2 <https://github.com/robosoft-ai/SMACC2>`_
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue

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
- Ignored packages not to be released
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup for sm_respira_1
  - Updated smacc2_rta command across readmes
  - Cleaned up sm_atomic_24hr
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Added galactic CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Updated c_cpp_properties.json
  - Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Updated doxygen links
  - More Readme Updates
  - Created new sm from sm_respira_1
  - Feature/core and navigation fixes
  - Several core improvements during navigation testing
  - Progress in aws navigation demo
  - New feature, cb_wait_topic_message: asynchronous client behavior
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic

### Contributors
- brettpac, Denis Štogl, Ubuntu 20-04-02-amd64

```rst
Section_2
=========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Optionally select nodes to wait for.
- Progress in AWS navigation demo.
- Base for the `sm_aws_warehouse` navigation.
- Navigation parameters fixes on `sm_dance_bot`.
- New client behavior: `cb_pause_slam`.
- Visualizing `turtlebot3` in `sm_dance_bot_lite`.
- `dance_bot` launch `gz_lidar` choice.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.
- Got `sm_multi_stage_1` working (barely).
- Brettpac branch.
- Remove `neo_simulation2` package.
- Diverse improvements in navigation and performance.

Changed
-------
- Minor formatting improvements.

Fixed
-----
- Remove some compile warnings.

Removed
-------
- `neo_simulation2` package.

Contributors
------------
- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_3

### Added

- Feature/diverse improvements navigation performance (#117)
  - Diverse improvements in navigation and performance.
  - Additional linting and formatting.
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
  - Introducing smacc2::deep_history syntax.
  - Testing sm_dance_bot with slam pausing/resuming functionality.
- Feature/more_sm_dance_bot_fixes
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern.
- First working version of sm template and template generator. (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases.
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2.
  - Fixing errors introduced on formatting.
  - Progressing in the moveit migration testing.
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
  - More testing on moveit behaviors.
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)
  - Multistage modes.
  - Sequences for sm_multi_stage.
  - Steps for sm_multi_state_1.
  - Finishing touches.
- Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and refactorings on navigation clients and behaviors.
  - More on aws demo.
- Waypoint Inputs (#178)
- sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
  - More changes and headless.

### Changed

- Move method after the method it calls. Otherwise, recursion could happen. (#126)
- Noticed "Finnaly" typo, corrected to "Finally."
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back, and removed some comments.
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Removing sm_dance_bot_msgs.
- Removing parameters smacc (#147)
- Update readme (#164)
- More readme updates.

### Fixed

- Fixing broken master build.
- Fixing pipeline error.
- Minor configuration.
- Fixing compiling issues.
- Format (#180).

### Removed

- Remove merge markers from a Python file. (#119).
- Removing test from main moveit CMake.
- Removing test ur5.
- Removing parameters smacc.
- Removing sm_dance_bot_msgs.
- Removing some comments in the past.

### Contributors

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
- Brettpac branch (#184)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)
- Pure spinning behavior missing files
- Minor changes (#190)
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
- Foxy backport (#206)
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- Rename header files and correct format
- Add workflow for checking doc build
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference
- Update name of package and package.xml to pass liter
- Reset all versions to 0.0.0
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Update smacc_sm_reference_library/sm_atomic/README.md
- Reactivating smacc2 nav clients for rolling via submodules
- Renamed tracing events after
- Bug in smacc2 component
- Reverted markdowns to html
- Added README tutorial for Dockerfile
- Additional cleanup
- Edited tracing.md to reflect new tracing event names
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2

Changed
-------
- Ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- First ensure you have the necessary package installed
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
- Branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
```

*pabloinigoblasco*

```rst
Section 5
=========

Added
-----

- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files. (#58)
- Added more Readme Updates (#72).
- Added more Readme (#74).
- Added new sm from sm_respira_1 (#76).
- Added base for the sm_aws_aarehouse navigation.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait.
- Added navigation parameters fixes on sm_dance_bot.

Changed
-------

- Renamed folders, deleted tracing.md, and edited README.md.
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated README.md launch command.
- Corrected all linters and formatters.

Fixed
-----

- Corrected trailing spaces.
- Fixed source CI and corrected README overview. (#62).
- Fixed pre-commit.
- Attempted pre-commit fixes.

Removed
-------

- Do not execute clang-format on smacc2_sm_reference_library package.

Authors
-------

- Pablo Iñigo Blasco
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
  - Implemented base for the sm_aws_warehouse navigation
  - Made progress in AWS navigation
  - Introduced several core improvements during navigation testing
  - Improved formatting
  - Added new feature: cb_wait_topic_message for asynchronous client behavior waiting for a topic message and optionally checking its contents for success
  - Added new client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and waiting for them to be alive
  - Continued progress in AWS navigation demo

Changed
-------

- Feature/sm dance bot fixes (#95)
  - Improved navigation parameters on sm_dance_bot

- Feature/cb pause slam (#98)
  - Added cb pause slam client behavior

- sm_dance_bot_lite (#99)
  - Updated yaml
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- Rename doxygen deployment workflow (#100)

- sm_dance_bot visualizing turtlebot3 (#101)
  - Added cleaning and lidar show/hide option
  - Fixed formatting

- Feature/dance bot launch gz lidar choice (#102)
  - Added cleaning files and lidar show/hide option

- Feature/sm dance bot lite gazebo fixes (#104)
  - Fixed gazebo to show the robot and the lidar
  - Fixed formatting

- sm_multi_stage_1 doubling (#103)
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Fixed gazebo for sm_dance_bot_strikes_back

- precommit cleanup run (#106)
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- aws demo (#108)
  - Demonstrated AWS functionality

- got sm_multi_stage_1 working (barely) (#109)
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- Brettpac branch (#110)
  - Made progress on sm_multi_stage_1

- Brettpac branch (#111)
  - Extended sm_multi_stage_1 to 5 stages
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- a3 (#113)
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- Remove neo_simulation2 package. (#112)
  - Removed neo_simulation2 package
  - Corrected formatting
  - Enabled source build on PR for testing
  - Adjusted build packages of source CI

- mm (#115)
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- diverse improvements navigation and performance (#116)
  - Introduced diverse improvements in navigation and performance

- Feature/diverse improvements navigation performance (#117)
  - Added additional linting and formatting

- Remove merge markers from a python file. (#119)

- Feature/slam toggle and smacc deep history (#122)
  - Progressed in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introduced smacc2::deep_history syntax
  - Added slam pausing/resuming functionality to sm_dance_bot

- more changes in sm_dance_bot (#125)

- Move method after the method it calls. Otherwise, recursion could happen. (#126)

- Feature/dance bot s pattern (#128)
  - Polished sm_dance_bot and s-pattern
  - Corrected typo ("Finnaly" to "Finally")

- Feature/dance bot s pattern (#129)
  - Continued refinement in sm_dance_bot
```
**Author:** Pablo Iñigo Blasco (pabloinigoblasco)

Section 7
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
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- initial migration to smacc2 (#151)
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
- Feature/planner changes 16 12 (#191)

Changed
-------
- minor tweaks (#130)
- minor format issues (#134)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Add dependencies for husky simulation.
- Update dependencies for husky in rolling and galactic.

Fixed
-----
- minor tuning to mitigate overshot issue cases
- fixing some errors introduced on formatting
- fixing some more linting warnings
- fixing compiling issues
- fixing broken master build
- fixing pipeline error
- fixing broken build

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

Author
------
- Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_8
=========

Version 0.1.0 (2022-01-01)
---------------------------

Added
~~~~~

- Feature/replanning 16 dec (#193): Replanning for all examples.
- Feature/undo motion 20 12 (#196): Improving undo motion navigation in warehouse2.
- Feature/warehouse2 22 12 (#200): Finishing warehouse2.
- Feature/warehouse2 23 12 (#201): Tuning and fixes (#202).
- Feature/minor tune (#203): Fixing warehouse 3 problems and other core improvements (#204).
- Merging code from backport foxy and updates about autoware (#208): Backport to foxy.
- Foxy backport (#206): Minor formatting fixes, trailing spaces, codespell corrections, linters adjustments, CI build for Galactic, ament_cpplint changes, tf2_ros dependency, ccache version bump, licenses added, and more.

Changed
~~~~~~~

- Renamed header files and corrected format.
- Updated doxygen-check-build.yml and created doxygen-deploy.yml.
- Renamed to smacc2 and smacc2_msgs.
- Updated name of package and package.xml.
- Reset all versions to 0.0.0.
- Updated description table and changelogs.

Fixed
~~~~~

- Fixed minor broken build.
- Enabled cppcheck and corrected formatting of python files.
- Included necessary package and edited Threesome launch.
- Reactivated smacc2 nav clients for Rolling via submodules.
- Fixed bug in smacc2 component.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Added smacc2_performance_tools for performance tests improvements.

Removed
~~~~~~~

- Deleted tracing directory.
- Removed manual installation of ros-rolling-ros2trace.
- Ignored all packages except smacc2 and smacc2_msgs.
- Removed galactic builds from master and kept only rolling.
``` 

*pabloinigoblasco*

```rst
Section_9
=========

Added
-----

- Added galactic CI setup and renamed rolling files. (#58)
- Added more readme updates. (#72)
- Added more readme updates. (#74)
- Added new state machine from sm_respira_1. (#76)
- Added feature/core and navigation fixes. (#78)
- Added base for the sm_aws_aarehouse navigation.
- Added progress in AWS navigation.
- Added several core improvements during navigation testing.
- Added progress in AWS navigation demo.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.
- Added progress in AWS navigation demo.
- Added navigation parameters fixes on sm_dance_bot.

Changed
-------

- Changed launch command to 'ros2 launch sm_respira_1 sm_respira_1.launch'. (#69)
- Changed doxygen links. (#70)
- Changed source CI and corrected README overview. (#62)

Fixed
-----

- Fixed trailing spaces.
- Fixed pre-commit.
- Fixed pre-commit.
- Fixed pre-commit.
- Corrected all linters and formatters.

Removed
-------

- Removed execution of clang-format on smacc2_sm_reference_library package.

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

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- Gazebo fixes to show the robot and the lidar.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo progress.
- Source build enabled on PR for testing.
- `smacc2::deep_history` syntax introduced.
- `sm_dance_bot` introducing slam pausing/resuming functionality.
- First working version of `sm` template and template generator.

Changed
-------

- Minor format improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Cleaning and lidar show/hide option in `sm_dance_bot`.
- Polishing `sm_dance_bot` and `s-pattern`.
- Typo correction: "Finnaly" to "Finally".
- Method moved after the method it calls to prevent recursion.

Fixed
-----

- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Remove merge markers from a Python file.
- Additional linting and formatting.

Removed
-------

- `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
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
- Added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/undo motion 20 12 (#196)

Changed
-------

- Minor tuning to mitigate overshot issue cases
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Fix CI: format fix python version (#148)
- Initial migration to smacc2
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Progress on moveit migration testing
- Improving dockerfile for building local tests
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----

- Resolve compile warnings (#137)
- Noticed launch command was incorrect in README.md
- Fixing compiling issues
- Fixing broken master build
- Fixing broken build
- Several fixes (#194)

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing node creation and create only a logger. (#149)
- Removing parameters smacc
- Removing dependencies for husky in rolling and galactic.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_12
-----------

Added
~~~~~
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
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
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter.
- Reset all versions to 0.0.0
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table.
- Update table
- Copy initial docs
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic

Changed
~~~~~~~
- improving undo motion navigation warehouse2
- tuning warehouse3 (#197)
- undo tuning and errors
- format issues
- replanning for all our examples
- finishing warehouse2
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- fixing docker for foxy and galactic
- removing warnings (#213)
- some reordering fixes
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Change extension of imports.
- Correct GitHub branch reference.
- Execute on master update
- changed wording "smacc application" to "SMACC2 library"
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh

Fixed
~~~~~
- minor broken build

Removed
~~~~~~~
- weird moveit not downloaded repo

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>

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
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Renamed smacc2_sm_reference_library to sm_reference_library.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.

Fixed
~~~~
- Bug in smacc2 component.
- Reverted markdowns to html.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Optimized dependencies in move_base_z_planners_common.
- Minor formatting fixes.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Minor format fixes.
- Pre-commit fixes.

Removed
~~~~~~~
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other
~~~~~
- Some progress on navigation rolling.
- More changes on performance tests.
- More on performance and other issues.
- More on navigation.
- Progress in aws navigation.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress

```rst
Section_14
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Nodes to wait can be optionally selected.
- Introduced `cb_pause_slam` client behavior.
- Added `sm_dance_bot_lite` feature for visualizing Turtlebot3.
- Added `sm_multi_stage_1` doubling feature.
- Added gazebo fixes for `sm_dance_bot_strikes_back`.
- Implemented AWS demo.

Changed
-------
- Improved core functionality during navigation testing.
- Made formatting improvements.
- Progressed in AWS navigation demo.
- Fixed navigation parameters on `sm_dance_bot`.
- Cleaned and added lidar show/hide option for `sm_dance_bot`.
- Enhanced formatting for various features.
- Updated yaml files.
- Adjusted build packages for source CI.
- Improved navigation and performance.

Fixed
-----
- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting issues.
- Removed merge markers from a Python file.

Removed
-------
- Removed `neo_simulation2` package.

Contributors
------------
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by: pabloinigoblasco <pablo@ibrobotics.com>
```

Section 15
===========

Added
-----
- Introducing slam pausing/resuming functionality to sm_dance_bot (#125)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Add SM core test (#138)
- Add QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation (#174)
- Waypoint Inputs (#178)

Changed
-------
- Move method after the method it calls to prevent recursion (#126)
- Renamed state machine transition timestamp to initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Minor navigation improvements (#141)
- Using local action messages instead of sm_dance_bot_msgs (#139)
- Renamed sm_advanced_recovery_1 to sm_advanced_recovery_1 renaming (#171)
- Reworked sm_multi_stage_1 (#172)
- Progress on moveit migration testing (#151)
- Added .reps dependencies and fixed build errors (#151)
- Improved Dockerfile for building local tests (#151)
- Added reliability qos config to SmaccPublisherClient (#163)

Fixed
-----
- Noticed typo "Finnaly" corrected to "Finally"
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148)
- Fixed CI format for Python version (#148)
- Fixed compiling issues (#164)
- Fixed broken master build (#167)
- Fixed pipeline error (#167)
- Fixed warehouse2 formatting (#180)
- Fixed some formatting and templating on SrConditional (#168)
- Moved trigger logic into headers in SrConditional (#168)
- Lint fixes in SrConditional (#168)

Removed
-------
- Removed node creation and created only a logger (#149)
- Removed parameters smacc (#147)
- Removed test from main moveit CMake (#151)
- Removed some comments in the past from launch command for sm_dance_bot_strikes_back (#148)
- Removed sm_dance_bot_msgs (#139)

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_16
==========

Added
-----
- Added finetuning waypoints (#187) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Added Feature/cb pure spinning (#188)
- Added Feature/cb pure spinning (#189)
- Added pure spinning behavior missing files
- Added Feature/planner changes 16 12 (#191)
- Added Feature/replanning 16 dec (#193)
- Added several fixes (#194)
- Added Feature/undo motion 20 12 (#196)
- Added tuning warehouse3 (#197)
- Added Feature/undo motion 20 12 (#198)
- Added Feature/sync 21 12 (#199)
- Added Feature/warehouse2 22 12 (#200)
- Added Feature/warehouse2 23 12 (#201)
- Added Feature/minor tune (#203)
- Added fixing warehouse 3 problems, and other core improvements (#204)
- Added fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Added weird moveit not downloaded repo
- Added missing file from warehouse2 (#205)
- Added docker files for different revisions, warnings removal and more testing on navigation
- Added fixing docker for foxy and galactic
- Added docker build files for all versions
- Added dockerfiles (#225)
- Added Feature/retry behavior warehouse 1 (#226)
- Added Foxy backport (#206)
- Added workflow for checking doc build
- Added doxygen-check-build.yml
- Added doxygen-deploy.yml
- Added workflow for testing prerelease builds
- Added Dockerfile w/ ROS distro as argument
- Added new folder for additional tracing contents
- Deleted tracing directory

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Fixed
-----
- Fixed minor broken build
- Fixed trailing spaces
- Fixed codespell
- Fixed python linters warnings
- Fixed other build issues

Removed
-------
- Removed use of node in the sm performance template
- Removed ament_cpplint
- Removed cpplint and cppcheck linters
- Removed disabled packages
- Removed disabled packages
- Removed some packages and update workflows
- Removed further packages
- Removed disabled packages
- Removed disabled packages
- Removed disabled packages

Authors
-------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

Section 17
===========

Added
-----
- Moved tracing.md to tracing directory.
- Added setupTracing.sh script to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally, you can select the nodes to wait.

Changed
-------
- Renamed "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Minor formatting changes.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Corrected trailing spaces.
- Updated c_cpp_properties.json.
- Fixed source CI and corrected README overview.
- Corrected all linters and formatters.

Fixed
-----
- Bug in smacc2 component.
- Reverted markdowns to HTML.
- Fixed build of missing rolling repositories.
- Fixed Navigation2 for semi-binary build.
- Fixed pre-commit issues.

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed manual tracing.md file.
- Removed galactic builds from master, keeping only rolling.
- Removed submodules and use .repos file.

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>

```rst
Section_18
==========

Added
~~~~~
- Feature/aws demo progress (#92)
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and ensure they are alive
- Feature/cb pause slam (#98)
- cb pause slam client behavior
- sm_dance_bot visualizing turtlebot3 (#101)
- Feature/dance bot launch gz lidar choice (#102)
- sm_dance_bot_lite gazebo fixes (#104)
- gazebo fixes for sm_dance_bot_strikes_back
- aws demo (#108)
- Brettpac branch (#110, #111)
- a3 (#113)

Changed
~~~~~~~
- Navigation parameters fixes on sm_dance_bot

Fixed
~~~~
- Remove some compile warnings (#96)
- Remove neo_simulation2 package (#112)

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section 19
-----------

Added
~~~~~
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Diverse improvements in navigation and performance.
- Additional linting and formatting.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introducing slam pausing/resuming functionality in sm_dance_bot.
- First working version of sm template and template generator.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Rolling Docker environment to be executed from any environment.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors.
- More on AWS demo.

Changed
~~~~~~~
- Move method after the method it calls to prevent recursion.
- Renamed SMs to smacc2_performance_tools.
- Minor navigation improvements.
- Using local action messages.
- Removed node creation and create only a logger.
- Husky launch file in sm_dance_bot.
- Update dependencies for husky in rolling and galactic.

Fixed
~~~~
- Remove merge markers from a python file.
- Fix CI: format fix python version.
- Noticed launch command was incorrect in README.md.
- Fixing broken master build.
- Fixing pipeline error.
- Fixing compiling issues.

Removed
~~~~~~~
- Removing sm_dance_bot_msgs.
- Removing parameters smacc.

Authors
~~~~~~~
- Pablo Iñigo Blasco (pabloinigoblasco)
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

- Waypoint Inputs (#178)
- wharehouse2 progress (#179)
- sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- finetuning waypoints (#187)
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
- Feature/improvements warehouse3 (#228)
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

Changed
-------

- SrConditional fixes and formatting (#168)
- redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- move trigger logic into headers
- minor changes
- replanning for all our examples
- improving undo motion navigation warehouse2
- format issues
- fixing format and minor
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- fixing docker for foxy and galactic
- retry behavior warehouse 1
- other minor changes
- progress in barrel husky
- making models local
- red picuup
- multiple controllable leds plugin
- progress in husky demo
- improving navigation behaviors
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
- Change extension of imports.

Removed
-------

- Only rolling version should be pre-released on on master. (#230)
- Correct codespell.
- Correct python linters warnings.
- Add ignition file and update repos files.
- progressing in husky demo
- Disable disabled packages
- Update ci-build-source.yml
- Enable cppcheck

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
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
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
- Created workflow for testing prerelease builds.
- Created setupTracing.sh to install necessary packages and configure tracing group.
- Added alternative ManualTracing.
- Added new sm markdowns.
- Added Dockerfile for Rolling and Galactic.
- Added smacc2_performance_tools.
- Added performance tests improvements.
- Added more on performance and other issues.
- Added sm_atomic_24hr.
- Added sm_atomic_performance_trace_1.
- Added sm_respira_test_2.
- Added sm_respira_1 format cleanup.
- Added sm_respira_1 format cleanup pre-commit.
- Added sm_atomic_24hr cleanup.
- Added optimized dependencies in move_base_z_planners_common.
- Added renaming of event generator library.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated smacc2_rta command across readmes.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated README.md.

Changed
-------
- Renamed ros2 launch sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Renamed header files and corrected format.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Renamed to smacc2 and smacc2_msgs.
- Corrected GitHub branch reference.
- Updated name of package and package.xml to pass liter.
- Executed master update.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Updated description table.
- Updated table.
- Copied initial docs.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Made some progress on navigation rolling.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Fixed bug in smacc2 component.
- Reverted markdowns to html.
- Edited tracing.md to reflect new tracing event names.
- Corrected trailing spaces.
- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax.
- Cleaned up sm_atomic_24hr.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Tracing directory.
- Manually installed ros-rolling-ros2trace from README.md under "Getting started".
- Deleted tracing.md.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed tracing.md.
- Removed submodules and used .repos file.
- Removed galactic builds from master and kept only rolling.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Contributors
------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_22
==========

Added
~~~~~
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92, #93, #94, #95, #98)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, Denis Štogl <destogl@users.noreply.github.com>, Denis Štogl <denis@stogl.de>

- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)

Changed
~~~~~~~
- Corrected all linters and formatters (#82)

Fixed
~~~~
- Navigation parameters fixes on sm_dance_bot (#93, #95)

Removed
~~~~~~~
- Removed some compile warnings (#96)

Miscellaneous
~~~~~~~~~~~~~
- Minor hotfix (#100)
- Updates yaml (#99)
- Precommit fixes (#99)
- Format improvements (#100, #99)
- Progress in AWS navigation demo (#92, #93, #94, #95, #98)
- Formatting improvements (#92, #93, #94, #95, #98)
- Several core improvements during navigation testing (#92, #93, #94, #95, #98)
- Base for the sm_aws_warehouse navigation (#81, #82, #92, #93, #94, #95, #98)
- Progressing in AWS navigation (#81, #82, #92, #93, #94, #95, #98)
- More on navigation (#81, #82, #92, #93, #94, #95, #98)
- Merge and progress (#94)
- Cleaning and lidar show/hide option (#102, #104)
- Cleaning files and making formatting work (#102, #104)
- More fixes (#102, #104)
- Gazebo fixes to show the robot and the lidar (#104)
- Sm_dance_bot visualizing turtlebot3 (#101, #104)
- Sm_dance_bot_lite (#99)
- Sm_multi_stage_1 doubling (#103)
- Attempting precommit fixes (#81)
```

```rst
Section_23
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
- Got sm_multi_stage_1 working (barely) (#109)
- Brettpac branch (#110)
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
- Feature/dance bot s pattern (#128)
- Minor tweaks (#130)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
  - Build fix
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
  - Navigation 2 stack renaming
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140)
  - Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Removing parameters smacc (#147)
  - Workflows update
  - Noticed launch command was incorrect in README.md
    - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Fix CI: format fix python version (#148)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing errors introduced on formatting
  - Missing dependency
  - Fixing linting warnings
  - Removing test from main moveit cmake
  - Progressing in the moveit migration testing
  - Adding .reps dependencies and fixing build errors
  - Adding dependency to ur5 client
  - Docker refactoring
  - Progress on move_it PR
  - Improving dockerfile for building local tests
  - Fixing compiling issues
- Update readme (#164)
  - More readme updates

Changed
-------
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Pre-commit cleanup
- Add QOS durability to SmaccPublisherClient (#163)
  - Added qos durability to SmaccPublisherClient
  - Added reliability qos config
  - Removed line
  - Fixed missing colon

Removed
-------
- Remove neo_simulation2 package (#112)
  - Correct formatting
  - Enable source build on PR for testing
  - Adjust build packages of source CI
```

*pabloinigoblasco*

Section_24
===========

Added
-----
- Added `sm_pubsub_1` (#169) with contributions from Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Added `sm_pubsub_1 part 2` (#170) with contributions from Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Added `sm_advanced_recovery_1 renaming` (#171) with contributions from Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Added `sm_multi_stage_1 reworking` (#172).
- Added `sm_multi_stage sequences`.
- Added `sm_multi_state_1 steps`.
- Added `sm_multi_stage_1 sequence d`.
- Added `sm_multi_stage_1 c sequence`.
- Added `mode_5_sequence_b`.
- Added `mode_4_sequence_b`.
- Added `sm_multi_stage_1 most`.
- Added `finishing touches 1`.
- Added `readme`.
- Added `Feature/aws navigation sm dance bot` (#174).
- Added repository dependency.
- Added `husky launch file` in `sm_dance_bot`.
- Added dependencies for `husky simulation`.
- Added `progress on aws navigation` and refactorings on navigation clients and behaviors.
- Added more content on `aws demo`.
- Added `warehouse2` (#177).
- Added `Waypoint Inputs` (#178).
- Added `wharehouse2 progress` (#179).
- Added `format` (#180).
- Added `sm_dance_bot_warehouse_3` (#181).
- Added `Feature/sm warehouse 2 13 dec 2` (#182).
- Added more changes and `headless` mode.
- Added `merge` functionality.
- Added `default values`.
- Added `Brettpac branch` (#184).
- Added `Redoing sm_dance_bot_warehouse_3 waypoints`.
- Added more `Waypoints`.
- Added `SrConditional fixes and formatting` (#168).
- Added `fix` for formatting and templating on `SrConditional`.
- Added `fix` to move trigger logic into headers.
- Added `fix` for linting.
- Added `Feature/wharehouse2 dec 14` (#185).
- Added `Feature/sm warehouse 2 13 dec 2` (#186).
- Added `finetuning waypoints` (#187).
- Added `Feature/cb pure spinning` (#188).
- Added `pure spinning behavior missing files`.
- Added `Feature/planner changes 16 12` (#191).
- Added more fixes.
- Added `Feature/replanning 16 dec` (#193).
- Added `replanning for all examples`.
- Added `several fixes` (#194).
- Added `Feature/undo motion 20 12` (#196).
- Added `improving undo motion navigation warehouse2`.
- Added `tuning warehouse3` (#197).
- Added `Feature/sync 21 12` (#199).
- Added `format issues`.
- Added `Feature/warehouse2 22 12` (#200).
- Added `finishing warehouse2`.
- Added `Feature/warehouse2 23 12` (#201).
- Added `tuning and fixes` (#202).
- Added `Feature/minor tune` (#203).
- Added `tuning and fixes`.
- Added `fixing warehouse 3 problems` and other core improvements (#204).
- Added `weird moveit not downloaded repo`.
- Added missing file from `warehouse2` (#205).
- Added `backport to foxy`.
- Added `minor format`.
- Added `minor linking errors foxy`.
- Added `updating subscriber publisher components`.
- Added `progress in autoware machine`.
- Added `refining cp subscriber cp publisher`.
- Added improvements in `smacc core` with more components developed for autoware demo.
- Added `autoware demo`.
- Added `foxy ci`.
- Added `fix` for broken build.
- Added `some reordering fixes`.
- Added `docker files` for different revisions, warnings removal, and more testing on navigation.
- Added `fixing docker for foxy and galactic`.
- Added `docker build files` for all versions.
- Added `barrel demo`.
- Added `barrel search build fix` and `warehouse3`.
- Added `fixing startup problems in warehouse 3`.
- Added `fixing format` and `minor`.
- Added `progress in barrel husky`.
- Added `barrel demo progress`.
- Added `testing dance bot demos`.
- Added `updating galactic repos`.
- Added `runtime dependency`.
- Added `restoring ur dependency`.

Changed
-------
- Updated dependencies for `husky` in `rolling` and `galactic`.

Fixed
-----
- Fixed `minor configuration`.
- Fixed `pipeline error`.
- Fixed `broken master build`.
- Fixed `formatting`.
- Fixed `lint`.
- Fixed `format issues`.
- Fixed `formatting and templating`.
- Fixed `move trigger logic`.
- Fixed `broken build`.
- Fixed `missing files`.
- Fixed `errors`.
- Fixed `deadlock` issues.
- Fixed `continuous integration` to be green.
- Fixed `missing sm`.
- Fixed `format issues`.
- Fixed `startup problems`.
- Fixed `linking errors`.
- Fixed `broken build`.

Removed
--------
- Removed `weird moveit not downloaded repo`.

Co-authored-by
--------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Denis Štogl <denis@stogl.de>.
- Denis Štogl <destogl@users.noreply.github.com>.
- Declan Dury <44791484+DecDury@users.noreply.github.com>.
- DecDury <declandury@gmail.com>.
- reelrbtx <brett2@reelrobotics.com>.
- brettpac <brett@robosoft.ai>.
- David Revay <MrBlenny@users.noreply.github.com>.
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>.

## Version 0.1.0 (2022-01-01)

### Added
- Build-status table.
- Detailed install instructions ([source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme)).
- `setupTracing.sh`: Installs necessary packages and configures tracing group.

### Changed
- Default build type set to `Release` for faster build and smaller executables.
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
- Reverted commit regarding package selection.
- Refactored getLogger functionality.
- Reorganized project structure.
- Updated README.md.
- Edited tracing.md to reflect new tracing event names.
- Performance tests improvements and related fixes.
- Do not execute clang-format on `smacc2_sm_reference_library`.
- Reactivated `smacc2` nav clients for Rolling via submodules.

*Contributors: Denis Štogl, Pablo Iñigo Blasco, DecDury, reelrbtx, brettpac*

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
