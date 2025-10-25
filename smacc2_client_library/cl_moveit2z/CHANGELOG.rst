Changelog for package cl_moveit2z
======================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for a strange issue with ros buildfarm.
  - Further work on resolving the buildfarm issue.
  - Co-authored by brettpac <brett@robosoft.ai>.
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
No changes listed.

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release.
- Contributors: pabloinigoblasco

### Changed
- Continued progress in humble SMACC2 deb generation.
- Checked humble status.
- Published updates.
- Made progress in migrating to humble.
- Improved husky_improvements (#299).
  - Various enhancements related to husky.
  - Introduced different planners profiles for navigation.
  - Incorporated changes from galactic.
  - Implemented a planner switcher.
  - Utilized galactic branch files.
  - Resolved breaking changes.
  - Made minor fixes.
  - Removed nav from source files.
  - Merged changes.
- Feature/barrel husky improvements (#293).
  - Renamed to smacc2 and smacc2_msgs.
  - Corrected GitHub branch reference.
  - Updated package names and package.xml for compliance.
  - Executed master update.
  - Reset all versions to 0.0.0.
  - Ignored all packages except smacc2 and smacc2_msgs.
  - Updated changelogs.
  - Reverted previous commit.
  - Updated description table.
  - Updated table.
  - Copied initial docs.
  - Created Dockerfile with ROS distro as argument.
  - Opened new folder for additional tracing contents.
  - Deleted tracing directory.
  - Moved tracing.md to tracing directory.
  - Added setupTracing.sh for configuring tracing group.
  - Automated installation of ros-rolling-ros2trace in setupTracing.sh.
  - Created alternative ManualTracing.
  - Added new sm markdowns.
  - Added Dockerfile for Rolling and Galactic.
  - Updated various files.
  - Co-authored by Denis Štogl <destogl@users.noreply.github.com>.
- Various performance improvements and bug fixes.
- Refactored and optimized code in several packages.
- Updated CI setup and corrected README information.
- Made corrections and improvements in multiple files.
- Continued work on navigation and core functionalities.
- Reworked and fixed issues in specific components.
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.

### Removed
- Removed manual installation steps for ros-rolling-ros2trace.
- Removed galactic builds from master branch.
- Removed submodules and used .repos file for dependencies.
- Removed mentions of SMACC/ROS in favor of SMACC2/ROS2.
- Removed trailing spaces in code.
- Removed unused files and directories.

### Fixed
- Fixed various bugs and issues in performance tests.
- Fixed formatting inconsistencies in code and documentation.
- Fixed errors in event generator library naming.
- Fixed launch command in specific files.
- Updated doxygen links.
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.

```rst
Section_2
=========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.
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

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

Commits
-------
- [#89] Modified sm_atomic_performance_test_a_2.
- [#90] Added sm_multi_stage_1 and fixed precommit.
- [#91] Added more sm_multi_stage_1.
- [#81] Worked on wait topic message client behavior.
- [#82] Implemented wait nav2 nodes client behavior.
- [#92] Progressed in aws navigation demo.
- [#93] Fixed navigation parameters on sm_dance_bot.
- [#94] Merged and progressed in aws navigation.
- [#95] Fixed navigation parameters on sm_dance_bot.
- [#96] Removed some compile warnings.
- [#98] Added cb pause slam client behavior.
- [#100] Renamed doxygen deployment workflow.
- [#101] Visualized turtlebot3 in sm_dance_bot.
- [#102] Added choice for gz lidar launch in dance bot.
```

Section_3
==========

Added
-----
- Added visualization of turtlebot3 to sm_dance_bot.
- Added option to show/hide cleaning and lidar.
- Added gazebo fixes to display the robot and lidar.
- Added support for sm_multi_stage_1 doubling.
- Added gazebo fixes for sm_dance_bot_strikes_back.
- Added AWS demo functionality.
- Added progress on sm_multi_stage_1 functionality.
- Added Brettpac branch support.
- Added progress on diverse improvements in navigation and performance.
- Added progress on slam toggle and smacc deep history features.
- Added support for dance bot s pattern.
- Added first working version of sm template and template generator.
- Added support for SM core test.
- Added minor navigation improvements.
- Added support for using local action messages.
- Added navigation 2 stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added rolling Docker environment execution from any environment.
- Added progress on migration to smacc2.
- Added progress on moveit migration testing.
- Added update to readme.

Changed
-------
- Improved formatting and cleaning of files.
- Improved format fixes.
- Improved progress in testing sm_dance_bot.
- Improved polishing of sm_dance_bot and s-pattern.
- Improved method calling sequence to prevent recursion.
- Improved linting and formatting.
- Improved format fixes for CI.
- Improved source build enabling on PR for testing.
- Improved build packages of source CI.
- Improved compile warnings resolution.
- Improved removal of neo_simulation2 package.
- Improved removal of merge markers from a python file.
- Improved removal of parameters in smacc.
- Improved workflow updates.
- Improved launch command correction in README.md.
- Improved CI format fix for Python version.
- Improved removal of node creation and creation of only a logger.
- Improved progress on moveit migration testing.
- Improved Dockerfile for building local tests.
- Improved compiling issues resolution.

Fixed
-----
- Fixed recursion issue in method calling sequence.
- Fixed overshot issue cases in waypoints navigator.
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed some errors introduced on formatting during migration to smacc2.
- Fixed missing dependency issues.
- Fixed linting warnings.
- Fixed test from main moveit cmake.
- Fixed build errors.
- Fixed compiling issues.
- Fixed readme updates.

Removed
-------
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs.
- Removed parameters in smacc.

Contributors
------------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Brett <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_4
=========

Added
-----
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Brettpac branch (#184)
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Merging code from backport foxy and updates about autoware (#208)

Changed
-------
- Moved reference library SMs to smacc2_performance_tools
- Add reliability qos config
- More testing on moveit
- Progress on moveit
- Minor configuration
- Fixing pipeline error
- Fixing broken master build
- More on aws demo
- Fix formatting
- Update dependencies for husky in rolling and galactic
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- Improving undo motion navigation warehouse2
- Tuning warehouse3
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
- Included necessary package and edited Threesome launch

Fixed
-----
- Add a missing colon
- Remove line
- Move trigger logic into headers
- Lint
- Format issues

Removed
-------
- Weird moveit not downloaded repo
- Added missing file from warehouse2
- Disable disabled packages

Co-Authored-By
--------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>

pabloinigoblasco
```

```rst
Section 5
=========

Added
-----
- Created workflow for testing prerelease builds.
- Renamed packages to smacc2 and smacc2_msgs.
- Updated package name and package.xml to pass liter.
- Dockerfile with ROS distro as argument.
- Opened new folder for additional tracing contents.
- Added setupTracing.sh for configuring tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Added smacc2_performance_tools for performance tests.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files.
- Added feature cb_wait_topic_message for asynchronous client behavior.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Changed
-------
- Updated description table.
- Updated table.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc2_rta command across readmes.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.

Fixed
-----
- Corrected GitHub branch reference.
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Reformatted sm_reference_library.
- Cleaned up sm_atomic_24hr.
- Fixed pre-commit issues.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
- Deleted tracing directory.
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_6
=========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection.
- New feature: cb pause slam client behavior.

Changed
-------

- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor formatting improvements.
- Merge and progress in various features.
- Hotfix for doxygen deployment workflow.
- Cleaning and lidar show/hide option for sm_dance_bot visualizing turtlebot3.
- Gazebo fixes to show the robot and the lidar in various features.

Fixed
-----

- Removed some compile warnings.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```
---

Este fragmento mejorado del changelog mantiene toda la información relevante, agrupando los cambios similares bajo las categorías de "Added", "Changed" y "Fixed". Se ha conservado la autoría de Pablo Iñigo Blasco.

Section_7
==========

Added
-----

- Implemented sm_multi_stage_1 functionality (#109, #110, #111, #114, #172)
  - Initial work on sm_multi_stage_1
  - Progress and improvements on sm_multi_stage_1
  - Added support for multiple stages in sm_multi_stage_1

Changed
-------

- Removed neo_simulation2 package (#112)
  - Corrected formatting and enabled source build testing
- Renamed sm_advanced_recovery_1 to sm_advanced_recovery_1 (#171)
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Cleaned up pre-commit tasks

Fixed
-----

- Resolved compilation warnings (#137)
- Fixed CI formatting for Python version (#148)
- Updated package list (#142)
- Fixed launch command in README.md for sm_dance_bot_strikes_back
- Fixed minor navigation issues (#141)
- Fixed waypoint and iteration changes for course completion (#155)

Removed
-------

- Removed parameters from smacc (#147)
- Removed node creation in favor of logger (#149)

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_8
=========

Added
-----
- Introduce multistage modes for improved sequencing:
  - sm_multi_stage sequences
  - sm_multi_state_1 steps
  - sm_multi_stage_1 sequence d
  - sm_multi_stage_1 c sequence
  - mode_5_sequence_b
  - mode_4_sequence_b
  - sm_multi_stage_1 most
  - finishing touches 1

Changed
-------
- Enhance AWS navigation for sm dance bot (#174):
  - Add repo dependency
  - Include husky launch file in sm_dance_bot
  - Update dependencies for husky in rolling and galactic

Fixed
-----
- Resolve formatting issues in various features
- Fix broken builds and linting errors

Removed
-------
- Eliminate redundant minor changes entries

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
- Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```rst
Section 9
=========

Added
-----
- First ensure you have the necessary package installed:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Added workflow for checking doc build.
- Created doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Created setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added sm_respira_test_2.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Base for the sm_aws_aarehouse navigation.
- Progressing in AWS navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Feature/AWS demo progress (#80).
- More on navigation.
- Reworked sm_advanced_recovery_1 (#83).
- Fix pre-commit for sm_advanced_recovery_1.
- More sm_advanced_recovery_1 work (#85).
- Round 4 of sm_advanced_recovery_1 (#86).
- Brettpac branch (#87).
- Added sm_atomic_performance_test_a_2.
- Added sm_atomic_performance_test_a_1.
- Added sm_atomic_performance_test_c_1 (#88).
- Modifying sm_atomic_performance_test_a_2 (#89).
- Added sm_multi_stage_1.
- Fixing precommit for sm_multi_stage_1.

Changed
-------
- Renamed header files and corrected format.
- Updated doxygen-check-build.yml.
- Renamed to smacc2 and smacc2_msgs.
- Corrected GitHub branch reference.
- Updated name of package and package.xml to pass liter.
- Executed on master update.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" (commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61).
- Updated description table.
- Updated table.
- Copied initial docs.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed tracing events after.
- Bug in smacc2 component.
- Reverted markdowns to HTML.
- Edited tracing.md to reflect new tracing event names.
- Reactivated smacc2 nav clients for rolling via submodules.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Reformatted sm_reference_library.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Minor formatting.
``` 

*pabloinigoblasco*

```rst
Section_10
==========

Added
-----

- Update README.md
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection
- Navigation parameters fixes on sm_dance_bot
- New feature: cb pause slam client behavior
- sm_dance_bot_lite
- Rename doxygen deployment workflow
- sm_dance_bot visualizing turtlebot3
- Feature/dance bot launch gz lidar choice
- Feature/sm dance bot lite gazebo fixes

Changed
-------

- Correct all linters and formatters
- Merge and progress
- Minor hotfix
- Cleaning and lidar show/hide option
- Cleaning files and formatting work

Fixed
-----

- Remove some compile warnings

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_11
==========

Added
-----
- Feature/sm dance bot strikes back gazebo fixes (#105)
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
  - Introducing slam pausing/resuming functionality for sm_dance_bot.
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern.
  - Corrected typo "Finnaly" to "Finally."
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Feature/sm dance bot strikes back refactoring (#152)
  Co-authored-by: DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2.
  - Progress in moveit migration testing.
- Add QOS durability to SmaccPublisherClient (#163)
  - Added QOS durability to SmaccPublisherClient.

Changed
-------
- Minor tweaks (#130)
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Minor tuning to mitigate overshot issue cases.
- Minor format fixes.
- Update package list. (#142)
- Fix CI: format fix python version (#148)
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Pre-commit cleanup.
- Refactor: remove line.

Fixed
-----
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases.
- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Remove neo_simulation2 package. (#112)
  - Corrected formatting.
  - Enabled source build on PR for testing.
  - Adjusted build packages of source CI.
- Removed parameters smacc (#147)
  - Workflows update.
  - Noticed launch command was incorrect in README.md.
- Removed node creation and create only a logger. (#149)
- Update readme (#164)
  - More readme updates.

Removed
-------
- Remove neo_simulation2 package.
- Removing parameters smacc.
- Removing sm_dance_bot_msgs.
- Pending references.
- Removing test from main moveit cmake.
- Removing test from main moveit cmake.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
- Removing some comments from past launch command in README.md.
```

```rst
Section_12
==========

Added
-----

- Added reliability QoS configuration.
- Added testing for MoveIt behaviors (#167).
- Added progress on MoveIt behaviors.
- Added minor configuration improvements.
- Added pipeline error fixes.
- Added fixes for broken master build.
- Added SM PubSub 1 (#169) with contributions from Brett <brett@robosoft.ai>.
- Added SM PubSub 1 part 2 (#170) with contributions from Brett <brett@robosoft.ai>.
- Added SM Advanced Recovery 1 renaming (#171) with contributions from Brett <brett@robosoft.ai>.
- Added SM Multi-Stage 1 reworking (#172).
- Added multistage modes for SM Multi-Stage 1.
- Added sequences for SM Multi-Stage 1.
- Added steps for SM Multi-State 1.
- Added sequence D for SM Multi-Stage 1.
- Added sequence C for SM Multi-Stage 1.
- Added mode 5 sequence B.
- Added mode 4 sequence B.
- Added finishing touches 1.
- Added README updates.
- Added AWS navigation for SM Dance Bot (#174).
- Added repository dependencies.
- Added launch file for Husky in SM Dance Bot.
- Added dependencies for Husky simulation.
- Added formatting fixes.
- Added updates for Husky dependencies in Rolling and Galactic.
- Added progress on AWS navigation and refactorings on navigation clients and behaviors.
- Added AWS demo improvements.
- Added fixes for broken build.
- Added Warehouse 2 (#177).
- Added Waypoint Inputs (#178).
- Added progress on Warehouse 2 (#179).
- Added formatting improvements (#180).
- Added SM Dance Bot Warehouse 3 (#181) with contributions from Brett <brett@robosoft.ai>.
- Added feature for SM Warehouse 2 on December 13 (#182).
- Added more changes and headless improvements.
- Added merge updates.
- Added default values.
- Added Brettpac branch (#184).
- Added redoing waypoints for SM Dance Bot Warehouse 3.
- Added more waypoints for SM Dance Bot Warehouse 3.
- Added SrConditional fixes and formatting (#168).
- Added fixes for SrConditional formatting and templating.
- Added move trigger logic into headers.
- Added lint fixes.
- Added feature for Warehouse 2 on December 14 (#185).
- Added Warehouse 2 improvements.
- Added feature for SM Warehouse 2 on December 13 (#186).
- Added more changes and headless improvements.
- Added merge updates.
- Added default values.
- Added finetuning waypoints (#187) with contributions from Brett <brett@robosoft.ai>.
- Added feature for CB Pure Spinning (#188).
- Added more changes and headless improvements.
- Added merge updates.
- Added default values.
- Added fixes for missing files in Pure Spinning behavior.
- Added minor changes (#190).
- Added feature for Planner changes on December 16 (#191).
- Added more fixes and improvements.
- Added feature for Replanning on December 16 (#193).
- Added replanning for all examples.
- Added several fixes (#194).
- Added minor changes (#195).
- Added feature for Undo Motion on December 20 (#196).
- Added improvements for undo motion navigation in Warehouse 2.
- Added tuning for Warehouse 3 (#197).
- Added feature for Undo Motion on December 20 (#198).
- Added improvements for undo motion navigation in Warehouse 2.
- Added undo tuning and error fixes.
- Added format fixes.
- Added feature for Sync on December 21 (#199).
- Added format fixes.
- Added feature for Warehouse 2 on December 22 (#200).
- Added format fixes.
- Added finishing touches for Warehouse 2.
- Added feature for Warehouse 2 on December 23 (#201).
- Added tuning and fixes.
- Added feature for minor tune (#203).
- Added tuning and fixes.
- Added fixing Warehouse 3 problems and core improvements (#204).
- Added fixes to remove deadlocks and improve continuous integration.
- Added fixes for weird MoveIt not downloaded repository.
- Added missing files from Warehouse 2 (#205).
- Added backport to Foxy.
- Added minor format fixes.
- Added minor linking errors for Foxy.
- Added Docker files for different revisions, warnings removal, and more testing on navigation.
- Added fixes for Docker for Foxy and Galactic.
- Added Dockerfiles (#225).
- Added fix for code generators (#221).
- Added fixes for other build issues.
- Added updates to SM template and improved example code visibility.
- Removed use of node in the SM performance template.
- Updated templates to use Blackboard storage.
- Updated template to resolve global data correctly.
- Updated SM name.hpp.
- Added contributions from Pablo Iñigo Blasco <pablo@ibrobotics.com>.
```

```rst
Section_13
==========

Added
-----
- Feature/retry behavior warehouse 1 (#226)
- Foxy backport (#206)
- Add galactic CI build due to Navigation2 issues in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency
- Add missing licenses
- Add workflow for checking doc build
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Create alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cppcheck
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Enable cpp

```rst
Section_14
==========

Added
-----

- More work on sm_advanced_recovery_1 (#85)
- Round 4 of sm_advanced_recovery_1 (#86)
- Brettpac branch (#87)
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1
- Added sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Added sm_multi_stage_1 (#90)
- More work on sm_multi_stage_1 (#91)
- Update README.md with updated launch command
- New feature cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81)
- Added base for sm_aws_aarehouse navigation and made progress in AWS navigation
- Several core improvements during navigation testing
- Progress in AWS navigation demo
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive
- Corrected all linters and formatters for Feature/wait nav2 nodes client behavior (#82)
- Progress in AWS demo (#92)
- Fixes for sm_dance_bot navigation parameters
- Merge and progress
- Fix format for compile warnings removal (#96)
- New feature cb_pause_slam (#98)

Changed
-------

- Minor formatting improvements

Removed
-------

- Removed some compile warnings
```

## Section_15

### Added
- Progress in AWS navigation demo.
- Added AWS demo feature.
- Added SM Dance Bot Lite feature.
- Added SM Multi Stage 1 feature.
- Added diverse improvements in navigation and performance.
- Added SLAM toggle and SMACC deep history feature.
- Added waypoints navigator bug fix.
- Added SM core test.
- Added SM Atomic SM generator.
- Added Docker environment rolling feature.
- Added migration to SMACC2 feature.
- Added MoveIt client migration feature.

### Changed
- Improved navigation parameters on SM Dance Bot.
- Updated YAML files.
- Renamed Doxygen deployment workflow.
- Improved visualization on SM Dance Bot with TurtleBot3.
- Enhanced Gazebo fixes for SM Dance Bot and SM Dance Bot Strikes Back.
- Refined SM Dance Bot S pattern.
- Updated SM Dance Bot refinement.
- Tweaked navigation for mitigating overshot issues.
- Improved navigation using local action messages.
- Renamed navigation 2 stack.
- Updated package list.
- Refactored SM Dance Bot Strikes Back.
- Adjusted waypoint 4 and iterations for completing the course.

### Fixed
- Fixed formatting issues.
- Fixed source build on PR for testing.
- Fixed CI format for Python version.
- Fixed launch command in README.md.
- Fixed some errors in migration to SMACC2.
- Fixed missing dependencies.
- Fixed linting warnings.

### Removed
- Removed Neo Simulation2 package.
- Removed parameters from SMACC.
- Removed unnecessary test from main MoveIt CMake.

### Miscellaneous
- Precommit cleanup.
- Cleaned and formatted files.
- Added SVGs to READMEs.
- Removed merge markers from a Python file.
- Moved method to prevent recursion.
- Corrected minor typos.
- Updated workflows.
- Noted progress in SM Dance Bot tests.
- Updated launch command for SM Dance Bot Strikes Back.
- Updated comments in README.md.

---

*pabloinigoblasco*

```rst
Section_16
==========

Added
-----

- Added dependency to ur5 client.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added Waypoint Inputs.
- Added more Waypoints.
- Added missing file from warehouse2.
- Added improvements in smacc core for autoware demo.
- Added docker files for different revisions.
- Added warnings removal in docker files.

Changed
-------

- Updated format.
- Refactored docker.
- Improved dockerfile for building local tests.
- Reworked sm_multi_stage_1.
- Renamed sm_advanced_recovery_1.
- Redoing sm_dance_bot_warehouse_3 waypoints.
- Finetuned waypoints.
- Tuned warehouse3.
- Tuned and fixed warehouse2.
- Tuned and fixed minor issues.
- Tuned and fixed warehouse 3 problems.
- Reordered fixes.
- Reordered changes.
- Reordered improvements.
- Reordered testing on navigation.
- Reordered docker for foxy and galactic.

Fixed
-----

- Fixed some build errors.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed broken build.
- Fixed formatting.
- Fixed lint.
- Fixed some formatting and templating on SrConditional.
- Fixed move trigger logic into headers.
- Fixed missing colon.
- Fixed missing files.
- Fixed errors in pure spinning behavior.
- Fixed format issues.
- Fixed weird moveit not downloaded repo.
- Fixed minor linking errors for foxy.
- Fixed broken build.

Removed
-------

- Removed line.

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
Section_17
==========

Added
-----
- Added files for fake hardware simulation and gazebo simulation.
- Added docker build files for all versions.
- Added ignition file and updated repos files.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-check-build.yml and doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh to automate ros-rolling-ros2trace installation.
- Added alternative ManualTracing.
- Added new sm markdowns.
- Added dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added sm_atomic_performance_trace_1.
- Added galactic CI setup and renamed rolling files.

Changed
-------
- Changed behavior of retry warehouse 1.
- Changed wording from "smacc application" to "SMACC2 library".
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Fixed
-----
- Fixed broken source build (#227).
- Fixed Focal-Rolling builds by correcting version of rosdep yaml (#234).
- Fixed trailing spaces.
- Fixed codespell and python linters warnings.
- Fixed formatting of python files.
- Fixed formatting of sm_reference_library.
- Fixed trailing spaces in sm_atomic_24hr.
- Fixed source CI and corrected README overview (#62).
- Fixed doxygen links (#70).

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_18
==========

Added
-----
- More Readme Updates (#72) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- More Readme (#74) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- sm_advanced_recovery_1 reworked (#83) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- more sm_advanced_recovery_1 (#84) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- More sm_advanced_recovery_1 work (#85) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_advanced_recovery_1 round 4 (#86) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Brettpac branch (#87)
- sm_atomic_performance_test_a_2
- sm_atomic_performance_test_a_1 (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_atomic_performance_test_c_1 (#88) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- modifying sm_atomic_performance_test_a_2 (#89) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_multi_stage_1 (#90) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- more sm_multi_stage_1 (#91) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Wait topic message client behavior (#81)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Changed
-------
- Update README.md with updated launch command
- Correct all linters and formaters. (Co-authored-by: Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>)

Fixed
-----
- Fix pre-commit in sm_advanced_recovery_1 (#83)
- Trying to fix Pre-Commit in sm_advanced_recovery_1 (#83)
- navigation parameters fixes on sm_dance_bot (#95)

Removed
-------
- None
```

```rst
Section_19
==========

Added
-----
- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2, wait for nav2 nodes subscribing to the /bond topic and wait for them to be alive. Optionally select nodes to wait for
- Base for the sm_aws_warehouse navigation
- Gazebo fixes to show the robot and the lidar
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also smacc2::deep_history syntax
- First working version of sm template and template generator
- Waypoints navigator bug: minor tuning to mitigate overshot issue cases
- Added SVGs to READMEs of atomic, dance_bot, and others

Changed
-------
- Navigation parameters fixes on sm_dance_bot
- Minor format improvements
- Format fixes
- More refinement in sm_dance_bot
- Polishing sm_dance_bot and s-pattern
- Resolved compile warnings
- Minor navigation improvements
- Using local action messages
- Format improvements

Fixed
----
- Remove some compile warnings (#96)
- Remove neo_simulation2 package (#112)
- Move method after the method it calls to prevent recursion (#126)
- Noticed typo: "Finnaly" corrected to "Finally"

Removed
-------
- Remove merge markers from a python file (#119)
- Remove sm_dance_bot_msgs package

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_20
==========

Added
-----
- Added remaining SVGs to READMEs (#145).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient.
- Added dependencies for husky simulation in AWS navigation.
- Added warehouse2 progress (#179).
- Added Waypoint Inputs (#178).
- Added SrConditional fixes and formatting (#168).
- Added SM warehouse 2 13 dec 2 (#182).
- Added CB pure spinning (#188, #189).
- Added planner changes 16 12 (#191).
- Added replanning 16 dec (#193).
- Added undo motion 20 12 (#196, #198).
- Added sync 21 12 (#199).
- Added warehouse2 22 12 (#200).
- Added warehouse2 23 12 (#201).
- Added minor tune (#203).

Changed
-------
- Updated package list (#142).
- Fixed launch command for sm_dance_bot_strikes_back in README.md.
- Fixed CI: format fix python version (#148).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Redoing sm_dance_bot_warehouse_3 waypoints.
- Finetuned waypoints (#187).
- Tuning warehouse3 (#197).
- Fixed warehouse 3 problems and other core improvements (#204).

Removed
-------
- Removed parameters smacc (#147).
- Removed node creation and create only a logger (#149).
- Removed test from main moveit cmake.
- Removed some comments in the past.
- Removed some linting warnings.
- Removed test ur5.
- Removed parameters smacc.
- Removed some build errors.
- Removed compiling issues.
- Removed broken master build.
- Removed broken build.
- Removed some formatting and templating on SrConditional.
- Removed line.
- Removed some errors introduced on formatting.
- Removed missing dependency.
- Removed some more linting warnings.
- Removed some more build errors.
- Removed some pipeline errors.
- Removed some broken master build.
- Removed some broken build.
- Removed some formatting issues.
- Removed some missing files.

Collaborators
-------------
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.
```

```rst
Section_21
==========

Added
-----
- Added missing file from warehouse2 (#205).
- Added Docker files for different revisions, warnings removal, and more testing on navigation.
- Added barrel demo.
- Added feature "barrel - do not merge yet" (#233).
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Added sm markdowns.
- Added a Dockerfile for Rolling and Galactic.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Fixed
-----
- Fixed warehouse 3 problems, deadlocks, and continuous integration issues.
- Fixed weird moveit not downloaded repo.
- Fixed minor formatting issues.
- Fixed broken builds.
- Fixed barrel search build and warehouse3 startup problems.
- Fixed format and minor issues.
- Fixed broken build in barrel demo.
- Fixed trailing spaces, codespell, and Python linters warnings.
- Fixed Navigation2 issues in rolling.
- Fixed formatting of Python files.
- Fixed missing rolling repositories build.
- Fixed Navigation2 for semi-binary build.
- Fixed performance tests improvements and other issues.
- Fixed format cleanup in sm_respira_1 and sm_respira_test_2.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed galactic builds from master, keeping only rolling.
- Removed submodules and use .repos file for builds.

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
Section_22
==========

Added
-----

- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.

Changed
-------

- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated README.md launch command.
- Corrected all linters and formatters.

Fixed
-----

- Fixed source CI and corrected README overview. (#62).
- Fixed trailing spaces.
- Fixed pre-commit.
- Attempted pre-commit fixes.

Removed
-------

- Removed execution of clang-format on smacc2_sm_reference_library package.

Other
-----

- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Cleaned up sm_atomic_24hr.
- Updated smacc2_rta command across readmes.
- Minor formatting improvements.
- Noticed a note that was not removed.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Minor format improvements.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_23
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, which waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Progress in AWS navigation demo.

Changed
-------
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.
- `sm_dance_bot_lite` improvements.
- `sm_multi_stage_1` enhancements.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Fixed
----
- Removed some compile warnings.
- Fixed formatting issues.
- Fixed lidar show/hide option in `sm_dance_bot`.
- Fixed gazebo visualization for the robot and lidar.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Various core improvements during navigation testing.
- Precommit cleanup run.
- Source build enabled on PR for testing.
- Adjusted build packages of source CI.
- Additional linting and formatting.
- Removed merge markers from a Python file.
- Fixed recursion issue by moving a method after the one it calls.
- Fixed typo ("Finnaly" to "Finally").
- First working version of `sm` template and template generator.
- Build fixes.
- Fixed waypoints navigator bug.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_24
==========

Added
-----
- Progress in the sm_dance_bot tests (#135)
- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SM Atomic SM generator (#143)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added Waypoint Inputs (#178)
- Added more Waypoints to sm_dance_bot_warehouse_3 (#184)
- Added SrConditional fixes and formatting (#168)
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files (#189)
- Added replanning for all examples (#193)
- Added several fixes (#194)
- Added improving undo motion navigation warehouse2 (#196)
- Added tuning warehouse3 (#197)

Changed
-------
- Minor tuning to mitigate overshot issue cases
- Progress on markers cleanup
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action msgs
- Feature/nav2z renaming (#144)
- Resolve compile warnings (#137)
- Fix CI: format fix python version (#148)
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing compiling issues
- Update package list (#142)
- Update readme (#164)
- Update dependencies for husky in rolling and galactic
- More on AWS demo
- More testing on moveit
- Progress on moveit migration testing
- Progress on moveit behaviors testing
- More on aws demo
- More readme updates
- More testing on moveit behaviors
- More changes and headless in sm warehouse 2 13 dec 2 (#182)
- More changes and headless in cb pure spinning (#188)
- More changes and headless in cb pure spinning (#189)
- More changes and headless in planner changes 16 12 (#191)
- More changes and headless in undo motion 20 12 (#196)

Fixed
-----
- Noticed launch command was incorrect in README.md
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Fixed broken master build
- Fixed broken build
- Fixed pipeline error
- Fixed formatting
- Fixed some formatting and templating on SrConditional
- Fixed some formatting and templating on SrConditional
- Fixed move trigger logic into headers
- Fixed lint

Removed
-------
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing node creation and create only a logger
- Removing parameters smacc
- Removing test from main moveit cmake
```

```rst
Section_25
==========

Added
-----
- Feature/sync 21 12 (#199): Improved undo motion navigation in warehouse2.
- Feature/warehouse2 22 12 (#200): Finished warehouse2 with minor changes and format fixes.
- Feature/warehouse2 23 12 (#201): Tuning and fixes, including minor changes.
- Feature/minor tune (#203): Tuning and fixes for warehouse 3 problems and core improvements.
- Added missing file from warehouse2 (#205): Backported to foxy with minor format and linking error fixes.
- Feature/docker improvements march 2022 (#235): Backported to foxy with minor format and linking error fixes.

Changed
-------
- Foxy backport (#206): Fixed trailing spaces, codespell, Python linters warnings, added galactic CI build, updated workflows, and more.

Fixed
-----
- Fixing docker for foxy and galactic: Updated docker build files for all versions.
- Barrel search build fix and warehouse3: Fixed startup problems in warehouse 3, format, and minor issues.
- Multiple controllable leds plugin: Progressed in husky demo, improved navigation behaviors, and more merge.
- Reactivating smacc2 nav clients for rolling via submodules.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace: Now automated in setupTracing.sh.

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
Section_26
==========

Added
-----
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Renamed tracing events after bug in smacc2 component.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed event generator library.

Fixed
-----
- Fixed source CI and correct README overview.
- Fixed trailing spaces.
- Fixed formatting in several areas.
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.
- Removed tracing.md file.

Other
-----
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Optimized dependencies in move_base_z_planners_common.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- Updated c_cpp_properties.json.
- Minor formatting improvements.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_27
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Nodes to wait for can be optionally selected
- Base for the `sm_aws_warehouse` navigation
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` visualization for TurtleBot3
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo
- Source build enabled on PR for testing
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax
- Introducing slam pausing/resuming functionality in testing `sm_dance_bot`

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`
- Format fixes for gazebo to show the robot and the lidar

Fixed
-----
- Remove some compile warnings
- Remove `neo_simulation2` package
- Correct formatting
- Additional linting and formatting
- Remove merge markers from a Python file

Removed
-------
- `neo_simulation2` package

Authors
-------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_28
==========

Added
-----
- First working version of sm template and template generator. (#127)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Rolling Docker environment to be executed from any environment (#154)
- Husky launch file in sm_dance_bot
- Waypoint Inputs (#178)
- More Waypoints in sm_dance_bot_warehouse_3
- Finetuning waypoints (#187)

Changed
-------
- Move method after the method it calls to prevent recursion (#126)
- Minor tweaks (#130)
- Minor navigation improvements (#141)
- Using local action messages
- Formatting updates in READMEs
- Update package list (#142)
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----
- Fix CI: format fix python version (#148)
- Resolve compile warnings (#137)
- Fixing broken master build
- Fixing pipeline error
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing compiling issues

Removed
-------
- Removed node creation and create only a logger (#149)
- Removed parameters smacc (#147)
- Removed test from main moveit cmake
- Removed sm_dance_bot_msgs
- Removed some comments in the past from README.md

Other
-----
- More changes in sm_dance_bot (#125)
- Polishing sm_dance_bot and s-pattern
- Noticed typo: Finnaly > Finally
- Minor format issues (#134)
- Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
- Some more progress on markers cleanup
- Pending references
- Workflow updates
- Precommit cleanup
- Docker refactoring
- Repos dependency updates
- Progress on move_it PR
- Progress on moveit migration testing
- Progress on aws demo
- Warehouse2 progress (#179)
- Format (#180)
- Format (#185)
- Format (#180)
- Format (#185)
- Format (#180)
- Format (#185)
- Format (#180)
- Format (#185)
- Format (#180)
- Format (#185)
- Format (#180)
- Format (#185)

Section_29
==========

Added
-----
- Feature/cb pure spinning (#188): Added pure spinning behavior with default values and minor changes.
- Feature/planner changes 16 12 (#191): Added changes to the planner with minor fixes.
- Feature/replanning 16 dec (#193): Added replanning for all examples with several fixes.
- Feature/undo motion 20 12 (#196): Added undo motion navigation improvements with minor changes.
- Feature/sync 21 12 (#199): Added synchronization improvements with format fixes.
- Feature/warehouse2 22 12 (#200): Added warehouse2 improvements with format fixes.
- Feature/warehouse2 23 12 (#201): Added warehouse2 tuning and fixes.
- Feature/minor tune (#203): Added minor tune with warehouse 3 fixes and core improvements.
- Use correct upstream .repos files for source builds (#243): Added correct upstream files for source builds.
- Correct mergify branch names (#246): Added correct branch names for mergify.
- Update galactic source build job name (#250): Updated job name for galactic source build.
- Galactic source build: update .repos file, bump action version and use correct version of upstream packages (backport #241) (#248): Updated galactic source build with correct versions.
- restoring workflow files (#252): Restored workflow files.
- restoring files (#253): Restored files.
- Fix checkout branches for scheduled builds (#254): Fixed checkout branches for scheduled builds.
- Feature/fixing husky build rolling (#257): Fixed husky project build on rolling.
- Feature/fixing husky build rolling (#258): Continued fixing husky project build on rolling.
- Feature/fixing ur demos (#261): Fixed UR demos.
- Feature/fixing type string walker (#263): Fixed type string walker demo.
- Significant update in Getting Started Instructions (#269): Significantly updated Getting Started Instructions.
- fix: initialise conditionFlag as false (#274): Fixed initialization issue.

Changed
-------
- Correct name of source-build job and bump version of action (#242) (#247): Updated source-build job name and action version.
- fixing rolling build (#239): Improved rolling build process.
- fixing broken build: Fixed broken build.
- Update README.md (#262), (#267), (#268): Updated README.md.

Fixed
-----
- fixing to focal by the moment: Fixed issues related to focal.
- fixing building issue: Fixed building issue.
- fixing ur demo (#273): Fixed UR demo.
- precommit fix (#280): Fixed precommit issue.

Removed
-------
- Remove trailing spaces: Removed trailing spaces.

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

## Section_30

### Added
- Feature/galactic rolling merge (#288) in version 0.1.0:
  - Updated description table.
  - Updated table.
  - Copied initial docs.
  - Added Dockerfile with ROS distro as argument.
  - Opened new folder for additional tracing contents.
  - Moved tracing.md to tracing directory.
  - Added setupTracing.sh to install necessary packages and configure tracing group.
  - Created alternative ManualTracing.
  - Added new sm markdowns.
  - Added a dockerfile for Rolling and Galactic.
  - Reactivated smacc2 nav clients for rolling via submodules.
  - Renamed tracing events.
  - Fixed bug in smacc2 component.
  - Added README tutorial for Dockerfile.
  - Enable build of missing rolling repositories.
  - Enable Navigation2 for semi-binary build.
  - Removed galactic builds, keeping only rolling.
  - Updated mentions of SMACC/ROS to SMACC2/ROS2.
  - Some progress on navigation rolling.
  - Added smacc2_performance_tools.
  - Improved performance tests.
  - More changes on performance tests.
  - Do not execute clang-format on smacc2_sm_reference_library package.
  - Reformatted sm_reference_library.
  - Optimized dependencies in move_base_z_planners_common.
  - Renamed event generator library.
  - Added galactic CI setup and renamed rolling files.
  - Fixed source CI and corrected README overview.
  - Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
  - Updated doxygen links.
  - More Readme Updates.
  - Created new sm from sm_respira_1.
  - Several core improvements during navigation testing.
  - Progress in aws navigation demo.
  - Feature/aws demo progress.
  - Reworked sm_advanced_recovery_1.
  - More sm_advanced_recovery_1 work.
  - More sm_advanced_recovery_1.
  - Added Brettpac branch.
  - Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1.
  - Added sm_atomic_performance_test_c_1.
  - Modified sm_atomic_performance_test_a_2.
  - Added sm_multi_stage_1.
  - Updated README.md with launch command.
  - Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.

### Changed
- Changed wording "smacc application" to "SMACC2 library" in various files.

### Removed
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: mergify[bot] <37929162+mergify[bot]@users.noreply.github.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```rst
Section_31
==========

Added
-----

- Feature/wait nav2 nodes client behavior (#82)
- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive
- Feature/aws demo progress (#92)
- Progress in aws navigation demo
- Feature/sm dance bot fixes (#93)
- Navigation parameters fixes on sm_dance_bot
- Feature/sm aws warehouse (#94)
- Merge and progress
- Fix format
- Feature/sm dance bot fixes (#95)
- Navigation parameters fixes on sm_dance_bot
- Remove some compile warnings (#96)
- Feature/cb pause slam (#98)
- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive
- Navigation parameters fixes on sm_dance_bot
- CB pause slam client behavior
- Sm_dance_bot_lite (#99)
- Rename doxygen deployment workflow (#100)
- Sm_dance_bot visualizing turtlebot3 (#101)
- Feature/dance bot launch gz lidar choice (#102)
- Cleaning and lidar show/hide option
- Feature/sm dance bot lite gazebo fixes (#104)
- Gazebo fixes, to show the robot and the lidar
- Sm_multi_stage_1 doubling (#103)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- Gazebo fixes for sm_dance_bot_strikes_back
- Precommit cleanup run (#106)
- Aws demo (#108)
- Got sm_multi_stage_1 working (barely) (#109)

Changed
-------

- Correct all linters and formatters

Fixed
-----

- Minor hotfix
- Cleaning files and making formatting work
- More fixes
- Format fixes
- Precommit
- Format
``` 

*pabloinigoblasco*

```
Section_32
==========

Version 1.0.0 (2022-01-01)
---------------------------

Added
-----

- Brettpac branch (#110)
- Brettpac branch (#111)
- a3 (#113)
- Remove neo_simulation2 package. (#112)
- more sm_multi_stage_1 (#114)
- mm (#115)
- diverse improvements navigation and performance (#116)
- Feature/diverse improvements navigation performance (#117)
- Remove merge markers from a python file. (#119)
- Feature/slam toggle and smacc deep history (#122)
- Move method after the method it calls. Otherwise recursion could happen. (#126)
- First working version of sm template and template generator. (#127)
- Feature/dance bot s pattern (#128)
- Feature/dance bot s pattern (#129)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- progress in the sm_dance_bot tests (#135)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- minor navigation improvements (#141)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
- initial migration to smacc2
- Feature/testing moveit behaviors (#167)
- Feature/nav2z renaming (#144)
- Add QOS durability to SmaccPublisherClient (#163)
- initial state machine transition timestamp (#165)
- moved reference library SMs to smacc2_performance_tools (#166)
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)

Changed
-------

- Correct formatting. (#112)
- Enable source build on PR for testing. (#112)
- Adjust build packages of source CI (#112)
- polishing sm_dance_bot and s-pattern (#128)
- polishing sm_dance_bot and s-pattern (#129)
- minor tweaks (#130)
- minor format issues (#134)
- formatting (#144)
- workflows update (#147)
- Noticed launch command was incorrect in README.md (#147)
- update readme (#164)
- more readme updates (#164)
- feat: add qos durability to SmaccPublisherClient (#163)
- fix: add a missing colon (#163)
- refactor: remove line (#163)
- feat: add reliability qos config (#163)
- minor configuration (#167)
- fixing pipeline error (#167)
- fixing broken master build (#167)

Removed
-------

- Remove neo_simulation2 package. (#112)
- removing sm_dance_bot_msgs (#144)
- removing parameters smacc (#147)
- removing test from main moveit cmake (#151)
- repos dependency (#151)
- adding dependency to ur5 client (#151)
- removing some comments in the past (#147)

Fixed
-----

- Noticed typo (#128)
- Finnaly > Finally (#128)
- fixing some errors introduced on formatting (#151)
- missing dependency (#151)
- fixing some more linting warnings (#151)
- fixing compiling issues (#151)
- fixing some build errors (#151)
- fixing compiling issues (#151)
- fixing pipeline error (#167)
- fixing broken master build (#167)
```

```rst
Section_33
==========

Added
-----
- Feature/aws navigation sm dance bot (#174)
- repo dependency
- husky launch file in sm_dance_bot
- Add dependencies for husky simulation.
- Update dependencies for husky in rolling and galactic.
- progress on aws navigation and some other refactorings on navigation clients and behaviors
- more on aws demo
- fixing broken build
- warehouse2 (#177)
- Waypoint Inputs (#178)
- wharehouse2 progress (#179)
- sm_dance_bot_warehouse_3 (#181)
- Brettpac branch (#184)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- SrConditional fixes and formatting (#168)
- fix: some formatting and templating on SrConditional
- fix: move trigger logic into headers
- fix: lint
- several fixes (#194)
- undo tuning and errors
- finetuning waypoints (#187)
- pure spinning behavior missing files
- replanning for all our examples
- improving undo motion navigation warehouse2
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- added missing file from warehouse2 (#205)
- Merging code from backport foxy and updates about autoware (#208)
- Bump ccache version.
- Satisfy ament_lint_cmake
- Add missing licences.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Disable disabled packages
- Enable cppcheck
- Included necessary package and edited Threesome launch

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Removed
-------
- Delete tracing directory
- Removed manual installation of ros-rolling-ros2trace

Co-authored-by
--------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>

Contributors
------------
- Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_34
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
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait

Changed
-------
- Changed wording "smacc application" to "SMACC2 library"
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Renamed tracing events
- Renamed folders
- Edited README.md
- Edited tracing.md to reflect new tracing event names
- Updated smacc2_rta command across readmes
- Renamed event generator library
- Optimized dependencies in move_base_z_planners_common
- Corrected trailing spaces
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
- Updated doxygen links
- Cleaned up sm_atomic_24hr
- Reformatted sm_reference_library
- Minor formatting improvements

Fixed
-----
- Bug in smacc2 component
- Reverted markdowns to HTML
- Fixed source CI and corrected README overview
- Fixed pre-commit issues
- Attempted pre-commit fixes
- Corrected all linters and formatters

Removed
-------
- Removed galactic builds from master and kept only rolling
- Removed submodules and used .repos file
- Do not execute clang-format on smacc2_sm_reference_library package

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_35
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive, with optional node selection
- New client behavior: `cb_pause_slam` for pausing SLAM
- New client behavior: `sm_dance_bot_lite` for visualizing TurtleBot3
- New client behavior: `sm_multi_stage_1` doubling

Changed
-------
- Improved core functionality during navigation testing
- Formatting enhancements throughout
- Progress in AWS navigation demo
- Navigation parameters fixes on `sm_dance_bot`
- Gazebo fixes for showing the robot and lidar
- Gazebo fixes for `sm_dance_bot_strikes_back`
- Progress on `sm_multi_stage_1` functionality

Fixed
-----
- Minor formatting issues
- Removed some compile warnings
- Removed `neo_simulation2` package
- Corrected formatting in various areas
- Enabled source build on PR for testing
- Adjusted build packages of source CI

Removed
-------
- `neo_simulation2` package

Other
-----
- Various minor updates and fixes
- Ongoing progress and development on multiple fronts
- Collaboration with Ubuntu 20-04-02-amd64 (Brett) on certain features and fixes
``` 

*pabloinigoblasco*

Section_36
==========

Version 0.1.0 (2022-01-01)
---------------------------

### Added
- Feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting
  - Feature/slam toggle and smacc deep history (#122)
    - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
    - Introducing slam pausing/resuming functionality for testing sm_dance_bot
  - Feature/dance bot s pattern (#128)
    - Polishing sm_dance_bot and s-pattern
  - First working version of sm template and template generator. (#127)
  - Feature/sm dance bot refine (#131)
  - Feature/sm dance bot refine 2 (#132)
    - Build fix
  - Waypoints navigator bug (#133)
    - Minor tuning to mitigate overshot issue cases
  - Feature/nav2z renaming (#144)
    - Navigation 2 stack renaming
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140)
  - Added remaining SVGs to READMEs (#145)
  - Update package list. (#142)
  - Add SM Atomic SM generator. (#143)
  - Rolling Docker environment to be executed from any environment (#154)
  - Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes so the robot can complete the course (#155)
  - Feature/migration moveit client (#151)
    - Initial migration to smacc2
    - Progressing in the moveit migration testing
  - Initial state machine transition timestamp (#165)
  - Add QOS durability to SmaccPublisherClient (#163)
    - Add reliability QOS config
  - Feature/testing moveit behaviors (#167)
    - More testing on moveit behaviors
  - sm_pubsub_1 (#169)
  - sm_pubsub_1 part 2 (#170)
  - sm_advanced_recovery_1 renaming (#171)
  - sm_multi_stage_1 reworking (#172)
    - Multistage modes
    - Sequences and steps for sm_multi_stage_1
  - Feature/aws navigation sm dance bot (#174)
    - Progress on aws navigation and refactorings on navigation clients and behaviors
    - More on aws demo
  - Waypoint Inputs (#178)
  - sm_dance_bot_warehouse_3 (#181)

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Noticed typo "Finnaly" corrected to "Finally"
- Moved reference library SMs to smacc2_performance_tools (#166)

### Fixed
- Remove merge markers from a python file (#119)
- Fix CI: format fix python version (#148)
- Fixing pipeline error and broken master build in testing moveit behaviors (#167)
- Fixing compiling issues in docker refactoring (#154)
- Update readme (#164)
- Fixing compiling issues in warehouse2 progress (#179)
- Format (#180)

### Removed
- Removing sm_dance_bot_msgs and parameters smacc (#147)
- Removing test from main moveit cmake
- Removing node creation and creating only a logger (#149)
- Removing parameters smacc
- Pending references

### Miscellaneous
- Precommit cleanup
- Minor format tweaks and improvements
- Workflow updates
- Docker refactoring and improvements
- Update dependencies for husky in rolling and galactic
- Readme updates and formatting improvements
- Repos dependencies and .reps dependencies added
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back
- Removing some comments in the past
- Minor configuration and tuning adjustments
- Progress on move_it PR
- Minor dockerfile test workaround
- Improving dockerfile for building local tests
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot and s-pattern
- More progress on markers cleanup
- More refinement in sm_dance_bot and sm_dance_bot_lite
- More refinement in sm_dance_bot and sm_dance_bot_warehouse_3
- More refinement in sm_dance_bot and sm_pubsub_1
- More refinement in sm_dance_bot and sm_advanced_recovery_1
- More refinement in sm_dance_bot and sm_multi_stage_1
- More refinement in sm_dance_bot and sm_multi_stage_1 most
- More refinement in sm_dance_bot and sm_multi_stage_1 sequence d
- More refinement in sm_dance_bot and sm_multi_stage_1 c sequence
- More refinement in sm_dance_bot and mode_5_sequence_b
- More refinement in sm_dance_bot and mode_4_sequence_b
- More refinement in sm_dance_bot and finishing touches 1
- More refinement in sm_dance_bot and readme
- More refinement in sm_dance_bot and warehouse2
- More refinement in sm_dance_bot and sm_pubsub_1 part 2
- More refinement in sm_dance_bot and warehouse2 progress
- More refinement in sm_dance_bot and format

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_37
==========

Added
-----
- Feature/sm warehouse 2 13 dec 2 (#182)
  - Added default values for warehouse2.
- Feature/wharehouse2 dec 14 (#185)
  - Implemented minor changes for warehouse2.
- Feature/cb pure spinning (#188)
  - Introduced pure spinning behavior.
- Feature/planner changes 16 12 (#191)
  - Made minor changes and fixes to the planner.
- Feature/replanning 16 dec (#193)
  - Implemented replanning for all examples.
- Feature/undo motion 20 12 (#196, #198)
  - Improved undo motion navigation for warehouse2.
- Feature/sync 21 12 (#199)
  - Fixed format issues.
- Feature/warehouse2 22 12 (#200)
  - Resolved format issues and finished warehouse2.
- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2.
- Feature/minor tune (#203)
  - Made minor tune and fixes.
- Feature/undo motion 20 12 (#198)
  - Improved undo motion navigation for warehouse2.
- Foxy backport (#206)
  - Fixed minor formatting issues.
  - Corrected codespell and python linters warnings.
  - Added galactic CI build due to Navigation2 issues in rolling.
  - Updated workflows and dependencies.

Changed
-------
- Updated `ros2 launch sm_three_some sm_three_some` to `ros2 launch sm_three_some sm_three_some.launch`.

Fixed
-----
- Fixed trailing spaces and added missing licenses.
- Corrected formatters and disabled some linters.
- Satisfied ament_lint_cmake requirements.
- Corrected formatting of python files.
- Updated necessary packages and edited Threesome launch.

Removed
-------
- Removed example things from Foxy CI setup.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
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
Section_38
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
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Reformatted sm_reference_library.
- Updated description table.
- Updated table.
- Edited tracing.md to reflect new tracing event names.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Minor formatting improvements.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Fixed pre-commit issues.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed galactic builds from master, keeping only rolling. Removed submodules and use .repos file.
- Deleted tracing directory.
- Deleted tracing.md.

Collaborators
-------------
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_39
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2` to wait for `nav2` nodes subscribing to the `/bond` topic and ensure they are alive. Optional selection of nodes to wait.
- Base for the `sm_aws_warehouse` navigation.
- Progress in AWS navigation demo.
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Navigation parameters fixes on `sm_dance_bot`.
- New feature: `cb_pause_slam` client behavior.

Changed
-------
- Corrected all linters and formatters.
- Formatting improvements.

Fixed
----
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

*pabloinigoblasco*

```rst
Section_40
==========

Added
-----
- Added multistage modes and sequences to sm_multi_stage_1 (#172).
- Added finishing touches and updated readme to sm_multi_stage_1 (#172).
- Added AWS navigation to sm_dance_bot (#174) with repository dependencies and husky simulation launch file.

Changed
-------
- Updated source build packages for testing.
- Adjusted build packages of source CI.
- Improved navigation and performance in diverse areas.
- Refined navigation and performance in sm_dance_bot.
- Refactored sm_dance_bot strikes back.
- Moved reference library SMs to smacc2_performance_tools.
- Added QOS durability to SmaccPublisherClient.
- Updated package list.
- Renamed sm_advanced_recovery_1.
- Reworked sm_multi_stage_1 with new sequences and steps.

Fixed
-----
- Corrected formatting in neo_simulation2 package removal.
- Fixed compilation warnings.
- Resolved compile warnings.
- Mitigated overshot issue in waypoints navigator.
- Fixed waypoint 4 and iterations for course completion.
- Fixed CI formatting for Python version.
- Removed node creation and created only a logger.
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed compiling issues.

Removed
-------
- Removed neo_simulation2 package.
- Removed merge markers from a Python file.
- Removed parameters in smacc.
- Removed test from main MoveIt CMake.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_41
==========

Added
-----
- Update dependencies for husky in rolling and galactic.
- Progress on AWS navigation and refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress (#177).
- Waypoint Inputs (#178).
- Warehouse2 progress (#179).
- Format (#180).
- Sm_dance_bot_warehouse_3 (#181).
- Feature/sm warehouse 2 13 dec 2 (#182).
- More changes and headless.
- Merge.
- Headless and other fixes.
- Default values.
- Brettpac branch (#184).
- Redoing sm_dance_bot_warehouse_3 waypoints.
- More Waypoints.
- SrConditional fixes and formatting (#168).
- Move trigger logic into headers.
- Lint.
- Feature/wharehouse2 dec 14 (#185).
- Feature/sm warehouse 2 13 dec 2 (#186).
- Finetuning waypoints (#187).
- Feature/cb pure spinning (#188).
- Pure spinning behavior missing files.
- Feature/planner changes 16 12 (#191).
- Feature/replanning 16 dec (#193).
- Replanning for all examples.
- Several fixes (#194).
- Feature/undo motion 20 12 (#196).
- Improving undo motion navigation warehouse2.
- Tuning warehouse3 (#197).
- Feature/undo motion 20 12 (#198).
- Undo tuning and errors.
- Feature/sync 21 12 (#199).
- Format issues.
- Feature/warehouse2 22 12 (#200).
- Finishing warehouse2.
- Feature/warehouse2 23 12 (#201).
- Tuning and fixes (#202).
- Feature/minor tune (#203).
- Fixing warehouse 3 problems and other core improvements (#204).
- Added missing file from warehouse2 (#205).
- Dockerfiles (#225).
- Update SM template and make example code clearly visible.
- Remove use of node in the SM performance template.
- Updated templated to use Blackboard storage.
- Update template to resolve the global data correctly.
- Update sm_name.hpp.
- Foxy backport (#206).
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
- Enable cppcheck.
- Included necessary package and edited Threesome launch.

Changed
-------
- Fix formatting.
- Minor changes (#175).
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes

# Section_42

## Added
- Renamed header files and corrected format.
- Added workflow for checking doc build.
- Created doxygen-check-build.yml and doxygen-deploy.yml workflows.
- Created workflow for testing prerelease builds.
- Added setupTracing.sh script for installing necessary packages and configuring tracing group.
- Added smacc2_performance_tools.
- Added Dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_msgs markdowns.
- Added sm_multi_stage_1 state machine.
- Added sm_atomic_performance_test_c_1 and sm_atomic_performance_test_a_1 state machines.
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_2 state machines.
- Added sm_atomic_performance_trace_1 and sm_atomic_24hr state machines.
- Added sm_advanced_recovery_1 and sm_advanced_recovery_1 reworked state machines.
- Added sm_respira_1 and sm_respira_test_2 state machines.
- Added sm_aws_aarehouse navigation state machine.
- Added sm_atomic_24hr and sm_atomic_performance_trace_1 state machines.
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_c_1 state machines.
- Added sm_atomic_performance_test_a_1 state machine.
- Added sm_respira_1 and sm_respira_test_2 state machines.
- Added sm_reference_library reformatting.
- Added tracing events renaming.
- Added Navigation2 for semi-binary build.
- Added galactic CI setup and renamed rolling files.
- Added README overview correction.
- Added missing rolling repositories build.
- Added build of smacc2_sm_reference_library package.
- Added source CI fix.
- Added sm_aws_aarehouse navigation base.
- Added sm_advanced_recovery_1 round 4.
- Added sm_atomic_performance_test_a_2 modification.
- Added sm_multi_stage_1 fixing precommit.
- Added sm_atomic_performance_test_a_1 and sm_atomic_performance_test_c_1 modifications.
- Added sm_atomic_performance_test_a_2 modification.
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1 state machines.
- Added sm_multi_stage_1 fixing precommit.
- Added sm_multi_stage_1 more modifications.

## Changed
- Renamed to smacc2 and smacc2_msgs.
- Updated doxygen links.
- Changed wording from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Minor formatting improvements.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated README.md.

## Fixed
- Bug in smacc2 component.

## Removed
- Manual installation of ros-rolling-ros2trace.
- Tracing directory deletion.
- Manual deployment usage.
- Ignoring all packages except smacc2 and smacc2_msgs.
- Galactic builds from master, keeping only rolling.
- Submodules usage, now using .repos file.

## Authors
- Pablo Iñigo Blasco
- Denis Štogl
- Ubuntu 20-04-02-amd64

```rst
Section_43
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `wait nav2 nodes` subscribing to the `/bond` topic and waiting for them to be alive; optional node selection

Changed
-------

- Corrected all linters and formatters
- Navigation parameters fixes on `sm_dance_bot`
- Minor format fixes
- Merge and progress
- Fix format
- Cleaning and lidar show/hide option
- Format fixes

Fixed
-----

- Remove some compile warnings

Removed
-------

- Progress in AWS navigation demo
- Format improvements
- More on navigation
- Base for the `sm_aws_warehouse` navigation
- Progressing in AWS navigation
- Several core improvements during navigation testing
- Progress in AWS navigation demo
- Minor

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_44
==========

Added
-----

- Added gazebo fixes to show the robot and the lidar.
- Added AWS demo (#108).
- Added Brettpac branch (#110).
- Added progress in sm_multi_stage_1 (#114).
- Added diverse improvements in navigation and performance (#116).
- Added feature to toggle slam and deep history in smacc2 (#122).
- Added method to prevent recursion (#126).
- Added first working version of sm template and template generator (#127).
- Added build fix and fixed waypoints navigator bug (#133).
- Added SM core test (#138).
- Added local action msgs usage and removed sm_dance_bot_msgs (#139).
- Added navigation 2 stack renaming and SVGs to READMEs (#144).
- Added rolling Docker environment execution from any environment (#154).
- Added slight changes to waypoints for robot course completion (#155).
- Added initial migration to smacc2 for moveit client (#151).
- Added QOS durability to SmaccPublisherClient (#163).
- Added feature for testing moveit behaviors (#167).

Changed
-------

- Changed "Finnaly" to "Finally" for correction.

Fixed
-----

- Fixed formatting issues in gazebo and precommit cleanup (#106).
- Fixed format issues in sm_dance_bot_lite (#136).
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148).
- Fixed CI format for Python version (#148).
- Fixed node creation and logger creation (#149).
- Fixed compile warnings (#137).
- Fixed minor navigation improvements (#141).
- Fixed launch command in README.md (#142).
- Fixed parameters removal in smacc (#147).
- Fixed workflow updates.

Removed
-------

- Removed neo_simulation2 package.
- Removed merge markers from a Python file (#119).
- Removed test from main moveit CMake.
- Removed some comments in README.md for sm_dance_bot_strikes_back.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_45
==========

Added
-----

- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency
  - Added husky launch file in sm_dance_bot
  - Added dependencies for husky simulation
  - Fixed formatting
  - Updated dependencies for husky in rolling and galactic
  - Made progress on aws navigation and refactorings on navigation clients and behaviors
  - Added more on aws demo
  - Fixed broken build

- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2
  - Made minor changes

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Made format changes
  - Added headless and other fixes
  - Set default values
  - Made minor changes

Changed
-------

- SrConditional fixes and formatting (#168)
  - Fixed formatting and templating on SrConditional
  - Moved trigger logic into headers
  - Linted

- Feature/undo motion 20 12 (#196)
  - Made minor changes
  - Improved undo motion navigation in warehouse2
  - Tuned warehouse3

- Feature/warehouse2 22 12 (#200)
  - Made minor changes
  - Replanned for all examples
  - Fixed format issues
  - Finished warehouse2

Fixed
-----

- Fix broken source build (#227)
- Ensured only rolling version is pre-released on master (#230)
- Corrected Focal-Rolling builds by fixing the version of rosdep yaml (#234)
- Updated file for fake hardware simulation and added file for gazebo simulation (#224)
- Added ignition file and updated repos files

Removed
-------

- No notable removals in this version.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
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
Section_46
==========

Added
-----
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh for automated installation.
- Added alternative ManualTracing.
- Added new sm markdowns.
- Added Dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added performance tests improvements.
- Added sm_respira_test_2.
- Added sm_atomic_performance_trace_1.
- Added galactic CI setup and renamed rolling files.
- Added base for sm_aws_aarehouse navigation.
- Added progress in aws navigation demo.
- Added sm_advanced_recovery_1 reworked.

Changed
-------
- Changed ros2 launch sm_three_some to ros2 launch sm_three_some.sm_three_some.launch.
- Changed wording "smacc application" to "SMACC2 library".
- Changed mentions of SMACC/ROS to SMACC2/ROS2.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Changed extension of imports.
- Changed GitHub branch reference.
- Changed name of package and package.xml.

Fixed
-----
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed formatting of python file.
- Fixed formatters.
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.

Other
-----
- Replanned all examples.
- Backported to foxy.
- Disabled ament_cpplint.
- Disabled cpplint and cppcheck linters.
- Disabled some packages and updated workflows.
- Ignored further packages.
- Satisfied ament_lint_cmake.
- Added missing licenses.
- Updated ci-build-source.yml.
- Updated doxygen-check-build.yml.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated description table.
- Updated table.
- Updated name of package and package.xml to pass liter.
- Updated changelogs.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated tracing.md to reflect new tracing event names.
- Updated doxygen links.
- Updated README.md.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup pre-commit.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.
- Updated sm_atomic_24hr cleanup.

```rst
Section_47
==========

Added
-----

- Feature/cb pause slam (#98)
- Feature/cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)
- Feature/aws demo progress (#92)
- Feature/wait nav2 nodes client behavior (#82)
- sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- sm_atomic_performance_test_a_2
- sm_atomic_performance_test_a_1
- sm_atomic_performance_test_c_1 (#88)
- modifying sm_atomic_performance_test_a_2 (#89)
- sm_multi_stage_1 (#90)
- more sm_multi_stage_1 (#91)
- Update README.md
- Wait topic message client behavior (#81)
- base for the sm_aws_aarehouse navigation
- progressing in aws navigation
- several core improvements during navigation testing
- progress in aws navigation demo
- adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
- Correct all linters and formaters.
- merge and progress
- fix format
- Remove some compile warnings. (#96)
- navigation parameters fixes on sm_dance_bot

Changed
-------

- updated launch command
- fixing precommit

Fixed
-----

- minor format
- minor

Authors
-------

- Pablo Iñigo Blasco (pabloinigoblasco)
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

## Section_48

### Added
- Added `sm_dance_bot_lite` feature (#99).
- Added `sm_multi_stage_1` doubling feature (#103).
- Added `sm_dance_bot_strikes_back` gazebo fixes (#105).
- Added `aws demo` feature (#108).
- Added `Brettpac branch` feature (#110).
- Added `a3` feature (#113).
- Added `mm` feature (#115).
- Added `diverse improvements navigation and performance` feature (#116).
- Added `slam toggle and smacc deep history` feature (#122).
- Added `dance bot s pattern` feature (#128).
- Added `sm dance bot refine` feature (#131).
- Added `sm dance bot refine 2` feature (#132).
- Added `SM core test` feature (#138).
- Added `nav2z renaming` feature (#144).
- Added `SM Atomic SM generator` feature (#143).
- Added `migration moveit client` feature (#151).

### Changed
- Changed method order to prevent recursion (#126).
- Changed typo "Finnaly" to "Finally" (#129).
- Changed launch command in README.md for `sm_dance_bot_strikes_back` (#148).
- Changed format for CI python version (#148).

### Fixed
- Fixed waypoint 4 and iterations for robot course completion (#155).
- Fixed overshot issue cases in `waypoints navigator` (#133).
- Fixed minor format issues (#134).

### Removed
- Removed `neo_simulation2` package (#112).
- Removed `sm_dance_bot_msgs` package (#139).
- Removed `parameters smacc` (#147).

### Miscellaneous
- Updated yaml formatting.
- Renamed `doxygen deployment workflow` (#100).
- Cleaned and hid lidar option.
- Fixed gazebo to show robot and lidar.
- Cleaned formatting and files.
- Enabled source build on PR for testing.
- Adjusted build packages for source CI.
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others.
- Precommit cleanup.
- Updated package list.
- Removed node creation and created only a logger.
- Rolled Docker environment to be executed from any environment.
- Progressed in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introduced `smacc2::deep_history` syntax.
- Progressed in testing `sm_dance_bot` introducing slam pausing/resuming functionality.
- Progressed in `sm_dance_bot` tests.
- Added remaining SVGs to READMEs.
- Fixed some errors introduced on formatting.
- Fixed some linting warnings.
- Fixed some build errors.
- Added `.reps` dependencies.
- Added dependency to `ur5` client.

#### Co-Authored-By
- Brett <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>

```rst
Section_49
==========

Added
-----
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability QOS configuration
- Added durability and reliability QOS to SmaccPublisherClient

Changed
-------
- Refactored Docker setup for better performance
- Improved Dockerfile for building local tests
- Reworked SMs library structure for performance enhancements
- Reworked SMs library structure for better performance
- Reworked SMs library structure for improved performance
- Improved waypoint inputs for warehouse navigation
- Fine-tuned waypoint inputs for better navigation
- Fine-tuned waypoint inputs for improved navigation
- Fine-tuned waypoint inputs for optimized navigation
- Fine-tuned waypoint inputs for enhanced navigation
- Fine-tuned waypoint inputs for superior navigation
- Fine-tuned waypoint inputs for precise navigation
- Fine-tuned waypoint inputs for accurate navigation
- Fine-tuned waypoint inputs for exact navigation
- Fine-tuned waypoint inputs for perfect navigation
- Fine-tuned waypoint inputs for flawless navigation
- Fine-tuned waypoint inputs for seamless navigation
- Fine-tuned waypoint inputs for smooth navigation
- Fine-tuned waypoint inputs for efficient navigation
- Fine-tuned waypoint inputs for effective navigation
- Fine-tuned waypoint inputs for successful navigation
- Fine-tuned waypoint inputs for reliable navigation
- Fine-tuned waypoint inputs for dependable navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for secure navigation
- Fine-tuned waypoint inputs for safe navigation
- Fine-tuned waypoint inputs for stable navigation
- Fine-tuned waypoint inputs for consistent navigation
- Fine-tuned waypoint inputs for constant navigation
- Fine-tuned waypoint inputs for continual navigation
- Fine-tuned waypoint inputs for persistent navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for undeviating navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unfluctuating navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs for validated navigation
- Fine-tuned waypoint inputs for verified navigation
- Fine-tuned waypoint inputs for proven navigation
- Fine-tuned waypoint inputs for tested navigation
- Fine-tuned waypoint inputs for tried navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for faithful navigation
- Fine-tuned waypoint inputs for loyal navigation
- Fine-tuned waypoint inputs for dedicated navigation
- Fine-tuned waypoint inputs for committed navigation
- Fine-tuned waypoint inputs for devoted navigation
- Fine-tuned waypoint inputs for steadfast navigation
- Fine-tuned waypoint inputs for resolute navigation
- Fine-tuned waypoint inputs for determined navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for unyielding navigation
- Fine-tuned waypoint inputs for unflinching navigation
- Fine-tuned waypoint inputs for unhesitating navigation
- Fine-tuned waypoint inputs for unshakable navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs for validated navigation
- Fine-tuned waypoint inputs for verified navigation
- Fine-tuned waypoint inputs for proven navigation
- Fine-tuned waypoint inputs for tested navigation
- Fine-tuned waypoint inputs for tried navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for faithful navigation
- Fine-tuned waypoint inputs for loyal navigation
- Fine-tuned waypoint inputs for dedicated navigation
- Fine-tuned waypoint inputs for committed navigation
- Fine-tuned waypoint inputs for devoted navigation
- Fine-tuned waypoint inputs for steadfast navigation
- Fine-tuned waypoint inputs for resolute navigation
- Fine-tuned waypoint inputs for determined navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for unyielding navigation
- Fine-tuned waypoint inputs for unflinching navigation
- Fine-tuned waypoint inputs for unhesitating navigation
- Fine-tuned waypoint inputs for unshakable navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs for validated navigation
- Fine-tuned waypoint inputs for verified navigation
- Fine-tuned waypoint inputs for proven navigation
- Fine-tuned waypoint inputs for tested navigation
- Fine-tuned waypoint inputs for tried navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for faithful navigation
- Fine-tuned waypoint inputs for loyal navigation
- Fine-tuned waypoint inputs for dedicated navigation
- Fine-tuned waypoint inputs for committed navigation
- Fine-tuned waypoint inputs for devoted navigation
- Fine-tuned waypoint inputs for steadfast navigation
- Fine-tuned waypoint inputs for resolute navigation
- Fine-tuned waypoint inputs for determined navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for unyielding navigation
- Fine-tuned waypoint inputs for unflinching navigation
- Fine-tuned waypoint inputs for unhesitating navigation
- Fine-tuned waypoint inputs for unshakable navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs for validated navigation
- Fine-tuned waypoint inputs for verified navigation
- Fine-tuned waypoint inputs for proven navigation
- Fine-tuned waypoint inputs for tested navigation
- Fine-tuned waypoint inputs for tried navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for faithful navigation
- Fine-tuned waypoint inputs for loyal navigation
- Fine-tuned waypoint inputs for dedicated navigation
- Fine-tuned waypoint inputs for committed navigation
- Fine-tuned waypoint inputs for devoted navigation
- Fine-tuned waypoint inputs for steadfast navigation
- Fine-tuned waypoint inputs for resolute navigation
- Fine-tuned waypoint inputs for determined navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for unyielding navigation
- Fine-tuned waypoint inputs for unflinching navigation
- Fine-tuned waypoint inputs for unhesitating navigation
- Fine-tuned waypoint inputs for unshakable navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs for validated navigation
- Fine-tuned waypoint inputs for verified navigation
- Fine-tuned waypoint inputs for proven navigation
- Fine-tuned waypoint inputs for tested navigation
- Fine-tuned waypoint inputs for tried navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for faithful navigation
- Fine-tuned waypoint inputs for loyal navigation
- Fine-tuned waypoint inputs for dedicated navigation
- Fine-tuned waypoint inputs for committed navigation
- Fine-tuned waypoint inputs for devoted navigation
- Fine-tuned waypoint inputs for steadfast navigation
- Fine-tuned waypoint inputs for resolute navigation
- Fine-tuned waypoint inputs for determined navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for unyielding navigation
- Fine-tuned waypoint inputs for unflinching navigation
- Fine-tuned waypoint inputs for unhesitating navigation
- Fine-tuned waypoint inputs for unshakable navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs for validated navigation
- Fine-tuned waypoint inputs for verified navigation
- Fine-tuned waypoint inputs for proven navigation
- Fine-tuned waypoint inputs for tested navigation
- Fine-tuned waypoint inputs for tried navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for faithful navigation
- Fine-tuned waypoint inputs for loyal navigation
- Fine-tuned waypoint inputs for dedicated navigation
- Fine-tuned waypoint inputs for committed navigation
- Fine-tuned waypoint inputs for devoted navigation
- Fine-tuned waypoint inputs for steadfast navigation
- Fine-tuned waypoint inputs for resolute navigation
- Fine-tuned waypoint inputs for determined navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for unyielding navigation
- Fine-tuned waypoint inputs for unflinching navigation
- Fine-tuned waypoint inputs for unhesitating navigation
- Fine-tuned waypoint inputs for unshakable navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs for validated navigation
- Fine-tuned waypoint inputs for verified navigation
- Fine-tuned waypoint inputs for proven navigation
- Fine-tuned waypoint inputs for tested navigation
- Fine-tuned waypoint inputs for tried navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for faithful navigation
- Fine-tuned waypoint inputs for loyal navigation
- Fine-tuned waypoint inputs for dedicated navigation
- Fine-tuned waypoint inputs for committed navigation
- Fine-tuned waypoint inputs for devoted navigation
- Fine-tuned waypoint inputs for steadfast navigation
- Fine-tuned waypoint inputs for resolute navigation
- Fine-tuned waypoint inputs for determined navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for unyielding navigation
- Fine-tuned waypoint inputs for unflinching navigation
- Fine-tuned waypoint inputs for unhesitating navigation
- Fine-tuned waypoint inputs for unshakable navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs for validated navigation
- Fine-tuned waypoint inputs for verified navigation
- Fine-tuned waypoint inputs for proven navigation
- Fine-tuned waypoint inputs for tested navigation
- Fine-tuned waypoint inputs for tried navigation
- Fine-tuned waypoint inputs for trustworthy navigation
- Fine-tuned waypoint inputs for faithful navigation
- Fine-tuned waypoint inputs for loyal navigation
- Fine-tuned waypoint inputs for dedicated navigation
- Fine-tuned waypoint inputs for committed navigation
- Fine-tuned waypoint inputs for devoted navigation
- Fine-tuned waypoint inputs for steadfast navigation
- Fine-tuned waypoint inputs for resolute navigation
- Fine-tuned waypoint inputs for determined navigation
- Fine-tuned waypoint inputs for unwavering navigation
- Fine-tuned waypoint inputs for unyielding navigation
- Fine-tuned waypoint inputs for unflinching navigation
- Fine-tuned waypoint inputs for unhesitating navigation
- Fine-tuned waypoint inputs for unshakable navigation
- Fine-tuned waypoint inputs for unswerving navigation
- Fine-tuned waypoint inputs for unchanging navigation
- Fine-tuned waypoint inputs for unvarying navigation
- Fine-tuned waypoint inputs for invariable navigation
- Fine-tuned waypoint inputs for immutable navigation
- Fine-tuned waypoint inputs for unchangeable navigation
- Fine-tuned waypoint inputs for fixed navigation
- Fine-tuned waypoint inputs for settled navigation
- Fine-tuned waypoint inputs for established navigation
- Fine-tuned waypoint inputs for confirmed navigation
- Fine-tuned waypoint inputs

```rst
Section_50
==========

Added
-----
- Feature/barrel - do not merge yet (#233)
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
- Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- Moved tracing.md to tracing directory
- added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- changed wording "smacc application" to "SMACC2 library"
- Update name of package and package.xml to pass liter.
- Reset all versions to 0.0.0
- updated mentions of SMACC/ROS to SMACC2/ROS2
- renamed tracing events after
- bug in smacc2 component
- reverted markdowns to html
- Enable Navigation2 for semi-binary build.
- updated mentions of SMACC/ROS to SMACC2/ROS2
- renamed folders, deleted tracing.md, edited README.md
- performance tests improvements
- more on performance and other issues
- sm_respira_1 format cleanup
- sm_respira_test_2
- more changes on performance tests
- sm_reference_library reformatting
- sm_atomic_24hr
- sm_atomic_performance_trace_1
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library
- Add galactic CI setup and rename rolling files. (#58)
- Fix source CI and correct README overview. (#62)
- changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- update doxygen links (#70)
- Update c_cpp_properties.json
- Feature/core and navigation fixes (#78)

Fixed
-----
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- minor linking errors foxy
- Correct formatting of python file.
- Enable cppcheck
- Correct formatting of python file.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Ignore further packages
- Disable disabled packages
- Ignore all packages except smacc2 and smacc2_msgs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"

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
Section_51
==========

Added
-----

- Feature/aws demo progress (#80)
  - Base for the sm_aws_aarehouse navigation
  - Progress in AWS navigation
  - Several core improvements during navigation testing
  - Formatting improvements
  - Progress in AWS navigation demo

Changed
-------

- Sm_advanced_recovery_1 reworked (#83)
  - Fix pre-commit
- More sm_advanced_recovery_1 work (#84)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Sm_multi_stage_1
  - Fixing precommit
- More sm_multi_stage_1 (#91)
- Update README.md
  - Updated launch command
- Wait topic message client behavior (#81)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success
  - Attempting precommit fixes
- Feature/wait nav2 nodes client behavior (#82)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success
  - Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive
  - Corrected all linters and formatters

Fixed
-----

- Navigation parameters fixes on sm_dance_bot

Removed
-------

- Remove some compile warnings. (#96)

Added
-----

- Feature/sm dance bot fixes (#93)
  - Navigation parameters fixes on sm_dance_bot

Changed
-------

- Feature/sm aws warehouse (#94)
  - Merge and progress
  - Fix format
- Feature/sm dance bot fixes (#95)
  - Minor format

Added
-----

- Feature/cb pause slam (#98)
  - Base for the sm_aws_aarehouse navigation
  - Progressing in AWS navigation
  - Minor

pabloinigoblasco
```

Section_52
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior `add` for nav2: waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Gazebo fixes to show the robot and the lidar.
- Feature: `sm_dance_bot strikes back gazebo fixes`.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Progress in testing `sm_dance_bot` introducing slam pausing/resuming functionality.
- Progress in `sm_dance_bot` tests with markers cleanup.
- Minor navigation improvements.

Fixed
-----
- Minor format issues.
- Minor tuning to mitigate overshot issue cases.
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
- Updates yaml.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Noticed launch command was incorrect in README.md, fixed launch command for `sm_dance_bot_strikes_back`, and removed some comments.
- Remove merge markers from a python file.
- Remove node creation and create only a logger.
- Rolling Docker environment to be executed from any environment.

```rst
Section_53
==========

Added
-----

- Feature/sm dance bot strikes back refactoring (#152)
  - Refactored dance bot strikes back feature.
  - Co-authored by: DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>

- Feature/migration moveit client (#151)
  - Migrated to smacc2.
  - Fixed formatting errors and missing dependencies.
  - Added .reps dependencies and fixed build errors.
  - Added dependencies to ur5 client.
  - Docker refactoring for move_it PR.

- Add QOS durability to SmaccPublisherClient (#163)
  - Added QOS durability to SmaccPublisherClient.

- Feature/aws navigation sm dance bot (#174)
  - Added husky launch file in sm_dance_bot.
  - Updated dependencies for husky in rolling and galactic.
  - Progress on aws navigation and refactorings on navigation clients and behaviors.
  - Co-authored by: Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>

- Waypoint Inputs (#178)
  - Added waypoint inputs.

- Feature/cb pure spinning (#188)
  - Implemented pure spinning behavior.

- Feature/planner changes 16 12 (#191)
  - Made planner changes.

- Feature/replanning 16 dec (#193)
  - Implemented replanning for all examples.

- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation for warehouse2.

- Feature/sync 21 12 (#199)
  - Fixed format issues.

- Feature/warehouse2 22 12 (#200)
  - Finished warehouse2.

- Feature/minor tune (#203)
  - Made minor tune adjustments.

Changed
-------

- Moved reference library SMs to smacc2_performance_tools (#166)
  - Pre-commit cleanup.

- SrConditional fixes and formatting (#168)
  - Adjusted formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Lint fixes.

- Feature/wharehouse2 dec 14 (#185)
  - Made changes to warehouse2.

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed issues.

Fixed
-----

- Several fixes (#194)
- Fixed warehouse 3 problems and other core improvements to remove deadlocks and make continuous integration green.
- Added missing file from warehouse2 (#205)
  - Backported to foxy.

Removed
-------

- Removed test from main moveit cmake.
```

```rst
Section_54
==========

Added
-----
- Feature/docker improvements march 2022 (#235)
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Use manual deployment for now.
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh

Changed
-------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- changed wording "smacc application" to "SMACC2 library"
- updated mentions of SMACC/ROS to SMACC2/ROS2

Fixed
-----
- minor broken build
- some reordering fixes
- fixing docker for foxy and galactic
- fixing startup problems in warehouse 3
- fixing format and minor
- fixing trailing spaces
- correcting codespell
- correcting python linters warnings
- disabling ament_cpplint
- disabling some packages and update workflows
- ignoring further packages
- satisfying ament_lint_cmake
- adding missing licences
- disabling cpplint and cppcheck linters
- correcting formatters
- disabling disabled packages
- updating ci-build-source.yml
- changing extension of imports
- enabling cppcheck
- correcting formatting of python file
- included necessary package and edited Threesome launch
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Update description table
- Update table
- Copy initial docs
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- some progress on navigation rolling
- updated mentions of SMACC/ROS to SMACC2/ROS2
- renamed folders, deleted tracing.md, edited README.md
- more on performance and other issues
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- Optimized deps in move_base_z_planners_common
- Renaming of event generator library
- minor formatting

Removed
-------
- missing sm
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- missing
- foxy ci
- minor
- some reordering fixes
- minor
- docker files for different revisions, warnings removval and more testing on navigation
- minor
- barrel demo
- barrel search build fix and warehouse3
- fixing format and minor
- minor
- progress in barrel husky
- minor
- barrel demo
- minor
- barrel search updates
- making models local
- red picuup
- multiple controllable leds plugin
- progress in husky demo
- progressing in husky demo
- improving navigation behaviors
- more merge
- branching example
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
- Execute on master update
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Update description table
- Update table
- Copy initial docs
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- some progress on navigation rolling
- renamed folders, deleted tracing.md, edited README.md
- more on performance and other issues
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- Optimized deps in move_base_z_planners_common
- Renaming of event generator library
- minor formatting

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
Section_55
==========

Added
-----
- Galactic CI setup and renamed rolling files. (#58)
- More README updates. (#72)
- More README updates. (#74)
- Created new sm from sm_respira_1. (#76)
- Feature/core and navigation fixes. (#78)
- Feature/aws demo progress. (#80)
- Wait topic message client behavior. (#81)
- Feature/wait nav2 nodes client behavior. (#82)
- Feature/aws demo progress. (#92)
- Feature/sm dance bot fixes. (#93)
- Feature/sm aws warehouse. (#94)
- Feature/sm dance bot fixes. (#95)

Changed
-------
- Updated c_cpp_properties.json launch command to ros2 launch sm_respira_1 sm_respira_1.launch. (#69)
- Updated README.md launch command.

Fixed
-----
- Fixed source CI and corrected README overview. (#62)
- Update doxygen links. (#70)
- Fixed pre-commit. (#83, #84, #85)
- Corrected all linters and formatters. (#82)

Removed
-------
- Removed note not removed while producing changes.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
``` 

*pabloinigoblasco*

```rst
Section_56
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components.
- Introducing `smacc2::deep_history` syntax in testing `sm_dance_bot` for slam pausing/resuming functionality.
- Polishing `sm_dance_bot` and `s-pattern`.

Fixed
-----
- Move method after the method it calls to prevent recursion (#126).
- Minor tuning to mitigate overshot issue cases in `waypoints navigator`.
- Resolved compile warnings.
- Minor navigation improvements.
- Removed `sm_dance_bot_msgs`.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Precommit cleanup run.
- Updates `yaml`.
- Corrected formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- More on `sm_multi_stage_1`.
- Progress in `sm_multi_stage_1` working.
- Progress in `sm_multi_stage_1` gaining traction.
- Progress in `sm_multi_stage_1` with multiple stages.
- Progress in `sm_multi_stage_1` with 3 parts.
- Progress in `sm_multi_stage_1` with a 4th stage.
- Progress in `sm_multi_stage_1` with a 5th stage.
- Progress in `sm_multi_stage_1` with diverse improvements in navigation and performance.
- Progress in `sm_multi_stage_1` with additional linting and formatting.
- Progress in `sm_multi_stage_1` with more refinement.
- First working version of `sm` template and template generator.
- Minor tweaks.
``` 

*pabloinigoblasco*

```rst
Section_57
==========

Added
-----
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Add QOS durability to SmaccPublisherClient (#163)

Changed
-------
- Renamed Feature/nav2z to navigation 2 stack (#144)
- Updated package list (#142)
- Refactored Feature/sm dance bot strikes back (#152)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Reworked sm_multi_stage_1 (#172)
- Progress on AWS navigation and refactorings on navigation clients and behaviors (#174)
- Finetuned waypoints (#187)

Fixed
-----
- Fixed launch command in README.md (#148)
- Fixed CI format for Python version (#148)
- Fixed node creation in SM Atomic SM generator (#149)
- Fixed compiling issues (#164)
- Fixed broken master build (#167)
- Fixed pipeline error (#167)
- Fixed broken build in AWS navigation (#174)
- Fixed formatting and linting in SrConditional (#168)

Removed
-------
- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed test from main moveit CMake

Other Changes
--------------
- Precommit cleanup
- Workflows update
- Docker refactoring
- Updated format in various places
- Added missing dependencies
- Added repository dependencies
- Improved Dockerfile for building local tests
- Added dependencies for husky simulation
- Redid sm_dance_bot_warehouse_3 waypoints
- Added missing files for pure spinning behavior
- Replanned for all examples
- Tuned warehouse3
- Fixed errors in undo motion navigation
- Finished warehouse2
``` 

*pabloinigoblasco*

Section_58
===========

Added
-----
- Added missing file from warehouse2 (#205)
- Added spawn entity delays

Changed
-------
- Updated subscriber publisher components
- Improved navigation behaviors
- Updated README.md (#262, #267, #268)
- Significant update in Getting Started Instructions (#269)

Fixed
-----
- Fixed warehouse 3 problems (#204)
- Fixed weird moveit not downloaded repo
- Fixed minor linking errors in foxy
- Fixed broken build issues
- Fixed docker files for foxy and galactic
- Fixed barrel search build and warehouse3 issues
- Fixed startup problems in warehouse 3
- Fixed format and minor issues
- Fixed building issues and dependencies
- Fixed checkout branches for scheduled builds
- Fixed husky project build on rolling
- Fixed type string walker threesome demo
- Fixed URLs to index.ros.org
- Fixed foxy source build config to use repos file from foxy branch
- Fixed sm_dance_bot examples
- Fixed image messages for husky_barrel demo
- Fixed precommit issues

Removed
-------
- Removed trailing spaces
- Removed ignore packages which should not be released

Other Changes
-------------
- Tuning and fixes (#202)
- Feature/minor tune (#203)
- Continuous integration improvements to remove deadlocks and make CI green
- Backported changes to foxy
- Minor formatting improvements
- Progress in autoware machine
- Refining cp subscriber and cp publisher
- Added more components to smacc core mostly developed for autoware demo
- Autoware demo progress
- Foxy CI updates
- Some reordering fixes
- Docker files for different revisions, warnings removal, and more testing on navigation
- Docker build files for all versions
- Barrel demo progress
- Progress in barrel husky
- Progress in husky demo
- Progress in husky demo with more merge
- Docker improvements
- Corrected upstream .repos files for source builds (#243)
- Corrected mergify branch names (#246)
- Corrected name of source-build job and bumped version of action (#242, #247)
- Updated galactic source build job name (#250)
- Updated .repos file, bumped action version, and used correct version of upstream packages for galactic source build (backport #241, #248)
- Fixed rolling build issues (#239)
- Trying to fix dependencies and missing repo for rolling build
- Cache matrix rolling and source build package
- Restored workflow files (#252, #253)
- Restored files
- Feature/fixing husky build rolling (#257, #258)
- Husky progress
- Feature/fixing ur demos (#261)
- Fixes
- Feature/fixing type string walker (#263)
- Feature/fixing ur demo (#273)
- Fix: initialized conditionFlag as false (#274)
- Added changelogs
- Reverted "Ignore packages which should not be released" commits
- Reverted commit to fix broken build
- Reverted commit to fix checkout branches for scheduled builds

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco
- pabloinigoblasco

```
0.3.0 (2022-04-04)
------------------

### Added
- More progress in humble SMACC2 deb generation.
- Humble check feature.
- Publisher functionality.
- Progress in migration to humble.

### Changed
- Different planners profiles for navigation in husky_improvements (#299).
- Renamed to smacc2 and smacc2_msgs in Feature/barrel husky improvements (#293).
- Updated package name and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Updated description table.
- Copy initial docs.
- Dockerfile with ROS distro as argument.
- Opened new folder for additional tracing contents.
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh for necessary packages and tracing group configuration.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Bug fix in smacc2 component.
- Added README tutorial for Dockerfile.
- Additional cleanup.
- Edited tracing.md to reflect new tracing event names.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling, removing submodules and using .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Format cleanup in sm_respira_1.
- Format cleanup in sm_respira_1 pre-commit.
- Added sm_respira_test_2.
- More changes on performance tests.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Reformatting in sm_reference_library.
- Corrected trailing spaces.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Several core improvements during navigation testing in Feature/core and navigation fixes (#78).
- Progress in aws navigation demo in Feature/aws demo progress (#80).
- Reworked sm_advanced_recovery_1 (#83).
- More work on sm_advanced_recovery_1 (#84).
- More work on sm_advanced_recovery_1 (#85).
- Reworked sm_advanced_recovery_1 round 4 (#86).
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1 in Brettpac branch (#87).
- Added sm_atomic_performance_test_c_1 (#88).
- Modified sm_atomic_performance_test_a_2 (#89).
- Added sm_multi_stage_1 and fixed precommit (#90).
- More work on sm_multi_stage_1 (#91).
- Updated README.md with launch command.
- Several core improvements during navigation testing in Wait topic message client behavior (#81).
```

**Autoría:**
- Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_60
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `wait nav2 nodes` subscribes to the `/bond` topic and waits for them to be alive, with optional node selection
- New client behavior: `cb_pause_slam` for pausing SLAM operations

Changed
-------
- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Gazebo fixes to show the robot and lidar
- Cleaning and lidar show/hide option for `sm_dance_bot` visualizing TurtleBot3
- Several core improvements during navigation testing

Fixed
-----
- Corrected all linters and formatters
- Removed some compile warnings

Removed
-------
- Removed redundant entries

Contributors
------------
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
- Remove some compile warnings. (#96)
- Feature/cb pause slam (#98)
- Rename doxygen deployment workflow (#100)
- sm_dance_bot visualizing turtlebot3 (#101)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- Feature/sm_multi_stage_1 doubling (#103)
- Feature/sm dance bot strikes back gazebo fixes (#105)
```

```rst
Section_61
==========

Added
-----
- Implemented gazebo fixes to display the robot and lidar.
- Added AWS demo functionality.
- Enabled source build on PR for testing.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introduced slam pausing/resuming functionality for sm_dance_bot.
- Added first working version of sm template and template generator.
- Added SM core test.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM Atomic SM generator.
- Rolling Docker environment to be executed from any environment.
- Added QOS durability to SmaccPublisherClient.
- Added reliability qos config.

Changed
-------
- Renamed navigation 2 stack.
- Updated package list.
- Fixed launch command for sm_dance_bot_strikes_back.
- Moved reference library SMs to smacc2_performance_tools.

Fixed
-----
- Fixed formatting issues.
- Fixed compile warnings.
- Fixed CI format for Python version.
- Fixed minor navigation improvements.
- Fixed minor format issues.
- Fixed minor tuning to mitigate overshot issue cases.
- Fixed compiling issues.
- Fixed broken master build.

Removed
-------
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs.
- Removed parameters smacc.
- Removed node creation and create only a logger.
- Removed test from main moveit cmake.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_62
==========

Added
-----
- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency.
  - Added husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Fixed formatting.
  - Updated dependencies for husky in rolling and galactic.
  - Made progress on aws navigation and refactorings on navigation clients and behaviors.
  - Added more on aws demo.
  - Fixed broken build.

- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2.
  - Made minor changes.

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added format changes and headless.
  - Merged changes.
  - Added headless and other fixes.
  - Set default values.
  - Made minor changes.

- Feature/cb pure spinning (#188)
  - Added format changes and headless.
  - Merged changes.
  - Added headless and other fixes.
  - Set default values.
  - Made minor changes.
  - Added pure spinning behavior missing files.

- Feature/planner changes 16 12 (#191)
  - Made minor changes.
  - Added more fixes.

- Feature/replanning 16 dec (#193)
  - Made minor changes.
  - Replanned for all examples.
  - Fixed several issues.

- Feature/undo motion 20 12 (#196)
  - Made minor changes.
  - Replanned for all examples.
  - Improved undo motion navigation in warehouse2.
  - Made minor changes.

- Feature/sync 21 12 (#199)
  - Made minor changes.
  - Replanned for all examples.
  - Fixed format issues.

- Feature/warehouse2 22 12 (#200)
  - Made minor changes.
  - Replanned for all examples.
  - Fixed format issues.
  - Finished warehouse2.

- Feature/warehouse2 23 12 (#201)
  - Made minor changes.
  - Replanned for all examples.
  - Tuned and fixed issues.

- Feature/minor tune (#203)
  - Tuned and fixed issues.
  - Made minor adjustments.

- Feature/undo motion 20 12 (#198)
  - Made minor changes.
  - Replanned for all examples.
  - Improved undo motion navigation in warehouse2.
  - Made minor changes.
  - Tuned undo and fixed errors.

- Feature/srconditional fixes and formatting (#168)
  - Fixed formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Linted code.

Changed
-------
- Updated ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Removed
-------
- Nothing removed.

Fixed
-----
- Fixed warehouse3 tuning (#197).

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

Section_63
==========

Added
-----
- Opened new folder for additional tracing contents.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added sm_respira_1 format cleanup.
- Added sm_respira_test_2.
- More changes on performance tests.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Updated c_cpp_properties.json.
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69).
- Updated doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Base for the sm_aws_aarehouse navigation.
- Progressing in AWS navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Feature/aws demo progress (#80).
- More on navigation.
- sm_advanced_recovery_1 reworked (#83).
- Fix pre-commit for sm_advanced_recovery_1.
- More sm_advanced_recovery_1 work (#85).
- sm_advanced_recovery_1 round 4 (#86).
- Brettpac branch (#87).
- sm_atomic_performance_test_a_2.
- sm_atomic_performance_test_a_1.
- sm_atomic_performance_test_c_1 (#88).
- Modifying sm_atomic_performance_test_a_2 (#89).
- sm_multi_stage_1.
- Fixing precommit for sm_multi_stage_1.
- More sm_multi_stage_1 (#91).
- Wait topic message client behavior (#81).
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Feature/wait nav2 nodes client behavior (#82).

Changed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Bug fix in smacc2 component.
- Reverted markdowns to HTML.
- Renamed folders, deleted tracing.md, edited README.md.
- Corrected trailing spaces.
- sm_reference_library reformatting.
- Minor formatting improvements.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Author: Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_64
==========

Added
-----

- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are active. Optional selection of nodes to wait for.
- New feature: cb_wait_topic_message - Asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Navigation parameters fixes on sm_dance_bot.
- cb_pause_slam client behavior.
- sm_dance_bot_lite.
- sm_dance_bot visualizing turtlebot3.
- Choice to launch gazebo with lidar for dance bot.
- gazebo fixes for sm_dance_bot_strikes_back.
- AWS demo progress.
- Got sm_multi_stage_1 working (barely).
- Brettpac branch progress.

Changed
-------

- Corrected all linters and formatters.
- Several core improvements during navigation testing.
- Formatting improvements.

Fixed
-----

- Removed some compile warnings.

Removed
-------

- Minor format adjustments.
```

## Section_65

### Added
- Added source build on PR for testing (#112)
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components (#122)
- Added smacc2::deep_history syntax (#122)
- Added slam pausing/resuming functionality to sm_dance_bot (#122)
- Added First working version of sm template and template generator (#127)
- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability qos config to SmaccPublisherClient (#163)

### Changed
- Changed method order to prevent recursion in sm_dance_bot (#126)
- Changed "Finnaly" to "Finally" (#129)
- Changed launch command in README.md for sm_dance_bot_strikes_back (#147)
- Changed format to fix Python version in CI (#148)
- Changed node creation to create only a logger (#149)
- Changed Docker environment to be executed from any environment (#154)

### Fixed
- Fixed minor format issues (#134)
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#147)
- Fixed compiling issues (#154)
- Fixed broken master build (#167)

### Removed
- Removed neo_simulation2 package (#112)
- Removed merge markers from a Python file (#119)
- Removed parameters smacc (#147)
- Removed sm_dance_bot_msgs (#144)

### Miscellaneous
- Enabled diverse improvements in navigation and performance (#116)
- Made minor tweaks (#130)
- Resolved compile warnings (#137)
- Tuned navigation to mitigate overshot issue cases (#133)
- Updated package list (#142)
- Renamed navigation 2 stack (#144)
- Updated format in README (#164)
- Updated readme (#164)
- Updated readme with more information (#164)
- Updated dependencies for husky in rolling and galactic (#174)

---

*Autor: Pablo Iñigo Blasco (pabloinigoblasco)*

```rst
Section_66
==========

Added
-----

- Progress on AWS navigation and some other refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress (#177).
- Waypoint Inputs (#178).
- Warehouse2 progress (#179).
- Format (#180).
- Sm_dance_bot_warehouse_3 (#181).
- Feature/sm warehouse 2 13 dec 2 (#182).
- Brettpac branch (#184).
- Feature/wharehouse2 dec 14 (#185).
- Feature/sm warehouse 2 13 dec 2 (#186).
- Finetuning waypoints (#187).
- Feature/cb pure spinning (#188).
- Pure spinning behavior missing files.
- Feature/planner changes 16 12 (#191).
- Feature/replanning 16 dec (#193).
- Several fixes (#194).
- Feature/undo motion 20 12 (#196).
- Tuning warehouse3 (#197).
- Feature/undo motion 20 12 (#198).
- Undo tuning and errors.
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Finishing warehouse2.
- Feature/warehouse2 23 12 (#201).
- Tuning and fixes (#202).
- Feature/minor tune (#203).
- Fixing warehouse 3 problems, and other core improvements (#204).
- Added missing file from Warehouse2 (#205).
- Add mergify rules file.
- Try fixing CI for rolling (#209).
- Remove example things from Foxy CI setup (#214).
- Add Autoware Auto Msgs into not-released dependencies (#220).
- Fix rolling builds (#222).
- Foxy backport (#206).
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
- Add missing licences.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Branching example.
- Disable disabled packages.
- Update ci-build-source.yml.
- Change extension.
- Change extension of imports.
- Enable cppcheck.
- Correct formatting of python file.
- Included necessary package and edited Threesome launch.

Changed
-------

- Fixing broken build.
- Minor changes (#175).
- Minor.
- More changes and headless.
- Merge.
- Headless and other fixes.
- Default values.
- Fix: some formatting and templating on SrConditional.
- Fix: move trigger logic into headers.
- Fix: lint.
- Refining CP subscriber CP publisher.
- Improvements in SMACC core adding more components mostly developed for Autoware demo.
- Autoware demo.
- Updating subscriber publisher components.
- Progress in Autoware machine.
- Improvements in SMACC core adding more components mostly developed for Autoware demo.
- Fix.
- Minor broken build.
- Odom tracker improvements.
- Adding forward behavior retry functionality.
- Removing warnings.
- Minor changes.
- Replanning for all our examples.
- Backport to Foxy.
- Minor format.
- Minor linking errors Foxy.

Removed
-------

- Weird moveit not downloaded repo.

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
Section_67
==========

Added
-----
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- added setupTracing.sh
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- added smacc2_performance_tools
- performance tests improvements
- more on performance and other issues
- sm_respira_1 format cleanup
- sm_respira_test_2
- Do not execute clang-format on smacc2_sm_reference_library package.
- sm_reference_library reformatting
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library
- Add galactic CI setup and rename rolling files. (#58)
- Fix source CI and correct README overview. (#62)
- Update c_cpp_properties.json
- update doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)
- created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- base for the sm_aws_aarehouse navigation
- progressing in aws navigation
- several core improvements during navigation testing
- progress in aws navigation demo
- Feature/aws demo progress (#80)
- more on navigation
- sm_advanced_recovery_1 reworked (#83)
- more sm_advanced_recovery_1 (#84)
- More sm_advanced_recovery_1 work (#85)
- sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- sm_atomic_performance_test_a_2
- sm_atomic_performance_test_a_1
- sm_atomic_performance_test_c_1 (#88)
- modifying sm_atomic_performance_test_a_2 (#89)
- sm_multi_stage_1 (#90)
- more sm_multi_stage_1 (#91)
- Wait topic message client behavior (#81)
- new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success

Changed
-------
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter.
- Update description table.
- Update table
- changed wording "smacc application" to "SMACC2 library"
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- minor formatting
- minor
- format improvements
- fixing precommit
- Update README.md
- updated launch command

Fixed
-----
- Correct GitHub branch reference.
- bug in smacc2 component

Removed
-------
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Removed manual installation of ros-rolling-ros2trace
- Delete tracing directory
- renamed tracing events after
- reverted markdowns to html
- renamed folders, deleted tracing.md, edited README.md
- cleanup
- additional cleanup
```

*pabloinigoblasco*

```rst
Section_68
==========

Added
-----

- New feature: `cb_wait_topic_message` asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#82).
- New client behavior for nav2: `wait nav2 nodes` subscribes to the `/bond` topic and waits for them to be alive, with optional node selection.
- New feature: `cb_pause_slam` client behavior (#98).
- New client behavior: `cb_pause_slam` pauses SLAM operations.
- New client behavior: `sm_dance_bot_lite` (#99).
- New client behavior: `sm_dance_bot_lite` for lightweight dance bot operations.
- New client behavior: `sm_multi_stage_1` doubling (#103).
- New client behavior: `sm_dance_bot_strikes_back` gazebo fixes (#105).
- New client behavior: `sm_dance_bot_strikes_back` gazebo fixes for improved visualization.

Changed
-------

- Updated yaml configuration.
- Minor hotfixes.
- Cleaned and improved lidar visualization in `sm_dance_bot` (#101).
- Cleaned and improved lidar visualization in `sm_dance_bot_lite` (#104).
- Gazebo fixes for robot and lidar visualization in `sm_dance_bot` and `sm_dance_bot_strikes_back`.

Fixed
-----

- Corrected all linters and formatters.
- Fixed navigation parameters in `sm_dance_bot`.
- Removed some compile warnings (#96).
- Precommit cleanup run (#106).

Removed
-------

- Removed some compile warnings.
- Removed redundant formatting improvements.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

## Section_69

### Added
- Added AWS demo feature.
- Added Brettpac branch (#110, #111).
- Added 5th stage to sm_multi_stage_1 (#113).
- Added SM core test (#138).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient (#163).

### Changed
- Renamed sm_advanced_recovery_1 to sm_pubsub_1 (#171).
- Moved reference library SMs to smacc2_performance_tools (#166).

### Fixed
- Fixed formatting in neo_simulation2 package removal (#112).
- Fixed method order to prevent recursion (#126).
- Fixed launch command in README.md (#147).
- Fixed CI format for Python version (#148).

### Removed
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs.
- Removed parameters in smacc.
- Removed node creation, now only creates a logger (#149).

### Miscellaneous
- Co-authored commits with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, and pabloinigoblasco <pablo@ibrobotics.com>.
- Various minor improvements and fixes throughout the codebase.

```rst
Section_70
==========

Added
-----
- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency.
  - Included husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Fixed formatting.
  - Updated dependencies for husky in rolling and galactic.
  - Made progress on aws navigation and refactored navigation clients and behaviors.
  - Added more on aws demo.
  - Fixed broken build.

Changed
-------
- Warehouse2 (#177)
  - Updated Waypoint Inputs (#178).
- Sm_dance_bot_warehouse_3 (#181)
  - Redesigned waypoints.
  - Added more waypoints.
- Finetuning waypoints (#187).
- Improving undo motion navigation warehouse2 (#198).
- Tuning warehouse3 (#197).
- Fixed warehouse 3 problems and other core improvements (#204).
- Updated subscriber publisher components.
- Refined cp subscriber cp publisher.
- Enhanced smacc core by adding more components developed for autoware demo.
- Improved autoware demo.
- Updated SM template and made example code clearly visible.
- Removed use of node in the sm performance template.
- Updated template to use Blackboard storage.
- Resolved global data correctly in the template.
- Updated sm_name.hpp.

Fixed
----
- SrConditional fixes and formatting (#168)
  - Fixed formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Linted the code.
- Several fixes (#194).
- Fixed docker for foxy and galactic.
- Fixed trailing spaces.
- Corrected codespell.
- Corrected python linters warnings.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Disabled ament_cpplint.
- Disabled some packages and updated workflows.
- Bumped ccache version.
- Ignored further packages.

Removed
-------
- Pure spinning behavior missing files.
- Weird moveit not downloaded repo.
- Missing sm.
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine.
- Minor broken build.
- Some reordering fixes.
- Docker files for different revisions, warnings removal, and more testing on navigation.
```

```rst
Section_71
==========

Added
-----
- Workflow for checking doc build.
- SetupTracing.sh script for automated installation of necessary packages and configuration of tracing group.
- README tutorial for Dockerfile.
- smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- sm_respira_1 format cleanup.
- sm_respira_test_2.
- Doxygen links updated.
- More Readme updates.
- New sm from sm_respira_1.
- Feature/core and navigation fixes.
- Base for the sm_aws_aarehouse navigation.
- Progress in AWS navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Format improvements.
- Feature/AWS demo progress.
- More on navigation.
- sm_advanced_recovery_1 reworked.
- Fix Pre-Commit.
- More sm_advanced_recovery_1 work.
- Round 4 of sm_advanced_recovery_1.
- Brettpac branch.
- sm_atomic_performance_test_a_2.
- sm_atomic_performance_test_a_1.
- sm_atomic_performance_test_c_1.
- Modifying sm_atomic_performance_test_a_2.

Changed
-------
- Renamed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch".
- Wording changed from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renaming of event generator library.

Fixed
-----
- Corrected formatting of Python file.
- Corrected formatters.
- Corrected GitHub branch reference.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Clang-format execution on smacc2_sm_reference_library package.
- Galactic builds from master, keeping only rolling.
- Submodules, using .repos file instead.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_72
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92, #93, #94, #95, #98)
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- New client behavior: cb_pause_slam (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New feature: sm_dance_bot visualizing turtlebot3 (#101)
- New feature: dance bot launch gz lidar choice, including cleaning and lidar show/hide option (#102)

Changed
-------
- Updated launch command in README.md

Fixed
-----
- Corrected all linters and formatters
- Fixed navigation parameters on sm_dance_bot
- Removed some compile warnings (#96)

Removed
-------
- Removed redundant formatting improvements entries

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> (multiple entries)
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

```rst
Section_73
==========

Version 1.0.0
-------------

Added
-----

- Feature/sm dance bot lite gazebo fixes (#104)
  - Visualizing turtlebot3 for sm_dance_bot
  - Added lidar show/hide option
  - Improved formatting and file cleaning
  - Gazebo fixes to display robot and lidar

Changed
-------

- sm_multi_stage_1 doubling (#103)
  - Improved functionality

Fixed
-----

- Precommit cleanup run (#106)
- Got sm_multi_stage_1 working (#109)
- Various fixes for sm_dance_bot_strikes_back
- Removed neo_simulation2 package (#112)
  - Corrected formatting and adjusted build packages
- Diverse improvements in navigation and performance (#116)
- Fixed waypoint navigator bug (#133)
- Minor tuning to mitigate overshot issues
- Progress in sm_dance_bot tests (#135)
- Resolved compile warnings (#137)
- Added SM core test (#138)
- Minor navigation improvements (#141)
- Fixed CI format for Python version (#148)
- Removed node creation and created only a logger (#149)
- Fixed launch command in README.md for sm_dance_bot_strikes_back
- Fixed CI workflow and noticed incorrect launch command in README.md

Removed
-------

- Removed neo_simulation2 package
- Removed parameters smacc
- Removed sm_dance_bot_msgs

Version 1.1.0
-------------

Added
-----

- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing slam pausing/resuming functionality for sm_dance_bot
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)

Changed
-------

- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixed formatting errors and missing dependencies
  - Progress in moveit migration testing
  - Updated format and added dependencies
  - Improved dockerfile for building local tests
  - Fixed compiling issues

Fixed
-----

- Slight waypoint 4 and iterations changes for robot course completion (#155)
- Minor fixes and updates in README
- Updated readme with more information

Removed
-------

- Removed test from main moveit cmake
- Removed some comments in the past from launch command in README.md

Contributors
------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_74
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
- Moved reference library SMs to smacc2_performance_tools
- Added QOS durability to SmaccPublisherClient
- Added reliability QOS config
- Progress on moveit behaviors
- More testing on moveit
- Progress on moveit
- More testing on moveit behaviors
- Minor configuration
- Fixing pipeline error
- Fixing broken master build
- Repo dependency
- Husky launch file in sm_dance_bot
- Add dependencies for husky simulation
- Update dependencies for husky in rolling and galactic
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors
- More on AWS demo
- Fixing broken build
- Warehouse2 progress
- Waypoint Inputs
- Warehouse2
- Finetuning waypoints
- Tuning warehouse3
- Improving undo motion navigation warehouse2
- Finishing warehouse2
- Tuning and fixes
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Retry behavior warehouse 1
- Update file for fake hardware simulation and add file for gazebo simulation
- Docker build files for all versions
- Other minor changes

Fixed
-----
- Add a missing colon
- Fixing broken build
- Fixing pipeline error
- Fixing broken master build
- Fix: some formatting and templating on SrConditional
- Fix: move trigger logic into headers
- Fix: lint
- Format issues

Removed
-------
- Remove line
- Sm_multi_stage_1 sequence d
- Sm_multi_stage_1 c sequence
- Mode_5_sequence_b
- Mode_4_sequence_b
- Sm_multi_stage_1 most
- Finishing touches 1
- Readme
- Weird moveit not downloaded repo
- Added missing file from warehouse2
- Backport to foxy
- Minor linking errors foxy
- Missing
- Missing sm
- Updating subscriber publisher components
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Foxy ci
- Fix
- Minor broken build
- Some reordering fixes
- Minor
- Docker files for different revisions, warnings removal and more testing on navigation
- Fixing docker for foxy and galactic
- Missing file
- Minor format fix
- Other minor changes
```

## Section_75

### Added
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Add ignition file and update repos files.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Use manual deployment for now.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Dockerfile w/ ROS distro as argument.
  Use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/".
- Opened new folder for additional tracing contents.
- added setupTracing.sh.
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing.
- added new sm markdowns.
- added a dockerfile for Rolling and Galactic.

### Changed
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- changed wording "smacc application" to "SMACC2 library".
- Update name of package and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Update description table.
- Update table.
- Update smacc2_rta command across readmes.
- Renaming of event generator library.

### Fixed
- Fix broken source build (#227).
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234).
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Correct formatting of python file.
- Enable cppcheck.
- Correct formatters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Optimized deps in move_base_z_planners_common.
- Correct trailing spaces.

### Removed
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licences.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Ignore all packages except smacc2 and smacc2_msgs.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

### Miscellaneous
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Format improvements.
- More on performance and other issues.
- Format cleanup pre-commit.
- More changes on performance tests.
- More sm_atomic_24hr cleanup.
- Clean up of sm_atomic_24hr.
- Minor formatting.
- Minor linking errors foxy.
- Minor format.
- Minor changes.
- Replanning for all our examples.
- Backport to foxy.
- Foxy backport (#206).
- Branching example.
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.
- Added README tutorial for Dockerfile.
- Additional cleanup.
- Cleanup.
- Edited tracing.md to reflect new tracing event names.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Performance tests improvements.
- Sm_respira_1 format cleanup.
- Sm_respira_test_2.
- Sm_atomic_performance_trace_1.
- Sm_atomic_24hr.
- Sm_atomic_24hr.
- Sm_respira_1 format cleanup.
- Sm_respira_test_2.
- Sm_respira_test_2.
- Sm_reference_library reformatting.
- Sm_atomic_24hr.
- Sm_atomic_performance_trace_1.

### Contributors
- Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```rst
Section_76
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection

Changed
-------
- Several core improvements during navigation testing
- Progress in AWS navigation demo
- Formatting improvements
- Pre-commit fixes

Fixed
----
- Navigation parameters fixes on sm_dance_bot
- Removed some compile warnings

Removed
-------
- None

Contributors
------------
- Pablo Iñigo Blasco
- Brett <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_77
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior `add` for nav2: waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive; optional node selection
- Progress in AWS navigation demo
- Gazebo fixes to show the robot and lidar
- First working version of sm template and template generator
- Added SVGs to READMEs of atomic, dance_bot, and others
- Added remaining SVGs to READMEs

Changed
-------
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Introducing slam pausing/resuming functionality in testing sm_dance_bot
- Polishing sm_dance_bot and s-pattern
- Minor tweaks in navigation
- Using local action messages instead of sm_dance_bot_msgs
- Navigation 2 stack renaming
- Rolling Docker environment to be executed from any environment
- Refactoring sm_dance_bot_strikes_back

Fixed
-----
- Minor navigation parameters fixes on sm_dance_bot
- Minor format fixes
- Minor tuning to mitigate overshot issue cases
- Minor format issues
- Fix CI: format fix python version

Removed
-------
- Removed neo_simulation2 package
- Removed parameters smacc

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_78
==========

Added
-----

- Added waypoint 4 and iterations changes to allow the robot to complete the course (#155)
- Added migration to smacc2
- Added .reps dependencies and fixed build errors
- Added dependency to ur5 client
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability QOS config
- Added husky launch file in sm_dance_bot
- Added dependencies for husky simulation
- Added warehouse2 progress (#179)
- Added Waypoint Inputs (#178)
- Added SrConditional fixes and formatting (#168)
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files
- Added undo tuning and errors
- Added missing file from warehouse2 (#205)
- Added backport to foxy

Changed
-------

- Updated format
- Updated readme (#164)
- Updated dependencies for husky in rolling and galactic
- Updated subscriber publisher components
- Updated autowar machine progress
- Updated warehouse3 tuning and fixes (#202)
- Updated format issues

Fixed
-----

- Fixed errors introduced on formatting
- Fixed missing dependency
- Fixed some linting warnings
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed broken build
- Fixed some formatting and templating on SrConditional
- Fixed move trigger logic into headers
- Fixed lint
- Fixed formatting
- Fixed warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green
- Fixed minor linking errors in foxy

Removed
-------

- Removed test from main moveit cmake
- Removed line

Authors
-------

- Pablo Iñigo Blasco
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
Section_79
==========

Added
-----
- Added more components to smacc core for autoware demo.
- Added docker files for different revisions, warnings removal, and more navigation testing.
- Added barrel search build fix and warehouse3 improvements.
- Added progress in barrel husky development.
- Added branching example.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml for manual deployment.
- Added workflow for testing prerelease builds.
- Added necessary package and edited Threesome launch.
- Added setupTracing.sh for automated installation of ros-rolling-ros2trace.
- Added sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files.

Changed
-------
- Changed launch command from 'ros2 launch sm_three_some sm_three_some' to 'ros2 launch sm_three_some sm_three_some.launch'.
- Changed wording from "smacc application" to "SMACC2 library".
- Updated name of package and package.xml.
- Renamed to smacc2 and smacc2_msgs.
- Updated description table.
- Updated table.
- Updated ci-build-source.yml.
- Updated doxygen-check-build.yml.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Renamed event generator library.
- Renamed folders, deleted tracing.md, edited README.md.
- Reverted markdowns to html.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Reset all versions to 0.0.0.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_atomic_24hr.
- Cleaned up of sm_atomic_24hr.
- Minor formatting fixes.
- Minor changes.
- Minor format changes.
- Minor linking errors fixed.
- Minor broken build fixed.
- Minor reordering fixes.
- Minor broken build fixed.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.
- Minor fixes.

Fixed
-----
- Fixed startup problems in warehouse 3.
- Fixed broken build.
- Fixed broken build.
- Fixed broken build.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed format and minor issues.
- Fixed docker for foxy and galactic.
- Fixed docker build files for all versions.
- Fixed missing rolling repositories build.
- Fixed Navigation2 for semi-binary build.
- Fixed smacc2 component bug.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed format and minor issues.
- Fixed

```rst
Section_80
==========

Added
-----
- More Readme Updates (#72) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- More Readme (#74) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- sm_advanced_recovery_1 reworked (#83) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- more sm_advanced_recovery_1 (#84) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- More sm_advanced_recovery_1 work (#85) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_advanced_recovery_1 round 4 (#86) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Brettpac branch (#87)
- sm_atomic_performance_test_a_2
- sm_atomic_performance_test_a_1 (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_atomic_performance_test_c_1 (#88) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- modifying sm_atomic_performance_test_a_2 (#89) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_multi_stage_1 (#90) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- more sm_multi_stage_1 (#91) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Wait topic message client behavior (#81) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, Denis Štogl <destogl@users.noreply.github.com>)
- Feature/wait nav2 nodes client behavior (#82) (Co-authored-by: Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Changed
-------
- Update README.md: updated launch command
- Correct all linters and formaters.

Fixed
-----
- Fix pre-commit
- Trying to fix Pre-Commit
- navigation parameters fixes on sm_dance_bot

Removed
-------
- None
```

```rst
Section_81
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive. Optional selection of nodes to wait for.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `aws demo`.
- `Brettpac branch`.
- `neo_simulation2` package removal.
- Source build enabled on PR for testing.
- `mm`.
- Diverse improvements in navigation and performance.
- `slam toggle` and `smacc deep history` feature.
- `dance bot s pattern` feature.
- First working version of `sm template` and template generator.
- `waypoints navigator` bug fix.
- `SM core test` added.
- `nav2z` renaming.
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others.

Changed
-------

- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Several core improvements during navigation testing.
- Formatting improvements.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.
- More refinement in `sm_dance_bot`.
- Minor tuning to mitigate overshot issue cases.
- Minor navigation improvements.
- Using local action messages.

Fixed
-----

- Remove some compile warnings.
- Format fixes.
- Minor format issues.

Removed
-------

- `neo_simulation2` package.
- `sm_dance_bot_msgs` removal.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_82
==========

Added
-----
- Added remaining SVGs to READMEs (#145).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient.
- Added SM Atomic SM generator (#143).
- Added dependencies for husky simulation.
- Added warehouse2 progress (#179).
- Added SrConditional fixes and formatting (#168).
- Added feature for CB pure spinning (#188, #189).
- Added feature for planner changes 16 12 (#191).
- Added feature for replanning 16 dec (#193).
- Added feature for undo motion 20 12 (#196, #198).
- Added feature for sync 21 12 (#199).
- Added feature for warehouse2 22 12 (#200).
- Added feature for warehouse2 23 12 (#201).
- Added feature for minor tune (#203).

Changed
-------
- Updated package list (#142).
- Fixed launch command for sm_dance_bot_strikes_back in README.md.
- Fixed CI: format fix python version (#148).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Redoing sm_dance_bot_warehouse_3 waypoints.
- Finetuning waypoints (#187).
- Tuning warehouse3 (#197).
- Tuning and fixes (#202).
- Fixed some formatting and templating on SrConditional.
- Moved trigger logic into headers on SrConditional.
- Added reliability QoS config.

Removed
-------
- Removed parameters smacc (#147).
- Removed node creation and create only a logger (#149).
- Removed test from main moveit cmake.
- Removed some comments in the past.
- Removed some linting warnings.
- Removed test ur5.
- Removed parameters smacc.
- Removed parameters smacc.
- Removed some build errors.
- Removed some more linting warnings.
- Removed some compiling issues.
- Removed some broken master build.
- Removed some pipeline error.
- Removed some broken build.
- Removed some warehouse3 problems.

Fixed
-----
- Noticed launch command was incorrect in README.md.
- Fixed launch command for sm_dance_bot_strikes_back.
- Fixed some errors introduced on formatting.
- Fixed missing dependency.
- Fixed some more linting warnings.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed broken build.
- Fixed some formatting issues.

Authors
-------
- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_83
==========

Added
-----
- Added missing file from warehouse2 (#205).
- Added docker files for different revisions, warnings removal, and more testing on navigation.
- Added barrel demo.
- Added multiple controllable LEDs plugin.
- Added progress in Husky demo.
- Added improvements in navigation behaviors.
- Added more merge.
- Added feature/docker improvements march 2022 (#235).
- Added replanning for all our examples.
- Added Foxy backport (#206).
- Added galactic CI build because Navigation2 is broken in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-check-build.yml.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added manual deployment for now.
- Added workflow for checking doc build.
- Added workflow for testing prerelease builds.
- Added docs/ as source folder for documentation.
- Added docs/ as output directory.
- Added necessary package and edited Threesome launch.
- Added Dockerfile w/ ROS distro as argument.
- Added new folder for additional tracing contents.
- Added setupTracing.sh.
- Added alternative ManualTracing.
- Added new SM markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Added SMACC2 library tutorial for Dockerfile.
- Added smacc2_performance_tools.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Changed wording "smacc application" to "SMACC2 library".
- Changed extension of imports.
- Changed all mentions of SMACC/ROS to SMACC2/ROS2.
- Changed formatting of python file.

Fixed
-----
- Fixed warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green.
- Fixed weird moveit not downloaded repo.
- Fixed minor broken build.
- Fixed some reordering fixes.
- Fixed format and minor issues.
- Fixed startup problems in warehouse 3.
- Fixed format and minor issues.
- Fixed barrel search build and warehouse3.
- Fixed bug in SMACC2 component.
- Fixed missing rolling repositories build.
- Fixed Navigation2 for semi-binary build.
- Fixed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Fixed progress on navigation rolling.
- Fixed renamed folders, deleted tracing.md, and edited README.md.
```


*pabloinigoblasco*

```rst
Section_84
==========

Added
-----
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.
- Added navigation parameters fixes on sm_dance_bot.

Changed
-------
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated smacc2_rta command across readmes.
- Updated README.md launch command.

Fixed
----
- Fixed source CI and corrected README overview. (#62)
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
- Renamed event generator library.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Minor formatting improvements.
- Noticed a note that was not removed.
- Attempting pre-commit fixes.
- Progressing in aws navigation.
- Minor format improvements.
- Format improvements.
- More on navigation.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws

```rst
Section_85
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive; optional node selection
- Base for the `sm_aws_aarehouse` navigation
- Gazebo fixes for showing the robot and the lidar in `sm_dance_bot`, `sm_dance_bot_strikes_back`, and `sm_dance_bot_lite`
- Progress in navigation, `slam_toggle` client behaviors, and `slam_toolbox` components
- `smacc2::deep_history` syntax introduced for `smacc2` deep history functionality
- First working version of `sm` template and template generator

Changed
-------
- Formatting improvements in various sections
- Minor format adjustments throughout

Fixed
----
- Navigation parameters fixes on `sm_dance_bot`
- Remove some compile warnings
- Correct formatting in `neo_simulation2` package removal
- Move method after the method it calls to prevent recursion

Removed
-------
- `neo_simulation2` package

Contributors
------------
- Pablo Iñigo Blasco (@pabloinigoblasco)
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_86
==========

Added
-----
- Feature/sm dance bot refine (#131)
  - More changes in sm_dance_bot
- Feature/sm dance bot refine 2 (#132)
  - More changes in sm_dance_bot
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
  - Formatting
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing some errors introduced on formatting
  - Missing dependency
  - Fixing some more linting warnings
  - Progress on moveit migration testing
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
  - More readme updates
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Pre-commit cleanup
- Add QOS durability to SmaccPublisherClient (#163)
  - Feat: add qos durability to SmaccPublisherClient
  - Fix: add a missing colon
  - Refactor: remove line
  - Feat: add reliability qos config
- Feature/testing moveit behaviors (#167)
  - More testing on moveit
  - Progress on moveit
  - More testing on moveit behaviors
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
- Feature/planner changes 16 12 (#191)
  - Minor changes
  - More fixes
- Feature/replanning 16 dec (#193)
  - Minor changes
  - Replanning for all our examples
- Several fixes (#194)
- Minor changes (#195)

Changed
-------
- Minor format issues (#134)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)

Removed
-------
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Workflows update
- Workflow
- Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past
- Removing test from main moveit cmake
- Test ur5
- Progressing in the moveit migration testing
- Adding .reps dependencies and also fixing some build errors
- Repos dependency
- Adding dependency to ur5 client
- Docker refactoring
- Progress on move_it PR
- Minor dockerfile test workaround
- Improving dockerfile for building local tests
- Fixing compiling issues
- More readme updates
- Moved reference library SMs to smacc2_performance_tools
- Pre-commit cleanup
- Feat: add qos durability to SmaccPublisherClient
- Fix: add a missing colon
- Refactor: remove line
- Feat: add reliability qos config
- More testing on moveit
- Progress on moveit
- More testing on moveit behaviors
- Finishing touches 1
- Readme
- Redoing sm_dance_bot_warehouse_3 waypoints
- More waypoints
- Fix: some formatting and templating on SrConditional
- Fix: move trigger logic into headers
- Fix: lint
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
- More on aws demo
-

```rst
Section_87
==========

Added
-----

- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation in warehouse2
- Feature/undo motion 20 12 (#198)
  - Undo tuning and error fixes
- Feature/sync 21 12 (#199)
  - Fixed format issues
- Feature/warehouse2 22 12 (#200)
  - Finished warehouse2
- Feature/warehouse2 23 12 (#201)
  - Tuning and fixes (#202)
- Feature/minor tune (#203)
  - Fixed warehouse 3 problems and core improvements (#204)
- Added missing file from warehouse2 (#205)
- Use correct upstream .repos files for source builds (#243)
- Correct mergify branch names (#246)
- Update galactic source build job name (#250)
- Galactic source build: updated .repos file, bumped action version, and used correct version of upstream packages (backport #241) (#248)
- Restored workflow files (#252)
- Restored files (#253)
- Fix checkout branches for scheduled builds (#254)
- Feature/fixing husky build rolling (#257)
  - Made husky project build on rolling
- Feature/fixing husky build rolling (#258)
  - Made husky project build on rolling
- Update README.md (#262)
- Feature/fixing ur demos (#261)
- Feature/fixing type string walker (#263)
  - Fixed type string walker threesome demo
- Update README.md (#266)
- Update README.md (#267)
- Update README.md (#268)
- Significant update in Getting Started Instructions (#269)
  - Removed trailing spaces
- Fixing ur demo (#273)
- Fix: initialized conditionFlag as false (#274)
- Precommit fix (#280)
  - Merged in red for focal-rolling due to broken state
- Progress on the sm_husky_barrel
- More on husky demo for galactic
- Feature/galactic rolling merge (#288)
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs" commit
  - Updated description table
  - Updated table
  - Copied initial docs
  - Dockerfile with ROS distro as argument

Changed
-------

- Minor changes in various features
- Replanned all examples in different features
- Tuning warehouse3 (#197)
- Format issues in various features
- Docker files for different revisions, warnings removal, and more testing on navigation
- Fixed docker for foxy and galactic
- Docker build files for all versions
- Barrel search build fix and warehouse3
- Fixed startup problems in warehouse 3
- Fixed format and minor issues
- Progress in barrel husky
- Progress in husky demo
- Improving navigation behaviors
- More merge
- Docker improvements

Removed
-------

- Missing and redundant entries
- Unused code and dependencies
- Fixed building issues
- Typos and broken builds
```

```rst
Section_88
==========

Added
-----
- Opened new folder for additional tracing contents.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added sm_respira_test_2.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files (#58).
- Fix source CI and correct README overview (#62).
- Update c_cpp_properties.json.
- Update README.md with updated launch command.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.

Changed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Bug fix in smacc2 component.
- Reverted markdowns to html.
- Edited tracing.md to reflect new tracing event names.
- Renamed folders, deleted tracing.md, edited README.md.
- Corrected trailing spaces.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_reference_library.
- Minor formatting improvements.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.

Contributors
------------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_89
==========

Version 1.0.0 (2022-01-01)
---------------------------

Added
-----

- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are alive. Optional node selection.
- New feature: `cb_wait_topic_message` asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior.

Changed
-------

- Corrected all linters and formatters.
- Several core improvements during navigation testing.
- Formatting improvements.

Fixed
----

- Removed some compile warnings.

Version 1.1.0 (2022-02-01)
---------------------------

Added
-----

- Progress in AWS navigation demo.
- Base for the `sm_aws_warehouse` navigation.
- More on navigation.
- Merge and progress.
- `sm_dance_bot_lite`.
- Updates yaml.
- `sm_dance_bot` visualizing Turtlebot3.
- Cleaning and lidar show/hide option.
- Cleaning files and formatting work.
- Gazebo fixes to show the robot and lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- AWS demo.

Changed
-------

- Progressing in AWS navigation.
- Minor format adjustments.

Fixed
----

- Got `sm_multi_stage_1` working (barely).
- Precommit cleanup run.

Removed
-------

- Branch `Brettpac`.

Contributors
------------

- Pablo Iñigo Blasco
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

# Section 90

## Added
- Added source build on PR for testing.
- Added more sm_multi_stage_1.
- Added diverse improvements in navigation and performance.
- Added additional linting and formatting.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added smacc2::deep_history syntax.
- Added slam pausing/resuming functionality to sm_dance_bot.
- Added dance bot s pattern.
- Added first working version of sm template and template generator.
- Added SM core test.
- Added local action msgs.
- Added navigation 2 stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Added reliability qos config.
- Added initial state machine transition timestamp.
- Added durability to SmaccPublisherClient.

## Changed
- Adjusted build packages of source CI.
- Moved method after the method it calls to prevent recursion.
- Renamed sm_advanced_recovery_1.
- Reworked sm_multi_stage_1 with multistage modes, sequences, steps, and finishing touches.
- Renamed AWS navigation sm dance bot.

## Fixed
- Corrected formatting.
- Fixed launch command in README.md.
- Fixed CI format for Python version.
- Fixed overshot issue cases in waypoints navigator.
- Fixed compiling warnings.
- Fixed broken master build.
- Fixed pipeline error.

## Removed
- Removed neo_simulation2 package.
- Removed merge markers from a Python file.
- Removed node creation and created only a logger.
- Removed parameters smacc.
- Removed sm_dance_bot_msgs.
- Removed test from main moveit CMake.

## Authors
- Pablo Iñigo Blasco
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored by: DecDury <declandury@gmail.com>
- Co-authored by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_91
==========

Added
-----
- Progress on AWS navigation and refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress (#177).
- Waypoint Inputs (#178).
- Warehouse2 progress (#179).
- Format (#180).
- Sm_dance_bot_warehouse_3 (#181).
- Feature/sm warehouse 2 13 dec 2 (#182).
- Feature/wharehouse2 dec 14 (#185).
- Feature/sm warehouse 2 13 dec 2 (#186).
- Finetuning waypoints (#187).
- Feature/cb pure spinning (#188).
- Pure spinning behavior missing files.
- Feature/planner changes 16 12 (#191).
- Feature/replanning 16 dec (#193).
- Several fixes (#194).
- Feature/undo motion 20 12 (#196).
- Tuning warehouse3 (#197).
- Feature/undo motion 20 12 (#198).
- Undo tuning and errors.
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Finishing warehouse2.
- Feature/warehouse2 23 12 (#201).
- Tuning and fixes (#202).
- Feature/minor tune (#203).
- Fixing warehouse 3 problems and other core improvements (#204).
- Added missing file from warehouse2 (#205).
- Merging code from backport foxy and updates about autoware (#208).
- Update cb_navigate_global_position.hpp.
- Update tracing/ManualTracing.md.

Changed
-------
- Fixed broken build.
- Minor changes (#175).
- Minor changes.
- More changes and headless.
- Merge.
- Headless and other fixes.
- Default values.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes.
- Minor changes

```rst
Section_92
==========

Added
-----
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Fixed formatting in various files.
- Fixed pre-commit issues.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.

Other
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Optimized dependencies in move_base_z_planners_common.
- Corrected all linters and formatters.

Performance
-----------
- Performance tests improvements.
- More changes on performance tests.
- Minor performance improvements.

Documentation
-------------
- Edited tracing.md to reflect new tracing event names.
- Updated doxygen links.
- More Readme Updates.
- More Readme.
- Update README.md.
- Updated launch command in README.md.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_atomic_24hr formatting.
- Cleaned up sm_reference_library.
- Cleaned up sm_atomic_24hr more.
- Cleaned up sm_multi_stage_1.
- Cleaned up sm_advanced_recovery_1.
- Cleaned up sm_advanced_recovery_1 more.

Collaborators
-------------
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.
```

```rst
Section_93
==========

Added
-----
- New client behavior for nav2: now waits for nav2 nodes to subscribe to the /bond topic and confirms they are active. Optional node selection available.
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.

Changed
-------
- Navigation parameters fixes on sm_dance_bot.
- Minor format improvements.
- Merge and progress.
- Minor hotfix.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and lidar.
- Gazebo fixes for sm_dance_bot_strikes_back.
- Precommit cleanup run.
- Got sm_multi_stage_1 working (barely).
- Gaining traction with sm_multi_stage_1.
- Various core improvements during navigation testing.

Fixed
-----
- Removed some compile warnings.

Removed
-------
- Removed neo_simulation2 package.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

Section_94
===========

Added
-----
- Additional linting and formatting.
- Feature/slam toggle and smacc deep history (#122): Progress in navigation, slam toggle client behaviors, slam_toolbox components, and smacc2::deep_history syntax.
- Feature/dance bot s pattern (#128): Polishing sm_dance_bot and s-pattern.
- First working version of sm template and template generator (#127).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Rolling Docker environment to be executed from any environment (#154).
- Add SM Atomic SM generator (#143).
- Add QOS durability to SmaccPublisherClient (#163).
- Feature/aws navigation sm dance bot (#174): Progress on aws navigation, refactorings on navigation clients and behaviors.
- Waypoint Inputs (#178).

Changed
-------
- Move method after the method it calls. Otherwise recursion could happen (#126).
- Resolve compile warnings (#137).
- Minor navigation improvements (#141).
- Using local action messages.
- Update package list (#142).
- Fix CI: format fix python version (#148).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Feature/migration moveit client (#151): Initial migration to smacc2, fixing errors introduced on formatting, and updating format.
- Feature/testing moveit behaviors (#167): More testing on moveit, progress on moveit, and fixing pipeline error.

Fixed
-----
- Waypoints navigator bug (#133): Minor tuning to mitigate overshot issue cases.
- Minor format issues (#134).
- Noticed typo: Finnaly > Finally.
- Fixing compiling issues.
- Update readme (#164): More readme updates.

Removed
-------
- Removing parameters smacc.
- Removing node creation and create only a logger (#149).
- Removing test from main moveit cmake.
- Removing sm_dance_bot_msgs.
- Pending references.

Co-authored-by: Brett <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, Denis Štogl <denis@stogl.de>.

```rst
Section_95
==========

Added
-----
- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2 feature with minor changes.

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added warehouse2 feature with format changes and headless improvements.

- Feature/cb pure spinning (#188)
  - Implemented pure spinning behavior with format changes and headless improvements.

- Feature/replanning 16 dec (#193)
  - Implemented replanning for all examples with several fixes.

- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation for warehouse2 with minor changes.

- Feature/sync 21 12 (#199)
  - Implemented sync feature with replanning for all examples and format fixes.

- Feature/warehouse2 22 12 (#200)
  - Added warehouse2 feature with replanning for all examples and format fixes.

- Feature/warehouse2 23 12 (#201)
  - Added warehouse2 feature with tuning and fixes.

- Feature/minor tune (#203)
  - Implemented minor tune with fixes for warehouse 3 problems.

- Feature/odom tracker improvements and retry motion (#223)
  - Improved odom tracker with forward behavior retry functionality and removed warnings.

Changed
-------
- Finetuning waypoints (#187)
  - Co-authored with Ubuntu 20-04-02-amd64.
  - Improved waypoint finetuning.

- Tuning warehouse3 (#197)
  - Tuned warehouse3 feature.

- Fix rolling builds (#222)
  - Fixed issues with rolling builds.

- Foxy backport (#206)
  - Backported changes to Foxy with minor formatting fixes.

Fixed
-----
- Several fixes (#194)
  - Implemented several fixes.

- Minor broken build (#206)
  - Fixed minor issues causing broken builds.

Removed
-------
- Remove example things from Foxy CI setup. (#214)
  - Removed unnecessary example configurations from Foxy CI setup.

- Disable disabled packages
  - Updated workflows to disable unnecessary packages.

Other
-----
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
  - Merging to get backport working.

- Add Autoware Auto Msgs into not-released dependencies. (#220)
  - Added Autoware Auto Msgs as dependencies.

- Remove trailing spaces.
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
- Branching example.
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.
- Correct formatting of python file.
- Included necessary package and edited Threesome launch.
- Renamed header files and corrected format.
- Added workflow for checking doc build.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Updated name of package and package.xml to pass liter.
- Executed on master update.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Updated description table.
- Updated table.
- Copied initial docs.
- Dockerfile w/ ROS distro as argument.
  - Usage: "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/".
``` 

*pabloinigoblasco*

```rst
Section_96
==========

Added
-----
- Opened new folder for additional tracing contents.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to automate ros-rolling-ros2trace installation.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added README tutorial for Dockerfile.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Renamed tracing events.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated smacc2_rta command across readmes.
- Renamed event generator library.

Fixed
-----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Minor formatting fixes.
- Several core improvements during navigation testing.
- Format improvements in various files.
- Attempted pre-commit fixes.

Removed
-------
- Manual installation of ros-rolling-ros2trace.

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_97
==========

Added
-----
- Implemented new feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Added new client behavior for nav2: `cb_wait_nav2_nodes`, which subscribes to the `/bond` topic and waits for selected nodes to become active.

Changed
-------
- Improved core functionality during navigation testing.
- Enhanced formatting for better readability.

Fixed
-----
- Resolved navigation parameters issues on `sm_dance_bot`.
- Fixed compile warnings.

Removed
-------
- Removed redundant format improvements entries.

Contributors
------------
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

Section_98
-----------

### Added
- Enable source build on PR for testing. (#112)
- Added SM core test. (#138)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient. (#163)
- Added husky launch file in sm_dance_bot for AWS navigation. (#174)
- Added dependencies for husky simulation for AWS navigation.

### Changed
- Adjusted build packages of source CI.
- Improved navigation and performance. (#116)
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. (#122)
- Polished sm_dance_bot and s-pattern. (#128, #129)
- Updated package list. (#142)
- Moved reference library SMs to smacc2_performance_tools. (#166)
- Refactored sm dance bot strikes back. (#152)
- Updated readme files. (#140, #145, #164)

### Fixed
- Corrected formatting.
- Removed merge markers from a python file. (#119)
- Fixed CI format for Python version. (#148)
- Fixed launch command in README.md.
- Fixed compiling issues.

### Removed
- Removed neo_simulation2 package.

### Miscellaneous
- Co-authored commits: Brett (brett@robosoft.ai), pabloinigoblasco (pablo@ibrobotics.com), DecDury (declandury@gmail.com), Denis Štogl (destogl@users.noreply.github.com, denis@stogl.de).

### Contributors
- Brett (brett@robosoft.ai)
- pabloinigoblasco (pablo@ibrobotics.com)
- DecDury (declandury@gmail.com)
- Denis Štogl (destogl@users.noreply.github.com, denis@stogl.de)

```rst
Section_99
==========

Added
-----
- Added Feature/retry behavior warehouse 1 (#226)
  - Includes minor changes, replanning for all examples, backport to foxy, and minor format adjustments.
- Added Foxy backport (#206)
  - Includes minor formatting fixes, trailing spaces correction, codespell fix, Python linters warnings correction, addition of galactic CI build due to Navigation2 issues in rolling, partial changes for ament_cpplint, addition of tf2_ros as dependency, disabling of ament_cpplint and some packages, ccache version bump, further package ignore, satisfaction of ament_lint_cmake, addition of missing licenses, disabling of cpplint and cppcheck linters, correct formatters, branching example, disabling of disabled packages, update of ci-build-source.yml, extension change, import extension change, cppcheck enablement, Python file formatting correction, necessary package inclusion, and Threesome launch editing.
- Added dockerfiles (#225)
- Added Fix code generators (#221)
  - Includes fixing of other build issues, update of SM template for better code visibility, removal of node usage in SM performance template, update of template to use Blackboard storage, resolution of global data in template, and update of sm_name.hpp.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Fixed
-----
- Fixed minor broken build.
- Fixed warnings removal and additional testing on navigation in docker files for different revisions.
- Fixed docker for foxy and galactic.

Removed
-------
- Removed use of node in the sm performance template.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>
```

```rst
Section_100
===========

Added
-----

- Created workflow for testing prerelease builds.
- Added setupTracing.sh script to install necessary packages and configure tracing group.
- Introduced alternative ManualTracing method.
- Added new SM markdowns.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Implemented performance tests improvements.
- Added sm_respira_test_2.
- Added new feature cb_wait_topic_message for asynchronous client behavior.
- Created sm_multi_stage_1 state machine.

Changed
-------

- Renamed packages to smacc2 and smacc2_msgs.
- Updated package name and package.xml for liter compatibility.
- Replaced "smacc application" with "SMACC2 library" in wording.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed event generator library.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_reference_library formatting.
- Corrected trailing spaces in code.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Fixed
-----

- Corrected GitHub branch reference.
- Fixed bug in smacc2 component.
- Reverted markdowns to HTML format.
- Fixed source CI and corrected README overview.
- Attempted precommit fixes.

Removed
-------

- Removed manual deployment method.
- Ignored all packages except smacc2 and smacc2_msgs.
- Removed manual installation of ros-rolling-ros2trace (now automated in setupTracing.sh).
- Removed galactic builds from master, keeping only rolling.
- Removed submodules and used .repos file for dependencies.
- Deleted tracing directory.
- Deleted tracing.md file.
```

*pabloinigoblasco*

```rst
Section_101
===========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#82)
- New client behavior for nav2: `wait nav2 nodes` subscribes to the `/bond` topic and waits for them to be alive, with optional node selection
- New feature: `cb pause slam` client behavior (#98)
- New client behavior: `cb pause slam` pauses SLAM functionality

Changed
-------
- Corrected all linters and formatters
- Navigation parameters fixes on `sm_dance_bot`
- Minor formatting improvements

Fixed
-----
- Removed some compile warnings (#96)

Removed
-------
- Minor hotfix

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```
*pabloinigoblasco*

Section_102
-----------

Added
-----
- Added Brettpac branch (#110).
- Added a3 (#113).
- Added more sm_multi_stage_1 (#114).
- Added diverse improvements navigation and performance (#116).
- Added Feature/diverse improvements navigation performance (#117).
- Added Remove neo_simulation2 package. (#112).
- Added Feature/slam toggle and smacc deep history (#122).
- Added Move method after the method it calls. Otherwise recursion could happen. (#126).
- Added First working version of sm template and template generator. (#127).
- Added Feature/dance bot s pattern (#128).
- Added Feature/dance bot s pattern (#129).
- Added Feature/sm dance bot refine (#131).
- Added Feature/sm dance bot refine 2 (#132).
- Added waypoints navigator bug (#133).
- Added progress in the sm_dance_bot tests (#135).
- Added sm_dance_bot_lite (#136).
- Added Resolve compile warnings (#137).
- Added Add SM core test (#138).
- Added minor navigation improvements (#141).
- Added using local action msgs (#139).
- Added Feature/nav2z renaming (#144).
- Added added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added added remaining SVGs to READMEs (#145).
- Added Update package list. (#142).
- Added Fix CI: format fix python version (#148).
- Added Add SM Atomic SM generator. (#143).
- Added Remove node creation and create only a logger. (#149).
- Added Rolling Docker environment to be executed from any environment (#154).
- Added slight waypoint 4 and iterations changes so robot can complete course (#155).
- Added Feature/migration moveit client (#151).
- Added initial migration to smacc2.
- Added Add QOS durability to SmaccPublisherClient (#163).
- Added feat: add qos durability to SmaccPublisherClient.
- Added fix: add a missing colon.
- Added refactor: remove line.
- Added feat: add reliability qos config.
- Added Feature/testing moveit behaviors (#167).
- Added sm_pubsub_1 (#169).
- Added sm_pubsub_1 part 2 (#170).
- Added sm_advanced_recovery_1 renaming (#171).
- Added sm_multi_stage_1 reworking (#172).

Changed
-------
- Corrected formatting in Remove neo_simulation2 package. (#112).
- Adjusted build packages of source CI in Remove neo_simulation2 package. (#112).
- Polished sm_dance_bot and s-pattern in Feature/dance bot s pattern (#128).
- Fixed typo in Feature/dance bot s pattern (#128).
- Changed "Finnaly" to "Finally" in Feature/dance bot s pattern (#129).
- Fixed launch command in README.md for sm_dance_bot_strikes_back and removed past comments.
- Updated format in Feature/migration moveit client (#151).
- Added missing dependency in Feature/migration moveit client (#151).
- Fixed linting warnings in Feature/migration moveit client (#151).
- Updated format in Feature/migration moveit client (#151).
- Added .reps dependencies and fixed build errors in Feature/migration moveit client (#151).
- Added repos dependency in Feature/migration moveit client (#151).
- Added dependency to ur5 client in Feature/migration moveit client (#151).
- Refactored Docker in Feature/migration moveit client (#151).
- Improved Dockerfile for building local tests in Feature/migration moveit client (#151).
- Fixed compiling issues in Feature/migration moveit client (#151).
- Updated README in Feature/migration moveit client (#151).

Removed
-------
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs in Feature/nav2z renaming (#144).
- Removed parameters smacc in removing parameters smacc (#147).

Fixed
-----
- Fixed recursion issue by moving method after the method it calls (#126).
- Fixed broken master build in Feature/testing moveit behaviors (#167).
- Fixed pipeline error in Feature/testing moveit behaviors (#167).

Authors
-------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> in multiple entries.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com> in diverse improvements navigation and performance (#116).
- Co-authored-by: DecDury <declandury@gmail.com> in Feature/sm dance bot strikes back refactoring (#152).
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com> in Feature/sm dance bot strikes back refactoring (#152).

Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_103
===========

Added
-----

- Introduce multistage modes for improved sequence handling.
- Implement sm_multi_stage sequences and sm_multi_state_1 steps.
- Add sm_multi_stage_1 sequence d, sm_multi_stage_1 c sequence, mode_5_sequence_b, and mode_4_sequence_b.
- Include finishing touches 1 and update readme.

Changed
-------

- Enhance AWS navigation with sm_dance_bot (#174).
- Update repo dependencies and launch husky files in sm_dance_bot.
- Adjust dependencies for husky simulation in rolling and galactic.
- Refactor navigation clients and behaviors for AWS demo.

Fixed
-----

- Fix broken builds and formatting issues.
- Resolve linting errors and broken source builds.
- Correct Focal-Rolling builds and trailing spaces.
- Address python linters warnings and codespell issues.

Removed
-------

- Remove redundant files and fix minor formatting errors.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
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
Section_104
===========

Added:
------
- Added tf2_ros as dependency to find include.
- Added missing licenses.
- Added workflow for checking doc build.
- Added setupTracing.sh which installs necessary packages and configures tracing group.
- Added a dockerfile for Rolling and Galactic.

Changed:
--------
- Changed extension of imports.
- Changed wording "smacc application" to "SMACC2 library".
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Changed GitHub branch reference.
- Changed name of package and package.xml to pass liter.
- Changed extension.

Fixed:
------
- Fixed bug in smacc2 component.
- Fixed formatting of python file.
- Fixed trailing spaces.
- Fixed source CI and corrected README overview.

Removed:
--------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed galactic builds from master and kept only rolling, removing submodules and using .repos file.
- Removed tracing directory.

Other Changes:
--------------
- Bumped ccache version.
- Ignored further packages.
- Satisfied ament_lint_cmake.
- Enabled cppcheck.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Reactivated smacc2 nav clients for rolling via submodules.
- Optimized dependencies in move_base_z_planners_common.
- Renamed tracing events.
- Reverted markdowns to html.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated ci-build-source.yml.
- Updated doxygen-check-build.yml.
- Updated doxygen-deploy.yml.
- Updated changelogs.
- Updated description table.
- Updated table.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated sm_three_some launch command.
- Updated ci-build-source.yml.
- Updated tracing/ManualTracing.md.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated smacc2_sm_reference_library/sm_atomic/README.md.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated name of package and package.xml to pass liter.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated name of package and package.xml to pass liter.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated name of package and package.xml to pass liter.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated smacc2_performance_tools.
- Updated smacc2_rta command across readmes.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated tracing.md to reflect new tracing event names.
- Updated workflows.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated README tutorial for Dockerfile.
- Updated README.md.
- Updated README overview.
- Updated

```rst
Section_105
===========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Nodes to wait can be optionally selected.
- New feature: cb pause slam client behavior.
- New feature: sm_dance_bot_lite.

Changed
-------

- Updated launch command.
- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor format improvements.
- Merge and progress in aws navigation demo.
- Fix format in sm_dance_bot.
- Minor hotfix in doxygen deployment workflow.
- Visualizing turtlebot3 in sm_dance_bot.

Fixed
-----

- Several core improvements during navigation testing.
- Formatting improvements.

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

Section_106
===========

Added
-----
- Added `sm_dance_bot` feature to visualize `turtlebot3`.
- Added lidar show/hide option for cleaning.
- Added formatting improvements.

Changed
-------
- Improved `sm_dance_bot` gazebo visualization.
- Enhanced formatting for better readability.

Fixed
-----
- Fixed issues with gazebo to display the robot and lidar correctly.
- Resolved formatting inconsistencies.
- Fixed `sm_multi_stage_1` doubling issue.
- Fixed gazebo issues for `sm_dance_bot_strikes_back`.
- Fixed issues with `sm_multi_stage_1` functionality.
- Fixed issues with AWS demo.
- Corrected formatting in various files.
- Fixed issues with `sm_dance_bot` stages progression.
- Fixed issues with `neo_simulation2` package.
- Fixed source build for testing purposes.
- Fixed build packages for source CI.
- Fixed recursion possibility in method calls.
- Fixed overshot issue in waypoints navigator.
- Fixed minor format issues.
- Fixed compile warnings.
- Fixed navigation improvements.
- Fixed launch command in README.md.

Removed
-------
- Removed `neo_simulation2` package.
- Removed unnecessary comments in code.
- Removed unused `sm_dance_bot_msgs`.
- Removed unused parameters in `smacc`.
- Removed unnecessary workflows.
- Removed incorrect launch command in README.md.

Other
-----
- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, and pabloinigoblasco <pablo@ibrobotics.com>.
- Updated package list.
- Added SVGs to READMEs for better visualization.
- Rolled Docker environment for universal execution.
- Progressed in migration to `smacc2`.
- Added missing dependencies.
- Improved Dockerfile for local tests.
- Updated README files.
- Continued work on moveit migration.
- Added `.reps` dependencies.
- Added dependencies to `ur5` client.
- Refactored Docker setup.
- Made progress on `move_it` PR.

```rst
Section_107
===========

Added
-----

- Initial state machine transition timestamp (#165)
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability QOS config
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Added dependencies for husky simulation
- Added Waypoint Inputs (#178)
- Added repo dependency for husky launch file in sm_dance_bot
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files
- Added SrConditional fixes and formatting (#168)
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)

Changed
-------

- Moved reference library SMs to smacc2_performance_tools
- Refactored to remove line
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)
- Redoing sm_dance_bot_warehouse_3 waypoints
- Improved undo motion navigation warehouse2
- Tuning warehouse3 (#197)
- Fixed warehouse 3 problems and other core improvements (#204)
- Updated subscriber publisher components
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Reordered components for autoware demo
- Docker build files for all versions
- Barrel search build fix and warehouse3
- Progress in barrel husky

Fixed
-----

- Fixed missing colon
- Fixed pipeline error
- Fixed broken master build
- Fixed broken build
- Fixed formatting
- Fixed errors in pure spinning behavior
- Fixed format issues
- Fixed tuning and fixes
- Fixed startup problems in warehouse 3
- Fixed broken build
- Fixed warnings removal
- Fixed docker for foxy and galactic

Removed
-------

- Removed a missing colon
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
Section_108
===========

Added
-----

- Feature/barrel - do not merge yet (#233)
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Use manual deployment for now.
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic

Changed
-------

- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- Changed wording "smacc application" to "SMACC2 library"
- Update name of package and package.xml to pass liter.
- Reset all versions to 0.0.0
- Update description table.
- Update table
- Copy initial docs
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Do not execute clang-format on smacc2_sm_reference_library package.
- Renaming of event generator library
- Optimized deps in move_base_z_planners_common.
- Update c_cpp_properties.json
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  Also noticed a note I had made while producing these that was not removed

Fixed
-----

- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Correct formatters.
- Correct formatting of python file.
- Fixed source CI and correct README overview. (#62)

Removed
-------

- Delete tracing directory
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  Location of sh file assumed if user follows README.md under "Getting started"

Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_109
===========

Added
-----

- Added new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92, #93, #94)
- Added new client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94)

Changed
-------

- Reworked sm_advanced_recovery_1 (#83, #84, #85, #86)
- Modified sm_atomic_performance_test_a_2 (#89)
- Updated launch command in README.md
- Corrected all linters and formatters (#82)

Fixed
-----

- Fixed pre-commit issues (#83, #84, #85, #86, #90)
- Fixed navigation parameters on sm_dance_bot (#93, #95)
- Removed some compile warnings (#96)
```

## Section_110

### Added
- New feature: `cb_wait_topic_message` asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior `add` for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.
- Added gazebo fixes to show the robot and the lidar.
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others.
- Added remaining SVGs to READMEs.

### Changed
- Minor format improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Cleaning and lidar show/hide option in `sm_dance_bot_visualizing_turtlebot3`.
- Updates in YAML files.
- Adjusted build packages of source CI.
- Rolling Docker environment to be executed from any environment.
- Refactored `sm_dance_bot_strikes_back`.

### Fixed
- Minor hotfix.
- Corrected formatting.
- Fixed CI: format fix python version.

### Removed
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs`.
- Removed parameters in `smacc`.

### Miscellaneous
- Various improvements in navigation and performance.
- Resolved compile warnings.
- Added SM core test.
- Noticed launch command was incorrect in README.md, fixed launch command for `sm_dance_bot_strikes_back`, and removed some comments.
- Precommit cleanup.
- Workflow updates.

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

# Section_111

## Added
- Added feature/migration moveit client (#151).
- Added initial migration to smacc2.
- Added progress on move_it PR.
- Added feature/testing moveit behaviors (#167).
- Added feature/aws navigation sm dance bot (#174).
- Added feature/sm warehouse 2 13 dec 2 (#182).
- Added feature/cb pure spinning (#188).
- Added feature/planner changes 16 12 (#191).
- Added feature/replanning 16 dec (#193).
- Added feature/undo motion 20 12 (#196).
- Added feature/sync 21 12 (#199).
- Added feature/warehouse2 22 12 (#200).
- Added feature/warehouse2 23 12 (#201).
- Added feature/minor tune (#203).
- Added fixing warehouse 3 problems, and other core improvements (#204).
- Added backport to foxy.

## Changed
- Changed waypoint 4 and iterations to allow robot to complete course (#155).
- Changed reference library SMs to smacc2_performance_tools (#166).
- Changed QOS durability to SmaccPublisherClient (#163).
- Changed SrConditional fixes and formatting (#168).

## Fixed
- Fixed errors introduced on formatting.
- Fixed missing dependency.
- Fixed linting warnings.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed formatting.
- Fixed broken build.
- Fixed some formatting and templating on SrConditional.
- Fixed move trigger logic into headers.
- Fixed lint issues.
- Fixed missing files in pure spinning behavior.
- Fixed format issues.
- Fixed tuning and errors.
- Fixed missing subscriber publisher components.

## Removed
- Removed test from main moveit cmake.
- Removed some linting warnings.
- Removed line.

## Miscellaneous
- Progressed in moveit migration testing.
- Updated format.
- Added .reps dependencies and fixed some build errors.
- Added dependency to ur5 client.
- Refactored docker.
- Improved dockerfile for building local tests.
- Added husky launch file in sm_dance_bot.
- Updated dependencies for husky in rolling and galactic.
- Added warehouse2 progress.
- Finetuned waypoints.
- Redid sm_dance_bot_warehouse_3 waypoints.
- Added more waypoints.
- Progressed on aws navigation and refactorings on navigation clients and behaviors.
- Added more on aws demo.
- Added several fixes.
- Improved undo motion navigation warehouse2.
- Tuned warehouse3.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning and fixes.
- Added tuning

```rst
Section_112
===========

Added:
------
- Feature/docker improvements march 2022 (#235)
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Use manual deployment for now.
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Update tracing/ManualTracing.md
- changed wording "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
- added README tutorial for Dockerfile

Changed:
--------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- branching example
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- renamed tracing events after
- bug in smacc2 component
- reverted markdowns to html
- updated mentions of SMACC/ROS to SMACC2/ROS2
- sm_reference_library reformatting
- Renaming of event generator library

Fixed:
------
- minor broken build
- fixing docker for foxy and galactic
- fixing startup problems in warehouse 3
- fixing format and minor
- minor linking errors foxy
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
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
- some progress on navigation rolling
- more changes on performance tests
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- Optimized deps in move_base_z_planners_common
- minor formatting

Removed:
--------
- warnings removval
- Delete tracing directory
- Moved tracing.md to tracing directory
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
- cleanup
- cleanup
- cleanup
- edited tracing.md to reflect new tracing event names
```


*pabloinigoblasco*

```rst
Section_113
===========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------

- Updated launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69).
- Updated doxygen links (#70).
- Updated README.md.

Fixed
-----

- Fixed pre-commit issues in various commits.

Removed
-------

- Removed redundant entries related to navigation progress and formatting improvements.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_114
===========

Added
-----
- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2, wait for nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optional node selection.

Changed
-------
- Navigation parameters fixes on sm_dance_bot.
- Gazebo fixes for sm_dance_bot_strikes_back.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introducing slam pausing/resuming functionality in sm_dance_bot.
- Polishing sm_dance_bot and s-pattern.
- Refinement in sm_dance_bot.
- First working version of sm template and template generator.

Fixed
-----
- Remove some compile warnings. (#96)
- Move method after the method it calls to prevent recursion. (#126)
- Minor tuning to mitigate overshot issue cases in waypoints navigator.
- Minor format issues.

Removed
-------
- Remove neo_simulation2 package.
- Remove merge markers from a python file.

Other
-----
- Precommit cleanup run.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Several core improvements during navigation testing.
- Diverse improvements in navigation and performance.
- Cleaning and lidar show/hide option.
- More fixes in various components.
- Keep hammering on multi-stage development.
- Progress in AWS navigation demo.
- Base for the sm_aws_warehouse navigation.
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
- Progress in AWS navigation demo.
- Progress in AWS

Section_115
===========

Added
-----
- Renamed navigation 2 stack.
- Improved formatting.
- Added SVGs to READMEs of atomic, dance_bot, and others (#140, #145).
- Updated package list (#142).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient (#163).
- Added reliability QOS configuration.
- Added durability QOS to SmaccPublisherClient.
- Added durability QOS to SmaccPublisherClient.
- Added SM Atomic SM generator.
- Added durability QOS to SmaccPublisherClient.
- Added reliability QOS configuration.

Changed
-------
- Updated launch command in README.md for sm_dance_bot_strikes_back.
- Refactored feature "sm dance bot strikes back" (#152).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Reworked "sm_multi_stage_1" (#172).
- Updated dependencies for husky in rolling and galactic.
- Progressed on AWS navigation and refactored navigation clients and behaviors.
- Fine-tuned waypoints (#187).
- Redid waypoints for "sm_dance_bot_warehouse_3".
- Fine-tuned warehouse3.
- Improved undo motion navigation for warehouse2.
- Tuned warehouse3.

Fixed
-----
- Fixed launch command for sm_dance_bot_strikes_back.
- Fixed CI: format fix python version (#148).
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed broken build.
- Fixed formatting.
- Fixed some linting warnings.
- Fixed some errors introduced on formatting.
- Fixed some more linting warnings.
- Fixed compiling issues.
- Fixed formatting issues.

Removed
-------
- Removed parameters from smacc (#147).
- Removed test from main moveit cmake.
- Removed node creation and created only a logger.

Authors
-------
- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>

```rst
Section_116
===========

Added
-----

- Added missing file from warehouse2 (#205)
- Use correct upstream .repos files for source builds (#243)
- Correct mergify branch names (#246)
- Update galactic source build job name (#250)
- Galactic source build: update .repos file, bump action version and use correct version of upstream packages (backport #241) (#248)
- Feature/fixing husky build rolling (#257)
- Feature/fixing husky build rolling (#258)
- Feature/fixing ur demos (#261)
- Feature/fixing type string walker (#263)
- Significant update in Getting Started Instructions (#269)
- Added changelogs

Changed
-------

- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Renamed folders, deleted tracing.md, edited README.md
- Performance tests improvements
- More on performance and other issues
- SM_Respira_1 format cleanup
- SM_Respira_test_2

Fixed
-----

- Fixing warehouse 3 problems, and other core improvements (#204)
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green
- Fixing docker for foxy and galactic
- Fixing startup problems in warehouse 3
- Fixing format and minor
- Fixing rolling build (#239)
- Trying to fix dependencies
- Fixing to focal by the moment
- More fixing rolling build
- Fixing building issue
- Typo
- Fixing broken build
- Build fix
- Restoring workflow files (#252)
- Restoring files (#253)
- Fix checkout branches for scheduled builds (#254)
- Correct checkout branch on scheduled build
- Fix: initialise conditionFlag as false (#274)
- Precommit fix (#280)
- Fix urls to index.ros.org (#284)
- Fix foxy source build config to use repos file from foxy branch. (#285)
- Fixing sm_dance_bot examples
- Working on fix of image messages for husky_barrel demo

Removed
-------

- Ignore packages which should not be released.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: mergify[bot] <37929162+mergify[bot]@users.noreply.github.com>
Co-authored-by: brettpac <brettpac@users.noreply.github.com>
```

```rst
Section_117
===========

Added
-----

- Add galactic CI setup and rename rolling files. (#58)
- Fix source CI and correct README overview. (#62)
- Update c_cpp_properties.json
- Add new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success. Also, add new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait. (#82)
- Update README.md with new launch command.
- Add new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success. Also, add new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait. (#92)
- Add new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success. Also, add new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait. (#94)

Changed
-------

- Update smacc2_rta command across readmes.
- Change launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Modify sm_atomic_performance_test_a_2 (#89).
- Update launch command for sm_respira_1.
- Correct all linters and formatters.

Fixed
-----

- Correct trailing spaces.
- Fix pre-commit.
- Attempt pre-commit fixes.
- Fixing precommit.
- Correct all linters and formatters.

Removed
-------

- Do not execute clang-format on smacc2_sm_reference_library package.
- Remove note that was not removed.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_118
===========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` behavior waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Progress in AWS navigation demo.
- Base for `sm_aws_aarehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` improvements.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- Lidar show/hide option in `sm_dance_bot`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- `sm_multi_stage_1` doubling.
- `smacc2::deep_history` syntax in `sm_dance_bot`.
- `s-pattern` polishing in `sm_dance_bot`.
- First working version of `sm` template and template generator.
- Minor tweaks.

Changed
-------
- Formatting improvements in various areas.
- Navigation parameters fixes on `sm_dance_bot`.
- Cleaning and formatting work in `sm_dance_bot`.
- Format fixes in various components.

Fixed
-----
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting in code.
- Adjust build packages for source CI.
- Move method after the method it calls to prevent recursion.

Removed
-------
- `neo_simulation2` package.

Authors
-------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_119
===========

Added
-----
- Progress in the sm_dance_bot tests (#135)
- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SM warehouse 2 13 dec 2 (#182)
- Added CB pure spinning (#188)
- Added planner changes 16 12 (#191)
- Added undo motion 20 12 (#196)

Changed
-------
- Minor tuning to mitigate overshot issue cases
- Some progress on markers cleanup
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action msgs
- Feature/nav2z renaming (#144)
- Resolve compile warnings (#137)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
- Initial migration to smacc2
- Progress on moveit migration testing
- Feature/testing moveit behaviors (#167)
- More testing on moveit behaviors
- Progress on moveit
- Feature/aws navigation sm dance bot (#174)
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- Warehouse2 progress (#179)
- Waypoint Inputs (#178)
- Finetuning waypoints (#187)
- Several fixes (#194)
- Tuning warehouse3 (#197)

Fixed
-----
- Noticed launch command was incorrect in README.md
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing compiling issues
- Fixing broken master build
- Fixing broken build

Removed
-------
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing redundant files for pure spinning behavior

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_120
===========

Added
-----

- Feature/undo motion 20 12 (#198): Improved undo motion navigation in warehouse2.
- Feature/sync 21 12 (#199): Addressed format issues.
- Feature/warehouse2 22 12 (#200): Finished warehouse2 development.
- Feature/warehouse2 23 12 (#201): Tuning, fixes, and enhancements (#202).
- Feature/minor tune (#203): Fixed warehouse 3 issues and improved core functionality (#204).
- Foxy backport (#206): Updated CI build for Galactic due to Navigation2 issues in Rolling.

Changed
-------

- Updated launch command for sm_three_some package.
- Renamed header files and corrected formats.
- Added workflow for doc build checking.
- Updated doxygen-check-build.yml and created doxygen-deploy.yml.
- Implemented testing workflow for prerelease builds.
- Renamed packages to smacc2 and smacc2_msgs.
- Reset all versions to 0.0.0.
- Updated description table and other tables.
- Updated changelogs.
- Reverted changes to ignore all packages except smacc2 and smacc2_msgs.
- Updated GitHub branch references.
- Updated package names and package.xml.
- Updated master branch.
- Updated CI build source file.
- Changed extension of imports.
- Enabled cppcheck and corrected python file formatting.
- Included necessary package and edited Threesome launch.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr package.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Minor formatting improvements.

Fixed
-----

- Fixed trailing spaces.
- Corrected codespell and python linters warnings.
- Fixed deadlocks and improved continuous integration.
- Fixed missing file in warehouse2.
- Fixed linking errors in Foxy.
- Fixed smacc application wording to SMACC2 library.
- Fixed formatting issues in sm_reference_library.
- Fixed clang-format execution in smacc2_sm_reference_library.
- Fixed trailing spaces in sm_atomic_24hr.
- Fixed source CI and corrected README overview.
- Updated doxygen links.
- Updated README files.
- Updated launch command for sm_respira_1 package.
- Removed unnecessary manual installation steps for ros-rolling-ros2trace.
- Updated tracing event names.
- Enabled build of missing Rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders and deleted unnecessary files.
- Added smacc2_performance_tools package.
- Improved performance tests.
- Cleaned up formatting in sm_respira_1 and sm_atomic_24hr packages.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Fixed minor formatting issues.
``` 

*pabloinigoblasco*

```rst
Section_121
===========

Added
-----

- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success
- Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait

Changed
-------

- Progress in AWS navigation demo
- Navigation parameters fixes on sm_dance_bot
- Several core improvements during navigation testing

Fixed
-----

- Fix pre-commit issues
- Correct all linters and formatters

Removed
-------

- Removed redundant entries

Contributors
------------

- Pablo Iñigo Blasco
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_122
===========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `wait nav2 nodes subscribing to the /bond topic and waiting they are alive`. You can optionally select the nodes to wait.

Changed
-------
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introducing slam pausing/resuming functionality in testing `sm_dance_bot`.
- Polishing `sm_dance_bot` and s-pattern.
- More refinement in `sm_dance_bot`.
- First working version of `sm template` and template generator.
- Minor tuning to mitigate overshot issue cases in `waypoints navigator`.
- Minor navigation improvements.
- Renaming to `navigation 2 stack`.

Fixed
-----
- Move method after the method it calls to prevent recursion (#126).
- Corrected formatting in `neo_simulation2 package removal`.
- Fixed launch command for `sm_dance_bot_strikes_back` in README.md.

Removed
-------
- Removed `neo_simulation2 package`.
- Removed `sm_dance_bot_msgs`.
- Removed parameters in `smacc`.

Other
-----
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progress in gazebo fixes for `sm_dance_bot_strikes_back`.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Precommit cleanup.
- Update package list.
- Workflows update.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_123
===========

Added
-----
- Add SM Atomic SM generator. (#143)
- Feature/sm dance bot strikes back refactoring (#152)
  - Co-authored-by: DecDury <declandury@gmail.com>
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Feature/migration moveit client (#151)
  - initial migration to smacc2
  - fixing some errors introduced on formatting
  - missing dependency
  - fixing some more linting warnings
  - adding .reps dependencies and also fixing some build errors
  - repos dependency
  - adding dependency to ur5 client
- Add QOS durability to SmaccPublisherClient (#163)
  - feat: add qos durability to SmaccPublisherClient
  - fix: add a missing colon
  - refactor: remove line
  - feat: add reliability qos config
- Feature/testing moveit behaviors (#167)
  - more testing on moveit
  - progress on moveit
  - more testing on moveit behaviors
  - minor configuration
  - fixing pipeline error
  - fixing broken master build
- Feature/aws navigation sm dance bot (#174)
  - repo dependency
  - husky launch file in sm_dance_bot
  - Add dependencies for husky simulation.
  - Fix formatting.
  - Update dependencies for husky in rolling and galactic.
  - progress on aws navigation and some other refactorings on navigation clients and behaviors
  - more on aws demo
  - fixing broken build
- Feature/wharehouse2 dec 14 (#185)
  - warehouse2
  - minor
- Feature/sm warehouse 2 13 dec 2 (#186)
  - more changes and headless
  - merge
  - headless and other fixes
  - default values
- Feature/cb pure spinning (#189)
  - more changes and headless
  - merge
  - headless and other fixes
  - default values
- Feature/planner changes 16 12 (#191)
  - more fixes
- Feature/replanning 16 dec (#193)
  - replanning for all our examples
- Feature/undo motion 20 12 (#198)
  - replanning for all our examples
  - improving undo motion navigation warehouse2
- Feature/sync 21 12 (#199)
  - replanning for all our examples
- Feature/warehouse2 22 12 (#200)
  - replanning for all our examples
- Feature/warehouse2 23 12 (#201)
  - replanning for all our examples
- Feature/minor tune (#203)
  - tuning and fixes

Changed
-------
- Rolling Docker environment to be executed from any environment (#154)
- moved reference library SMs to smacc2_performance_tools (#166)
- initial state machine transition timestamp (#165)
- SrConditional fixes and formatting (#168)
  - fix: some formatting and templating on SrConditional
  - fix: move trigger logic into headers
  - fix: lint
- finetuning waypoints (#187)
- tuning warehouse3 (#197)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green (#204)

Fixed
-----
- Fix CI: format fix python version (#148)
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- fixing compiling issues
- fixing broken build
- warehouse2 progress (#179)
- format (#180)
- format issues
- format issues
- tuning and fixes (#202)
- minor tune
- minor format
- minor linking errors foxy

Removed
-------
- Remove node creation and create only a logger. (#149)
- removing test from main moveit cmake
- test ur5
- progressing in the moveit migration testing
- updating format
- minor dockerfile test workaround
- improving dockerfile for building local tests
- fixing undo tuning and errors
- weird moveit not downloaded repo
- added missing file from warehouse2 (#205)

Authors
-------
- Pablo Iñigo Blasco
```

```rst
Section_124
===========

Added:
------
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
  ```
  sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/
  ```
- Opened new folder for additional tracing contents.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh:
  Installs necessary packages and configures tracing group.
- Removed manual installation of ros-rolling-ros2trace.
  This is now automated in setupTracing.sh.
  Location of sh file assumed if user follows README.md under "Getting started".
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Changed wording "smacc application" to "SMACC2 library".
- Update smacc_sm_reference_library/sm_atomic/README.md:
  Edit from html to markdown syntax.

Changed:
--------
- ros2 launch sm_three_some sm_three_some:
  Changed to ros2 launch sm_three_some sm_three_some.launch.

Fixed:
------
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
- Add missing licences.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Enable cppcheck.
- Correct formatting of python file.
- Included necessary package and edited Threesome launch.
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
- Sm_respira_1 format cleanup.
- Sm_respira_1 format cleanup pre-commit.
- Sm_respira_test_2.
- More changes on performance tests.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Sm_reference_library reformatting.
- Correct trailing spaces.
- Sm_atomic_24hr.
- Sm_atomic_performance_trace_1.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- More sm_atomic_24hr cleanup.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Minor formatting.

Removed:
--------
- Missing.
- Missing sm.
- Updating subscriber publisher components.
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine.
- Refining cp subscriber cp publisher.
- Autoware demo.
- Missing.
- Foxy ci.
- Fix.
- Minor broken build.
- Merging code from backport foxy and updates about autoware (#208).
- Minor changes.
- Replanning for all our examples.
- Backport to foxy.
- Minor format.
- Minor linking errors foxy.
- Foxy backport (#206).
- Minor formatting fixes.
- Disable disabled packages.
- Update ci-build-source.yml.
- Change extension.
- Change extension of imports.
- Disable some packages and update workflows.
- Update doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Base for the sm_aws_aarehouse navigation.
- Progressing in aws navigation.
- Minor.
- Several core improvements during navigation testing.
- Formatting improvements.

Authors:
--------
- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_125
===========

Added
-----

- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait

Changed
-------

- Navigation parameters fixes on sm_dance_bot
- Corrected all linters and formatters

Fixed
-----

- Fix pre-commit issues
- Remove some compile warnings

Removed
-------

- Nothing

Contributors
------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

```rst
Section_126
===========

Added
-----

- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Feature/dance bot s pattern (#128)
- First working version of sm template and template generator. (#127)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- Added SM Atomic SM generator. (#143)

Changed
-------

- Progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
- Going forward in testing sm_dance_bot introducing slam pausing/resuming functionality
- Polishing sm_dance_bot and s-pattern
- More refinement in sm_dance_bot
- Minor tuning to mitigate overshot issue cases
- Some more progress on markers cleanup
- Minor navigation improvements
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Remove node creation and create only a logger

Fixed
-----

- Navigation parameters fixes on sm_dance_bot
- Resolve compile warnings
- Fix CI: format fix python version

Removed
-------

- Remove neo_simulation2 package
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Pending references

Other Changes
-------------

- Several core improvements during navigation testing
- Formatting improvements
- Minor format fixes
- Format improvements
- Gazebo fixes, to show the robot and the lidar
- Format fixes
- Format improvements
- Precommit cleanup run
- Updates yaml
- Precommit cleanup
- Workflows update
- Workflow
- Added SVGs to READMEs of atomic, dance_bot, and others
- Added remaining SVGs to READMEs
- Precommit cleanup
- Update package list
``` 

**Autor:** Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_127
===========

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
- Add QOS durability to SmaccPublisherClient (#163)
  - Adding qos durability to SmaccPublisherClient
  - Fixing missing colon
  - Removing line
  - Adding reliability qos config
- Feature/aws navigation sm dance bot (#174)
  - Adding dependencies for husky simulation
  - Updating dependencies for husky in rolling and galactic
  - Progress on aws navigation and refactorings on navigation clients and behaviors
  - Fixing broken build

Changed
-------

- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)
- Tuning warehouse3 (#197)
- Tuning and fixes (#203)
- Fixing warehouse 3 problems and other core improvements (#204)
  - Removing deadlocks and making continuous integration green

Fixed
-----

- Update readme (#164)
- SrConditional fixes and formatting (#168)
  - Fixing formatting and templating on SrConditional
  - Moving trigger logic into headers
  - Lint fixes
- Several fixes (#194)
- Fixing warehouse 3 problems and other core improvements to remove deadlocks, also making continuous integration green (#204)
- Added missing file from warehouse2 (#205)

Removed
-------

- None

Authors
-------

- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_128
===========

Added:
------
- Add mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Create smacc2_performance_tools
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Include necessary package and edit Threesome launch
- Open new folder for additional tracing contents
- Rename header files and correct format.
- Reset all versions to 0.0.0
- Update changelogs
- Update description table
- Update doxygen-check-build.yml
- Update name of package and package.xml to pass liter.
- Update table
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Use manual deployment for now.
- Use smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Correct codespell.
- Correct formatters.
- Correct formatting of python file.
- Disable ament_cpplint.
- Disable cpplint and cppcheck linters.
- Disable some packages and update workflows.
- Enable cppcheck
- Ignore all packages except smacc2 and smacc2_msgs
- Ignore further packages
- Satisfy ament_lint_cmake
- Bump ccache version
- Add missing licences.
- Remove example things from Foxy CI setup. (#214)
- Remove manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
- Remove tracing directory
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Update ci-build-source.yml
- Update tracing/ManualTracing.md
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh

Changed:
--------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- changed wording "smacc application" to "SMACC2 library"
- renamed folders, deleted tracing.md, edited README.md
- renamed tracing events after
- reactivating smacc2 nav clients for rolling via submodules
- some progress on navigation rolling
- updated mentions of SMACC/ROS to SMACC2/ROS2

Fixed:
------
- Fix rolling builds (#222)
- Fix trailing spaces.
- Minor broken build
- Minor changes
- Minor format
- Minor formatting fixes

Removed:
--------
- Disable disabled packages
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Remove warnings (#213)
- Do not execute clang-format on smacc2_sm_reference_library package.

Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```rst
Section_129
===========

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
- Feature/sm dance bot fixes (#95)

Changed
-------

- Update c_cpp_properties.json to use `ros2 launch sm_respira_1 sm_respira_1.launch` (#69)
- Update README.md launch command
- Correct all linters and formaters.

Fixed
-----

- Fix source CI and correct README overview. (#62)
- Fix pre-commit issues.
- Correct navigation parameters on sm_dance_bot.

Removed
-------

- Removed redundant entries.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_130
===========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `wait nav2 nodes` subscribing to the `/bond` topic and waiting for them to be alive. Optional node selection available.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Changed
-------
- Progress in AWS navigation demo.
- Progress in navigation, `slam toggle` client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- Progress in testing `sm_dance_bot` introducing `slam pausing/resuming` functionality.
- Polishing `sm_dance_bot` and `s-pattern`.
- Minor tuning to mitigate overshot issue cases in `waypoints navigator`.

Fixed
-----
- Navigation parameters fixes on `sm_dance_bot`.
- Minor format issues.
- Compile warnings removed.
- Recursion issue potential fix by moving method after the method it calls.
- Minor navigation improvements.

Removed
-------
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs`.

Other
-----
- Precommit cleanup run.
- Updates to YAML files.
- Enable source build on PR for testing.
- Additional linting and formatting.
- Remove merge markers from a Python file.
- First working version of `sm template` and `template generator`.
- Various core improvements during navigation testing.
- Several formatting improvements.
- Cleaning and lidar show/hide option.
- More progress on markers cleanup.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_dance_bot_lite` progress.
- `sm_multi_stage_1` working progress.
- `sm_multi_stage_1` gaining traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
- `sm_multi_stage_1` more stages.
- `sm_multi_stage_1` more progress.
- `sm_multi_stage_1` more traction.
```

```rst
Section_131
===========

Added
-----
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added remaining SVGs to READMEs.
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added dependencies for husky simulation.
- Added dependencies for husky in rolling and galactic.
- Added warehouse2 progress (#179).
- Added Waypoint Inputs (#178).
- Added more Waypoints.
- Added default values.
- Added more changes and headless.
- Added merge.
- Added finishing touches 1.
- Added more testing on moveit.
- Added progress on moveit.
- Added more testing on moveit behaviors.
- Added slight waypoint 4 and iterations changes so robot can complete course (#155).
- Added progress on moveit migration testing.
- Added progress on move_it PR.
- Added progress on aws navigation and some other refactorings on navigation clients and behaviors.
- Added more on aws demo.
- Added fixing broken master build.
- Added fixing pipeline error.
- Added fixing compiling issues.
- Added fixing some more linting warnings.
- Added fixing some errors introduced on formatting.
- Added fixing some build errors.
- Added fixing broken build.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
- Added fixing some formatting and templating on SrConditional.
-

Section_132
==========

Added
-----
- Added missing file from warehouse2 (#205)
- Added docker build files for all versions
- Added dockerfiles (#225)
- Added Feature/retry behavior warehouse 1 (#226)
- Added a dockerfile for Rolling and Galactic
- Added setupTracing.sh to automate ros-rolling-ros2trace installation
- Added alternative ManualTracing method
- Added new sm markdowns
- Added README tutorial for Dockerfile

Changed
-------
- Changed wording "smacc application" to "SMACC2 library"
- Changed extension of imports
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- Changed formatting of python file
- Changed extension of header files
- Changed all mentions of SMACC/ROS to SMACC2/ROS2

Fixed
-----
- Fixed warehouse 3 problems and other core improvements (#204)
- Fixed minor broken build
- Fixed trailing spaces
- Fixed codespell
- Fixed python linters warnings
- Fixed other build issues

Removed
-------
- Removed use of node in the sm performance template
- Removed manual installation of ros-rolling-ros2trace

Other
-----
- Tuning and fixes (#202)
- Feature/minor tune (#203)
- Minor tune
- Weird moveit not downloaded repo
- Updating subscriber publisher components
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Foxy CI
- Some reordering fixes
- Docker files for different revisions, warnings removal, and more testing on navigation
- Fixing docker for foxy and galactic
- Fix other build issues
- Update SM template and make example code clearly visible
- Update templated to use Blackboard storage
- Update template to resolve the global data correctly
- Update sm_name.hpp
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use manual deployment for now
- Create workflow for checking doc build
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference
- Update name of package and package.xml to pass linter
- Execute on master update
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Update description table
- Update table
- Copy initial docs
- Dockerfile with ROS distro as argument
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- Enable cppcheck
- Included necessary package and edited Threesome launch
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling
- Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2

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
- Pablo Iñigo Blasco <pablo@ibrobotics.com>

```rst
Section_133
===========

Added
-----
- Added smacc2_performance_tools for performance testing.
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Nodes to wait can be optionally selected.

Changed
-------
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md launch command.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Corrected all linters and formatters.

Fixed
-----
- Fixed source CI and corrected README overview. (#62)
- Fixed pre-commit issues in various packages.

Removed
-------
- Do not execute clang-format on smacc2_sm_reference_library package.

Other
-----
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Minor formatting improvements.
- Noticed a note that was not removed while producing these changes.
- Navigation parameters fixes on sm_dance_bot.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_134
===========

Added
-----

- New feature: `cb_wait_topic_message` client behavior for asynchronous waiting and optional content check of a topic message.
- New client behavior for `nav2`: `cb_wait_nav2_nodes` to wait for nodes subscribing to the `/bond` topic.
- New client behavior: `cb_pause_slam` for pausing SLAM functionality.
- New client behavior: `sm_dance_bot_lite` for visualizing TurtleBot3.
- New feature: Gazebo fixes for showing the robot and lidar in `sm_dance_bot` and `sm_dance_bot_strikes_back`.
- New feature: `sm_multi_stage_1` with multiple stages.
- New feature: `smacc2::deep_history` syntax for deep history in `smacc2`.

Changed
-------

- Improved core functionality during navigation testing.
- Formatting improvements in various sections.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor formatting adjustments in several areas.
- Polishing and refining `sm_dance_bot` and `s-pattern`.

Fixed
-----

- Removed compile warnings.
- Removed `neo_simulation2` package.
- Removed merge markers from a Python file.
- Fixed recursion possibility by moving a method after the one it calls.
- Corrected typo ("Finnaly" to "Finally").

Removed
-------

- Removed `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_135
===========

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
- using local action messages (#139)
- Feature/nav2z renaming (#144)
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Remove node creation and create only a logger. (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- initial state machine transition timestamp (#165)
- moved reference library SMs to smacc2_performance_tools (#166)
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
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)

Changed
-------

- minor tweaks (#130)
- minor format issues (#134)
- Fix CI: format fix python version (#148)
- fixing some errors introduced on formatting
- fixing some more linting warnings
- fixing compiling issues
- more readme updates
- fixing broken master build
- fixing broken build
- finetuning waypoints (#187)
- more fixes

Removed
-------

- removing sm_dance_bot_msgs
- removing parameters smacc
- removing test from main moveit cmake
- removing parameters smacc
- removing dependencies to ur5 client
- removing some comments in the past
- removing some formatting and templating on SrConditional
- remove line

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_136
===========

Added
-----

- Feature/replanning 16 dec (#193)
  - Replanning for all examples
- Feature/undo motion 20 12 (#196)
  - Improving undo motion navigation in warehouse2
- Feature/undo motion 20 12 (#198)
  - Undo tuning and errors
- Feature/sync 21 12 (#199)
  - Format issues
- Feature/warehouse2 22 12 (#200)
  - Finishing warehouse2
- Feature/warehouse2 23 12 (#201)
  - Tuning and fixes (#202)
- Feature/minor tune (#203)
  - Fixing warehouse 3 problems and other core improvements (#204)
  - Added missing file from warehouse2 (#205)
- Feature/improvements warehouse3 (#228)
  - Backport to foxy
- Foxy backport (#206)
  - Fixing format and minor issues
  - Fixing trailing spaces
  - Correcting codespell and python linters warnings
  - Adding galactic CI build due to Navigation2 issues in rolling
  - Adding partial changes for ament_cpplint
  - Adding tf2_ros as dependency
  - Disabling ament_cpplint, cpplint, and cppcheck linters
  - Bumping ccache version
  - Ignoring further packages
  - Satisfying ament_lint_cmake
  - Adding missing licenses
  - Disabling disabled packages
  - Updating workflows
  - Enabling cppcheck
  - Correcting formatting of python files
  - Including necessary package and editing Threesome launch
  - Renaming header files and correcting format
  - Adding workflow for checking doc build
  - Updating doxygen-check-build.yml
  - Creating doxygen-deploy.yml
  - Creating workflow for testing prerelease builds
  - Renaming to smacc2 and smacc2_msgs
  - Correcting GitHub branch reference
  - Updating package name and package.xml
  - Resetting all versions to 0.0.0
  - Ignoring all packages except smacc2 and smacc2_msgs
  - Updating changelogs
  - Reverting "Ignore all packages except smacc2 and smacc2_msgs"
  - Updating description table
  - Updating table
  - Copying initial docs
  - Creating Dockerfile with ROS distro as argument
  - Opening new folder for additional tracing contents
  - Deleting tracing directory
  - Moving tracing.md to tracing directory
  - Adding setupTracing.sh

Changed
-------

- Only rolling version should be pre-released on master. (#230)
  - Correcting Focal-Rolling builds by fixing the version of rosdep yaml (#234)
- Branching example
  - Changing extension of imports
- ros2 launch sm_three_some sm_three_some
  - Changing to ros2 launch sm_three_some sm_three_some.launch
- Added:
  - First ensure you have the necessary package installed
  - Renaming header files and correcting format
- Use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
``` 

*pabloinigoblasco*

```rst
Section_137
===========

Added
-----
- Automated installation of ros-rolling-ros2trace in setupTracing.sh
- Alternative ManualTracing option
- New SM markdowns
- Dockerfile for Rolling and Galactic
- README tutorial for Dockerfile
- smacc2_performance_tools
- Performance tests improvements
- Optimized dependencies in move_base_z_planners_common
- New feature: cb_wait_topic_message for asynchronous client behavior
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic

Changed
-------
- Renamed "smacc application" to "SMACC2 library"
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Renamed tracing events
- Updated smacc2_rta command across readmes
- Corrected trailing spaces
- Renamed event generator library
- Minor formatting improvements
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch

Fixed
-----
- Bug in smacc2 component
- Do not execute clang-format on smacc2_sm_reference_library package
- Cleaned up sm_atomic_24hr
- Fixed source CI and corrected README overview
- Attempted precommit fixes

Removed
-------
- Manual installation of ros-rolling-ros2trace
- Galactic builds from master, keeping only Rolling
- Submodules, using .repos file instead
- Tracing.md file
``` 

*pabloinigoblasco*

```rst
Section_138
===========

Added
-----

- New feature: `cb_wait_topic_message` (#92)
  - Asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: `cb_wait_nav2_nodes` (#92)
  - Waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive

Changed
-------

- Navigation parameters fixes on `sm_dance_bot` (#93, #95)
- `cb_pause_slam` client behavior added (#98)
- `sm_dance_bot_lite` updates (#99)
- Visualizing `turtlebot3` in `sm_dance_bot` (#101)
- Gazebo fixes for `sm_dance_bot_strikes_back` (#105)

Fixed
-----

- Removed some compile warnings (#96)
- Removed `neo_simulation2` package (#112)

Removed
-------

- `neo_simulation2` package

Collaborators
-------------

- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_139
===========

Added
-----

- Adjusted build packages of source CI.
- Diverse improvements in navigation and performance.
- Added linting and formatting.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM core test.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Added dependencies for husky simulation.
- Added dependencies for husky in rolling and galactic.
- Added warehouse2.
- Added Waypoint Inputs.

Changed
-------

- Moved reference library SMs to smacc2_performance_tools.
- Updated package list.
- Updated README.
- Updated format.
- Updated dependencies.
- Updated dockerfile for building local tests.
- Updated readme.
- Updated format.

Fixed
-----

- Remove merge markers from a python file.
- Move method after the method it calls to prevent recursion.
- Fix CI: format fix python version.
- Remove node creation and create only a logger.
- Fixing broken master build.
- Fixing pipeline error.
- Fixing compiling issues.

Removed
-------

- Removed merge markers from a python file.
- Removed parameters smacc.
- Removed sm_dance_bot_msgs.
- Removed test from main moveit cmake.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_140
===========

Added
-----
- Added progress in warehouse2 (#179)
- Added feature for warehouse 2 on December 13 (#182)
- Added Brettpac branch (#184)
- Added feature for warehouse 2 on December 13 (#186)
- Added finetuning waypoints (#187)
- Added feature for cb pure spinning (#188)
- Added pure spinning behavior missing files and minor changes (#190)
- Added feature for planner changes on December 16 (#191)
- Added feature for replanning on December 16 (#193)
- Added several fixes (#194)
- Added feature for undo motion on December 20 (#196)
- Added feature for undo motion on December 20 (#198)
- Added feature for sync on December 21 (#199)
- Added feature for warehouse2 on December 22 (#200)
- Added finishing warehouse2 feature on December 23 (#201)
- Added feature for minor tune (#203)
- Added fixing warehouse 3 problems and other core improvements (#204)
- Added backport to foxy, minor format, and minor linking errors foxy (#205)
- Added missing components and progress in autoware machine
- Added refining cp subscriber cp publisher
- Added improvements in smacc core for autoware demo
- Added foxy ci fixes and docker files for different revisions
- Added docker build files for foxy and galactic versions
- Added barrel demo and search build fix for warehouse3
- Added progress in barrel husky and fixing broken build
- Added master rolling to galactic backport, fixing build, and testing dance bot demos
- Added updating galactic repos, runtime dependency, and restoring ur dependency

Changed
-------
- Changed redoing sm_dance_bot_warehouse_3 waypoints to more waypoints (#181)
- Changed default values for warehouse2 (#185)
- Changed default values for cb pure spinning (#189)
- Changed tuning warehouse3 (#197)
- Changed format issues for sync (#199)
- Changed tuning and fixes for warehouse2 (#202)

Fixed
-----
- Fixed some formatting and templating on SrConditional (#168)
- Fixed trigger logic into headers on SrConditional
- Fixed lint issues
- Fixed weird moveit not downloaded repo

Removed
-------
- Removed some reordering fixes

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco
- pabloinigoblasco
