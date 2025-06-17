Changelog for package sm_multi_stage_1
=======================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2`
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further work on buildfarm issue
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
- Feature/galactic rolling merge (#288)
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs"
  - Updated description table
  - Updated table
  - Copied initial docs
  - Dockerfile with ROS distro as argument
  - Opened new folder for additional tracing contents
  - Deleted tracing directory
  - Moved tracing.md to tracing directory
  - Added setupTracing.sh for necessary package installation and tracing group configuration
  - Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh
  - Created alternative ManualTracing
  - Added new sm markdowns
  - Added Dockerfile for Rolling and Galactic
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  - Renamed tracing events
  - Bug fixes in smacc2 component
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup for sm_respira_1
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Minor formatting changes
  - Added galactic CI setup and renamed rolling files (#58)
  - Fixed source CI and corrected README overview (#62)
  - Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  - Updated doxygen links (#70)
  - More Readme Updates (#72)
  - More Readme (#74)
  - Created new sm from sm_respira_1 (#76)
  - Feature/core and navigation fixes (#78)
  - Several core improvements during navigation testing
  - Progress in aws navigation demo
  - Feature/aws demo progress (#80)
  - Format improvements
  - Feature/aws demo progress (#80)
  - sm_advanced_recovery_1 reworked (#83)
  - More sm_advanced_recovery_1 work (#85)
  - sm_advanced_recovery_1 round 4 (#86)
  - sm_atomic_performance_test_a_2
  - sm_atomic_performance_test_a_1
  - sm_atomic_performance_test_c_1 (#88)
  - Modifying sm_atomic_performance_test_a_2 (#89)
  - sm_multi_stage_1
  - Update README.md
  - Wait topic message client behavior (#81)

```rst
Section_2
=========

Added
-----

- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.
- Navigation parameters fixes on sm_dance_bot.
- CB pause slam client behavior.
- Sm_dance_bot_lite.
- Sm_dance_bot visualizing turtlebot3.
- Dance bot launch gz lidar choice.
- Gazebo fixes to show the robot and the lidar.
- Sm_multi_stage_1 doubling.
- Sm dance bot strikes back gazebo fixes.

Changed
-------

- Several core improvements during navigation testing.

Fixed
-----

- Corrected all linters and formatters.
- Removed some compile warnings.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_3
=========

Added
-----
- Added AWS demo (#108, #109)
- Added Brettpac branch (#110, #111)
- Added a3 (#113)
- Added diverse improvements in navigation and performance (#116)
- Added feature to toggle SLAM and deep history in SMACC (#122)
- Added SM Atomic SM generator (#143)
- Added rolling Docker environment execution (#154)
- Added initial state machine transition timestamp (#165)
- Added QOS durability to SmaccPublisherClient (#163)
- Added testing for MoveIt behaviors (#167)
- Added sm_pubsub_1 (#169, #170)

Changed
-------
- Improved gazebo visualization for robot and lidar
- Improved formatting
- Improved functionality for sm_multi_stage_1 (#109, #114)
- Improved navigation and performance
- Refactored SM dance bot strikes back (#152)
- Updated package list (#142)
- Renamed navigation 2 stack
- Updated READMEs with SVGs
- Updated launch command in README.md
- Updated format for CI compatibility
- Updated Dockerfile for building local tests
- Updated readme (#164)
- Moved reference library SMs to smacc2_performance_tools
- Added QOS durability to SmaccPublisherClient

Fixed
-----
- Fixed recursion issue in method call order (#126)
- Fixed overshot issue in waypoints navigator (#133)
- Fixed format for CI Python version (#148)
- Fixed node creation in logger (#149)
- Fixed launch command in README.md
- Fixed linting warnings
- Fixed errors in moveit migration
- Fixed compiling issues
- Fixed broken master build
```

```rst
Section_4
=========

Added
-----

- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency.
  - Added husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Fixed formatting.
  - Updated dependencies for husky in rolling and galactic.
  - Added progress on aws navigation and refactorings on navigation clients and behaviors.
  - Added more on aws demo.
  - Fixed broken build.

- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2.
  - Made minor changes.

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added format changes.
  - Added more changes and headless.
  - Merged changes.
  - Added headless and other fixes.
  - Added default values.
  - Made minor changes.

Changed
-------

- sm_advanced_recovery_1 renaming (#171)
  - Renamed sm_advanced_recovery_1.

- SrConditional fixes and formatting (#168)
  - Fixed formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Linted code.

- Feature/undo motion 20 12 (#196)
  - Made minor changes.
  - Improved undo motion navigation in warehouse2.

- Feature/sync 21 12 (#199)
  - Made minor changes.
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

- Feature/planner changes 16 12 (#191)
  - Made minor changes.
  - Added more fixes.

- Feature/replanning 16 dec (#193)
  - Made minor changes.
  - Replanned for all examples.

- Feature/cb pure spinning (#188), Feature/cb pure spinning (#189)
  - Made minor changes.
  - Added pure spinning behavior files.
  - Made minor adjustments.

- Feature/undo motion 20 12 (#198)
  - Made minor changes.
  - Replanned for all examples.
  - Improved undo motion navigation in warehouse2.
  - Made minor adjustments.

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

Fixed
-----

- Several fixes (#194)
  - Made minor changes.

- Tuning warehouse3 (#197)
  - Tuned warehouse3.

- Fixing warehouse 3 problems, and other core improvements (#204)
  - Fixed warehouse 3 problems.
  - Made core improvements to remove deadlocks and ensure continuous integration is green.
  - Fixed weird moveit not downloaded repo.
  - Added missing file from warehouse2 (#205).

Removed
-------

- Removed trailing spaces.
- Removed unnecessary packages.
- Removed disabled packages.
- Removed further packages.
- Removed disabled cpplint and cppcheck linters.
- Removed trailing spaces.
- Removed further packages.
- Removed disabled packages.
- Removed further packages.
- Removed disabled cpplint and cppcheck linters.
- Removed trailing spaces.
- Removed further packages.
- Removed disabled packages.
- Removed further packages.
- Removed disabled cpplint and cppcheck linters.
- Removed trailing spaces.
- Removed further packages.
- Removed disabled packages.
- Removed further packages.
- Removed disabled cpplint and cppcheck linters.
- Removed trailing spaces.
- Removed further packages.
- Removed disabled packages.
- Removed further packages.
- Removed disabled cpplint and cppcheck linters.
- Removed trailing spaces.
- Removed further packages.
- Removed disabled packages.
- Removed further packages.
- Removed disabled cpplint and cppcheck linters.
- Removed trailing spaces.
- Removed further packages.
- Removed disabled packages.
- Removed further packages.
- Removed disabled cpplint and cppcheck linters.
- Removed trailing spaces.
- Removed further packages.
- Removed disabled packages.
- Removed further packages.
- Removed disabled cpplint and cppcheck linters.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section 5
=========

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
- More changes on performance tests.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Feature/aws demo progress (#80).
- Feature/wait nav2 nodes client behavior (#82).
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.

Changed
-------

- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.

Fixed
-----

- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Clean up of sm_atomic_24hr.
- Clean up of sm_advanced_recovery_1.
- Minor formatting improvements.

Removed
-------

- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.

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

- New client behavior for nav2: Now waits for nav2 nodes to subscribe to the /bond topic and confirms they are active. Users can select specific nodes to wait for.
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite`:
  - Visualizing turtlebot3.
  - Precommit updates.
- `sm_multi_stage_1` doubling.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Changed
-------

- Corrected all linters and formatters.
- Several core improvements during navigation testing.
- Formatting enhancements.

Fixed
-----

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

## Section_7

### Added
- Added source build on PR for testing.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added smacc2::deep_history syntax.
- Added slam pausing/resuming functionality to sm_dance_bot.
- Added first working version of sm template and template generator.
- Added SM core test.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Added reliability qos config.
- Added repo dependency for AWS navigation sm dance bot.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.

### Changed
- Changed method order to prevent recursion in sm_dance_bot.
- Changed "Finnaly" to "Finally" for correct spelling.
- Changed launch command in README.md for sm_dance_bot_strikes_back.
- Changed format for CI to fix python version.
- Changed node creation to create only a logger.
- Changed Docker environment to be executed from any environment.
- Changed state machine transition timestamp to initial state machine transition timestamp.
- Changed reference library SMs to smacc2_performance_tools.

### Fixed
- Fixed minor format issues.
- Fixed overshot issue cases in waypoints navigator.
- Fixed launch command in README.md.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.

### Removed
- Removed neo_simulation2 package.
- Removed merge markers from a python file.
- Removed parameters smacc.
- Removed sm_dance_bot_msgs.
- Removed test from main moveit cmake.

### Miscellaneous
- Minor tweaks.
- Minor navigation improvements.
- Minor format adjustments.
- Minor linting and formatting.
- Minor tuning to mitigate overshot issue cases.
- Minor configuration adjustments.
- Precommit cleanup.
- Workflow updates.
- Pending references.
- Noticed typo correction.
- Noticed launch command correction.
- Noticed dependency addition.
- Noticed dependency to ur5 client addition.
- Noticed line removal.
- Noticed README updates.
- Noticed repos dependency addition.
- Noticed Dockerfile improvements for building local tests.
- Noticed Docker refactoring.
- Noticed progress on move_it PR.
- Noticed progress in the moveit migration testing.
- Noticed progress on moveit behaviors testing.
- Noticed progress on markers cleanup.
- Noticed progress in the sm_dance_bot tests.
- Noticed progress in navigation.
- Noticed progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Noticed slight waypoint 4 and iterations changes for robot course completion.
- Noticed missing dependency addition.
- Noticed fixing some errors introduced on formatting.
- Noticed fixing some more linting warnings.
- Noticed fixing compiling issues.
- Noticed fixing some build errors.
- Noticed fixing broken master build.
- Noticed fixing pipeline error.
- Noticed fixing overshot issue cases in waypoints navigator.
- Noticed fixing launch command in README.md.
- Noticed fixing compiling issues.
- Noticed fixing broken master build.
- Noticed fixing pipeline error.

```rst
Section_8
=========

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
- Finetuning waypoints (#187).
- Feature/cb pure spinning (#188).
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
- Add mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies (#220).

Changed
-------
- Fixing broken build.
- Minor changes (#175).
- Fix: some formatting and templating on SrConditional.
- Move trigger logic into headers.
- Lint.
- Fix: some formatting and templating on SrConditional.
- Odom tracker improvements.
- Adding forward behavior retry functionality.
- Removing warnings (#213).
- Correct codespell.
- Correct python linters warnings.
- Fix trailing spaces.
- Rename header files and correct format.
- Change extension of imports.
- Correct formatting of python file.

Removed
-------
- Weird moveit not downloaded repo.
- Disable disabled packages.

Co-authored-by
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

## Section_9

### Added
- Created doxygen-deploy.yml for documentation deployment.
- Implemented manual deployment temporarily.
- Added workflow for testing prerelease builds.
- Set `docs/` as both source folder and output directory for documentation.
- Renamed to `smacc2` and `smacc2_msgs`.
- Added setupTracing.sh script for configuring tracing group.
- Introduced `smacc2_performance_tools` for performance testing improvements.
- Added new feature `cb_wait_topic_message` for asynchronous client behavior.

### Changed
- Updated GitHub branch reference.
- Updated package name and `package.xml` to pass liter.
- Replaced occurrences of "smacc application" with "SMACC2 library".
- Edited `smacc_sm_reference_library/sm_atomic/README.md` from HTML to Markdown syntax.
- Renamed tracing events.
- Updated mentions of `SMACC/ROS` to `SMACC2/ROS2`.
- Updated `smacc2_rta` command across READMEs.
- Optimized dependencies in `move_base_z_planners_common`.
- Renamed event generator library.

### Fixed
- Corrected trailing spaces.
- Fixed bug in `smacc2` component.
- Fixed source CI and corrected README overview.
- Fixed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.
- Fixed pre-commit issues.

### Removed
- Removed manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`.
- Removed galactic builds, keeping only rolling.
- Removed submodules and used `.repos` file for dependencies.

### Miscellaneous
- Various cleanups and formatting improvements.
- Created alternative `ManualTracing`.
- Opened new folder for additional tracing contents.
- Deleted tracing directory and moved `tracing.md` to tracing directory.
- Reactivated `smacc2` nav clients for rolling via submodules.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Progress made in AWS navigation demo.
- Worked on `sm_advanced_recovery_1` and `sm_multi_stage_1`.
- Added Dockerfile for Rolling and Galactic.
- Updated `smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh`.
- Updated `tracing/ManualTracing.md`.
- Updated `smacc2_sm_reference_library/sm_atomic/README.md`.
- Updated `README.md`.
- Updated `c_cpp_properties.json`.
- Updated doxygen links.

---

*Todos los cambios realizados mantienen la información original y han sido organizados para una mejor legibilidad por Pablo Iñigo Blasco (pabloinigoblasco).*

```rst
Section_10
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success. Co-authored by Denis Štogl.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive. You can optionally select the nodes to wait for. Co-authored by Denis Štogl.

Changed
-------
- Corrected all linters and formatters.

Fixed
-----
- Navigation parameters fixes on sm_dance_bot.

Removed
-------
- Removed some compile warnings.

Other
-----
- Precommit fixes.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- Merge and progress.
- Minor hotfix.
- Cleaning and lidar show/hide option.
- Cleaning files and making formatting work.
- More fixes.
- Gazebo fixes to show the robot and the lidar.
- Precommit cleanup run.
- Updates yaml.
``` 

*pabloinigoblasco*

```rst
Section_11
==========

Added
-----

- Added support for AWS demo.
- Implemented formatting improvements.
- Implemented initial stage 1 functionality (#109).
- Added Brettpac branch (#110).
- Added progress in sm_multi_stage_1 development.
- Added support for two stages and a 3-part process.
- Added 4th and 5th stages (#111).
- Introduced a3 functionality (#113).
- Added diverse improvements in navigation and performance (#116).
- Implemented source build testing on PR.
- Adjusted build packages for source CI.
- Added more functionality to sm_multi_stage_1 (#114).
- Added support for sm_dance_bot refinement (#131) and refinement 2 (#132).
- Added build fixes and resolved waypoints navigator bug (#133).
- Improved navigation and mitigated overshot issues.
- Made progress in sm_dance_bot tests (#135).
- Added SM core test (#138).
- Added minor navigation improvements (#141).
- Added support for using local action messages (#139).
- Renamed navigation 2 stack and added SVGs to READMEs (#144).
- Updated package list and removed parameters from smacc (#147).
- Fixed launch command in README.md and CI formatting (#148).
- Added SM Atomic SM generator (#143).
- Enabled Docker environment execution from any environment (#154).
- Refactored sm_dance_bot strikes back (#152).
- Made slight changes to waypoints and iterations for course completion (#155).
- Migrated moveit client to smacc2 (#151).
- Added dependencies and fixed linting warnings.
- Updated readme (#164).
- Added initial state machine transition timestamp (#165).
- Moved reference library SMs to smacc2_performance_tools.
- Added QOS durability to SmaccPublisherClient (#163).
- Added reliability QOS configuration.
- Added testing for moveit behaviors (#167).
- Added support for sm_pubsub_1 (#169) and part 2 (#170).
- Renamed sm_advanced_recovery_1 (#171).

Changed
-------

- Corrected formatting in various sections.
- Improved progress and traction in development.
- Made minor format adjustments.
- Updated format and dependencies for moveit migration.
- Improved Dockerfile for local test building.
- Updated readme files.
- Refactored Docker environment setup.
- Improved compilation process.
- Updated format for moveit testing.
- Fixed pipeline and build errors.

Removed
-------

- Removed neo_simulation2 package.
- Removed merge markers from a python file.
- Removed node creation and created only a logger.
- Removed test from main moveit cmake.
- Removed some comments from README.md.
- Removed sm_dance_bot_msgs package.
- Removed parameters from smacc.
- Removed line in reliability QOS configuration.

Fixed
-----

- Fixed recursion possibility by moving method after the method it calls.
- Fixed overshot issue cases in navigation.
- Fixed broken master build.
- Fixed compiling issues.
- Fixed broken pipeline error.
- Fixed CI formatting for Python version.
```

```rst
Section_12
==========

Added
~~~~~
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
~~~~~~~
- Finishing touches 1 (#172)
  - Reworked sm_multi_stage_1.
  - Improved multistage modes.
  - Enhanced sm_multi_stage sequences.
  - Updated sm_multi_state_1 steps.
  - Modified sm_multi_stage_1 sequence d.
  - Adjusted sm_multi_stage_1 c sequence.
  - Updated mode_5_sequence_b and mode_4_sequence_b.
  - Improved sm_multi_stage_1 most.

Fixed
~~~~~
- Warehouse2 (#177)
  - Fixed minor issues.
- Waypoint Inputs (#178)
  - Resolved formatting problems.
- SrConditional fixes and formatting (#168)
  - Fixed formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Linted code.

Removed
~~~~~~~
- Sm_dance_bot_warehouse_3 (#181)
  - Removed redundant waypoints.

Other Changes
~~~~~~~~~~~~~
- Feature/cb pure spinning (#188) and (#189)
  - Made format adjustments.
  - Implemented headless changes.
  - Merged changes.
  - Set default values.
  - Addressed minor issues.
- Feature/planner changes 16 12 (#191)
  - Made minor adjustments.
  - Fixed issues.
- Feature/replanning 16 dec (#193)
  - Made minor changes.
  - Improved replanning for all examples.
- Feature/undo motion 20 12 (#196) and (#198)
  - Made minor changes.
  - Improved undo motion navigation for warehouse2.
  - Tuned warehouse3.
- Feature/sync 21 12 (#199)
  - Made minor changes.
  - Resolved format issues.
- Feature/warehouse2 22 12 (#200) and 23 12 (#201)
  - Made minor changes.
  - Improved replanning for all examples.
  - Resolved format issues.
  - Finished warehouse2.
- Feature/minor tune (#203)
  - Tuned and fixed issues.
- Feature/undo motion 20 12 (#198)
  - Tuned undo motion and fixed errors.
- Feature/sync 21 12 (#199)
  - Resolved format issues.
- Feature/warehouse2 22 12 (#200)
  - Resolved format issues.
- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed issues.

Contributors
~~~~~~~~~~~~
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
Section_13
==========

Added
-----
- Satisfy ament_lint_cmake.
- Add missing licenses.
- Enable cppcheck.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Add smacc2_performance_tools.
- Performance tests improvements.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files.
- Fix source CI and correct README overview.
- Update c_cpp_properties.json.
- Update doxygen links.
- More Readme Updates.
- Update smacc2_rta command across readmes.
- Update changelogs.

Changed
-------
- Correct formatters.
- Change extension of imports.
- Update ci-build-source.yml.
- Change extension.
- Correct formatting of python file.
- Rename header files and correct format.
- Rename to smacc2 and smacc2_msgs.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Ignore all packages except smacc2 and smacc2_msgs.
- Update description table.
- Update table.
- Copy initial docs.
- Dockerfile w/ ROS distro as argument.
- Opened new folder for additional tracing contents.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Changed wording "smacc application" to "SMACC2 library".
- Update smacc_sm_reference_library/sm_atomic/README.md.
- Reactivating smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Bug in smacc2 component.
- Reverted markdowns to html.
- Added README tutorial for Dockerfile.
- Additional cleanup.
- Cleanup.
- Edited tracing.md to reflect new tracing event names.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Base for the sm_aws_aarehouse navigation.
- Progressing in AWS navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- More on navigation.
- Sm_advanced_recovery_1 reworked.
- Fix pre-commit.
- Trying to fix Pre-Commit.
- More sm_advanced_recovery_1 work.
- Sm_atomic_performance_test_a_2.
- Sm_atomic_performance_test_a_1.
- Sm_atomic_performance_test_c_1.
- Modifying sm_atomic_performance_test_a_2.

Removed
-------
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Revert "Ignore all packages except smacc2 and smacc2_msgs". This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh. Location of sh file assumed if user follows README.md under "Getting started".

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

```rst
Section_14
==========

Added
~~~~~
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #91, #92, #93, #94, #95, #98)
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- New client behavior: cb_pause_slam (#98)
- New client behavior: sm_dance_bot_lite (#99)
- Option to show/hide lidar in sm_dance_bot visualizing turtlebot3 (#102)

Changed
~~~~~~~
- Updated launch command in README.md
- Corrected all linters and formatters (#82)
- Merged and progressed in navigation (#94)
- Fixed navigation parameters on sm_dance_bot (#95)
- Minor hotfix in doxygen deployment workflow (#100)
- Cleaning and formatting improvements in dance bot launch gz lidar choice (#102)

Fixed
~~~~
- Fixed compile warnings (#96)
``` 

*pabloinigoblasco*

Section_15
==========

Added
-----
- Feature/sm dance bot lite gazebo fixes (#104)
  - Visualizing turtlebot3 for sm_dance_bot
  - Added option to show/hide lidar
  - Improved formatting and cleaned files
  - Implemented gazebo fixes to display robot and lidar

- Feature/sm_multi_stage_1 doubling (#103)
  - Implemented improvements for sm_multi_stage_1

- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Visualizing turtlebot3 for sm_dance_bot
  - Added option to show/hide lidar
  - Improved formatting and cleaned files
  - Implemented gazebo fixes for sm_dance_bot_strikes_back

- Precommit cleanup run (#106)
  - General cleanup and formatting

- AWS demo (#108)
  - Added AWS demo functionality

- Brettpac branch (#110)
  - Improved functionality for sm_multi_stage_1
  - Progress in multiple stages

- Remove neo_simulation2 package. (#112)
  - Removed neo_simulation2 package
  - Corrected formatting and adjusted build packages

- More sm_multi_stage_1 (#114)
  - Further improvements for sm_multi_stage_1

- MM (#115)
  - Minor improvements and optimizations

- Diverse improvements navigation and performance (#116)
  - Implemented diverse improvements for navigation and performance

- Feature/diverse improvements navigation performance (#117)
  - Implemented diverse improvements for navigation and performance
  - Added linting and formatting enhancements

- Remove merge markers from a python file. (#119)
  - Removed merge markers from a Python file

- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introduced smacc2::deep_history syntax
  - Added functionality for testing sm_dance_bot with slam pausing/resuming

- Move method after the method it calls. Otherwise recursion could happen. (#126)
  - Implemented method reordering to prevent recursion

- Feature/dance bot s pattern (#128)
  - Polished sm_dance_bot and s-pattern
  - Corrected typo

- First working version of sm template and template generator. (#127)
  - Implemented initial version of sm template and generator

- Feature/sm dance bot refine (#131)
  - Continued improvements for sm_dance_bot

- Feature/sm dance bot refine 2 (#132)
  - Continued improvements for sm_dance_bot
  - Fixed build issues

- Waypoints navigator bug (#133)
  - Tuned navigation to mitigate overshot issues

- Progress in the sm_dance_bot tests (#135)
  - Made progress in cleaning up markers

- Minor format issues (#134)
  - Fixed minor formatting issues

- Sm_dance_bot_lite (#136)
  - Implemented lite version of sm_dance_bot

- Resolve compile warnings (#137)
  - Resolved compile warnings

- Add SM core test (#138)
  - Added SM core test functionality

- Minor navigation improvements (#141)
  - Implemented minor navigation improvements

- Using local action msgs (#139)
  - Implemented usage of local action messages
  - Removed sm_dance_bot_msgs

- Feature/nav2z renaming (#144)
  - Renamed navigation 2 stack
  - Added formatting improvements

- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
  - Added SVGs to READMEs

- Added remaining SVGs to READMEs (#145)
  - Added remaining SVGs to READMEs

- Update package list. (#142)
  - Updated package list

- Removing parameters smacc (#147)
  - Removed parameters from smacc
  - Updated workflows

- Fix CI: format fix python version (#148)
  - Fixed CI formatting for Python version

- Add SM Atomic SM generator. (#143)
  - Added SM Atomic SM generator

- Rolling Docker environment to be executed from any environment (#154)
  - Implemented Docker environment for universal execution

- Feature/sm dance bot strikes back refactoring (#152)
  - Refactored sm_dance_bot_strikes_back

- Slight waypoint 4 and iterations changes so robot can complete course (#155)
  - Made slight adjustments for robot course completion

- Feature/migration moveit client (#151)
  - Migrated to smacc2
  - Fixed formatting errors and dependencies
  - Progressed in moveit migration testing
  - Updated readme and dependencies
  - Refactored docker environment

- Initial state machine transition timestamp (#165)
  - Moved reference library SMs to smacc2_performance_tools

Changed
-------
- Updated "Finnaly" to "Finally" (#129)

Removed
-------
- Removed test from main moveit cmake in Feature/migration moveit client (#151)
- Removed node creation and created only a logger in Remove node creation and create only a logger. (#149)
- Removed some comments in the past in Fix CI: format fix python version (#148)

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> (multiple entries)
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com> (Diverse improvements navigation and performance (#116))

```rst
Section_16
==========

Added
-----
- Added QOS durability to SmaccPublisherClient (#163).
- Added reliability QOS configuration.
- Added multistage modes, sequences, steps, and sequences for sm_multi_stage_1.
- Added Waypoint Inputs (#178).
- Added dependencies for husky simulation.
- Added redoing waypoints and more waypoints for sm_dance_bot_warehouse_3.
- Added pure spinning behavior missing files.
- Added replanning for all examples.
- Added improvements in undo motion navigation warehouse2.
- Added tuning and fixes for warehouse3.
- Added tuning and fixes for Feature/minor tune (#203).
- Added fixing warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green.
- Added missing subscriber publisher components.
- Added improvements in smacc core for autoware demo.
- Added docker files for different revisions, warnings removal, and more testing on navigation.
- Added update file for fake hardware simulation and file for gazebo simulation.
- Added retry behavior for warehouse 1.

Changed
-------
- Changed reference library SMs to smacc2_performance_tools.

Fixed
-----
- Fixed a missing colon.
- Fixed a pipeline error.
- Fixed a broken master build.
- Fixed formatting.
- Fixed some formatting and templating on SrConditional.
- Fixed lint.
- Fixed move trigger logic into headers.
- Fixed broken build.
- Fixed format issues.

Removed
-------
- Removed a line.

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

## Section_17

### Added
- Added `ignition` file and updated repos files.
- Added `galactic CI` build due to Navigation2 issues in rolling.
- Added partial changes for `ament_cpplint`.
- Added `tf2_ros` as dependency to find include.
- Added workflow for checking doc build.
- Added `doxygen-check-build.yml`.
- Created `doxygen-deploy.yml`.
- Created workflow for testing prerelease builds.
- Created `smacc2` and `smacc2_msgs` packages.
- Added `setupTracing.sh` for automated installation.
- Created alternative `ManualTracing`.
- Added new `sm` markdowns.
- Added Dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added `smacc2_performance_tools`.
- Added performance tests improvements.
- Added `sm_respira_1` format cleanup.
- Added `sm_atomic_performance_trace_1`.
- Added `sm_atomic_24hr`.
- Added `sm_atomic_24hr` cleanup.
- Optimized dependencies in `move_base_z_planners_common`.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files.

### Changed
- Changed `ros2 launch sm_three_some sm_three_some` to `ros2 launch sm_three_some sm_three_some.launch`.
- Changed wording from "smacc application" to "SMACC2 library".
- Updated name of package and `package.xml` to pass liter.
- Updated GitHub branch reference.
- Updated description table.
- Updated table.
- Updated `smacc_sm_reference_library/sm_atomic/README.md` to markdown syntax.
- Updated `smacc2_rta` command across readmes.
- Cleaned up `sm_atomic_24hr`.
- Renamed folders, deleted `tracing.md`, edited `README.md`.
- Renamed tracing events.
- Renamed `sm_reference_library` reformatting.
- Renamed `sm_respira_1` launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.
- Updated doxygen links.
- More updates in README.
- Created new `sm` from `sm_respira_1`.
- Progress in AWS navigation demo.

### Fixed
- Fixed broken source build (#227).
- Fixed version of rosdep yaml for Focal-Rolling builds (#234).
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed formatting of python file.

### Removed
- Removed manual installation of `ros-rolling-ros2trace`.
- Removed disabled packages.
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used `.repos` file.
- Ignored all packages except `smacc2` and `smacc2_msgs`.
- Do not execute clang-format on `smacc2_sm_reference_library` package.

### Miscellaneous
- Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Reset all versions to 0.0.0.
- Updated changelogs.
- Updated CI build source file.
- Changed extension of imports.
- Enabled `cppcheck`.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- More changes on performance tests.
- More on performance and other issues.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- Format improvements in AWS navigation demo.
- Cleanup in AWS navigation demo.
- Additional cleanup.

Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```rst
Section_18
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Nodes to wait can be optionally selected.
- Merge and progress in aws navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Remove some compile warnings (#96).
- Feature: cb_pause_slam (#98).

Changed
-------
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Attempting pre-commit fixes.
- Correct all linters and formatters.

Fixed
-----
- Formatting improvements.
- Fix pre-commit.

Removed
-------
- Minor format.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_19
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior `add` for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive; optional node selection
- Progress in AWS navigation demo
- Gazebo fixes to show the robot and lidar
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others
- Added remaining SVGs to READMEs
- Rolling Docker environment to be executed from any environment

Changed
-------
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components
- Refactored `sm_dance_bot_strikes_back`
- Minor navigation improvements
- Removed `neo_simulation2` package
- Adjusted build packages of source CI
- Corrected formatting
- Fixed launch command for `sm_dance_bot_strikes_back` in README.md

Fixed
-----
- Navigation parameters fixes on `sm_dance_bot`
- Minor format issues
- Minor hotfix
- Minor format improvements
- Format fixes
- Minor tweaks

Removed
-------
- Removed `neo_simulation2` package
- Removed parameters from `smacc`

Authors
-------
- Pablo Iñigo Blasco
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by: DecDury <declandury@gmail.com>
- Co-authored by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored by: pabloinigoblasco <pablo@ibrobotics.com>
```

# Section 20

## Added
- Added feature/migration moveit client (#151)
- Added initial migration to smacc2
- Added repos dependency to ur5 client
- Added docker refactoring
- Added progress on move_it PR
- Added pre-commit cleanup
- Added QOS durability to SmaccPublisherClient (#163)
- Added feature/aws navigation sm dance bot (#174)
- Added dependencies for husky simulation in sm_dance_bot
- Added warehouse2 progress (#179)
- Added waypoint inputs (#178)
- Added sm_dance_bot_warehouse_3
- Added redoing sm_dance_bot_warehouse_3 waypoints
- Added more waypoints to sm_dance_bot_warehouse_3
- Added SrConditional fixes and formatting (#168)
- Added feature/cb pure spinning (#188, #189)
- Added feature/planner changes 16 12 (#191)
- Added feature/replanning 16 dec (#193)
- Added feature/undo motion 20 12 (#196, #198)
- Added feature/sync 21 12 (#199)
- Added feature/warehouse2 22 12 (#200)
- Added feature/warehouse2 23 12 (#201)
- Added feature/minor tune (#203)
- Added fixing warehouse 3 problems and other core improvements (#204)
- Added backport to foxy
- Added missing file from warehouse2 (#205)

## Changed
- Changed waypoint 4 and iterations to allow robot to complete course (#155)
- Changed formatting errors
- Changed linting warnings
- Changed format updates
- Changed reliability QOS config
- Changed husky launch file in sm_dance_bot
- Changed dependencies update for husky in rolling and galactic
- Changed progress on aws navigation and refactorings on navigation clients and behaviors
- Changed finetuning waypoints (#187)
- Changed tuning warehouse3 (#197)
- Changed tuning and fixes (#202)
- Changed minor tune in feature/minor tune (#203)

## Fixed
- Fixed some errors introduced on formatting
- Fixed missing dependency
- Fixed some build errors
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed broken build
- Fixed pure spinning behavior missing files
- Fixed format issues
- Fixed minor linking errors in foxy
- Fixed missing subscriber publisher components
- Fixed progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Fixed refining cp subscriber cp publisher

## Removed
- Removed test from main moveit cmake

## Authors
- Pablo Iñigo Blasco

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>

```rst
Section_21
==========

Added
-----
- New components developed for Autoware demo in smacc core.
- Docker build files for different revisions, warnings removal, and enhanced navigation testing.
- Barrel demo progress with fixes in warehouse 3 startup problems.
- Feature branch "barrel" with instructions for necessary package installation.
- Workflow for checking documentation build and testing prerelease builds.
- Setup script for tracing group configuration in setupTracing.sh.
- Alternative manual tracing method.
- SMACC2 library markdowns and Dockerfiles for Rolling and Galactic.
- README tutorial for Dockerfile setup.
- SMACC2 performance tools and performance tests improvements.
- Renaming of event generator library.
- Optimized dependencies in move_base_z_planners_common.

Changed
-------
- Renamed "sm_three_some" launch command.
- Updated references from "SMACC/ROS" to "SMACC2/ROS2".
- Cleaned up formatting and trailing spaces in various files.
- Reverted markdowns to HTML temporarily.
- Reactivated SMACC2 nav clients for Rolling.
- Updated SMACC2 RTA command across readmes.
- Cleaned up sm_atomic_24hr package.
- Updated doxygen links.

Fixed
-----
- Fixed minor broken builds and broken build in barrel demo.
- Fixed format issues and linking errors in Foxy backport.
- Fixed trailing spaces, codespell, and Python linters warnings.
- Fixed build of missing Rolling repositories and Navigation2 issues.
- Fixed bug in smacc2 component.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
- Removed galactic builds from master, keeping only Rolling.
- Removed submodules and used .repos file for builds.

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
Section_22
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
```

```rst
Section_23
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior `add` for `nav2`, which waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `aws_demo`.
- `Brettpac` branch.
- `neo_simulation2` package removal.
- `mm` improvements.
- Diverse improvements in navigation and performance.
- `slam_toggle` client behaviors and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- `dance_bot_s_pattern`.
- First working version of `sm` template and template generator.
- `waypoints_navigator` bug minor tuning.
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others.

Changed
~~~~~~~
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Formatting improvements.
- Gazebo fixes to show the robot and the lidar.
- Minor format tweaks.
- Adjusted build packages of source CI.
- Minor navigation improvements.
- Using local action messages.

Fixed
~~~~
- Remove some compile warnings.
- Correct formatting.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.
- Noticed typo corrected (`Finnaly` to `Finally`).
- Minor format issues resolved.

Removed
~~~~~~~
- `neo_simulation2` package.
- `sm_dance_bot_msgs` removal.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

Section_24
===========

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
- Added CB pure spinning (#188).
- Added planner changes 16 12 (#191).
- Added replanning 16 dec (#193).
- Added undo motion 20 12 (#196).
- Added sync 21 12 (#199).
- Added warehouse2 22 12 (#200).
- Added warehouse2 23 12 (#201).
- Added minor tune (#203).

Changed
-------
- Updated package list (#142).
- Fixed launch command for sm_dance_bot_strikes_back and removed outdated comments.
- Fixed CI: format fix python version (#148).
- Removed node creation and created only a logger (#149).
- Rolling Docker environment can now be executed from any environment (#154).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Reworked sm_multi_stage_1 (#172).
- Redoing sm_dance_bot_warehouse_3 waypoints (#184).
- Finetuned waypoints (#187).
- Tuning warehouse3 (#197).
- Tuning and fixes (#202).
- Fixed warehouse 3 problems and other core improvements (#204).

Removed
-------
- Removed parameters smacc (#147).
- Removed test from main moveit cmake.
- Removed some linting warnings.
- Removed compiling issues.

Fixed
-----
- Noticed launch command was incorrect in README.md.
- Fixed some errors introduced on formatting.
- Fixed missing dependency.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed broken build.

Collaborators
-------------
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.

```rst
Section_25
==========

Added
-----
- Added missing file from warehouse2 (#205).
- Added docker files for different revisions, warnings removal, and more testing on navigation.
- Added barrel demo.
- Added multiple controllable LEDs plugin.
- Added progress in Husky demo.
- Added improvements in navigation behaviors.
- Added progress in barrel Husky.
- Added progress in autoware machine.
- Added progress in SMACC core, including more components developed for autoware demo.
- Added autoware demo.
- Added progress in barrel search updates.
- Added making models local.
- Added red pickup.
- Added progress in barrel Husky.
- Added progress in SMACC2 performance tools.
- Added a dockerfile for Rolling and Galactic.

Changed
-------
- Changed `ros2 launch sm_three_some sm_three_some` to `ros2 launch sm_three_some sm_three_some.launch`.
- Changed wording "smacc application" to "SMACC2 library".
- Changed extension of imports.
- Changed formatting of python file.

Fixed
-----
- Fixed warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green.
- Fixed weird moveit not downloaded repo.
- Fixed minor broken build.
- Fixed some reordering fixes.
- Fixed formatting and minor issues.
- Fixed startup problems in warehouse 3.
- Fixed format and minor issues.
- Fixed bug in SMACC2 component.
- Fixed missing rolling repositories build.
- Fixed Navigation2 for semi-binary build.
- Fixed galactic builds from master and kept only rolling, removed submodules and used .repos file.
- Fixed some progress on navigation rolling.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Backport
--------
- Backported to Foxy.
- Backported to Foxy.

Co-Authored-By
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

Section_26
-----------

### Added
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success. Also added a new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for. (#82, #92)
- Added navigation parameters fixes on sm_dance_bot. (#93)

### Changed
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`. (#69)
- Updated `smacc2_rta` command across readmes.
- Corrected all linters and formatters.

### Fixed
- Fixed source CI and corrected README overview. (#62)

### Removed
- Do not execute clang-format on `smacc2_sm_reference_library` package.

### Miscellaneous
- Performance tests improvements.
- More changes on performance tests.
- Corrected trailing spaces.
- Renamed event generator library.
- Minor formatting improvements.
- Progress in AWS navigation demo.
- Several core improvements during navigation testing.
- Optimized dependencies in `move_base_z_planners_common`.
- Updated `c_cpp_properties.json`.
- Attempted pre-commit fixes.

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_27
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive, with optional node selection
- New gazebo fixes for `sm_dance_bot_strikes_back`

Changed
-------
- Progress in AWS navigation demo
- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Cleaning and lidar show/hide option in `sm_dance_bot_visualizing_turtlebot3`
- Refinement in `sm_dance_bot` and `s-pattern`
- First working version of `sm_template` and template generator

Fixed
-----
- Remove some compile warnings
- Correct formatting in `neo_simulation2` package removal
- Enable source build on PR for testing
- Adjust build packages of source CI
- Move method after the method it calls to prevent recursion

Removed
-------
- Removed `neo_simulation2` package

Other
-----
- Several core improvements during navigation testing
- Formatting improvements
- Merge and progress in AWS navigation
- Base for the `sm_aws_warehouse` navigation
- Progress in AWS navigation demo
- Precommit cleanup run
- Updates yaml
- More on navigation
- Cleaning files and making formatting work
- More fixes in various components
- `sm_multi_stage_1` doubling
- Gaining traction in `sm_multi_stage_1`
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components
- Introducing slam pausing/resuming functionality in `sm_dance_bot`
- Noticed typo correction
- `smacc2::deep_history` syntax implementation
- Going forward in testing `sm_dance_bot` with slam functionality
- Hammering through stages in development
- Diverse improvements in navigation and performance
- Additional linting and formatting
- Remove merge markers from a Python file
- Finally > Finally
- Minor tweaks

Commits
-------
- Feature/sm dance bot fixes (#95)
- Feature/cb pause slam (#98)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- Feature/dance bot s pattern (#128)
- Feature/dance bot s pattern (#129)

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_28
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
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
  - Formatting
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing some errors introduced on formatting
  - Missing dependency
  - Progressing in the moveit migration testing
  - Updating format
  - Adding .reps dependencies and also fixing some build errors
  - Repos dependency
  - Adding dependency to ur5 client
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
  - Progress on aws navigation and some other refactorings on navigation clients and behaviors
  - More on aws demo
- Minor changes (#175)
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Wharehouse2 progress (#179)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
  - Redoing sm_dance_bot_warehouse_3 waypoints
  - More Waypoints
- SrConditional fixes and formatting (#168)
  - Fix: some formatting and templating on SrConditional
  - Fix: move trigger logic into headers
  - Fix: lint
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)
  - Pure spinning behavior missing files
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
  - Replanning for all our examples
- Several fixes (#194)
- Minor changes (#195)

Changed
-------

- Resolve compile warnings (#137)
- Using local action msgs
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Improving dockerfile for building local tests
- Fixing compiling issues
- Update readme (#164)
- More readme updates

Fixed
-----

- Build fix
- Minor format issues (#134)
- Fixing broken master build
- Fixing broken build
- Several fixes (#194)

Removed
-------

- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
```

Section_29
==========

Added
-----
- Feature/undo motion 20 12 (#196)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Added missing file from warehouse2 (#205)
- Use correct upstream .repos files for source builds (#243)
- Correct mergify branch names (#246)
- Update galactic source build job name (#250)
- Galactic source build: update .repos file, bump action version and use correct version of upstream packages (backport #241) (#248)
- Restoring workflow files (#252)
- Restoring files (#253)
- Fix checkout branches for scheduled builds (#254)
- Update README.md (#262)
- Update README.md (#266)
- Update README.md (#267)
- Update README.md (#268)
- Significant update in Getting Started Instructions (#269)
- Added changelogs.
- 0.4.0
- Fix foxy source build config to use repos file from foxy branch. (#285)

Changed
-------
- Minor changes in various features
- Replanning for all examples in multiple features
- Improving undo motion navigation in warehouse2
- Tuning warehouse3 (#197)
- Tuning and fixes in various features
- Fixing warehouse 3 problems, and other core improvements (#204)
- Fixing rolling build (#239)
- Fixing to focal by the moment in rolling build
- More fixing rolling build
- Cache matrix rolling and source build package
- Fixing building issue
- Fixing broken build
- Fix: initialise conditionFlag as false (#274)
- Precommit fix (#280)
- Fix urls to index.ros.org (#284)
- Fix foxy source build config to use repos file from foxy branch. (#285)

Fixed
-----
- Fixing warehouse 3 startup problems
- Fixing format and minor issues
- Fixing broken build in husky project
- Fixing type string walker threesome demo

Removed
-------
- Ignored packages which should not be released.

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

```rst
Section_30
==========

0.3.0 (2022-04-04)
------------------
- Reverted "Ignore packages which should not be released." (commit dec14a936a877b2ef722a6a32f1bf3df09312542)
- Contributors: Denis Štogl, Pablo Iñigo Blasco

0.0.0 (2022-11-09)
------------------
Added
~~~~~
- Feature/galactic rolling merge (#288)
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs" (commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61)
  - Updated description table
  - Updated table
  - Copied initial docs
  - Dockerfile with ROS distro as argument
    - Command: "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  - Opened new folder for additional tracing contents
  - Deleted tracing directory
  - Moved tracing.md to tracing directory
  - Added setupTracing.sh to install necessary packages and configure tracing group
  - Automated installation of ros-rolling-ros2trace in setupTracing.sh
  - Created alternative ManualTracing
  - Added new sm markdowns
  - Added a dockerfile for Rolling and Galactic
  - Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  - Updated tracing/ManualTracing.md
  - Changed wording "smacc application" to "SMACC2 library"
  - Updated smacc_sm_reference_library/sm_atomic/README.md to markdown syntax
  - Reactivated smacc2 nav clients for rolling via submodules
  - Renamed tracing events
  - Fixed bug in smacc2 component
  - Added README tutorial for Dockerfile
  - Enable build of missing rolling repositories
  - Enable Navigation2 for semi-binary build
  - Removed galactic builds from master and kept only rolling, removed submodules and used .repos file
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - More on performance and other issues
  - Format cleanup for sm_respira_1
  - Format cleanup for sm_respira_1 pre-commit
  - Added sm_respira_test_2
  - More changes on performance tests
  - Do not execute clang-format on smacc2_sm_reference_library package
  - Reformatting of sm_reference_library
  - Corrected trailing spaces
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
  - Progressing in AWS navigation
  - Several core improvements during navigation testing
  - Formatting improvements
  - Progress in AWS navigation demo
  - Feature/AWS demo progress (#80)
  - More on navigation
  - Reworked sm_advanced_recovery_1 (#83)
  - More sm_advanced_recovery_1 work (#85)
  - Round 4 of sm_advanced_recovery_1 (#86)
  - Brettpac branch (#87)
  - Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1
  - Added sm_atomic_performance_test_c_1 (#88)
  - Modified sm_atomic_performance_test_a_2 (#89)
  - Added sm_multi_stage_1
  - Fixed precommit for sm_multi_stage_1 (#91)
  - Updated README.md with launch command
  - Wait topic message client behavior (#81)
  - New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Attempting precommit fixes
```

```rst
Section_31
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection.
- New feature: cb_pause_slam client behavior.

Changed
-------
- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor formatting improvements.
- Merge and progress in various features.
- Cleaning and lidar show/hide option in sm_dance_bot visualizing turtlebot3.
- Gazebo fixes to show the robot and the lidar in various features.

Fixed
-----
- Removed some compile warnings.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

Section_32
===========

Added
-----

- Added Brettpac branch (#110).
- Added a3 (#113).
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
- Added Resolve compile warnings (#137).
- Added Add SM core test (#138).
- Added minor navigation improvements (#141).
- Added using local action msgs (#139).
- Added Feature/nav2z renaming (#144).
- Added added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added added remaining SVGs to READMEs (#145).
- Added Update package list. (#142).
- Added Remove node creation and create only a logger. (#149).
- Added Rolling Docker environment to be executed from any environment (#154).
- Added slight waypoint 4 and iterations changes so robot can complete course (#155).
- Added Feature/migration moveit client (#151).
- Added initial state machine transition timestamp (#165).
- Added moved reference library SMs to smacc2_performance_tools (#166).
- Added Add QOS durability to SmaccPublisherClient (#163).
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
- Fixed "Finnaly" to "Finally" in Feature/dance bot s pattern (#129).
- Fixed launch command in README.md for sm_dance_bot_strikes_back and removed some comments (#148).
- Fixed CI format in Fix CI: format fix python version (#148).
- Fixed some errors introduced on formatting in Feature/migration moveit client (#151).
- Fixed missing dependency in Feature/migration moveit client (#151).
- Fixed some linting warnings in Feature/migration moveit client (#151).
- Fixed compiling issues in Feature/migration moveit client (#151).
- Updated format in Feature/migration moveit client (#151).
- Added .reps dependencies and fixed some build errors in Feature/migration moveit client (#151).
- Added dependency to ur5 client in Feature/migration moveit client (#151).
- Improved dockerfile for building local tests in Feature/migration moveit client (#151).
- Added reliability qos config in Add QOS durability to SmaccPublisherClient (#163).
- Added qos durability to SmaccPublisherClient in Add QOS durability to SmaccPublisherClient (#163).

Removed
-------

- Removed neo_simulation2 package in Remove neo_simulation2 package. (#112).
- Removed parameters smacc in removing parameters smacc (#147).
- Removed sm_dance_bot_msgs in using local action msgs (#139).
- Removed pending references in Feature/nav2z renaming (#144).

Fixed
-----

- Fixed recursion issue by moving method after the method it calls in Move method after the method it calls. Otherwise recursion could happen. (#126).
- Fixed overshot issue cases in waypoints navigator bug (#133).
- Fixed format issues in minor format issues (#134).
- Fixed broken master build in Feature/testing moveit behaviors (#167).
- Fixed pipeline error in Feature/testing moveit behaviors (#167).

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> in multiple entries.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com> in diverse improvements navigation and performance (#116).
- Co-authored-by: DecDury <declandury@gmail.com> in Feature/sm dance bot strikes back refactoring (#152).
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com> in Feature/sm dance bot strikes back refactoring (#152).

```rst
Section_33
==========

Added
-----
- Feature/aws navigation sm dance bot (#174)
- Added repo dependency for husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added progress on aws navigation and refactorings on navigation clients and behaviors.
- Added more on aws demo.
- Added fixing broken build.
- Added warehouse2 (#177).
- Added Waypoint Inputs (#178).
- Added wharehouse2 progress (#179).
- Added sm_dance_bot_warehouse_3 (#181).
- Added Brettpac branch (#184).
- Added redoing sm_dance_bot_warehouse_3 waypoints and more waypoints.
- Added SrConditional fixes and formatting (#168).
- Added fix for some formatting and templating on SrConditional.
- Added move trigger logic into headers.
- Added lint fix.
- Added warehouse2 feature (#185).
- Added finetuning waypoints (#187).
- Added pure spinning behavior missing files.
- Added planner changes 16 12 (#191).
- Added replanning for all examples.
- Added several fixes (#194).
- Added undo motion 20 12 (#196).
- Added improving undo motion navigation warehouse2.
- Added tuning warehouse3 (#197).
- Added undo tuning and errors.
- Added sync 21 12 (#199).
- Added warehouse2 22 12 (#200).
- Added finishing warehouse2.
- Added warehouse2 23 12 (#201).
- Added tuning and fixes (#202).
- Added minor tune (#203).
- Added fixing warehouse 3 problems and other core improvements (#204).
- Added added missing file from warehouse2 (#205).
- Added merging code from backport foxy and updates about autoware (#208).
- Added backport to foxy.
- Added bump ccache version.
- Added satisfy ament_lint_cmake.
- Added add missing licences.
- Added branching example.
- Added update ci-build-source.yml.
- Added change extension of imports.
- Added enable cppcheck.
- Added correct formatting of python file.
- Added necessary package and edited Threesome launch.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Fixed
-----
- Fixed formatting in several places.

Removed
-------
- Removed tracing directory.
- Removed initial docs.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_34
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
- Renamed tracing events.
- Renamed folders.
- Renamed tracing events after.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Updated README.md.
- Edited tracing.md to reflect new tracing event names.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Fixed
-----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.

Removed
-------
- Manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_35
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior added for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and waits for them to be alive. Optional node selection available.
- `cb_pause_slam` client behavior added.
- `sm_dance_bot_lite` feature added for visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling feature added.
- `sm_dance_bot_strikes_back` feature added for gazebo fixes.
- `aws demo` feature added.
- `Brettpac branch` feature added, focusing on `sm_multi_stage_1`.
- `a3` feature added.
- `Remove neo_simulation2 package` feature added.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor hotfixes.
- Cleaning and lidar show/hide option added for `sm_dance_bot`.
- Gazebo fixes for showing the robot and the lidar.
- Formatting fixes.

Fixed
-----
- Removed some compile warnings.

Removed
-------
- `neo_simulation2` package removed.

Other
-----
- Various core improvements during navigation testing.
- Progress in AWS navigation.
- Formatting improvements.
- Merge and progress.
- Precommit cleanup run.
- Updates to YAML files.
- `sm_multi_stage_1` working progress.
- `sm_multi_stage_1` gaining traction.
- Continuous progress on multiple stages.
```

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
- First working version of sm template and template generator (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
  - Build fix
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Progressing in the moveit migration testing
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Sm_pubsub_1 (#169)
- Sm_pubsub_1 part 2 (#170)
- Sm_advanced_recovery_1 renaming (#171)
- Sm_multi_stage_1 reworking (#172)
  - Multistage modes, sequences, steps, and finishing touches
- Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and refactorings on navigation clients and behaviors

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Removing parameters smacc (#147)
- Update package list (#142)
- Remove node creation and create only a logger (#149)
- Warehouse2 progress (#179)

### Fixed
- Minor tuning to mitigate overshot issue cases (#133)
- Fix CI: format fix python version (#148)
- Fixing broken master build (#167)
- Fixing broken build (#174)

### Removed
- Remove merge markers from a python file (#119)
- Removing sm_dance_bot_msgs (#144)
- Removing test from main moveit cmake (#151)
- Removing parameters smacc (#147)

### Miscellaneous
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update README (#164)
- More readme updates (#164)
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back, and removed some comments (#147)
- Precommit cleanup
- Workflows update
- Pending references
- Repos dependency
- Docker refactoring
- Minor format issues (#134)
- Minor tweaks (#130)
- Minor (#124)
- Minor
- Noticed typo
- Finnaly > Finally

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_37
==========

Added
-----
- Feature/sm warehouse 2 13 dec 2 (#182)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/undo motion 20 12 (#196)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/replanning 16 dec (#193)
- Feature/planner changes 16 12 (#191)
- Feature/cb pure spinning (#189)
- Add mergify rules file. (#209)
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use manual deployment for now.
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update
- Reset all versions to 0.0.0

Changed
-------
- SrConditional fixes and formatting (#168)
- finetuning waypoints (#187)
- tuning warehouse3 (#197)
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green (#204)
- odom tracker improvements and adding forward behavior retry functionality (#213)

Fixed
-----
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Remove example things from Foxy CI setup. (#214)
- Fix rolling builds (#222)
- Fixing CI for rolling. (#209)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy CI setup. (#214)
- Remove example things from Foxy

```rst
Section_38
==========

Added
-----
- Initial docs copied.
- Dockerfile with ROS distro as argument.
- New folder for additional tracing contents opened.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Dockerfile for Rolling and Galactic added.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More changes on performance tests.
- sm_respira_1 format cleanup.
- sm_respira_test_2 added.
- Do not execute clang-format on smacc2_sm_reference_library package.
- sm_reference_library reformatting.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Add galactic CI setup and rename rolling files.
- Fix source CI and correct README overview.
- More Readme Updates.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Progress in aws navigation.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Feature/aws demo progress.
- More on navigation.
- sm_advanced_recovery_1 reworked.
- More sm_advanced_recovery_1 work.
- sm_atomic_performance_test_a_2 added.
- sm_atomic_performance_test_a_1 added.
- sm_atomic_performance_test_c_1 added.
- sm_multi_stage_1 added.
- Wait topic message client behavior added.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Feature/wait nav2 nodes client behavior added.

Changed
-------
- Wording changed from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Minor formatting changes.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More on performance and other issues.
- More on navigation.
- Attempted precommit fixes.

Removed
-------
- Manual installation of ros-rolling-ros2trace.
- Galatic builds from master, keeping only rolling.
- Submodules and use .repos file instead.
- Tracing.md file deleted.

Fixed
-----
- Bug in smacc2 component.
- Reverted markdowns to html.
- Edited tracing.md to reflect new tracing event names.
- Formatting improvements.
- Pre-commit fixes.
```

```rst
Section_39
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection.
- `cb_pause_slam` client behavior.

Changed
-------
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.
- `sm_multi_stage_1` doubling.

Fixed
-----
- Corrected all linters and formatters.
- Removed some compile warnings.

Removed
-------
- Removed redundant format improvements.

Contributors
------------
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
- sm_dance_bot_lite (#99)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- precommit cleanup run (#106)
- aws demo (#108)
- Brettpac branch (#110)
```

```rst
Section_40
==========

Added
-----

- Brettpac branch (#111)
- a3 (#113)
- Remove neo_simulation2 package. (#112)
- Feature/diverse improvements navigation performance (#117)
- Remove merge markers from a python file. (#119)
- Feature/slam toggle and smacc deep history (#122)
- Move method after the method it calls. Otherwise recursion could happen. (#126)
- First working version of sm template and template generator. (#127)
- Feature/dance bot s pattern (#128)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- Update package list. (#142)
- Remove node creation and create only a logger. (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
- initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)

Changed
-------

- Adjust build packages of source CI
- progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax
- more changes in sm_dance_bot
- polishing sm_dance_bot and s-pattern
- progress in the sm_dance_bot tests (#135)
- some more progress on markers cleanup
- formatting
- using local action msgs
- navigation 2 stack renaming
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- precommit cleanup
- fixing some errors introduced on formatting
- missing dependency
- fixing some more linting warnings
- removing test from main moveit cmake
- test ur5
- progressing in the moveit migration testing
- updating format
- adding .reps dependencies and also fixing some build errors
- repos dependency
- adding dependency to ur5 client
- docker refactoring
- improving dockerfile for building local tests
- fixing compiling issues
- update readme (#164)
- more readme updates
- feat: add qos durability to SmaccPublisherClient
- fix: add a missing colon
- refactor: remove line
- feat: add reliability qos config
- fixing pipeline error
- fixing broken master build

Fixed
-----

- Correct formatting.
- Noticed typo
- Finnaly > Finally
- minor format
- minor tweaks (#130)
- minor navigation improvements (#141)
- minor tuning to mitigate overshot issue cases
- minor format issues (#134)
- minor
- build fix
- workflows update
- workflow
- Noticed launch command was incorrect in README.md
- fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
- Fix CI: format fix python version (#148)
```

```rst
Section_41
==========

Added
-----
- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency for husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Added progress on aws navigation and refactorings on navigation clients and behaviors.
  - Added more on aws demo.
  - Added warehouse2 (#177).
  - Added Waypoint Inputs (#178).
  - Added warehouse2 progress (#179).
  - Added sm_dance_bot_warehouse_3 (#181).
  - Added finetuning waypoints (#187).
  - Added pure spinning behavior missing files.
  - Added undo motion navigation improvements for warehouse2.
  - Added tuning and fixes for warehouse2 (#202).
  - Added minor tune for Feature/minor tune (#203).
  - Added fixing warehouse 3 problems and other core improvements (#204).
  - Added backport to foxy for missing files.
  - Added updating subscriber publisher components.
  - Added progress in autoware machine.
  - Added refining cp subscriber cp publisher.
  - Added improvements in smacc core for autoware demo.
  - Added docker files for different revisions and testing on navigation.
  - Added fixing docker for foxy and galactic.

Changed
-------
- Updated dependencies for husky in rolling and galactic.
- Updated formatting.
- Updated default values.
- Updated trigger logic into headers.
- Updated format issues.
- Updated format issues for Feature/sync 21 12 (#199).
- Updated format issues for Feature/warehouse2 22 12 (#200).
- Updated tuning and fixes for Feature/warehouse2 23 12 (#201).
- Updated SM template and example code visibility.
- Updated templated to use Blackboard storage.
- Updated template to resolve global data correctly.
- Updated sm_name.hpp.

Fixed
-----
- Fixed broken build.
- Fixed broken build for foxy.
- Fixed some formatting and templating on SrConditional.
- Fixed lint issues.
- Fixed missing files from warehouse2.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed ament_cpplint issues.
- Fixed formatters.
- Fixed extension of imports.

Removed
-------
- Removed use of node in the sm performance template.
- Removed ament_cpplint.
- Removed cpplint and cppcheck linters.
- Removed disabled packages and update workflows.
- Removed further packages.
- Removed disabled packages.
- Removed some packages.
- Removed cpplint and cppcheck linters.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages.
- Removed further packages.
- Removed some packages.
- Removed disabled packages

```rst
Section_42
==========

Added
-----
- Ensure the necessary package is installed before running commands.
- Renamed header files and corrected formats.
- Added workflow for checking doc build.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Updated source and output directories for documentation.
- Renamed packages to smacc2 and smacc2_msgs.
- Corrected GitHub branch references.
- Updated package names and package.xml.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" commit.
- Updated description tables.
- Updated tables.
- Copied initial docs.
- Created Dockerfile with ROS distro as argument.
- Opened new folder for additional tracing contents.
- Deleted tracing directory and moved tracing.md.
- Added setupTracing.sh for automated installation.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added Dockerfile for Rolling and Galactic.
- Updated buildGalactic.sh script.
- Updated ManualTracing.md.
- Changed wording from "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library README.md.
- Reactivated smacc2 nav clients for Rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Reverted markdowns to html.
- Added README tutorial for Dockerfile.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds, kept only rolling, and removed submodules.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Progress on navigation rolling.
- Renamed folders, deleted tracing.md, and edited README.md.
- Added smacc2_performance_tools.
- Improved performance tests.
- More performance and other improvements.
- Cleaned up sm_respira_1 format.
- Cleaned up sm_respira_test_2.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More README updates.
- Created new sm from sm_respira_1.
- Fixed base for the sm_aws_aarehouse navigation.
- Progress in AWS navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- More format improvements.
- Reworked sm_advanced_recovery_1.
- Fixed pre-commit issues.
- More work on sm_advanced_recovery_1.
- Round 4 of sm_advanced_recovery_1.
- Created Brettpac branch.
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1.
- Added sm_atomic_performance_test_c_1.
- Modified sm_atomic_performance_test_a_2.
- Added sm_multi_stage_1.
- Fixed precommit issues in sm_multi_stage_1.
- More work on sm_multi_stage_1.
- Updated README.md with launch command changes.
- Resolved topic message client behavior.
``` 

*pabloinigoblasco*

```rst
Section_43
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior: add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive; optional node selection

Changed
-------
- Corrected all linters and formatters
- Navigation parameters fixes on sm_dance_bot
- Merge and progress
- Fix format
- Minor hotfix
- Cleaning and lidar show/hide option
- Cleaning files and making formatting work
- Gazebo fixes to show the robot and the lidar
- Format fixes
- Doubling in sm_multi_stage_1

Removed
-------
- Removed some compile warnings

Fixed
-----
- Navigation parameters fixes on sm_dance_bot
- Minor format

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

Section 44
-----------

Added
~~~~~
- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Visualizing turtlebot3 for sm_dance_bot
  - Added cleaning and lidar show/hide option
  - Implemented gazebo fixes to show the robot and lidar
  - Format fixes for gazebo
- Precommit cleanup run (#106)
- AWS demo (#108)
- Got sm_multi_stage_1 working (#109)
- Brettpac branch (#110)
- Added 5th stage for sm_multi_stage_1 (#111)
- Removed neo_simulation2 package (#112)
- More sm_multi_stage_1 changes (#114)
- MM (#115)
- Diverse improvements in navigation and performance (#116)
- Feature/diverse improvements in navigation performance (#117)
- Added slam toggle and smacc deep history feature (#122)
- Added waypoints navigator bug fix (#133)
- Added SM core test (#138)
- Added minor navigation improvements (#141)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment execution from any environment (#154)
- Feature/migration moveit client (#151)
- Added initial migration to smacc2
- Added QOS durability to SmaccPublisherClient (#163)
- Added durability and reliability QoS configuration to SmaccPublisherClient

Changed
~~~~~~~
- Moved reference library SMs to smacc2_performance_tools (#166)
- Updated package list (#142)

Fixed
~~~~~
- Fixed compilation warnings (#137)
- Fixed CI format for Python version (#148)
- Removed node creation and created only a logger (#149)
- Fixed launch command in README.md for sm_dance_bot_strikes_back
- Fixed minor format issues (#134)

Removed
~~~~~~~
- Removed neo_simulation2 package (#112)
- Removed parameters smacc (#147)

Authors
~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Pablo Iñigo Blasco

Section_45
==========

Added
-----
- Added `sm_pubsub_1` (#169), `sm_pubsub_1 part 2` (#170), `sm_advanced_recovery_1 renaming` (#171), `sm_multi_stage_1 reworking` (#172), `Feature/aws navigation sm dance bot` (#174), `warehouse2` (#177), `Waypoint Inputs` (#178), `wharehouse2 progress` (#179), `sm_dance_bot_warehouse_3` (#181), `Feature/sm warehouse 2 13 dec 2` (#182), `SrConditional fixes and formatting` (#168), `Feature/wharehouse2 dec 14` (#185), `Feature/cb pure spinning` (#188), `Feature/planner changes 16 12` (#191), `Feature/replanning 16 dec` (#193), `Feature/undo motion 20 12` (#196), `finetuning waypoints` (#187), `Feature/warehouse2 22 12` (#200), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 23 12` (#201), `Feature/minor tune` (#203), `Feature/undo motion 20 12` (#198), `Feature/sync 21 12` (#199), `Feature/warehouse2 22 12` (#200), `Feature/minor tune` (#203), `Feature/undo motion

```rst
Section_46
==========

Added
-----
- Added ignition file and updated repos files.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml for manual deployment.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh for automated installation.
- Added alternative ManualTracing.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added sm_atomic_24hr and sm_atomic_performance_trace_1.
- Added galactic CI setup and renamed rolling files.
- Added source CI and corrected README overview.
- Added more smacc2_rta commands across readmes.
- Added optimized deps in move_base_z_planners_common.
- Added renaming of event generator library.

Changed
-------
- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch".
- Changed wording from "smacc application" to "SMACC2 library".
- Changed extension of imports.
- Changed launch command to "ros2 launch sm_respira_1 sm_respira_1.launch".
- Changed formatting of python file.
- Changed name of package and package.xml to pass liter.
- Changed GitHub branch reference.
- Changed extension of header files and corrected format.
- Changed wording in various files for clarity.
- Changed folders' names and edited README.md.

Fixed
-----
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed formatting issues in various files.
- Fixed formatters.
- Fixed build of missing rolling repositories.
- Fixed Navigation2 for semi-binary build.
- Fixed some packages and updated workflows.
- Fixed some formatting issues.
- Fixed performance tests improvements.
- Fixed minor bugs in components.
- Fixed pre-commit issues.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
- Removed tracing directory.
- Removed disabled packages.
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Removed manual execution of clang-format on smacc2_sm_reference_library package.
- Removed unnecessary packages.

Other
-----
- Replanned all examples.
- Backported to foxy.
- Ignored further packages.
- Satisfied ament_lint_cmake.
- Added missing licenses.
- Disabled ament_cpplint, cpplint, and cppcheck linters.
- Disabled some packages and update workflows.
- Bumped ccache version.
- Ignored all packages except smacc2 and smacc2_msgs.
- Reset all versions to 0.0.0.
- Updated changelogs.
- Updated description table.
- Updated table.
- Updated ci-build-source.yml.
- Updated doxygen-check-build.yml.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated doxygen links.
- Updated README.md files.
- Updated tracing.md to reflect new tracing event names.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated smacc2 and smacc2_msgs to smacc2 and smacc2_msgs.
- Updated ci-build-source.yml.
- Updated workflows.
- Updated README.md under "Getting started".
- Updated smacc2_sm_reference_library package.
- Updated sm_three_some to sm_three_some.launch.
- Updated sm_respira_1.
- Updated sm_atomic_24hr.
- Updated sm_atomic_performance_trace_1.
- Updated sm_advanced_recovery_1.
- Updated sm_advanced_recovery_1.
- Updated sm_respira_1.
- Updated sm_respira_1.
- Updated sm_aws_warehouse navigation.
- Updated sm_advanced_recovery_1.
- Updated sm_advanced_recovery_1.
- Updated sm_respira_1.
- Updated sm_respira_1.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic_24hr.
- Updated sm_atomic

```rst
Section_47
==========

Added
-----

- More work on `sm_advanced_recovery_1` (#84, #85, #86, #87)
- Work on `sm_atomic_performance_test_a_1`, `sm_atomic_performance_test_a_2`, `sm_atomic_performance_test_c_1` (#88, #89)
- Development of `sm_multi_stage_1` (#90, #91)
- New feature `cb_wait_topic_message`: asynchronous client behavior waiting for a topic message and optionally checking its contents for success (#81, #82)
- Progress in AWS navigation demo and core improvements
- Adding new client behavior for Nav2, waiting for nodes subscribing to the `/bond` topic and ensuring they are alive
- Correcting all linters and formatters
- Navigation parameters fixes on `sm_dance_bot`
- Merge and progress
- Fixing compile warnings (#96)
- New feature `cb_pause_slam`

Changed
-------

- Updated launch command in README.md

Removed
-------

- Removed some compile warnings (#96)

Authors
-------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>
```

Section_48
----------

Added
~~~~~
- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.

Changed
~~~~~~~
- Minor formatting improvements.
- Navigation parameters fixes on sm_dance_bot.
- Updates yaml.
- Rename doxygen deployment workflow.
- Cleanup and lidar show/hide option in sm_dance_bot visualizing turtlebot3.
- Gazebo fixes to show the robot and lidar in sm_dance_bot visualizing turtlebot3.
- Gazebo fixes for sm_dance_bot_strikes_back.
- Got sm_multi_stage_1 working (barely).
- Brettpac branch: Gaining traction with sm_multi_stage_1.
- Diverse improvements in navigation and performance.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introducing slam pausing/resuming functionality in sm_dance_bot.
- More changes in sm_dance_bot.
- Polishing sm_dance_bot and s-pattern.
- First working version of sm template and template generator.
- Minor tweaks.
- Minor navigation improvements.
- Using local action messages.
- Pending references.
- Navigation 2 stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Rolling Docker environment to be executed from any environment.
- Slight waypoint 4 and iterations changes so the robot can complete the course.
- Initial migration to smacc2 in Feature/migration moveit client.

Fixed
~~~~
- Move method after the method it calls to prevent recursion.
- Minor tuning to mitigate overshot issue cases.
- Progress in the sm_dance_bot tests.
- Minor format issues.
- Fix CI: format fix python version.
- Remove node creation and create only a logger.

Removed
~~~~~~
- Remove neo_simulation2 package.
- Removing parameters in smacc.
- Workflows update.
- Removed some comments in the past from README.md.

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_49
==========

Added
~~~~~
- Added .reps dependencies and fixed build errors.
- Added dependency to ur5 client.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added Waypoint Inputs.
- Added default values.
- Added more Waypoints.
- Added missing files from warehouse2.
- Added missing subscriber publisher components.
- Added more components to smacc core for autoware demo.
- Added improvements in smacc core for autoware demo.
- Added more components developed for autoware demo.

Changed
~~~~~~~
- Updated format.
- Refactored docker.
- Improved dockerfile for local tests.
- Updated readme.
- Reworked SrConditional.
- Fine-tuned waypoints.
- Tuned warehouse3.
- Fixed formatting and templating on SrConditional.
- Moved trigger logic into headers.

Fixed
~~~~
- Fixed linting warnings.
- Fixed compiling issues.
- Fixed pipeline error.
- Fixed broken master build.
- Fixed broken build.
- Fixed formatting.
- Fixed errors in pure spinning behavior.
- Fixed warehouse 3 problems.
- Fixed linking errors for foxy CI.
- Fixed minor issues in broken build.

Removed
~~~~~~~
- Removed test from main moveit cmake.

Other
~~~~~
- Progressed in moveit migration testing.
- Progressed in move_it PR.
- Progressed in moveit behaviors testing.
- Progressed in aws navigation.
- Progressed in warehouse2.
- Progressed in sm_dance_bot_warehouse_3.
- Progressed in cb pure spinning.
- Progressed in planner changes.
- Progressed in replanning for examples.
- Progressed in undo motion navigation for warehouse2.
- Progressed in sync.
- Progressed in warehouse2 finishing.
- Progressed in minor tune.
- Progressed in core improvements to remove dead lock.
- Progressed in autoware machine.
- Progressed in refining cp subscriber cp publisher.
- Progressed in foxy CI.
- Progressed in reordering fixes.
```

*pabloinigoblasco*

```rst
Section_50
==========

Added
-----
- Docker build files for all versions.
- Barrel demo.
- Barrel search build fix and warehouse3.
- Progress in barrel husky.
- Feature/barrel - do not merge yet (#233).
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Workflow for checking doc build.
- Workflow for testing prerelease builds.
- Use manual deployment for now.
- Create workflow for testing prerelease builds.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- SM_RESPIRA_1 format cleanup.
- SM_RESPIRA_TEST_2.
- Do not execute clang-format on smacc2_sm_reference_library package.
- SM_REFERENCE_LIBRARY reformatting.
- SM_ATOMIC_24HR.
- SM_ATOMIC_PERFORMANCE_TRACE_1.
- Clean up of sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Renaming of event generator library.
- Fix source CI and correct README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).

Changed
-------
- ROS2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Wording "smacc application" to "SMACC2 library".

Fixed
-----
- Fixing Docker for Foxy and Galactic.
- Fixing startup problems in warehouse 3.
- Fixing broken build.
- Fix trailing spaces.
- Correct codespell.
- Correct Python linters warnings.
- Fixing format and minor.
- Minor formatting fixes.
- Correct formatters.
- Bug in smacc2 component.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Ignore further packages.
- Ignore all packages except smacc2 and smacc2_msgs.
- Revert "Ignore all packages except smacc2 and smacc2_msgs".

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

- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- More sm_advanced_recovery_1 work (#85)
- Brettpac branch (#87)
- sm_atomic_performance_test_c_1 (#88)
- sm_multi_stage_1 (#90)
- Wait topic message client behavior (#81)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Changed
-------

- Progress in aws navigation
- Several core improvements during navigation testing
- Formatting improvements
- Progress in aws navigation demo
- More on navigation
- Correct all linters and formaters
- Navigation parameters fixes on sm_dance_bot
- Merge and progress
- Fix format

Fixed
-----

- Fix pre-commit
- Trying to fix Pre-Commit
- Modifying sm_atomic_performance_test_a_2
- Fixing precommit

Removed
-------

- Minor

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
``` 

*pabloinigoblasco*

```rst
Section_52
==========

Added
-----
- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are alive. Optional node selection available.
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally checks its contents for success.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in AWS navigation.
- Several core improvements during navigation testing.
- Gazebo fixes to show the robot and lidar.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also `smacc2::deep_history` syntax.
- Progress in testing `sm_dance_bot` introducing slam pausing/resuming functionality.
- Polishing `sm_dance_bot` and S-pattern.

Fixed
-----
- Removed some compile warnings. (#96)
- Minor hotfix.
- Minor format issues.
- Minor navigation improvements.
- Minor tuning to mitigate overshot issue cases.
- Minor format issues.
- Minor tweaks.
- Build fix.

Removed
-------
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs`.
- Removed parameters from `smacc`.

Other
-----
- Base for the `sm_aws_warehouse` navigation.
- Precommit cleanup run.
- Updates YAML.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- More on navigation.
- Cleaning and lidar show/hide option.
- Cleaning files and making formatting work.
- More fixes.
- More refinement in `sm_dance_bot`.
- First working version of `sm` template and template generator.
- Noticed typo.
- Finally > Finally.
- Pending references.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Precommit cleanup.
- Update package list.
``` 

*pabloinigoblasco*

```rst
Section_53
==========

Added
-----

- Add SM Atomic SM generator. (#143)
- Add SM Atomic SM generator for creating state machines. Co-authored-by: DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>

- Add QOS durability to SmaccPublisherClient (#163)
- Add Quality of Service (QOS) durability configuration to SmaccPublisherClient for improved message reliability.

Changed
-------

- Update workflows for smoother operation.
- Update workflows to enhance performance.

- Refactor feature "sm dance bot strikes back" (#152)
- Refactor feature "sm dance bot strikes back" for improved functionality. Co-authored-by: DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>

- Update readme (#164)
- Update readme file with corrected information.

- Move reference library SMs to smacc2_performance_tools (#166)
- Move reference library State Machines to smacc2_performance_tools for better organization.

Fixed
-----

- Fix launch command in README.md (#148)
- Correct launch command for sm_dance_bot_strikes_back in README.md and remove unnecessary comments.

- Fix CI: format fix python version (#148)
- Fix Continuous Integration (CI) formatting issue related to Python version.

- Fix node creation in logger (#149)
- Correct node creation process to only create a logger for improved efficiency.

- Fix compiling issues in moveit migration (#151)
- Resolve compiling issues introduced during migration to smacc2.

- Fix broken build in aws navigation (#174)
- Fix broken build related to AWS navigation feature.

- Fix formatting in warehouse2 progress (#179)
- Correct formatting issues in warehouse2 progress.

- Fix formatting in SrConditional (#168)
- Address formatting and templating issues in SrConditional feature.

Removed
-------

- Remove unnecessary test from main moveit cmake.
- Remove unnecessary test from main moveit cmake configuration.

- Remove redundant node creation steps.
- Remove redundant steps related to node creation.

- Remove unnecessary dependencies in dockerfile.
- Remove unnecessary dependencies in dockerfile configuration.

- Remove redundant progress updates on moveit migration.
- Remove redundant progress updates related to moveit migration testing.

- Remove redundant changes in warehouse3 tuning.
- Remove redundant changes in warehouse3 tuning process.

- Remove redundant replanning for all examples.
- Remove redundant replanning steps for all examples.

- Remove redundant minor tune feature.
- Remove redundant minor tune adjustments.

- Remove redundant fixes in warehouse3 problems.
- Remove redundant fixes in warehouse3 problems and other core improvements.

- Remove redundant format issues in feature sync.
- Remove redundant format issues in feature sync.

- Remove redundant finishing steps in warehouse2 feature.
- Remove redundant finishing steps in warehouse2 feature.

- Remove redundant tuning and fixes in warehouse2 feature.
- Remove redundant tuning and fixes in warehouse2 feature.

- Remove redundant backport to foxy.
- Remove redundant backport to foxy version.

- Remove redundant minor format adjustments.
- Remove redundant minor format adjustments.

- Remove redundant weird moveit not downloaded repo.
- Remove redundant weird moveit not downloaded repository.

- Remove redundant added missing file from warehouse2.
- Remove redundant added missing file from warehouse2 feature.

- Remove redundant minor changes in undo motion feature.
- Remove redundant minor changes in undo motion feature.

- Remove redundant undo tuning and errors.
- Remove redundant undo tuning and errors.

- Remove redundant default values in feature cb pure spinning.
- Remove redundant default values in feature cb pure spinning.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature planner changes.
- Remove redundant default values in feature planner changes.

- Remove redundant default values in feature replanning.
- Remove redundant default values in feature replanning.

- Remove redundant default values in feature undo motion.
- Remove redundant default values in feature undo motion.

- Remove redundant default values in feature sync.
- Remove redundant default values in feature sync.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature added missing file from warehouse2.
- Remove redundant default values in feature added missing file from warehouse2.

- Remove redundant default values in feature backport to foxy.
- Remove redundant default values in feature backport to foxy.

- Remove redundant default values in feature minor format.
- Remove redundant default values in feature minor format.

- Remove redundant default values in feature weird moveit not downloaded repo.
- Remove redundant default values in feature weird moveit not downloaded repo.

- Remove redundant default values in feature undo tuning and errors.
- Remove redundant default values in feature undo tuning and errors.

- Remove redundant default values in feature format.
- Remove redundant default values in feature format.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor changes.
- Remove redundant default values in feature minor changes.

- Remove redundant default values in feature tuning and fixes.
- Remove redundant default values in feature tuning and fixes.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature added missing file from warehouse2.
- Remove redundant default values in feature added missing file from warehouse2.

- Remove redundant default values in feature backport to foxy.
- Remove redundant default values in feature backport to foxy.

- Remove redundant default values in feature minor format.
- Remove redundant default values in feature minor format.

- Remove redundant default values in feature weird moveit not downloaded repo.
- Remove redundant default values in feature weird moveit not downloaded repo.

- Remove redundant default values in feature undo tuning and errors.
- Remove redundant default values in feature undo tuning and errors.

- Remove redundant default values in feature format.
- Remove redundant default values in feature format.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor changes.
- Remove redundant default values in feature minor changes.

- Remove redundant default values in feature tuning and fixes.
- Remove redundant default values in feature tuning and fixes.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature added missing file from warehouse2.
- Remove redundant default values in feature added missing file from warehouse2.

- Remove redundant default values in feature backport to foxy.
- Remove redundant default values in feature backport to foxy.

- Remove redundant default values in feature minor format.
- Remove redundant default values in feature minor format.

- Remove redundant default values in feature weird moveit not downloaded repo.
- Remove redundant default values in feature weird moveit not downloaded repo.

- Remove redundant default values in feature undo tuning and errors.
- Remove redundant default values in feature undo tuning and errors.

- Remove redundant default values in feature format.
- Remove redundant default values in feature format.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor changes.
- Remove redundant default values in feature minor changes.

- Remove redundant default values in feature tuning and fixes.
- Remove redundant default values in feature tuning and fixes.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature added missing file from warehouse2.
- Remove redundant default values in feature added missing file from warehouse2.

- Remove redundant default values in feature backport to foxy.
- Remove redundant default values in feature backport to foxy.

- Remove redundant default values in feature minor format.
- Remove redundant default values in feature minor format.

- Remove redundant default values in feature weird moveit not downloaded repo.
- Remove redundant default values in feature weird moveit not downloaded repo.

- Remove redundant default values in feature undo tuning and errors.
- Remove redundant default values in feature undo tuning and errors.

- Remove redundant default values in feature format.
- Remove redundant default values in feature format.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor changes.
- Remove redundant default values in feature minor changes.

- Remove redundant default values in feature tuning and fixes.
- Remove redundant default values in feature tuning and fixes.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature added missing file from warehouse2.
- Remove redundant default values in feature added missing file from warehouse2.

- Remove redundant default values in feature backport to foxy.
- Remove redundant default values in feature backport to foxy.

- Remove redundant default values in feature minor format.
- Remove redundant default values in feature minor format.

- Remove redundant default values in feature weird moveit not downloaded repo.
- Remove redundant default values in feature weird moveit not downloaded repo.

- Remove redundant default values in feature undo tuning and errors.
- Remove redundant default values in feature undo tuning and errors.

- Remove redundant default values in feature format.
- Remove redundant default values in feature format.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor changes.
- Remove redundant default values in feature minor changes.

- Remove redundant default values in feature tuning and fixes.
- Remove redundant default values in feature tuning and fixes.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature added missing file from warehouse2.
- Remove redundant default values in feature added missing file from warehouse2.

- Remove redundant default values in feature backport to foxy.
- Remove redundant default values in feature backport to foxy.

- Remove redundant default values in feature minor format.
- Remove redundant default values in feature minor format.

- Remove redundant default values in feature weird moveit not downloaded repo.
- Remove redundant default values in feature weird moveit not downloaded repo.

- Remove redundant default values in feature undo tuning and errors.
- Remove redundant default values in feature undo tuning and errors.

- Remove redundant default values in feature format.
- Remove redundant default values in feature format.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor changes.
- Remove redundant default values in feature minor changes.

- Remove redundant default values in feature tuning and fixes.
- Remove redundant default values in feature tuning and fixes.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove redundant default values in feature fixing warehouse 3 problems.

- Remove redundant default values in feature added missing file from warehouse2.
- Remove redundant default values in feature added missing file from warehouse2.

- Remove redundant default values in feature backport to foxy.
- Remove redundant default values in feature backport to foxy.

- Remove redundant default values in feature minor format.
- Remove redundant default values in feature minor format.

- Remove redundant default values in feature weird moveit not downloaded repo.
- Remove redundant default values in feature weird moveit not downloaded repo.

- Remove redundant default values in feature undo tuning and errors.
- Remove redundant default values in feature undo tuning and errors.

- Remove redundant default values in feature format.
- Remove redundant default values in feature format.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature warehouse2.
- Remove redundant default values in feature warehouse2.

- Remove redundant default values in feature minor changes.
- Remove redundant default values in feature minor changes.

- Remove redundant default values in feature tuning and fixes.
- Remove redundant default values in feature tuning and fixes.

- Remove redundant default values in feature minor tune.
- Remove redundant default values in feature minor tune.

- Remove redundant default values in feature fixing warehouse 3 problems.
- Remove

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

Changed
-------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- changed wording "smacc application" to "SMACC2 library"
- updated mentions of SMACC/ROS to SMACC2/ROS2

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
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file

Removed
-------
- minor linking errors foxy
- missing
- missing sm
- autoware demo
- foxy ci
- fix
- minor broken build
- some reordering fixes
- minor
- barrel demo
- fixing format and minor
- progress in barrel husky
- barrel search updates
- making models local
- red picuup
- multiple controllable leds plugin
- progress in husky demo
- progressing in husky demo
- improving navigation behaviors
- more merge
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- fixing startup problems in warehouse 3
- some progress on navigation rolling
- renamed folders, deleted tracing.md, edited README.md
- added smacc2_performance_tools
- performance tests improvements
- more on performance and other issues
- sm_respira_1 format cleanup
- sm_respira_1 format cleanup pre-commit
- sm_respira_test_2
- more changes on performance tests

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
```

```rst
Section_55
==========

Added
~~~~~
- Add galactic CI setup and rename rolling files. (#58)
- Update c_cpp_properties.json
- Add new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait
- Wait topic message client behavior (#81)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)

Changed
~~~~~~~
- Update smacc2_rta command across readmes
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Update README.md
- Updated launch command
- Correct all linters and formaters.

Fixed
~~~~
- Correct trailing spaces.
- Fix source CI and correct README overview. (#62)
- Fix pre-commit
- Trying to fix Pre-Commit

Removed
~~~~~~~
- Do not execute clang-format on smacc2_sm_reference_library package.
- Renaming of event generator library
- Clean up of sm_atomic_24hr
- Optimized deps in move_base_z_planners_common.
- minor formatting
- minor
- several core improvements during navigation testing
- formatting improvements
- progress in aws navigation demo
- format improvements
- more on navigation
- navigation parameters fixes on sm_dance_bot
- minor format
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation
- more on navigation

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_56
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, which waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.

Changed
-------
- Minor formatting improvements.

Fixed
-----
- Format fixes.
- Navigation parameters fixes on `sm_dance_bot`.
- Remove some compile warnings.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- Gazebo fixes to show the robot and the lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Correct formatting in various places.
- Move method after the method it calls to prevent recursion.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Precommit cleanup run.
- Updates in `yaml`.
- Various core improvements during navigation testing.
- Additional linting and formatting.
- Remove merge markers from a Python file.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- First working version of `sm` template and template generator.
- Waypoints navigator bug.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_57
==========

Added
-----
- Progress in the sm_dance_bot tests (#135)
- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Husky launch file in sm_dance_bot
- Add dependencies for husky simulation
- Update dependencies for husky in rolling and galactic
- Waypoint Inputs (#178)
- More Waypoints in sm_dance_bot_warehouse_3
- Redoing sm_dance_bot_warehouse_3 waypoints
- Finetuning waypoints (#187)
- Pure spinning behavior missing files
- Replanning for all our examples
- Improving undo motion navigation warehouse2
- Tuning warehouse3 (#197)

Changed
-------
- Minor tuning to mitigate overshot issue cases
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Resolve compile warnings (#137)
- Format fix python version in CI (#148)
- Remove node creation and create only a logger (#149)
- Feature/nav2z renaming (#144)
- Feature/sm dance bot strikes back refactoring (#152)
- Feature/migration moveit client (#151)
- Feature/aws navigation sm dance bot (#174)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)

Fixed
-----
- Noticed launch command was incorrect in README.md
- Fix CI: format fix python version (#148)
- Fix formatting in warehouse2 (#177)
- Fix compiling issues
- Fix broken master build
- Fixing broken build

Removed
-------
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from

```rst
Section_58
==========

Added
-----
- Feature/sync 21 12 (#199): Added synchronization feature.
- Feature/warehouse2 22 12 (#200): Added warehouse2 feature.
- Feature/warehouse2 23 12 (#201): Added warehouse2 finishing touches.
- Feature/minor tune (#203): Added minor tuning feature.
- Added missing file from warehouse2 (#205).
- Use correct upstream .repos files for source builds (#243).
- Correct mergify branch names (#246).
- Update galactic source build job name (#250).
- Galactic source build: Updated .repos file and action version (#248).
- restoring workflow files (#252).
- restoring files (#253).
- Fix checkout branches for scheduled builds (#254).
- Feature/fixing husky build rolling (#257): Fixed husky project build.
- Feature/fixing husky build rolling (#258): Continued fixing husky project build.
- Update README.md (#262).
- Feature/fixing ur demos (#261): Fixed UR demos.
- Feature/fixing type string walker (#263): Fixed type string walker demo.
- Update README.md (#266).
- Update README.md (#267).
- Update README.md (#268).
- Significant update in Getting Started Instructions (#269).
- Fix urls to index.ros.org (#284).
- Fix foxy source build config to use repos file from foxy branch (#285).
- Added spawn entity delays.

Changed
-------
- Correct name of source-build job and bump version of action (#242) (#247).
- fixing rolling build (#239): Improved rolling build process.
- fixing to focal by the moment: Fixed issues related to Focal.
- fixing building issue: Resolved building issues.
- fixing broken build: Fixed broken build.
- precommit fix (#280): Improved precommit process.

Fixed
-----
- fixing warehouse 3 problems, and other core improvements (#204): Fixed warehouse 3 issues and made core improvements.
- fixing docker for foxy and galactic: Fixed Docker issues for Foxy and Galactic.
- barrel search build fix and warehouse3: Fixed barrel search and warehouse3 issues.
- fixing startup problems in warehouse 3: Resolved startup issues in warehouse 3.
- fixing format and minor: Fixed formatting and minor issues.
- fixing ur demo (#273): Fixed UR demo.
- fix: initialise conditionFlag as false (#274): Initialized conditionFlag as false.
- fixing sm_dance_bot examples: Fixed sm_dance_bot examples.
- working on fix of image messages for husky_barrel demo: Improved image messages for husky_barrel demo.

Removed
-------
- Revert "Ignore packages which should not be released.": Reverted package release changes.

Collaborators
-------------
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- mergify[bot] <37929162+mergify[bot]@users.noreply.github.com>
- brettpac <brettpac@users.noreply.github.com>
```

```rst
Section_59
==========

Added
-----
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Added smacc2_performance_tools.
- Performance tests improvements.
- Optimized dependencies in move_base_z_planners_common.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Base for the sm_aws_aarehouse navigation.
- Progressing in AWS navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.
- Corrected all linters and formatters.

Changed
-------
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed sm_respira_1 format cleanup to sm_respira_1 format cleanup pre-commit.
- Updated smacc2_rta command across readmes.
- Renamed event generator library.
- Minor formatting improvements.
- Updated c_cpp_properties.json.
- Updated launch command in README.md.
- Attempted precommit fixes.

Removed
-------
- Do not execute clang-format on smacc2_sm_reference_library package.

Fixed
-----
- Corrected trailing spaces.
- Cleaned up sm_atomic_24hr.
- Fixed pre-commit in sm_advanced_recovery_1.
- Fixed pre-commit in sm_atomic_performance_test_a_2.
- Fixed pre-commit in sm_multi_stage_1.
``` 

*pabloinigoblasco*

Section_60
==========

Added
-----
- New client behavior `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the `/bond` topic and wait for them to become active. Optionally select nodes to wait for.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` gazebo fixes: cleaning and lidar show/hide option.
- `sm_dance_bot_strikes_back` gazebo fixes: cleaning and lidar show/hide option.
- Progress in AWS navigation demo.
- Progress in navigation testing.
- Progress in `sm_multi_stage_1`.
- Progress in `sm_dance_bot` visualizing TurtleBot3.
- Progress in `sm_dance_bot` introducing slam pausing/resuming functionality.
- Slam toggle client behaviors and `slam_toolbox` components.
- `smacc2::deep_history` syntax.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor formatting improvements.
- Remove some compile warnings.
- Minor hotfixes.
- Correct formatting for `neo_simulation2` package removal.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Move method after the method it calls to prevent recursion.

Removed
-------
- `neo_simulation2` package.

Fixed
-----
- Various core improvements during navigation testing.

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>

```rst
Section_61
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
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Update package list. (#142)
- Update readme (#164)
- Finetuning waypoints (#187)

Fixed
-----

- Waypoints navigator bug (#133)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
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

- Noticed typo "Finnaly" corrected to "Finally"
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Minor tuning to mitigate overshot issue cases
- Some more progress on markers cleanup
- Progress in the sm_dance_bot tests (#135)
- Progress on moveit migration testing
- Progress on move_it PR
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- Warehouse2 progress (#179)
- More changes in sm_dance_bot
- More refinement in sm_dance_bot
- More testing on moveit
- More testing on moveit behaviors
- More on aws demo
- More readme updates
- More waypoints added
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints in sm_dance_bot_warehouse_3
- Slight waypoint 4 and iterations changes so robot can complete course
- Multistage modes in sm_multi_stage_1
- Sm_multi_stage sequences in sm_multi_stage_1
- Sm_multi_state_1 steps in sm_multi_stage_1
- Sm_multi_stage_1 sequence d in sm_multi_stage_1
- Sm_multi_stage_1 c sequence in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Sm_multi_stage_1 most in sm_multi_stage_1
- Finishing touches 1 in sm_multi_stage_1
- Readme updates in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
- Mode_4_sequence_b in sm_multi_stage_1
- Mode_5_sequence_b in sm_multi_stage_1
-

```rst
Section_62
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
- Foxy backport (#206)
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use manual deployment for now.
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Rename to smacc2 and smacc2_msgs
- Enable cppcheck
- Correct formatting of python file.
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Update changelogs
- Update description table.
- Update table
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- added setupTracing.sh
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Update smacc_sm_reference_library/sm_atomic/README.md
- Update smacc2_rta command across readmes
- Renaming of event generator library

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- First ensure you have the necessary package installed.
- Rename header files and correct format.
- Add workflow for checking doc build.
- Change extension of imports.
- Reset all versions to 0.0.0
- changed wording "smacc application" to "SMACC2 library"
- updated mentions of SMACC/ROS to SMACC2/ROS2

Fixed
-----
- several fixes (#194)
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements (#204)
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Correct GitHub branch reference.
- Correct formatters.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Optimized deps in move_base_z_planners_common.
- Correct trailing spaces.

Removed
-------
- Delete tracing directory
- Removed manual installation of ros-rolling-ros2trace
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages
- Ignore further packages
- Ignore all packages except smacc2 and smacc2_msgs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file

Other
-----
- more changes and headless
- merge
- headless and other fixes
- default values
- minor
- pure spinning behavior missing files
- minor changes (#190)
- minor changes
- more fixes
- replanning for all our examples
- several fixes (#194)
- minor changes (#195)
- minor
- improving undo motion navigation warehouse2
- minor
- tuning warehouse3 (#197)
- minor
- undo tuning and errors
- format
- minor changes
- format issues
- minor changes
- format issues
- finishing warehouse2
- tuning and fixes
- minor tune
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- weird moveit not downloaded repo
- added missing file from warehouse2
- minor format
- minor linking errors foxy
- minor formatting fixes
- some progress on navigation rolling
- renamed folders, deleted tracing.md, edited README.md
- performance tests improvements
- more on performance and other issues
- sm_respira_1 format cleanup
- sm_respira_1 format cleanup pre-commit
- sm_respira_test_2
- sm_respira_test_2
- more changes on performance tests
- Do not execute clang-format on smacc2_sm_reference_library package.
- sm_reference_library reformatting
- sm_atomic_24hr
- sm_atomic_performance_trace_1
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- minor formatting
- Add galactic CI setup and rename rolling files. (#58)

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_63
==========

Added
-----

- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait.

Changed
-------

- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links.

Fixed
-----

- Fixed source CI and corrected README overview.
- Fixed pre-commit issues.
- Corrected all linters and formatters.
- Fixed navigation parameters on sm_dance_bot.

Removed
-------

- Removed note that was not needed.

Other
-----

- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Formatting improvements.
- Progress in aws navigation.
- Merge and progress in development.

Contributors
------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_64
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.
- Progress in AWS navigation demo.
- Gazebo fixes for showing the robot and the lidar.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- Going forward in testing `sm_dance_bot` introducing `slam` pausing/resuming functionality.
- Polishing `sm_dance_bot` and `s-pattern`.
- More refinement in `sm_dance_bot`.
- First working version of `sm` template and template generator.

Fixed
-----

- Remove some compile warnings (#96).
- Remove `neo_simulation2` package (#112).
- Move method after the method it calls to prevent recursion (#126).
- Minor tuning to mitigate overshot issue cases in waypoints navigator (#133).
- Minor format issues (#134).

Removed
-------

- Remove `sm_dance_bot_msgs`.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_65
==========

Added
~~~~~
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174)
- Feature/testing moveit behaviors (#167)
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
~~~~~~~
- Renamed navigation 2 stack
- Updated package list. (#142)
- Fixed launch command for sm_dance_bot_strikes_back
- Refactored SM dance bot strikes back
- Moved reference library SMs to smacc2_performance_tools
- Redoing sm_dance_bot_warehouse_3 waypoints
- Finetuning waypoints (#187)
- Tuning warehouse3 (#197)

Fixed
~~~~~
- Noticed launch command was incorrect in README.md
- Fix CI: format fix python version (#148)
- Fix formatting.
- Fix compiling issues
- Fixing pipeline error
- Fixing broken master build
- Several fixes (#194)
- Tuning and fixes (#202)

Removed
~~~~~~~
- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed test from main moveit cmake

Other Changes
~~~~~~~~~~~~~
- Precommit cleanup
- Workflows update
- Docker refactoring
- Minor formatting improvements
- Minor configuration changes
- Minor changes in various files
- Update readme (#164)
- More readme updates
- Warehouse2 progress (#179)
- Format (#180)
- Format issues
- Default values
- Merge changes
- Headless and other fixes
- Pure spinning behavior missing files
- Improving undo motion navigation warehouse2
- Undo tuning and errors
- Progress on moveit migration testing
- Progress on aws navigation and other refactorings
- More on aws demo
- Progress on moveit
- More testing on moveit behaviors
- More testing on moveit
- Progressing in the moveit migration testing
- Progress on move_it PR
- Progress on warehouse2
- More changes and headless
- More on warehouse2
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
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless

Authors
~~~~~~~
- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_66
==========

Added
-----
- Added missing file from warehouse2 (#205)
- Added workflow for checking doc build
- Added galactic CI build because Navigation2 is broken in rolling
- Added tf2_ros as dependency to find include
- Added setupTracing.sh to automate installation of ros-rolling-ros2trace
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Tuning and fixes
- Minor tune
- Fixed warehouse 3 problems and other core improvements (#204)
- Fixed warehouse 3 problems and other core improvements to remove dead lock and make continuous integration green
- Updated subscriber publisher components
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Replanning for all examples
- Merging code from backport foxy and updates about autoware (#208)
- Renamed "sm_three_some" launch command
- Renamed header files and corrected format
- Renamed to smacc2 and smacc2_msgs
- Renamed tracing events
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Updated smacc2_rta command across readmes
- Cleaned up sm_atomic_24hr
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library

Fixed
-----
- Minor broken build
- Fixed trailing spaces
- Corrected codespell
- Corrected python linters warnings
- Corrected formatters
- Corrected formatting of python file
- Corrected GitHub branch reference
- Corrected wording "smacc application" to "SMACC2 library"
- Fixed bug in smacc2 component
- Enabled cppcheck
- Enabled build of missing rolling repositories
- Enabled Navigation2 for semi-binary build
- Removed galactic builds from master and kept only rolling
- Updated description table
- Updated table
- Updated name of package and package.xml to pass liter
- Updated ci-build-source.yml
- Updated doxygen-check-build.yml
- Created doxygen-deploy.yml
- Updated changelogs
- Updated tracing.md to reflect new tracing event names
- Updated smacc_sm_reference_library/sm_atomic/README.md
- Updated tracing/ManualTracing.md
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Updated smacc2_rta command across readmes
- Updated c_cpp_properties.json
- Changed extension of imports
- Updated workflows
- Updated doxygen-deploy.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml
- Updated ci-build-source.yml

Removed
-------
- Disable ament_cpplint
- Disable some packages and update workflows
- Ignore further packages
- Disable cpplint and cppcheck linters
- Disable disabled packages
- Ignore all packages except smacc2 and smacc2_msgs
- Reverted "Ignore all packages except smacc2 and smacc2_msgs"
- Do not execute clang-format on smacc2_sm_reference_library package
```

```rst
Section_67
==========

Added
-----

- Update doxygen links (#70) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More Readme Updates (#72) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More Readme (#74) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Base for the sm_aws_aarehouse navigation
- Progressing in AWS navigation
- Several core improvements during navigation testing
- Formatting improvements
- Progress in AWS navigation demo
- Feature/aws demo progress (#80)
- More on navigation
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait
- Correct all linters and formaters (#82) by Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>
- Progress in AWS navigation demo
- Navigation parameters fixes on sm_dance_bot
- Merge and progress
- Fix format

Changed
-------

- Sm_advanced_recovery_1 reworked (#83)
- Fix pre-commit
- Trying to fix Pre-Commit
- More sm_advanced_recovery_1 (#84)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Sm_multi_stage_1 (#90)
- Fixing precommit
- More sm_multi_stage_1 (#91)
- Update README.md updated launch command

Removed
-------

- Wait topic message client behavior (#81)
```

*pabloinigoblasco*

```rst
Section_68
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: waits for `nav2` nodes to subscribe to the `/bond` topic and waits for them to be alive; optional node selection
- Base for `sm_aws_warehouse` navigation
- Gazebo fixes to show the robot and the lidar
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- `Brettpac` branch
- `a3` feature
- Diverse improvements in navigation and performance
- `slam_toggle` client behaviors and `slam_toolbox` components; `smacc2::deep_history` syntax
- `sm_dance_bot` introducing slam pausing/resuming functionality
- `s-pattern` for `sm_dance_bot`
- First working version of `sm` template and template generator
- Waypoints navigator bug tuning to mitigate overshot issue cases
- `SM` core test
- Minor navigation improvements
- Renaming to `nav2z`

Changed
-------
- Progress in AWS navigation demo
- Progress in navigation testing
- Progress in `sm_dance_bot` tests
- More refinement in `sm_dance_bot`
- Polishing `sm_dance_bot` and `s-pattern`
- Minor tweaks and build fixes
- Format improvements

Fixed
-----
- Navigation parameters fixes on `sm_dance_bot`
- Remove some compile warnings
- Minor hotfix
- Correct formatting
- Additional linting and formatting
- Remove merge markers from a Python file
- Minor format issues

Removed
-------
- Remove `neo_simulation2` package
- Remove `sm_dance_bot_msgs`
- Pending references

Contributors
------------
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_69
==========

Added
-----
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added Waypoint Inputs (#178)

Changed
-------
- Updated package list (#142)
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Refactored feature/sm dance bot strikes back (#152)
- Reworked sm_multi_stage_1 (#172)
- Redoing sm_dance_bot_warehouse_3 waypoints (#184)

Fixed
-----
- Fixed CI: format fix python version (#148)
- Fixed compiling issues in moveit migration (#151)
- Fixed broken master build in moveit behaviors testing (#167)
- Fixed pipeline error in moveit behaviors testing (#167)
- Fixed broken build in AWS navigation (#174)
- Fixed formatting and templating on SrConditional (#168)
- Fixed some formatting and templating on SrConditional (#168)
- Fixed warehouse 3 problems and other core improvements (#204)

Removed
-------
- Removed parameters smacc (#147)
- Removed node creation and created only a logger (#149)
- Removed test from main moveit cmake
- Removed some comments in the past from launch command for sm_dance_bot_strikes_back

Other Changes
-------------
- Precommit cleanup
- Workflows update
- Noticed launch command was incorrect in README.md
- Rolling Docker environment to be executed from any environment (#154)
- Progress on moveit migration testing
- Progress on move_it PR
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More testing on moveit behaviors
- More on aws demo
- More readme updates
- More changes and headless in sm warehouse 2 (#182)
- More changes and headless in cb pure spinning (#188)
- More changes and headless in cb pure spinning (#189)
- More fixes in planner changes 16 12 (#191)
- More fixes in replanning 16 dec (#193)
- More fixes in several fixes (#194)
- More fixes in tuning and fixes (#202)
- Finishing warehouse2 in warehouse2 22 12 (#200)
- Finishing warehouse2 in warehouse2 23 12 (#201)
- Finetuning waypoints (#187)
- Tuning warehouse3 (#197)
- Tuning and fixes in minor tune (#203)
- Improving undo motion navigation warehouse2 in undo motion 20 12 (#196) and undo motion 20 12 (#198)
- Undo tuning and errors in undo motion 20 12 (#198)
- Format issues in sync 21 12 (#199) and warehouse2 22 12 (#200)
- Default values in sm dance bot warehouse 3 (#181) and cb pure spinning (#188) and (#189)
- Warehouse2 progress (#179)
- Warehouse2 in feature/wharehouse2 dec 14 (#185)
- Warehouse2 in feature/warehouse2 23 12 (#201)
- Warehouse2 in feature/warehouse2 22 12 (#200)
- Warehouse2 in feature/warehouse2 23 12 (#201)
- Warehouse2 in feature/minor tune (#203)
```

```rst
Section_70
==========

Added
-----

- Added missing file from warehouse2 (#205).
- Added mergify rules file.
- Added Autoware Auto Msgs into not-released dependencies. (#220).
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------

- Changed wording "smacc application" to "SMACC2 library".
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Changed extension of imports.
- Changed extension of header files and corrected format.
- Updated subscriber publisher components.
- Updated description table.
- Updated table.
- Updated name of package and package.xml to pass liter.
- Updated ci-build-source.yml.
- Updated tracing/ManualTracing.md.
- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax.
- Updated README tutorial for Dockerfile.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated changelogs.

Fixed
-----

- Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixed weird moveit not downloaded repo.
- Fixed minor broken build.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed rolling builds (#222).
- Fixed docker for foxy and galactic.
- Fixed warnings (#213).
- Fixed missing repositories build for rolling.
- Fixed Navigation2 for semi-binary build.

Removed
-------

- Removed example things from Foxy CI setup. (#214).
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed tracing directory.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed disabled packages and updated workflows.
- Removed disabled packages.
- Removed warnings.
- Removed tracing.md.

Other
-----

- Backported changes to foxy.
- Backported changes to foxy.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted markdowns to html.
- Reverted

## Section_71

### Added
- Added galactic CI setup and renamed rolling files. (#58)
- Added more Readme Updates (#72, #74)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait

### Changed
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69)
- Updated `smacc2_rta` command across readmes
- Updated `c_cpp_properties.json`
- Updated README.md with launch command
- Corrected all linters and formatters

### Fixed
- Fixed source CI and corrected README overview. (#62)
- Fixed trailing spaces
- Fixed pre-commit issues

### Removed
- Removed execution of clang-format on `smacc2_sm_reference_library` package

### Miscellaneous
- Optimized dependencies in `move_base_z_planners_common`
- Renamed event generator library
- Several core improvements during navigation testing
- Progress in AWS navigation demo
- Navigation parameters fixes on `sm_dance_bot`
- Minor formatting improvements

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_72
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive

Changed
~~~~~~~
- Navigation parameters fixes on `sm_dance_bot`
- Gazebo fixes for showing the robot and the lidar
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Introducing slam pausing/resuming functionality in `sm_dance_bot`
- Polishing `sm_dance_bot` and `s-pattern`
- Refinement in `sm_dance_bot`
- First working version of `sm` template and template generator

Fixed
~~~~
- Remove some compile warnings
- Minor hotfixes
- Correct formatting
- Adjust build packages of source CI
- Move method after the method it calls to prevent recursion
- Typo correction: "Finnaly" to "Finally"
- Minor tweaks

Removed
~~~~~~~
- Remove `neo_simulation2` package
- Remove merge markers from a Python file

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_73
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
- Feature/undo motion 20 12 (#196)

Changed
-------
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Fix CI: format fix python version (#148)
- Initial migration to smacc2
- Update readme (#164)
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----
- Resolve compile warnings (#137)
- Fixing broken master build
- Fixing pipeline error
- Fixing compiling issues
- Several fixes (#194)

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
Section_74
==========

Added
-----
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/retry behavior warehouse 1 (#226)
- dockerfiles (#225)
- Fix code generators (#221)
- Foxy backport (#206)

Changed
-------
- Improvements in undo motion navigation warehouse2
- Tuning warehouse3 (#197)
- Undo tuning and errors
- Replanning for all examples
- Format issues
- Tuning and fixes (#202)
- Fixing warehouse 3 problems and other core improvements (#204)
- Fix other build issues
- Update SM template and make example code clearly visible
- Remove use of node in the SM performance template
- Update template to use Blackboard storage
- Update template to resolve the global data correctly
- Update sm_name.hpp
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
- Add missing licenses
- Disable cpplint and cppcheck linters
- Correct formatters
- Disable disabled packages
- Update ci-build-source.yml
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- Rename header files and correct format
- Add workflow for checking doc build
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
- Update changelogs
- Update description table
- Update table
- Copy initial docs
- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch"
- Added instructions for installing necessary package
- Update tracing/ManualTracing.md
- Changed wording "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md (edited from html to markdown syntax)

Fixed
-----
- Fix trailing spaces
- Weird moveit not downloaded repo
- Minor broken build
- Some reordering fixes
- Minor linking errors foxy
- Minor format
- Minor changes
- Minor tune
- Format issues
- Missing sm
- Missing
- Fixing docker for foxy and galactic
- Delete tracing directory
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- This is now automated in setupTracing.sh
- Location of sh file assumed if user follows README.md under "Getting started"
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
``` 

*pabloinigoblasco*

```rst
Section_75
==========

Added
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.

Changed
-------
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Renamed sm_respira_1 format cleanup to sm_respira_test_2.
- Renamed sm_atomic_24hr to sm_atomic_performance_trace_1.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Corrected all linters and formatters.

Fixed
-----
- Bug in smacc2 component.
- Reverted markdowns to html.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Optimized dependencies in move_base_z_planners_common.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other Changes
-------------
- Some progress on navigation rolling.
- More changes on performance tests.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
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
-

```rst
Section_76
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2: `wait nav2 nodes subscribing to the /bond topic and waiting they are alive`. Nodes to wait can be optionally selected.
- Added `cb_pause_slam` client behavior.
- Added gazebo fixes for `sm_dance_bot_strikes_back`.

Changed
~~~~~~~
- Progress in AWS navigation demo.
- Minor navigation parameters fixes on `sm_dance_bot`.
- Formatting improvements.
- Cleaning and lidar show/hide option for `sm_dance_bot visualizing turtlebot3`.
- Updates yaml.
- Correct formatting for `neo_simulation2 package`.
- Adjusted build packages of source CI.
- Additional linting and formatting.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also `smacc2::deep_history` syntax.

Fixed
~~~~
- Removed some compile warnings.
- Removed `neo_simulation2 package`.
- Removed merge markers from a python file.

Removed
~~~~~~~
- Removed `a3`.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

## Section_77

### Added
- Introducing slam pausing/resuming functionality to `sm_dance_bot` (#125, #129)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Add QOS durability to SmaccPublisherClient (#163)

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Renamed state machine transition timestamp to `smacc2_performance_tools` (#166)
- Moved reference library SMs to `smacc2_performance_tools` (#166)
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors (#174)

### Fixed
- Minor format issues (#134)
- Fix CI: format fix python version (#148)
- Fixing some errors introduced on formatting during migration to smacc2 (#151)
- Fixing pipeline error during moveit testing (#167)
- Fixing broken master build during moveit testing (#167)

### Removed
- Removed node creation and create only a logger (#149)
- Removed parameters from `smacc` (#147)
- Removed `sm_dance_bot_msgs` (#144)

### Miscellaneous
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> (multiple commits)
- Co-authored-by: DecDury <declandury@gmail.com> (refactoring `sm_dance_bot strikes back`)
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com> (refactoring `sm_dance_bot strikes back`)
- Co-authored-by: Denis Štogl <denis@stogl.de> (AWS navigation improvements)
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com> (AWS navigation improvements)

```rst
Section_78
==========

Added
~~~~~
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/improvements warehouse3 (#228)
- Foxy backport (#206)
- Add ignition file and update repos files.
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
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update

Changed
~~~~~~~
- Only rolling version should be pre-released on on master. (#230)
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.

Fixed
~~~~~
- minor broken build
- some reordering fixes
- fixing docker for foxy and galactic
- fix broken source build (#227)
- fixing format and minor
- fixing startup problems in warehouse 3
- retry behavior warehouse 1
- minor format fix
- other minor changes

Removed
~~~~~~~
- weird moveit not downloaded repo
- missing file
- missing sm
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- missing
- foxy ci
- fix
- minor
- docker files for different revisions, warnings removval and more testing on navigation
- Update file for fake hardware simulation and add file for gazebo simulation.
- docker build files for all versions
- red picuup

Co-authored-by
~~~~~~~~~~~~~
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
Section_79
==========

Added
-----

- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Dockerfile now accepts ROS distro as argument
- Opened new folder for additional tracing contents
- Added setupTracing.sh for automated installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Renamed tracing events
- Added smacc2_performance_tools
- Performance tests improvements
- Optimized dependencies in move_base_z_planners_common
- Renaming of event generator library
- Add galactic CI setup and rename rolling files
- Fix source CI and correct README overview
- Update c_cpp_properties.json
- Update doxygen links
- More Readme Updates
- Created new sm from sm_respira_1
- Feature/core and navigation fixes
- Feature/aws demo progress
- Feature/wait nav2 nodes client behavior

Changed
-------

- Reverted "Ignore all packages except smacc2 and smacc2_msgs"
- Updated description table
- Updated table
- Changed wording "smacc application" to "SMACC2 library"
- Updated smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- Minor formatting changes
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch

Fixed
-----

- Bug in smacc2 component
- Do not execute clang-format on smacc2_sm_reference_library package
- Correct trailing spaces
- Several core improvements during navigation testing
- Attempted precommit fixes

Removed
-------

- Manual installation of ros-rolling-ros2trace
- Galactic builds from master, keeping only rolling
- Submodules, now using .repos file
- Tracing.md file
- Tracing directory
- ManualTracing.md file
- Removed tracing event names
- Removed markdowns, reverted to html
- Removed unnecessary folders
- Removed redundant files
- Removed unnecessary cleanup
- Removed unused tracing events
- Removed outdated README note
``` 

*pabloinigoblasco*

```rst
Section_80
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.

Changed
~~~~~~~
- Minor hotfixes.
- Cleaning and lidar show/hide option for `sm_dance_bot`.
- Format fixes for gazebo to show the robot and lidar.

Fixed
~~~~
- Navigation parameters fixes on `sm_dance_bot`.
- Remove some compile warnings.

Removed
~~~~~~~
- Some compile warnings.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Features
~~~~~~~~
- AWS demo progress (#92).
- SM dance bot fixes (#93).
- SM AWS warehouse (#94).
- SM dance bot fixes (#95).
- CB pause slam (#98).
- SM dance bot visualizing turtlebot3 (#101).
- Dance bot launch Gazebo lidar choice (#102).
- SM dance bot lite gazebo fixes (#104).
- SM dance bot strikes back gazebo fixes (#105).
- AWS demo (#108).
- Brettpac branch (#110).
``` 

*pabloinigoblasco*

```rst
Section_81
==========

Added
~~~~~
- Brettpac branch (#111)
- Added a3 (#113)
- Added diverse improvements in navigation and performance (#116)
- Added Feature/diverse improvements in navigation performance (#117)
- Added Feature/slam toggle and smacc deep history (#122)
- Added Feature/dance bot s pattern (#128)
- Added First working version of sm template and template generator (#127)
- Added Feature/sm dance bot refine (#131)
- Added Feature/sm dance bot refine 2 (#132)
- Added waypoints navigator bug (#133)
- Added Resolve compile warnings (#137)
- Added Add SM core test (#138)
- Added Feature/nav2z renaming (#144)
- Added Update package list (#142)
- Added Add SM Atomic SM generator (#143)
- Added Rolling Docker environment to be executed from any environment (#154)
- Added Feature/sm dance bot strikes back refactoring (#152)
- Added slight waypoint 4 and iterations changes so robot can complete course (#155)
- Added Feature/migration moveit client (#151)
- Added initial state machine transition timestamp (#165)
- Added Add QOS durability to SmaccPublisherClient (#163)
- Added Feature/testing moveit behaviors (#167)
- Added sm_pubsub_1 (#169)
- Added sm_pubsub_1 part 2 (#170)
- Added sm_advanced_recovery_1 renaming (#171)
- Added sm_multi_stage_1 reworking (#172)

Changed
~~~~~~~
- Corrected formatting in Remove neo_simulation2 package (#112)
- Enabled source build on PR for testing in Remove neo_simulation2 package (#112)
- Adjusted build packages of source CI in Remove neo_simulation2 package (#112)
- Moved method after the method it calls to prevent recursion in Move method after the method it calls (#126)
- Fixed typo in Finnaly to Finally
- Fixed launch command in README.md for sm_dance_bot_strikes_back and removed unnecessary comments
- Fixed CI format in Fix CI: format fix python version (#148)
- Removed node creation and created only a logger in Remove node creation and create only a logger (#149)
- Added reliability qos config in Add QOS durability to SmaccPublisherClient (#163)

Fixed
~~~~~
- Minor tuning to mitigate overshot issue cases in waypoints navigator bug (#133)
- Fixed compiling issues in fixing compiling issues (#164)
- Fixed broken master build in fixing broken master build (#167)
- Fixed pipeline error in fixing pipeline error (#167)

Removed
~~~~~~~
- Removed neo_simulation2 package in Remove neo_simulation2 package (#112)
- Removed merge markers from a python file in Remove merge markers from a python file (#119)
- Removed parameters smacc in removing parameters smacc (#147)
- Removed test from main moveit cmake in removing test from main moveit cmake (#151)
- Removed sm_dance_bot_msgs in removing sm_dance_bot_msgs (#144)

Authors
~~~~~~~
- Pablo Iñigo Blasco (@pabloinigoblasco)
- Brett (@brett@robosoft.ai)
- DecDury (@declandury@gmail.com)
- Denis Štogl (@destogl@users.noreply.github.com)
```

```rst
Section_82
==========

Added
-----

- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency for husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Added progress on aws navigation and refactorings on navigation clients and behaviors.
  - Added more on aws demo.

Changed
-------

- Updated dependencies for husky in rolling and galactic.
- Refactored warehouse2 (#177).
- Implemented Waypoint Inputs (#178).
- Improved sm_dance_bot_warehouse_3 (#181).
- Enhanced pure spinning behavior (#188, #189).
- Tweaked planner changes (#191).
- Enhanced undo motion navigation warehouse2 (#196, #198).
- Tuned warehouse3 (#197).
- Fine-tuned waypoints (#187).
- Improved sync (#199).
- Finalized warehouse2 (#200, #201).
- Tuned and fixed minor issues (#203).
- Fixed warehouse 3 problems and core improvements (#204).
- Added missing file from warehouse2 (#205).
- Updated subscriber publisher components.
- Refined cp subscriber cp publisher.
- Improved smacc core by adding more components for autoware demo.
- Updated docker files for different revisions.
- Fixed docker for foxy and galactic.
- Fixed barrel search build and warehouse3 startup problems.
- Progress in barrel husky.

Fixed
-----

- Fixed formatting.
- Fixed broken build issues.
- Fixed broken build in barrel demo.
- Fixed format and minor issues.
- Fixed warnings removal and more testing on navigation.
- Fixed runtime dependency issues.
- Restored ur dependency.

Removed
-------

- Removed weird moveit not downloaded repo.

Contributors
------------

- Denis Štogl
- Pablo Iñigo Blasco
```
