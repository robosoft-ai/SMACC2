Changelog for package sm_pubsub_1
==================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2 <https://github.com/robosoft-ai/SMACC2>`_
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Additional work on buildfarm issue
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
No significant changes.

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release
- Contributors: pabloinigoblasco

### Changed
- Various updates and improvements related to publisher features and documentation.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

### Fixed
- Multiple performance tests and cleanup tasks.
- Updates and corrections in various README files.
- Navigation improvements and bug fixes.
- Formatting and naming adjustments.
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

### Removed
- Redundant or outdated information.
- Unused files and directories.

### Miscellaneous
- Ongoing work on AWS navigation demo.
- Continued enhancements in core functionality.
- Pre-commit fixes and adjustments.

### Contributors
- brettpac
- pabloinigoblasco
- Denis Štogl
- Ubuntu 20-04-02-amd64

```rst
Section_2
=========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `wait nav2 nodes` subscribing to the `/bond` topic and waiting for them to be alive; optional node selection
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` gazebo fixes: cleaning, lidar show/hide option, formatting work, and more fixes
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes: cleaning, lidar show/hide option, formatting work, and more fixes
- `sm_dance_bot_visualizing_turtlebot3`
- `sm_aws_warehouse` navigation base
- Progress in AWS navigation demo
- Several core improvements during navigation testing
- Minor format improvements
- Merge and progress
- Correct all linters and formatters
- Remove some compile warnings
- Updates yaml
- Rename doxygen deployment workflow
- `sm_dance_bot` visualizing turtlebot3
- `sm_dance_bot` launch gazebo lidar choice
- `sm_dance_bot` fixes
- `sm_aws_warehouse` feature progress
- `sm_dance_bot` fixes

Changed
-------
- Formatting improvements
- Navigation parameters fixes on `sm_dance_bot`

Fixed
-----
- Attempted precommit fixes
- Fix format

Removed
-------
- Minor hotfix
- Minor format

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

Section_3
==========

Added
-----

- Added AWS demo (#108, #109)
- Added Brettpac branch (#110, #111)
- Added a3 (#113)
- Added diverse improvements in navigation and performance (#116)
- Added feature to toggle SLAM and deep history in SMACC (#122)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added sm_pubsub_1 (#169, #170)

Changed
-------

- Improved gazebo to show the robot and lidar
- Improved format
- Improved gazebo for sm_dance_bot_strikes_back
- Improved sm_multi_stage_1 functionality (#109, #114, #115)
- Improved navigation and performance
- Improved dance bot's pattern
- Improved navigation in sm_dance_bot tests (#135)
- Improved minor navigation issues
- Renamed navigation 2 stack
- Updated package list (#142)
- Refactored sm_dance_bot strikes back (#152)
- Updated readme (#164)
- Moved reference library SMs to smacc2_performance_tools
- Added reliability QOS config to SmaccPublisherClient

Fixed
-----

- Fixed neo_simulation2 package removal (#112)
- Fixed source build on PR for testing
- Fixed build packages of source CI
- Fixed waypoints navigator bug (#133)
- Fixed overshot issue cases in navigation
- Fixed format in CI for Python version (#148)
- Fixed node creation in SM Atomic SM generator
- Fixed launch command in README.md
- Fixed compiling issues

Removed
-------

- Removed neo_simulation2 package
- Removed sm_dance_bot_msgs
- Removed parameters in SMACC
- Removed test from main moveit cmake

Collaborators
-------------

- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by DecDury <declandury@gmail.com>
- Co-authored by Denis Štogl <destogl@users.noreply.github.com>
- Co-authored by pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_4
=========

Added
-----
- Feature/aws navigation sm dance bot (#174)
- Added dependencies for husky simulation.
- Added progress on aws navigation and refactorings on navigation clients and behaviors.
- Added more on aws demo.
- Added finishing touches 1.
- Added readme.

Changed
-------
- Renamed sm_advanced_recovery_1 (#171).
- Reworked sm_multi_stage_1 (#172).
- Updated dependencies for husky in rolling and galactic.
- Updated warehouse2 (#177).
- Updated Waypoint Inputs (#178).
- Updated sm_dance_bot_warehouse_3 (#181).
- Updated sm_dance_bot_warehouse_3 waypoints.
- Updated pure spinning behavior.
- Updated planner changes 16 12 (#191).
- Updated replanning for all examples.
- Improved undo motion navigation in warehouse2.
- Tuned warehouse3 (#197).
- Fixed warehouse 3 problems and other core improvements (#204).
- Merged code from backport foxy and updates about autoware (#208).
- Updated ci-build-source.yml.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Updated name of package and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".

Fixed
-----
- Fixed broken build.
- Fixed some formatting.
- Fixed some linting issues.
- Fixed some linking errors in foxy.
- Fixed trailing spaces.
- Fixed codespell and python linters warnings.
- Fixed weird moveit not downloaded repo.
- Fixed missing files in warehouse2.
- Fixed continuous integration issues.
- Fixed dead lock issues.
- Fixed minor formatting issues.

Removed
-------
- Removed unnecessary packages.
- Removed disabled packages.
- Removed trailing spaces.
- Removed some linting tools.
- Removed some unnecessary dependencies.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

```rst
Section 5
=========

Added
-----

- Opened new folder for additional tracing contents.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to automate installation of necessary packages and configure tracing group.
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
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More Readme Updates.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Feature/aws demo progress.
- Feature/wait nav2 nodes client behavior.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.

Changed
-------

- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Bug fix in smacc2 component.
- Minor formatting changes.
- Corrected trailing spaces.
- Reformatted sm_reference_library.
- Cleaned up sm_atomic_24hr.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_advanced_recovery_1.
- Fixed pre-commit issues.
- More work on sm_advanced_recovery_1.
- More work on sm_atomic_performance_test_a_2.
- More work on sm_multi_stage_1.
- Attempted pre-commit fixes.

Removed
-------

- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Deleted tracing.md.
- Removed tracing events after.
- Deleted tracing directory.

Contributors
------------

- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_6
=========

Added
-----

- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are active. Optional selection of nodes to wait for.
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Navigation parameters fixes on sm_dance_bot.
- cb_pause_slam client behavior.
- sm_dance_bot_lite gazebo fixes: Cleaning, lidar show/hide option, formatting improvements.
- gazebo fixes for sm_dance_bot_strikes_back.

Changed
-------

- Corrected all linters and formatters.
- Several core improvements during navigation testing.

Fixed
-----

- Removed some compile warnings.

Removed
-------

- Minor hotfix.

Contributors
------------

- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_7

### Added
- Added source build on PR for testing (#112).
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components (#122).
- Added smacc2::deep_history syntax for smacc2::deep_history syntax (#122).
- Added First working version of sm template and template generator (#127).
- Added SM core test (#138).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient (#163).
- Added husky launch file in sm_dance_bot for AWS navigation (#174).

### Changed
- Corrected formatting for neo_simulation2 package removal (#112).
- Adjusted build packages of source CI (#112).
- Moved method after the method it calls to prevent recursion (#126).
- Renamed sm_advanced_recovery_1 (#171).
- Reworked sm_multi_stage_1 with multistage modes and sequences (#172).
- Moved reference library SMs to smacc2_performance_tools (#166).

### Fixed
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#147).
- Fixed CI format for Python version (#148).
- Fixed compiling issues in Docker environment (#154).
- Fixed broken master build for moveit behaviors (#167).

### Removed
- Removed neo_simulation2 package (#112).
- Removed merge markers from a Python file (#119).
- Removed node creation and created only a logger (#149).
- Removed parameters smacc (#147).
- Removed sm_dance_bot_msgs (#144).

### Miscellaneous
- Minor tweaks and format adjustments (#130, #134, #136, #141, #152, #155).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Updated package list (#142).
- Updated README files (#164).
- Updated dependencies for husky in rolling and galactic (#174).
- Pre-commit cleanup for various changes.
- Added missing dependencies and fixed build errors for moveit migration (#151).
- Progressed in testing sm_dance_bot and moveit behaviors.
- Added reliability qos config for SmaccPublisherClient.
- Fixed pipeline errors and improved Dockerfile for building local tests.
- Updated format for navigation 2 stack renaming (#144).
- Added repo dependencies and fixed linting warnings for moveit migration.
- Updated format for move_it PR (#151).

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
- Several fixes (#194).
- Feature/undo motion 20 12 (#196).
- Tuning warehouse3 (#197).
- Feature/undo motion 20 12 (#198).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Fixing warehouse 3 problems and other core improvements (#204).
- Added missing file from warehouse2 (#205).
- Add mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies (#220).
- Fix rolling builds (#222).
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
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.
- Included necessary package and edited Threesome launch.

Changed
-------
- Fixing broken build.
- Minor changes (#175).
- Default values.
- Redoing sm_dance_bot_warehouse_3 waypoints.
- More waypoints.
- Replanning for all our examples.
- Improving undo motion navigation warehouse2.
- Tuning and fixes (#202).
- Refining cp subscriber cp publisher.
- Improvements in SMACC core adding more components mostly developed for Autoware demo.
- Format issues.
- Minor formatting fixes.
- Fix trailing spaces.
- Correct codespell.
- Correct Python linters warnings.
- Correct formatting of Python file.
- Rename header files and correct format.

Removed
-------
- Weird moveit not downloaded repo.
- Pure spinning behavior missing files.
- Missing SM.
- Updating subscriber publisher components.
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine.
- Minor broken build.
- Minor linking errors foxy.
- Disable disabled packages.
- Change extension.

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
Section_9
=========

Added
-----

- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Added smacc2_performance_tools
- Performance tests improvements
- More on performance and other issues
- Optimized deps in move_base_z_planners_common
- Renaming of event generator library
- Add galactic CI setup and rename rolling files
- Fix source CI and correct README overview
- Update c_cpp_properties.json
- Update README.md with new launch command
- Update doxygen links
- More Readme Updates
- Created new sm from sm_respira_1
- Feature/core and navigation fixes
- Base for the sm_aws_aarehouse navigation
- Progressing in AWS navigation
- Several core improvements during navigation testing
- Progress in AWS navigation demo
- Format improvements
- Feature/aws demo progress
- More on navigation
- Reworked sm_advanced_recovery_1
- Fix pre-commit
- More sm_advanced_recovery_1 work
- Sm_advanced_recovery_1 round 4
- Brettpac branch
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1
- Modifying sm_atomic_performance_test_a_2
- Sm_multi_stage_1
- More sm_multi_stage_1
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success

Changed
-------

- Correct GitHub branch reference
- Update description table
- Update table
- Changed wording "smacc application" to "SMACC2 library"
- Reactivating smacc2 nav clients for rolling via submodules
- Renamed tracing events after
- Bug in smacc2 component
- Reverted markdowns to HTML
- Edited tracing.md to reflect new tracing event names
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- More sm_atomic_24hr cleanup
- Correct trailing spaces
- Minor formatting

Removed
-------

- Ignore all packages except smacc2 and smacc2_msgs
- Removed manual installation of ros-rolling-ros2trace
- Delete tracing directory
- Cleanup

Fixed
-----

- Do not execute clang-format on smacc2_sm_reference_library package
- Sm_reference_library reformatting
- Fixing precommit
- Trying to fix Pre-Commit
- Wait topic message client behavior

Authors
-------

- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_10
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `wait nav2 nodes` subscribes to the `/bond` topic and waits for them to be alive. You can optionally select the nodes to wait for.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` gazebo fixes to show the robot and lidar.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.

Changed
-------

- Corrected all linters and formatters.

Fixed
-----

- Removed some compile warnings.

Removed
-------

- Minor hotfix.

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

- Added AWS demo.
- Added Brettpac branch (#110, #111).
- Added a3 (#113).
- Added diverse improvements in navigation and performance (#116).
- Added feature to toggle SLAM and SMACC deep history (#122).
- Added feature to refine SM dance bot (#131, #132).
- Added SM Atomic SM generator (#143).
- Added rolling Docker environment execution from any environment (#154).
- Added slight waypoint 4 and iterations changes for robot course completion (#155).
- Added initial state machine transition timestamp (#165).
- Added QOS durability to SmaccPublisherClient (#163).
- Added feature for testing MoveIt behaviors (#167).
- Added SM pubsub 1 (#169) and part 2 (#170).
- Renamed sm_advanced_recovery_1 (#171).

Changed
-------

- Improved formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- Made minor format tweaks.
- Updated package list.
- Refactored Dockerfile for building local tests.
- Updated READMEs with SVGs.
- Fixed launch command in README.md.
- Updated format for MoveIt migration.
- Added .reps dependencies and fixed build errors.
- Added dependency to UR5 client.
- Improved Dockerfile for building local tests.
- Added durability QOS to SmaccPublisherClient.
- Added reliability QOS config.
- Added missing colon in code.
- Moved reference library SMs to smacc2_performance_tools.

Fixed
-----

- Removed neo_simulation2 package.
- Corrected formatting.
- Mitigated overshot issue cases in navigation.
- Fixed compilation warnings.
- Fixed CI format for Python version.
- Removed node creation and created only a logger.
- Fixed compiling issues.

Removed
-------

- Removed neo_simulation2 package.
- Removed parameters from SMACC.
- Removed SM dance bot messages.

Collaborators
-------------

- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai> (#109, #112, #114, #115, #116, #124, #136, #138, #140, #142, #147, #154, #169, #170, #171).
- Co-authored by pabloinigoblasco <pablo@ibrobotics.com> (#116).
- Co-authored by DecDury <declandury@gmail.com> and Denis Štogl <destogl@users.noreply.github.com> (#152).
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
  - Updated dependencies for husky in rolling and galactic.
  - Improved AWS navigation and refactored navigation clients and behaviors.
  - Added more content for AWS demo.
  - Fixed broken build.

Changed
~~~~~~~
- Warehouse2 (#177)
  - Updated waypoint inputs.

Fixed
~~~~
- SrConditional fixes and formatting (#168)
  - Fixed formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Linted code.

Removed
~~~~~~~
- Pure spinning behavior missing files.
- Weird moveit not downloaded repo.

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
Section_13
==========

Added
-----
- Added missing licenses.
- Added workflow for checking doc build.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.

Changed
-------
- Changed extension of imports.
- Changed wording "smacc application" to "SMACC2 library".
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Fixed
-----
- Fixed formatting of python file.
- Fixed bug in smacc2 component.
- Fixed trailing spaces.
- Fixed source CI and corrected README overview.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Other
-----
- Satisfy ament_lint_cmake.
- Disable cpplint and cppcheck linters.
- Enable cppcheck.
- Updated ci-build-source.yml.
- Updated doxygen-check-build.yml and created doxygen-deploy.yml.
- Updated smacc2_rta command across readmes.
- Updated name of package and package.xml.
- Updated GitHub branch reference.
- Updated description table and table.
- Updated changelogs.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated to use docs/ as source and output directory.
- Updated all versions to 0.0.0.
- Updated tracing.md to reflect new tracing event names.
- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated c_cpp_properties.json.
- Updated doxygen links.
- Updated sm_three_some launch command to include .launch extension.
- Updated name of package to pass liter.
- Updated to build missing rolling repositories.
- Updated Navigation2 for semi-binary build.
- Updated Navigation2 for Rolling via submodules.
- Updated to ignore all packages except smacc2 and smacc2_msgs.
- Updated to remove galactic builds from master and keep only rolling, removing submodules and using .repos file.
- Updated to rename tracing events.
- Updated to optimize dependencies in move_base_z_planners_common.
- Updated to rename event generator library.
- Updated to enable build of missing rolling repositories.
- Updated to create alternative ManualTracing.
- Updated to rename folders, delete tracing.md, and edit README.md.
- Updated to add a dockerfile for Rolling and Galactic.
- Updated to copy initial docs.
- Updated to open new folder for additional tracing contents.
- Updated to delete tracing directory.
- Updated to move tracing.md to tracing directory.
- Updated to reactivate smacc2 nav clients for rolling via submodules.
- Updated to revert "Ignore all packages except smacc2 and smacc2_msgs".
- Updated to reset all versions to 0.0.0.
- Updated to rename to smacc2 and smacc2_msgs.
- Updated to rename header files and correct format.
- Updated to use manual deployment for now.
- Updated to create workflow for testing prerelease builds.
- Updated to use docs/ as source folder for documentation.
- Updated to use docs/ as output directory.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to rename tracing events after.
- Updated to

```rst
Section_14
==========

Added
~~~~~
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #91, #92, #93, #94, #95, #98)
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- New client behavior: cb_pause_slam (#98)
- New client behavior: sm_dance_bot_lite (#99)

Changed
~~~~~~~
- Updated launch command in README.md
- Corrected all linters and formatters (#82)
- Navigation parameters fixes on sm_dance_bot (#93, #95)
- Minor hotfix in doxygen deployment workflow (#100)
- Visualizing turtlebot3 in sm_dance_bot (#101)
- Added lidar show/hide option in dance bot launch (#102)

Fixed
~~~~
- Fixed precommit issues (#90, #91)
- Removed some compile warnings (#96)
- Fixed formatting issues in various parts of the codebase

Authors
~~~~~~~
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_15
==========

Added
-----
- Feature/sm dance bot lite gazebo fixes (#104)
  - Visualizing turtlebot3 for sm_dance_bot
  - Added lidar show/hide option
  - Improved formatting and cleaned files
  - Various fixes for gazebo to display the robot and lidar

- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Visualizing turtlebot3 for sm_dance_bot
  - Added lidar show/hide option
  - Improved formatting and cleaned files
  - Various fixes for gazebo to display the robot and lidar
  - Additional fixes for sm_dance_bot_strikes_back

Changed
-------
- sm_multi_stage_1 doubling (#103)
  - Improved functionality with doubling feature

- Precommit cleanup run (#106)
  - General cleanup and formatting improvements

- Brettpac branch (#110)
  - Continued improvements on sm_multi_stage_1 functionality

- Brettpac branch (#111)
  - Continued improvements on sm_multi_stage_1 functionality, now with five stages

- Remove neo_simulation2 package. (#112)
  - Removed neo_simulation2 package
  - Corrected formatting and adjusted build packages for source CI

- More sm_multi_stage_1 (#114)
  - Further enhancements to sm_multi_stage_1 functionality

- Diverse improvements navigation and performance (#116)
  - Various enhancements for navigation and performance

- Feature/diverse improvements navigation performance (#117)
  - Continued enhancements for navigation and performance
  - Added linting and formatting improvements

- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing slam pausing/resuming functionality for sm_dance_bot
  - Additional fixes for sm_dance_bot

- Move method after the method it calls. Otherwise recursion could happen. (#126)
  - Corrected method order to prevent recursion issues

- First working version of sm template and template generator. (#127)
  - Initial release of sm template and generator

- Feature/dance bot s pattern (#128)
  - Continued improvements for sm_dance_bot and s-pattern
  - Corrected typo "Finnaly" to "Finally"

- Feature/dance bot s pattern (#129)
  - Further refinements for sm_dance_bot and s-pattern

- Feature/sm dance bot refine (#131)
  - Continued improvements for sm_dance_bot

- Feature/sm dance bot refine 2 (#132)
  - Continued improvements for sm_dance_bot
  - Fixed build issues

- Waypoints navigator bug (#133)
  - Tuning to mitigate overshot issues

- Progress in the sm_dance_bot tests (#135)
  - Progress made on markers cleanup

- Minor navigation improvements (#141)
  - Minor enhancements for navigation

- Feature/nav2z renaming (#144)
  - Renamed navigation 2 stack components
  - Updated formatting and added SVGs to READMEs

- Update package list. (#142)
  - Updated package list

- Fix CI: format fix python version (#148)
  - Fixed CI formatting issues for Python version

- Add SM Atomic SM generator. (#143)
  - Added SM Atomic SM generator

- Remove node creation and create only a logger. (#149)
  - Removed node creation, now only creating a logger

- Rolling Docker environment to be executed from any environment (#154)
  - Improved Docker environment for universal execution

- Feature/sm dance bot strikes back refactoring (#152)
  - Refactored sm_dance_bot strikes back functionality

- Slight waypoint 4 and iterations changes so robot can complete course (#155)
  - Adjusted waypoints for course completion

- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixed errors in formatting and dependencies
  - Progress in moveit migration testing
  - Updated format and dependencies for UR5 client
  - Docker refactoring for local tests
  - Improved Dockerfile for building tests
  - Fixed compiling issues

- Update readme (#164)
  - Updated readme with additional information

Fixed
-----
- Resolve compile warnings (#137)
  - Resolved compile warnings

- Add SM core test (#138)
  - Added SM core test

- Minor format issues (#134)
  - Fixed minor format issues

- Fixing compiling issues
  - Updated readme with correct launch command for sm_dance_bot_strikes_back
  - Removed unnecessary comments

- Moved reference library SMs to smacc2_performance_tools (#166)
  - Improved organization by moving reference library SMs

Removed
-------
- Removing parameters smacc (#147)
  - Removed parameters for smacc
  - Updated workflows

- Removing sm_dance_bot_msgs
  - Removed sm_dance_bot_msgs
  - Updated workflows

- Removing test from main moveit cmake
  - Removed test from main moveit cmake

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters smacc
  - Removed parameters for smacc

- Removing parameters

```rst
Section_16
==========

Added
-----
- Added QOS durability to SmaccPublisherClient (#163) to improve message reliability.
- Added reliability QOS configuration for enhanced performance.
- Added multistage modes, sequences, and steps for improved functionality.
- Added warehouse2 progress (#179) for warehouse management.
- Added Waypoint Inputs (#178) for precise navigation.
- Added finetuning waypoints (#187) for improved navigation accuracy.
- Added pure spinning behavior for specific actions (#188, #189).
- Added undo motion feature (#196, #198) for reverting actions.
- Added sync feature (#199) for synchronization.
- Added minor tune feature (#203) for small adjustments.
- Added fixing warehouse 3 problems and core improvements (#204) to enhance stability.
- Added missing subscriber and publisher components for completeness.
- Added improvements in smacc core for Autoware demo.
- Added retry behavior for warehouse 1 for better performance.

Changed
-------
- Moved reference library SMs to smacc2_performance_tools for better organization.
- Refactored to remove unnecessary lines and improve code quality.
- Renamed sm_advanced_recovery_1 for clarity.
- Reworked sm_multi_stage_1 for optimization.
- Updated dependencies for husky simulation for compatibility.
- Updated dependencies for husky in rolling and galactic for consistency.
- Updated dependencies for warehouse2 for compatibility.
- Updated dependencies for fake hardware simulation and gazebo simulation for simulation accuracy.
- Updated docker build files for all versions for consistency.
- Updated formatting for readability.
- Updated trigger logic into headers for better logic management.
- Updated lint for code quality.
- Updated pipeline error for smoother development process.
- Updated broken master build for stability.
- Updated broken build for continuous integration.
- Updated broken build for Foxy and Galactic compatibility.
- Updated file for fake hardware simulation and added file for gazebo simulation for simulation accuracy.
- Updated warnings removal for cleaner code.
- Updated formatting issues for better readability.
- Updated tuning and fixes for optimization.
- Updated tuning and fixes for warehouse3 for better performance.
- Updated tuning and fixes for warehouse2 for better performance.
- Updated tuning and fixes for warehouse2 for finishing touches.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for finishing warehouse2.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.
- Updated tuning and fixes for warehouse2 for format issues.
- Updated tuning and fixes for warehouse2 for tuning and fixes.

```rst
Section_17
==========

Added
-----
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
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Copy initial docs
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
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- changed wording "smacc application" to "SMACC2 library"
- reactivating smacc2 nav clients for rolling via submodules
- renamed tracing events after
- bug in smacc2 component
- reverted markdowns to html
- updated mentions of SMACC/ROS to SMACC2/ROS2
- some progress on navigation rolling
- renamed folders, deleted tracing.md, edited README.md
- performance tests improvements
- more on performance and other issues
- sm_respira_1 format cleanup
- sm_respira_test_2
- more changes on performance tests
- Do not execute clang-format on smacc2_sm_reference_library package.
- sm_reference_library reformatting
- sm_atomic_24hr
- sm_atomic_performance_trace_1
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library
- minor formatting

Fixed
-----
- fix broken source build (#227)
- Only rolling version should be pre-released on on master. (#230)
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
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
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Enable galactic CI setup and rename rolling files. (#58)
- Fix source CI and correct README overview. (#62)
- changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- update doxygen links (#70)

Removed
-------
- Delete tracing directory
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
- cleanup
- cleanup
- cleanup
```

Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_18
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- New client behavior: add for nav2, which waits for nav2 nodes subscribing to the /bond topic and ensures they are alive, with optional node selection.

Changed
-------
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Formatting improvements.
- Pre-commit fixes.

Fixed
-----
- Navigation parameters fixes on sm_dance_bot.
- Removed some compile warnings.

Removed
-------
- Minor format adjustments.

Commits
-------
- sm_advanced_recovery_1 reworked (#83) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- More sm_advanced_recovery_1 work (#84) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- More sm_advanced_recovery_1 work (#85) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- sm_advanced_recovery_1 round 4 (#86) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Brettpac branch (#87) by Pablo Iñigo Blasco.
- sm_atomic_performance_test_a_2 by Pablo Iñigo Blasco.
- sm_atomic_performance_test_a_1 by Pablo Iñigo Blasco.
- sm_atomic_performance_test_c_1 (#88) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Modifying sm_atomic_performance_test_a_2 (#89) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- sm_multi_stage_1 (#90) by Pablo Iñigo Blasco.
- More sm_multi_stage_1 (#91) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Wait topic message client behavior (#81) by Pablo Iñigo Blasco.
- Feature/wait nav2 nodes client behavior (#82) by Pablo Iñigo Blasco.
- Feature/aws demo progress (#92) by Pablo Iñigo Blasco.
- Feature/sm dance bot fixes (#93) by Pablo Iñigo Blasco.
- Feature/sm aws warehouse (#94) by Pablo Iñigo Blasco.
- Feature/sm dance bot fixes (#95) by Pablo Iñigo Blasco.
- Remove some compile warnings. (#96) by Pablo Iñigo Blasco.
- Feature/cb pause slam (#98) by Pablo Iñigo Blasco.

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Denis Štogl <destogl@users.noreply.github.com>.
- Denis Štogl <denis@stogl.de>.
```

```rst
Section_19
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior `add` for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive; optional node selection
- Progress in AWS navigation demo
- Gazebo fixes to show the robot and the lidar
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others
- Added remaining SVGs to READMEs

Changed
-------

- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Minor hotfixes
- Cleaning and lidar show/hide option
- More fixes in `sm_dance_bot`
- Polishing `sm_dance_bot` and `s-pattern`
- Resolved compile warnings
- Minor navigation improvements
- Fixed launch command for `sm_dance_bot_strikes_back`
- Rolling Docker environment to be executed from any environment

Fixed
-----

- Move method after the method it calls to prevent recursion
- Minor tuning to mitigate overshot issue cases
- Progress in the `sm_dance_bot` tests
- Minor format issues

Removed
-------

- Removed `neo_simulation2` package
- Removed `sm_dance_bot_msgs`
- Removed parameters in `smacc`

Authors
-------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
```

# Changelog

## [Unreleased]

### Added
- Added QOS durability to SmaccPublisherClient (#163) [@pabloinigoblasco]
- Added dependencies for husky simulation in sm_dance_bot (#174) [@pabloinigoblasco, @denisstogl, @DecDury, @reelrbtx]
- Added warehouse2 progress (#179) [@pabloinigoblasco]
- Added Waypoint Inputs (#178) [@pabloinigoblasco]
- Added SrConditional fixes and formatting (#168) [@pabloinigoblasco]
- Added finetuning waypoints (#187) [@pabloinigoblasco]
- Added pure spinning behavior (#188, #189) [@pabloinigoblasco]
- Added planner changes (#191) [@pabloinigoblasco]
- Added replanning for all examples (#193, #195) [@pabloinigoblasco]
- Added undo motion improvements (#196, #198) [@pabloinigoblasco]
- Added sync feature (#199) [@pabloinigoblasco]
- Added warehouse2 features (#200, #201) [@pabloinigoblasco]
- Added minor tune feature (#203) [@pabloinigoblasco]
- Added core improvements and fixes to warehouse 3 (#204) [@pabloinigoblasco, @denisstogl, @DecDury, @reelrbtx, @MrBlenny]

### Changed
- Changed reference library SMs to smacc2_performance_tools (#166) [@pabloinigoblasco]
- Changed mode sequences in sm_multi_stage_1 (#172) [@pabloinigoblasco]
- Changed warehouse3 tuning (#197) [@pabloinigoblasco]

### Fixed
- Fixed errors introduced on formatting (#155) [@pabloinigoblasco]
- Fixed missing dependencies and build errors (#164) [@pabloinigoblasco]
- Fixed linting warnings (#164) [@pabloinigoblasco]
- Fixed compiling issues (#164) [@pabloinigoblasco]
- Fixed broken builds (#169, #174, #185, #204) [@pabloinigoblasco]
- Fixed formatting and templating on SrConditional (#168) [@pabloinigoblasco]
- Fixed missing files in pure spinning behavior (#190) [@pabloinigoblasco]
- Fixed linking errors in foxy backport (#205) [@pabloinigoblasco]

### Removed
- Removed test from main moveit cmake (#155) [@pabloinigoblasco]
- Removed unnecessary lines in refactorings (#163) [@pabloinigoblasco]

### Miscellaneous
- Progressed in moveit migration testing (#155) [@pabloinigoblasco]
- Improved dockerfile for local tests (#155) [@pabloinigoblasco]
- Updated format and dependencies (#155) [@pabloinigoblasco]
- Pre-commit cleanup (#166) [@pabloinigoblasco]
- Added reliability qos config (#166) [@pabloinigoblasco]
- Added missing colon (#163) [@pabloinigoblasco]
- Added husky launch file in sm_dance_bot (#174) [@pabloinigoblasco]
- Updated dependencies for husky in rolling and galactic (#174) [@pabloinigoblasco]
- Progressed on aws navigation and refactorings (#174) [@pabloinigoblasco]
- More on aws demo (#174) [@pabloinigoblasco]
- Progressed on moveit behaviors testing (#167) [@pabloinigoblasco]
- Improved undo motion navigation in warehouse2 (#196, #198) [@pabloinigoblasco]
- Reworked sm_dance_bot_warehouse_3 waypoints (#184) [@pabloinigoblasco]
- Added missing file from warehouse2 (#205) [@pabloinigoblasco]
- Updated subscriber publisher components (#205) [@pabloinigoblasco]
- Progressed in autowar machine refinement (#205) [@pabloinigoblasco]
- Refined cp subscriber and cp publisher (#205) [@pabloinigoblasco]

## [1.0.0] - 2021-12-23

### Added
- Initial migration to smacc2
- Added .reps dependencies and fixed build errors
- Added dependency to ur5 client
- Added docker refactoring
- Added progress on move_it PR
- Added minor dockerfile test workaround
- Added more testing on moveit behaviors
- Added minor configuration
- Added more readme updates
- Added more changes and headless
- Added merge changes
- Added default values

### Changed
- Moved reference library SMs to smacc2_performance_tools
- Changed multistage modes in sm_multi_stage_1
- Changed sm_multi_stage sequences
- Changed sm_multi_state_1 steps
- Changed sm_multi_stage_1 sequence d
- Changed sm_multi_stage_1 c sequence
- Changed mode_5_sequence_b
- Changed mode_4_sequence_b
- Changed sm_multi_stage_1 most
- Changed finishing touches 1

### Fixed
- Fixed warehouse2 formatting
- Fixed minor changes
- Fixed several issues

### Removed
- Removed weird moveit not downloaded repo

### Miscellaneous
- Updated readme

## [0.1.0] - 2021-12-16

### Added
- Added warehouse2 feature
- Added warehouse2 tuning and fixes

### Changed
- Changed minor features

### Fixed
- Fixed several issues

### Miscellaneous
- Updated format issues

## [0.0.1] - 2021-12-14

### Added
- Added warehouse2 feature

### Changed
- Changed minor features

### Fixed
- Fixed several issues

### Miscellaneous
- Updated format issues

```rst
Section_21
==========

Added
-----
- Added more components to smacc core, mainly for autoware demo.
- Added docker files for different revisions, warnings removal, and additional testing on navigation.
- Added barrel search build fix and warehouse3 progress.
- Added progress in barrel husky.
- Added branching example.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml for manual deployment.
- Added workflow for testing prerelease builds.
- Added necessary package and edited Threesome launch.
- Added Dockerfile for Rolling and Galactic.
- Added setupTracing.sh for automated installation.
- Added alternative ManualTracing.
- Added new sm markdowns.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added performance tests improvements.
- Added more changes on performance tests.
- Added sm_atomic_performance_trace_1.
- Added galactic CI setup and renamed rolling files.
- Added source CI fix and corrected README overview.
- Added optimized dependencies in move_base_z_planners_common.
- Added renaming of event generator library.

Changed
-------
- Changed "sm_three_some" launch command to "sm_three_some.launch".
- Changed wording from "smacc application" to "SMACC2 library".
- Changed references of SMACC/ROS to SMACC2/ROS2.
- Changed launch command to "ros2 launch sm_respira_1 sm_respira_1.launch".

Fixed
-----
- Fixed minor broken build.
- Fixed docker for Foxy and Galactic.
- Fixed startup problems in warehouse 3.
- Fixed format issues.
- Fixed trailing spaces.
- Fixed codespell and Python linters warnings.
- Fixed formatting of Python files.
- Fixed broken build in barrel demo.
- Fixed linking errors in Foxy.
- Fixed build of missing rolling repositories.
- Fixed Navigation2 for semi-binary build.
- Fixed bug in smacc2 component.
- Fixed formatting in sm_respira_1 and sm_atomic_24hr.
- Fixed trailing spaces in sm_reference_library.
- Fixed sm_atomic_24hr cleanup.
- Fixed sm_atomic_24hr formatting.
- Fixed sm_respira_test_2.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
- Removed tracing directory.
- Removed disabled packages.
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.

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
- sm_atomic_performance_test_a_1
- sm_atomic_performance_test_c_1 (#88) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- modifying sm_atomic_performance_test_a_2 (#89)
- sm_multi_stage_1 (#90)
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
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `aws demo`.
- `Brettpac branch`.
- `neo_simulation2` package removal.
- `mm`.
- Diverse improvements in navigation and performance.
- `slam toggle` client behaviors and `slam_toolbox` components.
- `smacc2::deep_history` syntax.
- `slam pausing/resuming` functionality.
- `s-pattern` for `sm_dance_bot`.
- First working version of `sm template` and template generator.
- `waypoints navigator` bug fixes.
- `SM core` test.
- `nav2z` renaming.

Changed
~~~~~~~
- Progress in `aws navigation demo`.
- Navigation parameters fixes on `sm_dance_bot`.
- Several core improvements during navigation testing.
- Formatting improvements.
- Cleaning and lidar show/hide option.
- More refinement in `sm_dance_bot`.
- Minor tweaks.
- Minor navigation improvements.
- Using local action messages.

Fixed
~~~~
- Remove some compile warnings.
- Correct formatting.
- Adjust build packages of source CI.
- Minor format issues.

Removed
~~~~~~~
- Remove `neo_simulation2` package.
- Remove merge markers from a python file.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

Section_24
===========

Added
-----
- Added remaining SVGs to READMEs (#145).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient (#163).
- Added dependencies for husky simulation in AWS navigation (#174).
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
- Fixed launch command for sm_dance_bot_strikes_back and removed unnecessary comments.
- Fixed CI: format fix python version (#148).
- Removed node creation and created only a logger (#149).
- Rolled Docker environment to be executed from any environment (#154).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Redoing sm_dance_bot_warehouse_3 waypoints (#184).
- Finetuned waypoints (#187).
- Tuning warehouse3 (#197).
- Fixed warehouse 3 problems and other core improvements (#204).

Fixed
-----
- Noticed launch command was incorrect in README.md.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed broken pipeline error.
- Fixed formatting issues.
- Fixed minor issues.

Removed
-------
- Removed parameters smacc.
- Removed test from main moveit cmake.

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

- Added missing file from warehouse2 (#205)
- Added Feature/docker improvements march 2022 (#235)
- Added workflow for checking doc build
- Added doxygen-deploy.yml
- Added workflow for testing prerelease builds
- Added Dockerfile w/ ROS distro as argument
- Added setupTracing.sh for automated installation of ros-rolling-ros2trace
- Added alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Added README tutorial for Dockerfile
- Added smacc2_performance_tools

Changed
-------

- Changed wording "smacc application" to "SMACC2 library"
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Fixed
-----

- Fixed warehouse 3 problems to remove dead lock
- Fixed weird moveit not downloaded repo
- Fixed minor broken build
- Fixed trailing spaces
- Fixed codespell
- Fixed python linters warnings
- Fixed docker for foxy and galactic
- Fixed startup problems in warehouse 3
- Fixed format and minor
- Fixed bug in smacc2 component

Removed
-------

- Removed manual installation of ros-rolling-ros2trace
- Removed tracing directory
- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file

Other Changes
-------------

- Backported changes to foxy
- Reordered components in smacc core adding more components mostly developed for autoware demo
- Progress in autoware machine
- Progress in barrel husky
- Progress in husky demo
- Progress in navigation behaviors
- Progress in navigation rolling
- Progress in barrel search updates
- Progress in husky demo
- Progress in barrel demo
- Progress in barrel search build fix and warehouse3
- Progress in barrel demo
- Progress in husky demo
- Progress in barrel search updates
- Progress in autoware demo
- Progress in smacc2 nav clients for rolling via submodules
- Progress in smacc2 library

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

Section_26
==========

Added
-----
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.

Changed
-------
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69).
- Updated doxygen links (#70).
- Updated README.md launch command.
- Corrected all linters and formatters.

Fixed
-----
- Fixed source CI and corrected README overview. (#62).
- Fixed trailing spaces.
- Fixed pre-commit issues.

Removed
-------
- Do not execute clang-format on `smacc2_sm_reference_library` package.

Other
-----
- Performance tests improvements.
- More changes on performance tests.
- Optimized dependencies in `move_base_z_planners_common`.
- Renamed event generator library.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.

```rst
Section_27
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `wait nav2 nodes subscribing to the /bond topic and waiting they are alive`. Nodes to wait can be optionally selected.
- `sm_dance_bot_lite` gazebo fixes.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `aws demo` progress.
- `sm_multi_stage_1` improvements.
- `slam toggle` client behaviors and `slam_toolbox` components.
- `smacc2::deep_history` syntax.
- `s-pattern` polishing in `sm_dance_bot`.

Changed
-------
- Formatting improvements in various sections.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor tweaks and refinements in different areas.

Fixed
-----
- Compile warnings removed.
- Recursion issue fixed by moving a method after the one it calls.
- Typo corrected.

Removed
-------
- `neo_simulation2` package removed.

Other
-----
- Various core improvements during navigation testing.
- Progress in AWS navigation demo.
- Merge and progress in development.
- Precommit cleanup run.
- Source build enabled on PR for testing.
- Linting and formatting adjustments.
- Merge markers removed from a Python file.
- First working version of `sm` template and template generator.

Contributors
------------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_28
==========

Added
-----
- Feature/sm dance bot refine (#131)
  - Refinement in sm_dance_bot functionality.
- Feature/sm dance bot refine 2 (#132)
  - Further refinement in sm_dance_bot.
- Waypoints navigator bug (#133)
  - Tuning to mitigate overshot issues.
- Progress in the sm_dance_bot tests (#135)
  - Progress on markers cleanup.
- sm_dance_bot_lite (#136)
  - Lightweight version of sm_dance_bot.
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Feature/nav2z renaming (#144)
  - Renaming navigation 2 stack.
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
  - Refactoring in sm_dance_bot functionality.
- Feature/migration moveit client (#151)
  - Migration to smacc2 with various fixes and dependencies.
- Update readme (#164)
  - Updates to project readme.
- Add QOS durability to SmaccPublisherClient (#163)
  - Added QOS durability feature to SmaccPublisherClient.
- Feature/testing moveit behaviors (#167)
  - Testing and improvements on moveit behaviors.
- Feature/aws navigation sm dance bot (#174)
  - Progress on AWS navigation and related refactorings.
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- sm_dance_bot_warehouse_3 (#181)
  - Progress on warehouse waypoints.
- Feature/sm warehouse 2 13 dec 2 (#182)
  - Updates and fixes in warehouse functionality.
- SrConditional fixes and formatting (#168)
  - Fixes and formatting improvements in SrConditional.
- Feature/wharehouse2 dec 14 (#185)
  - Updates in warehouse functionality.
- Feature/cb pure spinning (#188)
  - Changes and fixes in pure spinning behavior.
- Feature/planner changes 16 12 (#191)
  - Minor changes and fixes in planner functionality.
- Feature/replanning 16 dec (#193)
  - Replanning for all examples.

Changed
-------
- Minor format issues (#134)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing some errors introduced on formatting in migration moveit client.
- Fixing some more linting warnings in migration moveit client.
- Improving Dockerfile for building local tests.
- Fixing compiling issues.
- Finetuning waypoints (#187)

Fixed
-----
- Noticed launch command was incorrect in README.md (#147)
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments.
- Fixing broken master build in testing moveit behaviors.
- Fixing pipeline error in testing moveit behaviors.
- Pure spinning behavior missing files.
- Several fixes (#194)

Removed
-------
- Removing sm_dance_bot_msgs in nav2z renaming.
- Removing parameters smacc in migration moveit client.
- Removing test from main moveit cmake in migration moveit client.
- Removing parameters smacc in migration moveit client.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_29
===========

Added
-----
- Feature/undo motion 20 12 (#196): Improved undo motion navigation in warehouse2.
- Feature/undo motion 20 12 (#198): Continued improvements on undo motion navigation in warehouse2.
- Feature/sync 21 12 (#199): Addressed format issues.
- Feature/warehouse2 22 12 (#200): Finalized warehouse2 development.
- Feature/warehouse2 23 12 (#201): Tuning and fixes (#202).
- Feature/minor tune (#203): Tuning and fixes, including warehouse 3 problems (#204).
- Added missing file from warehouse2 (#205).
- Use correct upstream .repos files for source builds (#243).
- Correct mergify branch names (#246).
- Update galactic source build job name (#250).
- Galactic source build: Updated .repos file, action version, and upstream packages (#248).
- Restoring workflow files (#252).
- Restoring files (#253).
- Fix checkout branches for scheduled builds (#254).
- Update foxy-source-build.yml.
- Feature/fixing husky build rolling (#257): Ensured husky project builds on rolling.
- Feature/fixing husky build rolling (#258): Continued work on husky project builds on rolling.
- Update README.md (#262).
- Feature/fixing ur demos (#261): Fixed issues with ur demos.
- Feature/fixing type string walker (#263): Fixed issues with type string walker demo.
- Update README.md (#266).
- Update README.md (#267).
- Update README.md (#268).
- Significant update in Getting Started Instructions (#269).
- Fix urls to index.ros.org (#284).
- Fix foxy source build config to use repos file from foxy branch (#285).

Changed
-------
- Minor changes and replanning for all examples in various features.
- Tuning warehouse3 (#197).
- Fixed broken build issues.
- Reordered some elements for clarity.
- Fixed docker files for foxy and galactic.
- Updated barrel search build and warehouse3.
- Progress in autoware machine and husky demo.
- Improved navigation behaviors.
- Docker improvements.

Removed
-------
- Removed trailing spaces.
- Reverted commit for ignoring packages not to be released.

Fixed
-----
- Fixed rolling build issues.
- Fixed dependencies and building problems.
- Fixed broken build issues.
- Fixed precommit issues.
- Fixed URLs to index.ros.org.
- Fixed issues with image messages for husky_barrel demo.
- Fixed issues with sm_dance_bot examples.

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
### Added
- Publisher feature.

0.0.0 (2022-11-09)
------------------
### Added
- Feature/galactic rolling merge (#288)
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs" (commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61).
  - Updated description table.
  - Updated table.
  - Copied initial docs.
  - Dockerfile with ROS distro as argument.
  - Opened new folder for additional tracing contents.
  - Added setupTracing.sh for installing necessary packages and configuring tracing group.
  - Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
  - Created alternative ManualTracing.
  - Added new sm markdowns.
  - Added Dockerfile for Rolling and Galactic.
  - Updated tracing/ManualTracing.md.
  - Changed wording "smacc application" to "SMACC2 library".
  - Updated smacc_sm_reference_library/sm_atomic/README.md.
  - Reactivated smacc2 nav clients for rolling via submodules.
  - Renamed tracing events.
  - Fixed bug in smacc2 component.
  - Added README tutorial for Dockerfile.
  - Added smacc2_performance_tools.
  - Improved performance tests.
  - Cleaned up sm_respira_1 format.
  - Optimized dependencies in move_base_z_planners_common.
  - Renamed event generator library.
  - Added galactic CI setup and renamed rolling files.
  - Fixed source CI and corrected README overview.
  - Updated c_cpp_properties.json.
  - Changed launch command to "ros2 launch sm_respira_1 sm_respira_1.launch".
  - Updated doxygen links.
  - More Readme Updates.
  - Created new sm from sm_respira_1.
  - Added core and navigation fixes.
  - Added base for the sm_aws_aarehouse navigation.
  - Made progress in aws navigation.
  - Improved core during navigation testing.
  - Added AWS demo progress.
  - Reworked sm_advanced_recovery_1.
  - Fixed pre-commit issues.
  - Continued work on sm_advanced_recovery_1.
  - Worked on sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1.
  - Added sm_atomic_performance_test_c_1.
  - Modified sm_atomic_performance_test_a_2.
  - Added sm_multi_stage_1.
  - Fixed precommit for sm_multi_stage_1.
  - Updated README.md.
  - Added new feature: cb_wait_topic_message, an asynchronous client behavior.

### Removed
- Ignoring packages which should not be released.

### Contributors
- Denis Štogl
- Pablo Iñigo Blasco
- pabloinigoblasco
- brettpac
```

```rst
Section_31
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#82)
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection
- New feature: cb_pause_slam client behavior (#98)
- New feature: sm_dance_bot_lite gazebo fixes (#104)
- New feature: gazebo fixes for sm_dance_bot_strikes_back (#105)
- New feature: aws demo (#108)

Changed
-------

- Corrected all linters and formatters (#92)
- Navigation parameters fixes on sm_dance_bot
- Minor hotfixes
- Minor formatting improvements
- Precommit cleanup run

Fixed
-----

- Removed some compile warnings (#96)
- Fixed format issues

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_32
==========

Added
-----

- Implemented AWS demo.
- Added formatting improvements.
- Added support for multi-stage functionality (#109, #110, #111).
- Added 5 stages for sm_multi_stage_1.
- Added a3 functionality (#113).
- Added durability QoS to SmaccPublisherClient (#163).
- Added reliability QoS configuration.
- Added QoS durability to SmaccPublisherClient.
- Added SM core test (#138).
- Added SM Atomic SM generator (#143).
- Added Docker environment for execution in any environment (#154).
- Added SM Atomic SM generator.
- Added SM Atomic SM generator.
- Added remaining SVGs to READMEs of atomic, dance_bot, and others (#140, #145).
- Added remaining SVGs to READMEs.
- Added waypoints navigator bug fixes (#133).
- Added waypoints navigator bug fixes.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components

Section_33
==========

Added
-----
- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency for husky simulation.
  - Included husky launch file in sm_dance_bot.
  - Updated dependencies for husky in rolling and galactic.
  - Improved AWS navigation and refactored navigation clients and behaviors.
  - Added more content for AWS demo.
  - Fixed broken build.

Changed
-------
- SrConditional fixes and formatting (#168)
  - Adjusted formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Linted the code.

Fixed
-----
- Several fixes (#194)
  - Addressed various issues.

Removed
-------
- Deleted tracing directory.
- Removed tracing.md from the main directory.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_34
==========

Added
~~~~~
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
~~~~~~~
- Renamed "smacc application" to "SMACC2 library"
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Renamed tracing events
- Renamed event generator library

Fixed
~~~~
- Bug in smacc2 component
- Trailing spaces corrected
- Do not execute clang-format on smacc2_sm_reference_library package
- Cleaned up sm_atomic_24hr
- Cleaned up sm_reference_library
- Cleaned up sm_advanced_recovery_1
- Corrected README overview
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch

Removed
~~~~~~~
- Manual installation of ros-rolling-ros2trace
- Galactic builds from master, keeping only Rolling
- Submodules in favor of .repos file
- Tracing.md file
```

*pabloinigoblasco*

```rst
Section_35
==========

Added
~~~~~
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#92)
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#92)
- New client behavior: cb_pause_slam (#98)
- New client behavior: cb_pause_slam client behavior (#98)
- New client behavior: sm_dance_bot_lite (#99)
- New client behavior: sm_dance_bot visualizing turtlebot3 (#101)
- New client behavior: sm_dance_bot visualizing turtlebot3 (#102)
- New client behavior: gazebo fixes for sm_dance_bot_strikes_back (#105)
- New client behavior: aws demo (#108)
- New client behavior: got sm_multi_stage_1 working (barely) (#109)
- New client behavior: gaining traction sm_multi_stage_1 (#110)
- New client behavior: two stages, 3 part, 4th stage, 5th stage (#111)
- New client behavior: a3 (#113)

Changed
~~~~~~~
- Improved core functionality during navigation testing (#92, #93, #94, #95, #98)
- Formatting improvements in navigation demo and client behaviors (#92, #93, #94, #95, #98)
- Navigation parameters fixes on sm_dance_bot (#93, #95, #98)
- Precommit cleanup run (#106)
- Minor hotfixes and format corrections (#100, #101, #102, #104, #105, #106)
- Renamed doxygen deployment workflow (#100)

Removed
~~~~~~~
- Removed some compile warnings (#96)
- Removed neo_simulation2 package (#112)

Collaborators
~~~~~~~~~~~~~
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai> (#99, #103, #105, #106, #109, #110, #111, #113)
```

Section_36
==========

Added
-----
- Adjusted build packages of source CI.
- Diverse improvements in navigation and performance.
- Added linting and formatting.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- First working version of sm template and template generator.
- Added dependencies for husky simulation.
- Added dependencies for husky in rolling and galactic.
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors.
- Progress on AWS demo.
- Progress on moveit migration testing.
- Progress on moveit behaviors testing.
- Progress on move_it PR.
- Progress on sm_dance_bot tests.
- Progress on markers cleanup.
- Progress on warehouse2.
- Progress on waypoint inputs.

Changed
-------
- Moved reference library SMs to smacc2_performance_tools.
- Updated package list.
- Updated README.
- Updated format.
- Updated dependencies.
- Updated Docker environment to be executed from any environment.
- Updated format for building local tests.
- Updated format for compiling issues.
- Updated launch command in README.md.
- Updated dependencies for husky in rolling and galactic.
- Updated format for AWS navigation and some other refactorings on navigation clients and behaviors.
- Updated format for AWS demo.
- Updated format for moveit migration testing.
- Updated format for moveit behaviors testing.
- Updated format for move_it PR.
- Updated format for sm_dance_bot tests.
- Updated format for markers cleanup.
- Updated format for warehouse2.
- Updated format for waypoint inputs.

Fixed
-----
- Removed merge markers from a python file.
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments.
- Fixed CI: format fix python version.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed compiling issues.
- Fixed broken build.

Removed
-------
- Removed node creation and create only a logger.
- Removed parameters smacc.
- Removed sm_dance_bot_msgs.
- Removed test from main moveit cmake.

Contributors
------------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Brett (brett@robosoft.ai)
- DecDury (declandury@gmail.com)
- Denis Štogl (denis@stogl.de)
- Denis Štogl (destogl@users.noreply.github.com)

```rst
Section_37
==========

Added
-----
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
  Merging to get backport working.
- Remove example things from Foxy CI setup. (#214)
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Fix rolling builds (#222)
- do not merge yet - Feature/odom tracker improvements and retry motion (#223)
  - odom tracker improvements
  - adding forward behavior retry functionality
  - removing warnings
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licenses.
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

Changed
-------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch

Removed
-------
- minor broken build

Fixed
-----
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
```

```rst
Section_38
==========

Added
-----
- Initial setup:
  - Execute on master update
  - Reset all versions to 0.0.0
  - Ignore all packages except smacc2 and smacc2_msgs
  - Update changelogs
  - Copy initial docs
  - Dockerfile w/ ROS distro as argument
  - Opened new folder for additional tracing contents
  - Added setupTracing.sh for automated installation
  - Created alternative ManualTracing
  - Added new sm markdowns
  - Added a dockerfile for Rolling and Galactic
  - Enable build of missing rolling repositories
  - Enable Navigation2 for semi-binary build
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Added galactic CI setup and renamed rolling files
  - Added new feature cb_wait_topic_message
  - Added asynchronous client behavior cb_wait_topic_message
  - Added sm_multi_stage_1

Changed
-------
- Updated description table
- Updated table
- Changed wording "smacc application" to "SMACC2 library"
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Reactivated smacc2 nav clients for rolling via submodules
- Renamed tracing events
- Updated smacc2_rta command across readmes
- Cleaned up sm_reference_library
- Corrected trailing spaces
- Formatted sm_atomic_24hr
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch

Fixed
-----
- Fixed bug in smacc2 component
- Fixed source CI and corrected README overview
- Fixed formatting in sm_advanced_recovery_1
- Fixed pre-commit issues in sm_advanced_recovery_1
- Fixed pre-commit in sm_multi_stage_1
- Attempted pre-commit fixes

Removed
-------
- Removed manual installation of ros-rolling-ros2trace
- Removed galactic builds from master and kept only rolling
- Removed submodules and use .repos file
- Do not execute clang-format on smacc2_sm_reference_library package
- Deleted tracing directory
- Removed tracing.md
```

*pabloinigoblasco*

```rst
Section_39
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Nodes to wait can be optionally selected.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo progress.

Changed
-------

- Corrected all linters and formatters.
- Minor format fixes.
- Precommit cleanup run.
- Hotfix for `doxygen` deployment workflow.
- Cleaning and lidar show/hide option for `dance_bot_launch_gz_lidar_choice`.
- Format fixes for gazebo to show the robot and lidar.

Fixed
-----

- Removed some compile warnings.

Removed
-------

- Redundant entries.

Contributors
------------

- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```

*pabloinigoblasco*

```rst
Section_40
==========

Added
~~~~~
- Implemented "keep hammering" feature.
- Added support for "two stages" functionality.
- Introduced a new "3 part" feature.
- Implemented "4th stage" functionality.
- Added support for "5th stage" feature.

Changed
~~~~~~~
- Improved performance of "sm_multi_stage_1" feature.
- Enhanced "sm_multi_stage_1" functionality.
- Made progress in various areas.

Fixed
~~~~
- Corrected formatting issues.
- Resolved bugs in "sm_dance_bot" and "waypoints navigator."
- Mitigated overshot issue cases.
- Fixed compilation warnings.
- Updated package list.
- Fixed CI formatting issues.
- Corrected launch command in README.md.

Removed
~~~~~~~
- Removed "neo_simulation2" package.
- Eliminated redundant parameters in "smacc."
- Removed unnecessary comments in README.md.

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

## Section_41

### Added
- Added dependencies for husky simulation.
- Added missing file from warehouse2.
- Added tf2_ros as dependency to find include.
- Added partial changes for ament_cpplint.
- Added missing licenses.
- Added docker files for different revisions.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added branching example.

### Changed
- Updated dependencies for husky in rolling and galactic.
- Updated SM template and made example code clearly visible.
- Updated template to use Blackboard storage.
- Updated template to resolve the global data correctly.
- Updated `sm_name.hpp` file.
- Updated subscriber publisher components.
- Updated subscriber publisher components.
- Updated ci-build-source.yml file.
- Updated extension of imports.

### Fixed
- Fixed formatting.
- Fixed broken build.
- Fixed some formatting and templating on SrConditional.
- Fixed move trigger logic into headers.
- Fixed linting issues.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed docker for foxy and galactic.
- Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixed errors in undo motion navigation warehouse2.
- Fixed minor linking errors in foxy.
- Fixed warnings removal and more testing on navigation.

### Removed
- Removed use of node in the SM performance template.
- Removed disabled packages.
- Removed cpplint and cppcheck linters.
- Removed warnings from ament_lint_cmake.
- Removed disabled packages and updated workflows.
- Removed further packages.
- Removed ament_cpplint.
- Removed cpplint and cppcheck linters.

### Miscellaneous
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors.
- More on AWS demo.
- Several fixes.
- Replanning for all our examples.
- Improving undo motion navigation warehouse2.
- Tuning and fixes.
- Finishing warehouse2.
- Tuning warehouse3.
- Weird MoveIt not downloaded repo.
- Docker files for different revisions.
- Some reordering fixes.
- Backport to foxy.
- Bump ccache version.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Enable cppcheck.
- Enable cppcheck.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: Pablo Iñigo Blasco <pablo@ibrobotics.com>

```rst
Section_42
==========

Added
-----
- First ensure you have the necessary package installed by running:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```

Changed
-------
- Renamed `ros2 launch sm_three_some sm_three_some` to `ros2 launch sm_three_some sm_three_some.launch`
- Renamed header files and corrected format
- Updated doxygen-check-build.yml
- Created doxygen-deploy.yml
- Updated GitHub branch reference
- Updated package name and package.xml to pass liter
- Reset all versions to 0.0.0
- Ignored all packages except smacc2 and smacc2_msgs
- Updated changelogs
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" (commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61)
- Updated description table
- Updated table
- Renamed to smacc2 and smacc2_msgs
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Reverted markdowns to html
- Updated tracing.md to reflect new tracing event names
- Enabled build of missing rolling repositories
- Enabled Navigation2 for semi-binary build
- Removed galactic builds from master and kept only rolling, removing submodules and using .repos file
- Reactivated smacc2 nav clients for rolling via submodules
- Renamed tracing events
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library
- Added galactic CI setup and renamed rolling files (#58)
- Fixed source CI and corrected README overview (#62)
- Updated c_cpp_properties.json
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69)
- Updated doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Progress in aws navigation demo
- Feature/aws demo progress (#80)
- More on navigation
- sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 work (#85)
- sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- sm_atomic_performance_test_c_1 (#88)
- sm_multi_stage_1 (#90)
- More sm_multi_stage_1 (#91)
- Updated README.md with launch command

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh
- Tracing directory
- ManualTracing.md
- Additional cleanup
- Cleanup
- Deleted tracing.md
- Moved tracing.md to tracing directory
- Added setupTracing.sh to install necessary packages and configure tracing group
- Do not execute clang-format on smacc2_sm_reference_library package
- Cleaned up sm_atomic_24hr
- Cleaned up sm_atomic_performance_trace_1
- Cleaned up sm_atomic_24hr
- Minor formatting
- Fixing precommit
- More on performance tests
- Format improvements
- More on performance and other issues
- Format cleanup in sm_respira_1
- Format cleanup in sm_respira_test_2
- Format cleanup in sm_atomic_24hr
- Format cleanup in sm_atomic_performance_trace_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
- Format cleanup in sm_atomic_performance_test_a_2
- Format cleanup in sm_atomic_performance_test_a_1
- Format cleanup in sm_atomic_performance_test_c_1
- Format cleanup in sm_multi_stage_1
```

```rst
Section_43
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success (#81, #82, #92, #93, #94, #95, #98)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  Co-authored-by: Denis Štogl <denis@stogl.de>
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- New client behavior: cb_pause_slam (#98)
- New client behavior: cb_pause_slam client behavior for pausing SLAM operations (#98)
- New client behavior: sm_dance_bot_lite (#99)
- New client behavior: sm_dance_bot_lite for lightweight dance bot operations (#99)
- New client behavior: sm_dance_bot visualizing turtlebot3 (#101, #104)
- New feature: dance bot launch gazebo lidar choice, with cleaning and lidar show/hide option (#102)
- New feature: gazebo fixes to show the robot and the lidar (#104)
- New feature: sm_multi_stage_1 doubling (#103)

Changed
-------
- Corrected all linters and formatters (#82)
- Navigation parameters fixes on sm_dance_bot (#93, #95)
- Minor formatting improvements (#81, #82, #92, #93, #94, #95, #98)
- Merge and progress (#94)
- Fix format (#94)
- Updates yaml (#99)
- Minor hotfix (#100)
- Format fixes (#104)

Removed
-------
- Removed some compile warnings (#96)
- Removed unnecessary precommit fixes (#81)

```

*pabloinigoblasco*

```rst
Section_44
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
- Brettpac branch (#110)
- A3 (#113)
- More sm_multi_stage_1 (#114)
- MM (#115)
- Diverse improvements navigation and performance (#116)
- Feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting
- Remove neo_simulation2 package (#112)
  - Correct formatting
  - Enable source build on PR for testing
  - Adjust build packages of source CI
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Minor navigation improvements (#141)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
  - Formatting
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing errors introduced on formatting
  - Missing dependency
  - Fixing linting warnings
  - Progressing in the moveit migration testing
  - Updating format
  - Adding .reps dependencies and fixing build errors
  - Adding dependency to ur5 client
  - Docker refactoring
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
  - Add reliability QoS config

Changed
-------
- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Update package list (#142)
- Update readme (#164)

Fixed
-----
- Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Fix CI: format fix python version (#148)

Removed
-------
- Remove neo_simulation2 package (#112)
  - Removing sm_dance_bot_msgs
  - Pending references
- Removing parameters smacc (#147)
  - Workflows update
  - Workflow
```

*pabloinigoblasco*

Section_45
===========

Added
-----

- Added `sm_pubsub_1` (#169) with progress on moveit behaviors.
- Added `sm_pubsub_1 part 2` (#170) and continued work on it.
- Introduced `sm_advanced_recovery_1` renaming (#171).
- Implemented `sm_multi_stage_1` reworking (#172) with multistage modes, sequences, and steps.
- Added `Feature/aws navigation sm dance bot` (#174) with repo dependency updates and husky launch file in `sm_dance_bot`.
- Introduced `warehouse2` (#177) and `Waypoint Inputs` (#178).
- Added `sm_dance_bot_warehouse_3` (#181) and `Feature/sm warehouse 2 13 dec 2` (#182) with format changes and headless improvements.
- Implemented `Feature/cb pure spinning` (#188) with format changes and headless improvements.
- Added `Feature/planner changes 16 12` (#191) and `Feature/replanning 16 dec` (#193) with minor changes and fixes.
- Introduced `Feature/undo motion 20 12` (#196) and `Feature/sync 21 12` (#199) with format adjustments and feature enhancements.
- Added `Feature/warehouse2 22 12` (#200) and `Feature/warehouse2 23 12` (#201) with tuning, fixes, and finishing touches.
- Implemented `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/undo motion 20 12` (#198) with improvements in navigation and minor adjustments.
- Introduced `Feature/sync 21 12` (#199) with format fixes.
- Added `Feature/warehouse2 22 12` (#200) with format adjustments and finishing touches.
- Implemented `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and enhancements.
- Implemented `Feature/sync 21 12` (#199) with format adjustments.
- Added `Feature/warehouse2 22 12` (#200) with format fixes.
- Introduced `Feature/minor tune` (#203) with tuning and fixes.
- Added `Feature/warehouse2 23 12` (#201) with tuning, fixes, and

```rst
Section_46
==========

Added
-----
- Added ignition file and updated repos files.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as a dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh for automated installation.
- Added alternative ManualTracing.
- Added Dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added sm_respira_test_2.
- Added galactic CI setup and renamed rolling files.

Changed
-------
- Updated file for fake hardware simulation and added file for gazebo simulation.
- Updated warehouse3 feature/improvements (#228).
- Backported to foxy.
- Updated formatting, linking errors, and GitHub branch reference.
- Renamed header files and corrected format.
- Renamed to smacc2 and smacc2_msgs.
- Updated name of package and package.xml.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Updated changelogs.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More readme updates.
- Created new sm from sm_respira_1.
- Progressed in aws navigation demo.
- Reworked sm_advanced_recovery_1.

Fixed
-----
- Fixed trailing spaces.
- Corrected codespell.
- Corrected python linters warnings.
- Corrected formatters.
- Fixed bug in smacc2 component.
- Enabled cppcheck.
- Corrected formatting of python file.
- Fixed source CI and corrected README overview.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_47
==========

Added
-----

- Implement more sm_advanced_recovery_1 functionality (#84, #85, #86, #87)
- Develop sm_atomic_performance_test_a_1 and sm_atomic_performance_test_a_2
- Introduce sm_atomic_performance_test_c_1 (#88)
- Enhance sm_multi_stage_1 with additional features (#90, #91)
- Create new feature cb_wait_topic_message for asynchronous client behavior (#81, #82, #92)
- Add new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic (#82, #92)
- Implement navigation parameters fixes on sm_dance_bot (#93, #94, #95)
- Introduce cb_pause_slam feature (#98)

Changed
-------

- Update README.md and launch command
- Make several core improvements during navigation testing
- Correct all linters and formatters

Fixed
-----

- Resolve compile warnings (#96)
```

Section_48
----------

Added
~~~~~
- New client behavior for nav2: Now waits for nav2 nodes to subscribe to the /bond topic and ensures they are alive. Optional node selection available.
- Progress in AWS navigation demo.
- CB pause slam client behavior.
- Added SM Atomic SM generator.
- Rolling Docker environment to be executed from any environment.

Changed
~~~~~~~
- Navigation parameters fixes on sm_dance_bot.
- Minor hotfix.
- Format fixes.
- Minor format improvements.
- Adjust build packages of source CI.
- Minor navigation improvements.
- Format fix python version.

Fixed
~~~~
- Remove neo_simulation2 package: Corrected formatting and enabled source build on PR for testing.
- Move method after the method it calls to prevent recursion.
- Minor tuning to mitigate overshot issue cases.
- Fix CI: Format fix python version.
- Remove node creation and create only a logger.

Removed
~~~~~~~
- Remove neo_simulation2 package.
- Removing parameters smacc.
- Removing sm_dance_bot_msgs.

Other
~~~~~
- Various improvements in navigation and performance.
- Precommit cleanup run.
- Update package list.
- Noticed launch command was incorrect in README.md: Fixed launch command for sm_dance_bot_strikes_back and removed some comments.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Workflow updates.
- Pending references.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_49
==========

Added
-----
- Added .reps dependencies and fixed build errors.
- Added dependency to ur5 client.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added Waypoint Inputs.
- Added more Waypoints.
- Added missing file from warehouse2.

Changed
-------
- Updated format.
- Improved dockerfile for building local tests.
- Progressed in moveit migration testing.
- Progressed on move_it PR.
- Progressed on aws navigation and refactorings on navigation clients and behaviors.
- Finetuned waypoints.
- Tuned warehouse3.
- Tuned and fixed warehouse3 problems.
- Tuned and fixed minor issues.
- Tuned and fixed warehouse 3 problems, and other core improvements.
- Tuned and fixed formatting issues.

Fixed
-----
- Fixed compiling issues.
- Fixed pipeline error.
- Fixed broken master build.
- Fixed broken build.
- Fixed formatting.
- Fixed some linting warnings.
- Fixed some formatting and templating on SrConditional.
- Fixed move trigger logic into headers.
- Fixed lint.
- Fixed warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green.
- Fixed minor linking errors in foxy CI.
- Fixed minor broken build.

Removed
-------
- Removed test from main moveit cmake.

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
Section_50
==========

Added
-----
- Docker build files for all versions.
- Barrel demo.
- Barrel search build fix and Warehouse3.
- Progress in Barrel Husky.
- Feature/barrel - do not merge yet (#233).
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
- Workflow for checking doc build.
- Workflow for testing prerelease builds.
- Use manual deployment for now.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Added setupTracing.sh. Installs necessary packages and configures tracing group.
- Created alternative ManualTracing.
- Added new SM markdowns.
- Added a Dockerfile for Rolling and Galactic.
- README tutorial for Dockerfile.
- SMACC2 library performance tests improvements.
- SM Respira 1 format cleanup.
- SM Respira test 2.
- SM Atomic 24hr.
- SM Atomic performance trace 1.
- Clean up of SM Atomic 24hr.
- Optimized dependencies in move_base_z_planners_common.
- Renaming of event generator library.
- More on performance and other issues.
- More changes on performance tests.
- SM Reference Library reformatting.
- Clean up of SM Atomic 24hr.
- More SM Atomic 24hr cleanup.
- Minor formatting.
- Fix source CI and correct README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).

Changed
-------
- ROS2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.

Fixed
-----
- Fixing Docker for Foxy and Galactic.
- Fixing startup problems in Warehouse 3.
- Fixing broken build.
- Fix trailing spaces.
- Correct codespell.
- Correct Python linters warnings.
- Add Galactic CI build because Navigation2 is broken in Rolling.
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
- Correct formatting of Python file.
- Included necessary package and edited Threesome launch.
- Enable build of missing Rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove Galactic builds from master and keep only Rolling. Remove submodules and use .repos file.
- Some progress on Navigation Rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Reactivating SMACC2 nav clients for Rolling via submodules.
- Bug in SMACC2 component.
- Reverted markdowns to HTML.
- Edited tracing.md to reflect new tracing event names.
- Do not execute clang-format on SMACC2 SM Reference Library package.
- Update SMACC2 RTA command across READMEs.
- Correct trailing spaces.

Removed
-------
- Disable disabled packages.
- Ignore all packages except SMACC2 and SMACC2_msgs.
- Revert "Ignore all packages except SMACC2 and SMACC2_msgs".
```

```rst
Section_51
==========

Version 1.0.0 (2022-01-01)
---------------------------

Added
~~~~~

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
- Navigation parameters fixes on sm_dance_bot

Changed
~~~~~~~

- Sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 (#84)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Sm_multi_stage_1
- More sm_multi_stage_1 (#91)
- Correct all linters and formatters

Fixed
~~~~~

- Fix pre-commit
- Trying to fix Pre-Commit
- Merge and progress
- Fix format

Removed
~~~~~~~

- Update README.md: updated launch command
- Wait topic message client behavior (#81)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Authors
~~~~~~~

- Pablo Iñigo Blasco (pabloinigoblasco)
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_52
==========

Added
-----
- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are active. Optional node selection available.
- New feature: cb_wait_topic_message - Asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Progress in AWS navigation.
- Several core improvements during navigation testing.
- Format improvements.
- Gazebo fixes to show the robot and the lidar.

Fixed
-----
- Removed some compile warnings. (#96)
- Fixed formatting issues.
- Fixed lidar show/hide option.
- Fixed formatting in sm_dance_bot_lite.
- Fixed waypoint navigator bug by tuning to mitigate overshot issue cases.
- Fixed minor format issues.

Removed
-------
- Removed neo_simulation2 package.
- Removed merge markers from a Python file.
- Removed parameters in smacc.

Other
-----
- Base for the sm_aws_warehouse navigation.
- Precommit cleanup run.
- Updates yaml.
- Corrected formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Pending references cleanup.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```
Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_53
==========

Added
-----
- Add SM Atomic SM generator. (#143)
- Add SM Atomic SM generator. (#143)
- Add QOS durability to SmaccPublisherClient (#163)
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies for husky simulation.
- Add dependencies

```rst
Section_54
==========

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
- Delete tracing directory
- Moved tracing.md to tracing directory
- added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic

Changed:
--------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- changed wording "smacc application" to "SMACC2 library"
- updated mentions of SMACC/ROS to SMACC2/ROS2

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
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- some progress on navigation rolling

Removed:
--------
- minor linking errors foxy
- missing
- missing sm
- foxy ci
- minor broken build
- some reordering fixes
- minor
- fixing format and minor
- minor
- minor
- minor
- minor
- barrel demo
- fixing startup problems in warehouse 3
- fixing format and minor
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
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- barrel search build fix and warehouse3
- progress in barrel husky
- barrel demo
- cleanup
- cleanup
- cleanup
- edited tracing.md to reflect new tracing event names
- some changes on performance tests
- sm_respira_1 format cleanup
- sm_respira_1 format cleanup pre-commit
- sm_respira_test_2
- sm_respira_test_2
- more changes on performance tests

Authors:
--------
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
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait. (#82)
- Added navigation parameters fixes on sm_dance_bot. (#94)

Changed
-------
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated c_cpp_properties.json.
- Updated README.md launch command.
- Updated smacc2_rta command across readmes.

Fixed
-----
- Fixed source CI and corrected README overview. (#62)
- Fixed all linters and formatters.

Removed
-------
- Removed execution of clang-format on smacc2_sm_reference_library package.
- Removed trailing spaces.

Other
-----
- Refactored sm_reference_library.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_atomic_performance_trace_1.
- Minor formatting improvements.
- Noticed and removed a note left while producing these changes.
- Attempted pre-commit fixes.
- Progressed in aws navigation.
- Several core improvements during navigation testing.
- More work on navigation.
- Reworked sm_advanced_recovery_1.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed in aws navigation demo.
- Progressed

```rst
Section_56
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Optional selection of nodes to wait.
- Progress in AWS navigation demo.

Changed
-------
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior.
- Visualizing `turtlebot3` in `sm_dance_bot_lite`.
- Cleaning and lidar show/hide option in `sm_dance_bot_visualizing_turtlebot3`.
- Gazebo fixes to show the robot and lidar in various dance bot scenarios.
- `sm_multi_stage_1` doubling.

Fixed
-----
- Remove some compile warnings.
- Correct formatting in various files.
- Move method after the method it calls to prevent recursion.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Various core improvements during navigation testing.
- Precommit cleanup runs.
- Updates in YAML files.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Remove merge markers from a Python file.
- Waypoints navigator bug fixes.

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
- Added SM Atomic SM generator. (#143)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added Waypoint Inputs (#178)
- Added more Waypoints to sm_dance_bot_warehouse_3 (#184)
- Added SrConditional fixes and formatting (#168)
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files (#189)
- Added several fixes (#194)
- Added tuning to warehouse3 (#197)

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
- Update package list. (#142)
- Fixing launch command in README.md
- Fixing compiling issues
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Refactored to remove node creation and create only a logger (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Refactoring husky launch file in sm_dance_bot
- Finetuning waypoints (#187)

Fixed
-----

- Noticed launch command was incorrect in README.md

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing some comments in the past
- Removing parameters smacc
- Removing node creation and create only a logger
- Removing test workaround in dockerfile
- Removing test from main moveit cmake

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

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
- Galactic source build: Updated .repos file, action version, and upstream packages (#248).
- Restoring workflow files (#252).
- Restoring files (#253).
- Fix checkout branches for scheduled builds (#254).
- Feature/fixing husky build rolling (#257): Fixed husky project build on rolling.
- Feature/fixing husky build rolling (#258): Continued fixing husky project build on rolling.
- Update README.md (#262).
- Feature/fixing ur demos (#261): Fixed UR demos.
- Feature/fixing type string walker (#263): Fixed type string walker demo.
- Update README.md (#266).
- Update README.md (#267).
- Update README.md (#268).
- Significant update in Getting Started Instructions (#269): Significantly updated getting started instructions.
- Fix urls to index.ros.org (#284).
- Fix foxy source build config to use repos file from foxy branch (#285).
- Added spawn entity delays.

Changed
-------
- Correct name of source-build job and bump version of action (#242) (#247): Updated source-build job name and action version.

Fixed
-----
- Fixing warehouse 3 problems, and other core improvements (#204): Fixed warehouse 3 issues and made core improvements.
- Fixing docker for foxy and galactic: Fixed docker issues for foxy and galactic.
- Barrel search build fix and warehouse3: Fixed barrel search build and warehouse3 issues.
- Fixing startup problems in warehouse 3.
- Fixing format and minor issues.
- Fixing broken build.
- Fixing rolling build (#239): Fixed rolling build issues.
- Trying to fix dependencies.
- Fixing to focal by the moment.
- Fixing building issue.
- Fixing UR demo (#273).
- Fix: Initialized conditionFlag as false (#274).
- Precommit fix (#280): Fixed precommit issues.
- Fixing sm_dance_bot examples.
- Working on fix of image messages for husky_barrel demo.

Removed
-------
- Revert "Ignore packages which should not be released.": Reverted commit ignoring unreleased packages.
```

```rst
Section_59
==========

Added
-----
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Added smacc2_performance_tools.
- Performance tests improvements.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- More Readme Updates (#72).
- Created new sm from sm_respira_1 (#76).
- Base for the sm_aws_aarehouse navigation.
- Progress in AWS navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait.
- Corrected all linters and formatters.

Changed
-------
- Renamed folders, deleted tracing.md, edited README.md.
- Updated smacc2_rta command across readmes.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Minor formatting improvements.
- Updated launch command in README.md.

Removed
-------
- Removed trailing spaces.

Fixed
-----
- Corrected trailing spaces.

Co-authored-by
--------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Denis Štogl <destogl@users.noreply.github.com>.
- Denis Štogl <denis@stogl.de>.
```

```rst
Section_60
==========

Added
~~~~~
- New client behavior for nav2: Waits for nav2 nodes to subscribe to the /bond topic and confirms they are active. Optional node selection available.
- New feature: cb_wait_topic_message - Asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.

Changed
~~~~~~~
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Base for the sm_aws_warehouse navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- CB pause slam client behavior.
- Progress in AWS navigation.

Fixed
~~~~
- Minor format fixes.
- Remove some compile warnings.
- Minor hotfix.
- Cleaning and lidar show/hide option.
- More fixes in various features.
- Format fixes.
- Gazebo fixes for visualization.
- Correct formatting in code files.
- Adjust build packages for source CI.
- Additional linting and formatting.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.

Removed
~~~~~~~
- Removed neo_simulation2 package.

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

Section_61
-----------

### Added
- First working version of sm template and template generator. (#127)
- Add SM core test (#138)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Waypoint Inputs (#178)

### Changed
- Minor tweaks (#130)
- Minor navigation improvements (#141)
- Update package list. (#142)
- Update readme (#164)
- Finetuning waypoints (#187)

### Fixed
- Fix CI: format fix python version (#148)
- Fixing pipeline error
- Fixing broken master build
- SrConditional fixes and formatting (#168)

### Removed
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Remove node creation and create only a logger. (#149)

### Miscellaneous
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back and removed some comments. 
- Precommit cleanup
- Workflows update
- Docker refactoring
- Repos dependency
- Progress on move_it PR
- Improving dockerfile for building local tests
- Removing test from main moveit cmake
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing compiling issues
- Progress on moveit migration testing
- Progress on moveit
- Progress in the sm_dance_bot tests (#135)
- Some more progress on markers cleanup
- Minor format issues (#134)
- Minor tuning to mitigate overshot issue cases
- Minor configuration
- Minor changes
- More readme updates
- More
- More changes in sm_dance_bot
- More refinement in sm_dance_bot
- More changes and headless
- Merge
- Headless and other fixes
- Default values
- More on aws demo
- More on aws navigation and some other refactorings on navigation clients and behaviors
- More testing on moveit
- More testing on moveit behaviors
- More changes in sm_dance_bot
- More changes in sm_dance_bot
- More changes in sm_dance_bot
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
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless
- More changes and headless

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
- Add galactic CI build due to Navigation2 issues in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency for include resolution
- Create workflow for testing prerelease builds
- Use docs/ as source and output directory for documentation
- Dockerfile with ROS distro as argument
- Open new folder for additional tracing contents
- Added setupTracing.sh script for automated installation
- Created alternative ManualTracing
- Added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Update smacc_sm_reference_library/sm_atomic/README.md
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- Optimized dependencies in move_base_z_planners_common
- Renaming of event generator library
- Add galactic CI setup and rename rolling files (#58)

Changed
-------
- Rename header files and correct format
- Correct GitHub branch reference
- Update name of package and package.xml
- Reset all versions to 0.0.0
- Update description table
- Update table
- Copy initial docs
- Change extension of imports
- Correct formatting of python file
- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch"
- Update changelogs
- Update smacc2 and smacc2_msgs names
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cppcheck
- Enable cpp

```rst
Section_63
==========

Added
~~~~~
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success. Also, added a new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait. (#81, #82, #92)

Changed
~~~~~~~
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch. (#69)
- Updated doxygen links. (#70)
- Corrected all linters and formatters. (#82)

Fixed
~~~~
- Fixed source CI and corrected README overview. (#62)
- Fixed pre-commit issues. (#83, #84, #86, #87, #88, #89, #90, #91)
- Fixed formatting issues. (#80, #81, #82, #92, #93, #94, #95)
- Fixed navigation parameters on sm_dance_bot. (#93, #95)

Removed
~~~~~~~
- Removed note that was not needed. (#69)

Authors
~~~~~~~
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_64
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and waiting for them to be alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- Gazebo fixes for showing the robot and the lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Source build enabled on PR for testing.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- First working version of `sm` template and template generator.
- Minor tuning to mitigate overshot issue cases.
- SM core test added.
- Minor navigation improvements.
- Using local action messages.
- Pending references resolved.

Changed
-------

- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Formatting improvements.
- Cleaning and lidar show/hide option.
- More fixes in `sm_dance_bot`.
- Polishing `sm_dance_bot` and `s-pattern`.
- Minor tweaks.
- Minor format issues.

Fixed
-----

- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.
- Typo correction: "Finnaly" to "Finally".

Removed
-------

- `neo_simulation2` package.
- `sm_dance_bot_msgs`.

Contributors
------------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_65
==========

Added
-----
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)

Changed
-------
- Renamed navigation 2 stack
- Updated package list. (#142)
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Rolling Docker environment to be executed from any environment (#154)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----
- Noticed launch command was incorrect in README.md
- Fix CI: format fix python version (#148)
- Fix formatting.
- Fix compiling issues
- Fixing pipeline error
- Fixing broken master build
- Fixing broken build
- Several fixes (#194)
- Tuning and fixes (#202)

Removed
-------
- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed test from main moveit cmake

Collaborators
-------------
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_66
==========

Added
-----
- Added missing file from warehouse2 (#205)
- Added workflow for checking doc build
- Added setupTracing.sh to automate installation of ros-rolling-ros2trace
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Tuning and fixes
- Minor tune
- Fixed warehouse 3 problems and other core improvements (#204)
- Fixed warehouse 3 problems and core improvements to remove deadlocks and make continuous integration green
- Updated subscriber publisher components
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Replanning for all examples
- Reverted wording "smacc application" to "SMACC2 library"
- Reactivated smacc2 nav clients for rolling via submodules
- Renamed tracing events
- Bug fix in smacc2 component
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Some progress on navigation rolling
- Performance tests improvements
- More on performance and other issues
- Format cleanup in sm_respira_1 and sm_atomic_24hr
- Cleaned up sm_atomic_24hr
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library
- Minor formatting changes
- Renamed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)

Fixed
-----
- Minor broken build
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Enable cppcheck
- Correct formatting of python file
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Cleaned up sm_atomic_24hr
- Optimized deps in move_base_z_planners_common
- Renamed event generator library
- Minor formatting
- Fix source CI and correct README overview (#62)

Removed
-------
- Disable ament_cpplint
- Disable some packages and update workflows
- Ignore further packages
- Disable cpplint and cppcheck linters
- Disable disabled packages
- Ignore all packages except smacc2 and smacc2_msgs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
```


*pabloinigoblasco*

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
- Minor improvements during navigation testing
- Formatting improvements
- Progress in AWS navigation demo
- Feature/aws demo progress (#80)
- More on navigation
- Sm_advanced_recovery_1 reworked (#83)
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
- Wait topic message client behavior (#81)
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait
- Correct all linters and formaters
- Feature/wait nav2 nodes client behavior (#82)
- Progress in AWS navigation demo
- Navigation parameters fixes on sm_dance_bot

Changed
-------

- Update README.md with updated launch command

Removed
-------

- None
```

```rst
Section_68
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- Gazebo fixes to show the robot and lidar.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `sm_multi_stage_1` working progress.
- `sm_dance_bot` visualizing `turtlebot3`.
- `sm_dance_bot_lite` improvements.
- `slam` toggle client behaviors and `slam_toolbox` components progress.
- First working version of `sm` template and template generator.
- Minor tuning to mitigate overshot issue cases in waypoints navigator.
- Minor navigation improvements.
- Using local action messages.
- Navigation 2 stack renaming.

Changed
-------
- Progress in AWS navigation demo.
- Minor format improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Several core improvements during navigation testing.
- Additional linting and formatting.
- Move method after the method it calls to prevent recursion.
- Polishing `sm_dance_bot` and `s-pattern`.
- Refinement in `sm_dance_bot`.

Fixed
-----
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Remove merge markers from a Python file.
- Minor format issues.

Removed
-------
- `sm_dance_bot_msgs` package.

Contributors
------------
- Pablo Iñigo Blasco (@pabloinigoblasco)
- Ubuntu 20-04-02-amd64 (@brett@robosoft.ai)
```

```rst
Section_69
==========

Added
~~~~~
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added Waypoint Inputs (#178)

Changed
~~~~~~~
- Updated package list (#142)
- Fixed launch command for sm_dance_bot_strikes_back in README.md
- Refactored SM dance bot strikes back (#152)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Redoing sm_dance_bot_warehouse_3 waypoints (#184)
- Finetuned waypoints (#187)
- Improved undo motion navigation in warehouse2 (#198)
- Tuned warehouse3 (#197)

Fixed
~~~~~
- Fixed CI: format fix python version (#148)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed broken build in AWS navigation
- Fixed formatting in several features
- Fixed errors in undo motion and warehouse3
- Fixed warehouse 3 problems and other core improvements (#204)

Removed
~~~~~~~
- Removed parameters smacc (#147)
- Removed node creation and created only a logger (#149)
- Removed test from main moveit cmake
- Removed some comments in the past
- Removed some linting warnings
- Removed test workaround in minor dockerfile
- Removed some build errors
- Removed line in refactor
- Removed some formatting errors

Authors
~~~~~~~
- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_70
==========

Added
~~~~~
- Added missing file from warehouse2 (#205).
- Added mergify rules file.
- Added Autoware Auto Msgs into not-released dependencies. (#220).
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Added a dockerfile for Rolling and Galactic.

Changed
~~~~~~~
- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch".
- Changed wording "smacc application" to "SMACC2 library".

Fixed
~~~~
- Fixed warehouse 3 problems and other core improvements to remove deadlocks.
- Fixed weird moveit not downloaded repo.
- Fixed minor broken build.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed rolling builds (#222).
- Fixed docker for foxy and galactic.
- Fixed warnings (#213).
- Fixed formatting of python file.

Removed
~~~~~~~
- Removed example things from Foxy CI setup. (#214).
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed tracing directory.

Other Changes
~~~~~~~~~~~~~
- Backported changes to foxy.
- Updated subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components mostly developed for autoware demo.
- Autoware demo.
- Foxy CI.
- Branching example.
- Replanning for all our examples.
- Reverted markdowns to html.
- Additional cleanup.
- Cleanup.
- Edited tracing.md to reflect new tracing event names.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.

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
Section_71
==========

Added
-----

- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.

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
- Fixed pre-commit issues.

Removed
-------

- Removed execution of clang-format on smacc2_sm_reference_library package.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_72
===========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and waiting for them to be alive. Optionally, you can select the nodes to wait.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.
- Gazebo fixes to show the robot and the lidar.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.
- Going forward in testing `sm_dance_bot` introducing `slam` pausing/resuming functionality.
- Polishing `sm_dance_bot` and `s-pattern`.
- Noticed typo: "Finnaly" corrected to "Finally".
- More refinement in `sm_dance_bot`.

Fixed
-----

- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.

Removed
-------

- `neo_simulation2` package.

Collaborators
-------------

- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by: pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_73
==========

Added
~~~~~
- Feature/sm dance bot refine 2 (#132)
  - More changes in sm_dance_bot
  - Minor build fix
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
  - Some more progress on markers cleanup
- Sm_dance_bot_lite (#136)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
  - Formatting
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes so robot can complete course
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing some errors introduced on formatting
  - Missing dependency
  - Progressing in the moveit migration testing
  - Updating format
  - Adding .reps dependencies and also fixing some build errors
  - Repos dependency
  - Adding dependency to ur5 client
- Update readme (#164)
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Add QOS durability to SmaccPublisherClient (#163)
  - Add reliability QoS config
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
- Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and some other refactorings on navigation clients and behaviors
  - More on aws demo
- Minor changes (#175)
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Warehouse2 progress (#179)
- Format (#180)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
- Brettpac branch (#184)
  - Redoing sm_dance_bot_warehouse_3 waypoints
  - More waypoints
- SrConditional fixes and formatting (#168)
  - Fix: Some formatting and templating on SrConditional
  - Fix: Move trigger logic into headers
  - Fix: Lint
- Feature/warehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
  - Pure spinning behavior missing files
- Feature/cb pure spinning (#189)
- Minor changes (#190)
- Feature/planner changes 16 12 (#191)
  - More fixes
- Feature/replanning 16 dec (#193)
  - Replanning for all our examples
- Several fixes (#194)
- Minor changes (#195)
- Feature/undo motion 20 12 (#196)
  - Replanning for all our examples

Changed
~~~~~~~
- Fix CI: Format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Improving Dockerfile for building local tests

Removed
~~~~~~~
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit CMake
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
- Format issues
- Fixing warehouse 3 problems and other core improvements (#204)
- Minor tune
- Minor broken build
- Some reordering fixes
- Minor formatting fixes
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Add galactic CI build due to Navigation2 issues in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency
- Disable ament_cpplint, cpplint, and cppcheck linters
- Bump ccache version
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licenses
- Disable some packages and update workflows
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Update SM template and make example code visible
- Update template to use Blackboard storage
- Update template to resolve global data correctly
- Update sm_name.hpp
- Rename header files and correct format
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Update name of package and package.xml
- Reset all versions to 0.0.0
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table
- Update table
- Copy initial docs
- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch"
- Added instructions for necessary package installation
- Use docs/ as source and output directory for documentation
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference
- Execute on master update
- Update tracing/ManualTracing.md
- Edit wording from "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md

Fixed
-----
- Fix other build issues
- Fixing docker for foxy and galactic
- Remove use of node in the sm performance template
- Disable disabled packages
- Ignore all packages except smacc2 and smacc2_msgs

Removed
-------
- Weird moveit not downloaded repo
- Missing sm
- Missing
- Missing
- Delete tracing directory

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
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Renamed event generator library.

Fixed
-----
- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_atomic_24hr.
- Fixed source CI and corrected README overview.
- Corrected all linters and formatters.

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
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws

```rst
Section_76
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- `sm_multi_stage_1` doubling.
- Gazebo fixes for showing the robot and lidar.
- AWS demo.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in AWS navigation demo.
- Minor hotfixes.
- Cleaning and lidar show/hide option for `sm_dance_bot`.
- Formatting improvements for various features.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- `smacc2::deep_history` syntax.

Fixed
-----
- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- Removed merge markers from a Python file.

Removed
-------
- `neo_simulation2` package.

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
```

Section_77
===========

Added
-----
- Introducing slam pausing/resuming functionality to sm_dance_bot (#124, #125, #126, #128, #129, #131, #132, #135, #136, #137, #138, #141, #144, #145, #147, #149, #152, #155, #163, #164, #165, #166, #167, #169, #170, #171, #172, #174, #175, #177, #178, #179, #180, #181, #182, #184, #185, #186)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added QOS durability to SmaccPublisherClient (#163)
- Waypoint Inputs (#178)

Changed
-------
- Move method after the method it calls to prevent recursion (#126)
- Renamed state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Refactored SmaccPublisherClient to add QOS durability (#163)

Fixed
-----
- Minor format issues (#134, #140, #148, #164, #180, #186)
- Fixed launch command in README.md (#148)
- Fixed CI format for python version (#148)
- Fixed compiling issues (#154)
- Fixed broken master build (#167, #174)
- Fixed pipeline error (#167)
- Fixed warehouse2 progress (#179)
- Fixed SrConditional formatting and templating (#168)

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

```rst
Section_78
==========

Added
-----
- Finetuned waypoints (#187) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Added feature "cb pure spinning" (#188)
- Added feature "pure spinning behavior missing files"
- Added feature "planner changes 16 12" (#191)
- Added feature "replanning 16 dec" (#193)
- Added feature "undo motion 20 12" (#196)
- Added feature "sync 21 12" (#199)
- Added feature "warehouse2 22 12" (#200)
- Added feature "warehouse2 23 12" (#201)
- Added feature "minor tune" (#203)
- Added feature "improvements warehouse3" (#228)
- Added barrel demo
- Added multiple controllable leds plugin
- Added progress in husky demo
- Added ignition file and updated repos files
- Added partial changes for ament_cpplint
- Added tf2_ros as dependency
- Added galactic CI build
- Added workflow for checking doc build
- Created doxygen-deploy.yml
- Created workflow for testing prerelease builds
- Renamed header files and corrected format
- Renamed to smacc2 and smacc2_msgs
- Updated name of package and package.xml

Changed
-------
- Updated file for fake hardware simulation and added file for gazebo simulation
- Updated file for fake hardware simulation and added file for gazebo simulation (#224)
- Updated file for fake hardware simulation and added file for gazebo simulation (#224)
- Updated file for fake hardware simulation and added file for gazebo simulation
- Changed ros2 launch sm_three_some to ros2 launch sm_three_some.launch

Fixed
-----
- Fixed broken source build (#227)
- Fixed format and minor issues
- Fixed trailing spaces
- Fixed Focal-Rolling builds by fixing the version of rosdep yaml
- Fixed codespell
- Fixed python linters warnings
- Fixed navigation behaviors
- Fixed trailing spaces
- Fixed formatting of python file

Removed
-------
- Removed weird moveit not downloaded repo
- Removed some reordering fixes
- Removed minor broken build
- Removed some packages and updated workflows
- Removed further packages
- Removed disabled packages
- Removed disabled packages
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters
- Removed disabled cpplint and cppcheck linters

```rst
Section_79
==========

Added
-----
- Reset all versions to 0.0.0.
- Update changelogs.
- Added setupTracing.sh script for automated installation of tracing packages.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added Dockerfile for Rolling and Galactic ROS distros.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added new feature cb_wait_topic_message for asynchronous client behavior.
- Added feature wait nav2 nodes client behavior.

Changed
-------
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Updated description table.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc2_rta command across readmes.
- Renamed event generator library.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reformatted sm_reference_library.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed tracing events.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Fixed
-----
- Bug in smacc2 component.

Removed
-------
- Manual installation of ros-rolling-ros2trace now automated in setupTracing.sh.
- Removed galactic builds, keeping only rolling. Removed submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_80
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes to subscribe to the `/bond` topic and waits for them to be alive. Optional node selection.
- Base for the `sm_aws_warehouse` navigation.

Changed
~~~~~~~
- Minor format improvements.
- Progress in AWS navigation demo.
- Several core improvements during navigation testing.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.

Fixed
~~~~
- Corrected all linters and formatters.
- Removed some compile warnings.

Removed
~~~~~~~
- Unused code.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_81
==========

Added
-----

- Brettpac branch (#111)
- Added a3 (#113)
- Removed neo_simulation2 package (#112)
- Enabled source build on PR for testing
- Adjusted build packages of source CI
- Added diverse improvements in navigation and performance (#116)
- Introduced feature/diverse improvements in navigation performance (#117)
- Removed merge markers from a python file (#119)
- Added feature/slam toggle and smacc deep history (#122)
- Implemented dance bot s pattern (#128)
- Added first working version of sm template and template generator (#127)
- Added feature/sm dance bot refine (#131)
- Added feature/sm dance bot refine 2 (#132)
- Resolved compile warnings (#137)
- Added SM core test (#138)
- Added minor navigation improvements (#141)
- Added using local action messages (#139)
- Added pending references
- Renamed navigation 2 stack
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Updated package list (#142)
- Removed parameters smacc (#147)
- Fixed launch command for sm_dance_bot_strikes_back
- Fixed CI: format fix python version (#148)
- Added SM Atomic SM generator (#143)
- Rolled Docker environment to be executed from any environment (#154)
- Refactored feature/sm dance bot strikes back (#152)
- Slightly changed waypoint 4 and iterations for robot course completion (#155)
- Migrated moveit client to smacc2 (#151)
- Added QOS durability to SmaccPublisherClient (#163)
- Added feature/testing moveit behaviors (#167)
- Added sm_pubsub_1 (#169)
- Added sm_pubsub_1 part 2 (#170)
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)

Changed
-------

- Corrected formatting for neo_simulation2 package removal
- Improved navigation, slam toggle client behaviors, and slam_toolbox components
- Updated smacc2::deep_history syntax
- Introduced slam pausing/resuming functionality in testing sm_dance_bot
- Polished sm_dance_bot and s-pattern
- Moved method after the method it calls to prevent recursion
- Mitigated overshot issue cases in waypoints navigator
- Cleaned up markers in sm_dance_bot tests
- Updated format in moveit migration testing
- Added .reps dependencies and fixed build errors
- Added dependency to ur5 client
- Refactored Docker environment for local tests
- Improved Dockerfile for building local tests
- Fixed compiling issues
- Updated readme
- Added reliability qos config to SmaccPublisherClient
- Fixed pipeline error and broken master build

Removed
-------

- Removed neo_simulation2 package
- Removed sm_dance_bot_msgs
- Removed test from main moveit cmake

Fixed
-----

- Noticed typo in changelog (Finnaly > Finally)
- Fixed launch command for sm_dance_bot_strikes_back
- Fixed CI: format fix python version
- Removed line in reliability qos config
```

**Autor:** Pablo Iñigo Blasco (pabloinigoblasco)

Section_82
==========

Added
-----

- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency for husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Added progress on aws navigation and refactorings on navigation clients and behaviors.
  - Added more on aws demo.
  - Added fixing broken build.

Changed
-------

- Updated dependencies for husky in rolling and galactic.
- Minor changes (#175).
- Updated Waypoint Inputs (#178).
- Updated warehouse2 progress (#179).
- Updated sm_dance_bot_warehouse_3 (#181).
- Updated Brettpac branch (#184).
- Redone sm_dance_bot_warehouse_3 waypoints.
- Fine-tuned waypoints (#187).
- Improved undo motion navigation warehouse2.
- Tuned warehouse3 (#197).
- Tuned and fixed warehouse2 (#202).
- Tuned and fixed minor issues (#203).
- Fixed warehouse 3 problems and other core improvements to remove deadlocks, also making continuous integration green.
- Added missing file from warehouse2 (#205).
- Updated subscriber publisher components.
- Refined cp subscriber cp publisher.
- Added more components to smacc core mostly developed for autoware demo.
- Improved autoware demo.
- Fixed docker for foxy and galactic.
- Fixed startup problems in warehouse 3.
- Updated docker build files for all versions.
- Fixed broken build.

Fixed
-----

- Fixed formatting.
- Fixed some formatting and templating on SrConditional.
- Moved trigger logic into headers.
- Linted code.
- Fixed several issues.
- Fixed format issues.
- Fixed errors.
- Fixed missing files.
- Fixed broken build.

Removed
-------

- Removed pure spinning behavior missing files.

Contributors
------------

- Denis Štogl
- Pablo Iñigo Blasco
- pabloinigoblasco
