# Changelog for package backward_local_planner

## Version 2.3.16 (2023-07-16)
- Merged branch 'humble' from [robosoft-ai/SMACC2](https://github.com/robosoft-ai/SMACC2) into humble
- Brettpac branch:
  - Attempted to fix weird issue with ros buildfarm
  - More work on buildfarm issue
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

## Version 2.3.6 (2023-03-12)

## Version 1.22.1 (2022-11-09)
- Pre-release
- Contributors: pabloinigoblasco

## Version 0.3.0 (2022-04-04)

## Version 0.0.0 (2022-11-09)
- Humble check and progress in migration to humble
- Reverted "Ignore packages which should not be released."
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Performance tests improvements
- Format cleanup in various packages
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library
- Added new features and improvements for AWS navigation demo
- Added new client behavior for waiting topic messages
- Corrected linters and formatters
- Co-authored by various contributors including brettpac and Denis Štogl

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for Nav2: `cb_wait_nav2_nodes`: waits for Nav2 nodes subscribing to the `/bond` topic and ensures they are alive.

Changed
-------

- Improved core functionality during navigation testing.
- Formatting enhancements throughout the codebase.
- Progress made in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.
- `sm_dance_bot_lite` now visualizes TurtleBot3.
- `sm_multi_stage_1` functionality enhanced.

Fixed
-----

- Removed some compile warnings.
- Fixed formatting issues.
- Gazebo fixes for various components.
- `neo_simulation2` package removed.

Removed
-------

- `neo_simulation2` package removed.

Other
-----

- Various minor updates and fixes.
- Precommit cleanup run.
- Workflow updates.
- Collaborator contributions from Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
```

Section_3
=========

Version 0.1.0 (2022-01-01)
--------------------------

### Added
- Diverse improvements in navigation and performance (#116)
- Additional linting and formatting
- Feature to toggle slam and deep history in smacc (#122)
- First working version of sm template and template generator (#127)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added QOS durability to SmaccPublisherClient (#163)
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools
- Feature to test moveit behaviors (#167)
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors
- Waypoint Inputs (#178)

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Resolved compile warnings (#137)
- Minor navigation improvements (#141)
- Using local action messages
- Renamed navigation 2 stack
- Removed node creation and create only a logger (#149)
- Minor changes in warehouse2 (#177)

### Fixed
- Minor format issues (#134)
- Fixed launch command for sm_dance_bot_strikes_back in README.md
- Fixed CI format for python version (#148)
- Fixed broken master build
- Fixed pipeline error

### Removed
- Removed merge markers from a python file (#119)
- Removed parameters smacc
- Removed test from main moveit cmake
- Removed sm_dance_bot_msgs
- Removed some comments in the past from README.md

Version 0.2.0 (2022-02-01)
--------------------------

### Added
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components (#122)
- Introduced slam pausing/resuming functionality in sm_dance_bot
- More refinement in sm_dance_bot
- More progress on markers cleanup
- Added remaining SVGs to READMEs
- Added dependency to ur5 client
- Added .reps dependencies and fixed some build errors
- Added reliability qos config

### Changed
- Slight waypoint 4 and iterations changes so the robot can complete the course (#155)
- Progressing in the moveit migration testing
- Updated format
- Updated readme

### Fixed
- Minor tuning to mitigate overshot issue cases
- Fixed compiling issues

### Removed
- Removed some comments in the past from README.md

Version 0.3.0 (2022-03-01)
--------------------------

### Added
- Progress on moveit
- More testing on moveit behaviors
- Progress on sm_pubsub_1
- Progress on sm_pubsub_1 part 2
- Progress on sm_advanced_recovery_1 renaming
- Progress on sm_multi_stage_1 reworking
- Progress on sm_multi_stage_1 most
- Progress on sm_multi_stage_1 finishing touches 1
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- Progress on warehouse2
- Progress on sm_dance_bot_warehouse_3

### Changed
- Initial migration to smacc2
- Fixed some errors introduced on formatting
- Fixed some more linting warnings
- Fixed compiling issues
- Updated dependencies for husky in rolling and galactic

### Removed
- Removed test from main moveit cmake

### Authors
- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```rst
Section_4
=========

Added
-----
- Feature/sm warehouse 2 13 dec 2 (#182)
  - Implemented new warehouse feature.
- Feature/wharehouse2 dec 14 (#185)
  - Added enhancements to warehouse functionality.
- Feature/cb pure spinning (#188)
  - Introduced pure spinning behavior.
- Feature/planner changes 16 12 (#191)
  - Implemented planner changes.
- Feature/replanning 16 dec (#193)
  - Added replanning functionality.
- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation.
- Feature/sync 21 12 (#199)
  - Addressed synchronization issues.
- Feature/warehouse2 22 12 (#200)
  - Completed warehouse2 development.
- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2.
- Feature/minor tune (#203)
  - Made minor adjustments.
- Fix trailing spaces, codespell, linters warnings (#206)
  - Resolved various code issues.
- Update changelogs
  - Updated changelogs to reflect changes.

Changed
-------
- Updated description table.
- Updated table format.
- Renamed header files and corrected format.
- Changed extension of imports.
- Renamed to smacc2 and smacc2_msgs.
- Updated GitHub branch reference.
- Updated package name and package.xml.
- Reset all versions to 0.0.0.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Reverted markdowns to html format.
- Enabled build of missing rolling repositories.

Removed
-------
- Deleted tracing directory.
- Removed manual installation of ros-rolling-ros2trace.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

Section 5
----------

Added
-----
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links.
- Updated README.md launch command.

Fixed
----
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Corrected README overview. (#62).
- Fixed source CI.
- Corrected all linters and formatters.

Removed
-------
- Do not execute clang-format on smacc2_sm_reference_library package.

Other
-----
- Some progress on navigation rolling.
- Performance tests improvements.
- More changes on performance tests.
- Minor formatting.
- Progress in aws navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Attempting precommit fixes.
- Fixing precommit.
- Minor format.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.

```rst
Section_6
=========

Added
-----

- New client behavior `cb_wait_topic_message` added for nav2. It allows waiting for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Nodes to wait can be optionally selected.
- New feature `cb_pause_slam` added for pausing SLAM functionality.
- New feature `sm_dance_bot_lite` introduced for visualizing TurtleBot3 in Gazebo.
- New feature `sm_multi_stage_1` doubling functionality added.
- New feature `sm_dance_bot_strikes_back` gazebo fixes implemented.

Changed
-------

- Progress made in AWS navigation demo.
- Navigation parameters fixed for `sm_dance_bot`.
- Formatting improvements across various features.
- Gazebo fixes implemented to show the robot and lidar in different features.

Fixed
-----

- Compile warnings removed.
- Recursion issue fixed by moving a method after the one it calls.

Removed
-------

- `neo_simulation2` package removed.

Other
-----

- Various core improvements made during navigation testing.
- `smacc2::deep_history` syntax introduced for `slam_toolbox` components.
- Additional linting and formatting applied.
- Merge markers removed from a Python file.

Contributors
------------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_7
==========

Added
-----
- First working version of sm template and template generator. (#127)
- minor tweaks (#130)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- minor navigation improvements (#141)
- using local action messages (#139)
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- warehouse2 (#177)
- finetuning waypoints (#187)
- Feature/cb pure spinning (#188)

Changed
-------
- polishing sm_dance_bot and s-pattern
- noticed typo "Finnaly" corrected to "Finally"
- more refinement in sm_dance_bot
- minor format issues (#134)
- minor format issues (#180)
- fixing some errors introduced on formatting
- fixing some more linting warnings
- updating format
- fixing compiling issues
- update readme (#164)
- initial state machine transition timestamp (#165)
- moved reference library SMs to smacc2_performance_tools (#166)
- progress on moveit
- fixing pipeline error
- fixing broken master build
- husky launch file in sm_dance_bot
- Update dependencies for husky in rolling and galactic
- more on aws demo
- warehouse2 progress (#179)
- SrConditional fixes and formatting (#168)

Fixed
-----
- build fix
- waypoints navigator bug (#133)
- fixing broken build

Removed
-------
- removing sm_dance_bot_msgs
- removing parameters smacc
- removing test from main moveit cmake
- removing node creation and create only a logger

Co-authored-by
--------------
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
- First ensure you have the necessary package installed
- Rename header files and correct format
- Change extension of imports
- Correct GitHub branch reference
- Update name of package and package.xml to pass liter
- Correct wording "smacc application" to "SMACC2 library"

Fixed
-----
- Several fixes (#194)
- Tuning warehouse3 (#197)
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
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build

Removed
-------
- Delete tracing directory
- Reverted markdowns to html
- Additional cleanup
- Cleanup
- Edited tracing.md to reflect new tracing event names

Co-authored-by
--------------
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>

pabloinigoblasco
```

```rst
Section_9
=========

Added
-----
- Added `smacc2_performance_tools`.
- Added `galactic CI setup` and renamed rolling files. (#58)
- Added new feature, `cb_wait_topic_message`: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, `wait nav2 nodes`, subscribing to the `/bond` topic and waiting for them to be alive. You can optionally select the nodes to wait.

Changed
-------
- Updated mentions of `SMACC/ROS` to `SMACC2/ROS2`.
- Renamed folders, deleted `tracing.md`, and edited `README.md`.
- Updated `smacc2_rta` command across readmes.
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69).
- Corrected trailing spaces.
- Optimized dependencies in `move_base_z_planners_common`.
- Renamed event generator library.
- Updated `c_cpp_properties.json`.
- Updated `README.md` launch command.

Fixed
-----
- Do not execute `clang-format` on `smacc2_sm_reference_library` package.
- Fixed source CI and corrected `README` overview. (#62).
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use `.repos` file.

Other
-----
- Some progress on `navigation rolling`.
- More changes on performance tests.
- Noticed a note that was not removed.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- Progress in `aws navigation demo`.
- Progress in `aws navigation`.
- Progressing in `aws navigation`.
- Base for the `sm_aws_aarehouse navigation`.
- More on performance and other issues.
- More on navigation.
- More `Readme` updates.
- More `Readme`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.
- More on `navigation`.

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
- New client behavior `cb_wait_topic_message` added for nav2. It allows waiting for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Nodes to wait for can be optionally selected.
- New feature `cb_pause_slam` added for pausing SLAM functionality.
- New feature `sm_dance_bot_lite` added for visualizing Turtlebot3 in Gazebo.
- New feature `sm_multi_stage_1` added for multi-stage functionality.

Changed
-------
- Progress made in AWS navigation demo.
- Navigation parameters fixed for `sm_dance_bot`.
- Formatting improvements made.
- Gazebo fixes implemented to show the robot and lidar.

Fixed
-----
- Compile warnings removed.
- Recursion issue fixed by moving a method after the one it calls.

Removed
-------
- `neo_simulation2` package removed.

Other
-----
- Various core improvements made during navigation testing.
- Precommit cleanup run performed.
- Source build enabled on PR for testing.
- Adjustments made to build packages of source CI.
- Additional linting and formatting applied.
- Merge markers removed from a Python file.
- `smacc2::deep_history` syntax introduced for deep history functionality.
- Testing progress made for `sm_dance_bot` with slam pausing/resuming functionality.
- `sm_dance_bot` changes and fixes implemented.
- `sm_dance_bot_s_pattern` feature added.
```

Section 11
===========

Added
-----

- First working version of sm template and template generator. (#127)
- Minor tweaks (#130)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Warehouse2 (#177)
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)

Changed
-------

- Polishing sm_dance_bot and s-pattern
- Noticed typo "Finnaly" corrected to "Finally"
- More changes in sm_dance_bot
- More refinement in sm_dance_bot
- Minor format issues (#134)
- Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
- Minor configuration
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Progress on move_it PR
- Improving Dockerfile for building local tests
- Fixing compiling issues
- Moved reference library SMs to smacc2_performance_tools (#166)
- Pre-commit cleanup
- Finishing touches 1
- Readme updates
- More readme updates
- Fix: Add a missing colon
- Refactor: Remove line
- Feat: Add reliability QoS config

Fixed
-----

- Waypoints navigator bug (#133)
- Fix CI: Format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Fixing pipeline error
- Fixing broken master build

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit CMake
- Removing parameters smacc
- Removing test from main moveit CMake
- Removing some comments in the past
- Removing some comments in the past

Collaborators
-------------

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
- Foxy backport (#206)
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
- Include necessary package and edit Threesome launch
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update
- Reset all versions to 0.0.0
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table.
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

Changed
-------
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Minor broken build
- Some reordering fixes
- Format issues
- Finishing warehouse2
- Minor formatting fixes

Fixed
-----
- Headless and other fixes
- Default values
- Pure spinning behavior missing files
- Updating subscriber publisher components
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Fix minor linking errors foxy

Removed
-------
- Merge
- Minor
- More fixes
- Missing
- Missing sm
- Foxy ci
- Fix
- Minor broken build

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
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Minor formatting improvements.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed formatting in sm_advanced_recovery_1.
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other
-----
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Made some progress on navigation rolling.
- Made progress in aws navigation demo.
- Made several core improvements during navigation testing.
- Made more changes on performance tests.
- Made more sm_atomic_24hr cleanup.
- Made more sm_advanced_recovery_1 work.
- Made more sm_multi_stage_1 changes.
- Made more progress in aws navigation.
- Made more progress in aws navigation demo.
- Attempted precommit fixes.
- Corrected formatting.
- Added galactic CI setup and renamed rolling files.
- Updated doxygen links.
- Updated c_cpp_properties.json.
- Updated README.md.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Reverted markdowns to html.
- Edited tracing.md to reflect new tracing event names.
- Edited README.md.
- Edited sm_atomic_performance_trace_1.
- Edited sm_atomic_performance_test_a_2.
- Edited sm_atomic_performance_test_c_1.
- Edited sm_respira_1 format cleanup.
- Edited sm_respira_test_2.
- Edited sm_respira_1 format cleanup pre-commit.
- Edited sm_atomic_24hr.
- Edited sm_atomic_performance_test_a_1.
- Edited sm_atomic_performance_test_a_2.
- Edited sm_multi_stage_1.
- Edited sm_advanced_recovery_1.
- Edited sm_advanced_recovery_1 reworked.
- Edited sm_advanced_recovery_1 round 4.
- Edited sm_atomic_performance_test_c_1.
- Edited sm_atomic_performance_test_a_2.
- Edited sm_multi_stage_1.
- Edited sm_atomic_performance_test_a_1.
- Edited sm_atomic_performance_test_a_2.
- Edited sm_atomic_performance_test_c_1.
- Edited sm_multi_stage_1.
- Edited sm_aws_aarehouse navigation.
- Edited sm_advanced_recovery_1.
- Edited sm_advanced_recovery_1 reworked.
- Edited sm_advanced_recovery_1 round 4.
- Edited sm_atomic_performance_test_a_2.
- Edited sm_atomic_performance_test_a_1.
- Edited sm_atomic_performance_test_c_1.
- Edited sm_multi_stage_1.
- Edited wait topic message client behavior.
- Edited wait nav2 nodes client behavior.
```

```rst
Section_14
==========

Added
-----
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: Waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- `dance_bot` launch `gz` lidar choice feature.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo improvements.
- `sm_multi_stage_1` doubling feature.
- `sm_multi_stage_1` improvements.
- `neo_simulation2` package removal.
- Source build enabled on PR for testing.
- Build packages adjustment for source CI.
- Diverse improvements in navigation and performance.

Changed
-------
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Format fixes for gazebo to show the robot and the lidar.

Fixed
----
- Removed some compile warnings.

Removed
-------
- Removed `neo_simulation2` package.

Contributors
------------
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
```

# Section 15

## Added

- Feature/diverse improvements in navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122): Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Introducing smacc2::deep_history syntax. Testing sm_dance_bot with slam pausing/resuming functionality.
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174): Progress on aws navigation, refactorings on navigation clients and behaviors.

## Changed

- Move method after the method it calls to prevent recursion (#126)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Renamed sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172): Multistage modes, sequences, steps, and finishing touches.

## Fixed

- Minor tuning to mitigate overshot issue cases in waypoints navigator (#133)
- Fix CI: format fix python version (#148)
- Update package list (#142)
- Removed node creation and create only a logger (#149)
- Fixed compiling issues in docker environment (#164)
- Fixed broken master build in testing moveit behaviors (#167)
- Fixed broken build in aws navigation progress (#174)

## Removed

- Removed merge markers from a python file (#119)
- Removed parameters smacc (#147)
- Removed sm_dance_bot_msgs (#144)

## Authors

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
- Minor changes (#195)
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
- Branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
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

Removed
-------
- First ensure you have the necessary package installed
- ```
  sudo apt-get install ros-rolling-ros2trace
  ```
``` 

*pabloinigoblasco*

```rst
Section_17
==========

Added
-----
- Added setupTracing.sh script to install necessary packages and configure tracing group.
- Added README tutorial for Dockerfile.
- Added new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added sm_multi_stage_1 package.

Changed
-------
- Changed wording from "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Fixed formatting in various packages.
- Fixed pre-commit issues in multiple packages.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed galactic builds, keeping only rolling. Removed submodules and use .repos file.

Other
-----
- Reverted markdowns to HTML format.
- Reactivated smacc2 nav clients for rolling via submodules.
- Cleaned up tracing.md to reflect new tracing event names.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Made progress on navigation rolling.
- Various performance tests improvements.
- Continued work on AWS navigation demo.
- More changes and improvements in navigation testing.
- Attempted pre-commit fixes in multiple packages.
```

*pabloinigoblasco*

```rst
Section_18
==========

Added
-----
- Introduce new feature `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for nav2, `add`, to wait for nav2 nodes subscribing to the `/bond` topic and ensure they are alive. Optionally select nodes to wait for.
- Add base for the `sm_aws_aarehouse` navigation.
- Introduce `cb_pause_slam` client behavior.

Changed
-------
- Correct all linters and formatters.
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor hotfix.
- Cleaning and lidar show/hide option for `sm_dance_bot` visualizing `turtlebot3`.
- Gazebo fixes to show the robot and the lidar.
- Format fixes for `sm_multi_stage_1` and `sm_dance_bot_strikes_back`.

Fixed
----
- Remove some compile warnings.

Removed
-------
- Eliminate redundant entries.

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
- sm_dance_bot_lite (#99)
- Rename doxygen deployment workflow (#100)
- sm_dance_bot visualizing turtlebot3 (#101)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- sm_multi_stage_1 doubling (#103)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- precommit cleanup run (#106)
- aws demo (#108)
- Brettpac branch (#110)
- Brettpac branch (#111)
```

Section_19
===========

Added
-----

- Added multistage modes to sm_multi_stage_1.
- Added sequences to sm_multi_stage_1.
- Added steps to sm_multi_stage_1.
- Added finishing touches to sm_multi_stage_1.
- Added AWS navigation to sm_dance_bot.

Changed
-------

- Reworked sm_multi_stage_1 with new sequences and steps.

Fixed
-----

- Fixed waypoint 4 and iterations for robot course completion.
- Fixed CI format for Python version.

Removed
-------

- Removed neo_simulation2 package.
- Removed node creation in favor of logger.
- Removed parameters from smacc.
- Removed test from main MoveIt CMake.

Other
-----

- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, and pabloinigoblasco <pablo@ibrobotics.com>.
- Various minor improvements and fixes throughout.

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
- Finetuning waypoints.
- Feature/cb pure spinning.
- Pure spinning behavior missing files.
- Feature/planner changes 16 12.
- Feature/replanning 16 dec.
- Replanning for all examples.
- Several fixes.
- Feature/undo motion 20 12.
- Improving undo motion navigation warehouse2.
- Tuning warehouse3.
- Feature/sync 21 12.
- Format issues.
- Feature/warehouse2 22 12.
- Finishing warehouse2.
- Feature/warehouse2 23 12.
- Tuning and fixes.
- Feature/minor tune.
- Fixing warehouse 3 problems and other core improvements.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components.
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
- Minor format.
- Minor linking errors foxy.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Only rolling version should be pre-released on master.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.

Fixed
-----

- Fixing broken build.
- Fix broken source build.
- Fixing format and minor.
- Fixing startup problems in warehouse 3.

Removed
-------

- Weird moveit not downloaded repo.
- Missing sm.
- Missing.
- Foxy CI.
- Minor broken build.
- Some reordering fixes.
- Docker files for different revisions, warnings removal and more testing on navigation.
- Missing file.
- Other minor changes.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor.
- Minor

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
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------
- Rename header files and correct format.
- Change extension of imports.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Update description table.
- Update table.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Minor formatting.
- Fix source CI and correct README overview.
- Update c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Update doxygen links.
- More Readme Updates.
- More Readme.
- Feature/core and navigation fixes.
- Feature/aws demo progress.
- More on navigation.
- sm_advanced_recovery_1 reworked.
- Trying to fix Pre-Commit.
- More sm_advanced_recovery_1 work.
- sm_advanced_recovery_1 round 4.
- Brettpac branch.
- sm_atomic_performance_test_a_2.
- sm_atomic_performance_test_a_1.
- sm_atomic_performance_test_c_1.

Fixed
-----
- Correct formatters.
- Correct formatting of python file.
- Correct trailing spaces.
- Bug in smacc2 component.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Disable further packages.
- Ignore all packages except smacc2 and smacc2_msgs.
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
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection.
- New client behavior: cb_pause_slam for pausing SLAM functionality.
- New feature: sm_dance_bot_lite.

Changed
-------
- Updated launch command.
- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor format improvements.
- Merge and progress in AWS navigation demo.
- Fix format in various places.
- Minor hotfix.
- Visualizing TurtleBot3 in sm_dance_bot.

Fixed
-----
- Fixed compile warnings.

Removed
-------
- Removed some compile warnings.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

Section_23
==========

Added
-----
- Added `sm_dance_bot` feature to visualize `turtlebot3`.
- Added option to show/hide cleaning and lidar.
- Added `gazebo` fixes for `sm_dance_bot_lite` (#104).
- Added `gazebo` fixes to display the robot and lidar.
- Added `sm_multi_stage_1` doubling (#103).
- Added `aws demo` feature (#108).
- Added `Brettpac branch` (#110).
- Added `5th stage` to `sm_multi_stage_1`.
- Added `a3` feature (#113).
- Added `diverse improvements` for navigation and performance (#116).
- Added `slam toggle` and `smacc deep history` feature (#122).
- Added `dance bot s pattern` feature (#128).
- Added `waypoints navigator` bug fix (#133).
- Added `SM core test` (#138).
- Added `minor navigation improvements` (#141).
- Added `rolling Docker environment` for execution in any environment (#154).
- Added `migration moveit client` feature (#151).

Changed
-------
- Changed launch command in `README.md` for `sm_dance_bot_strikes_back`.
- Changed format for CI in `Fix CI` (#148).

Fixed
-----
- Fixed recursion issue by moving method after the one it calls (#126).
- Fixed overshot issue cases in `waypoints navigator`.
- Fixed format issues in `minor format issues` (#134).
- Fixed compile warnings in `Resolve compile warnings` (#137).
- Fixed node creation to only create a logger in `Remove node creation` (#149).

Removed
-------
- Removed `neo_simulation2` package (#112).
- Removed `sm_dance_bot_msgs` package.
- Removed parameters in `smacc` (#147).
- Removed test from main moveit cmake.
- Removed some comments in `README.md`.

Authors
-------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Brett <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>

Section_24
===========

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
- Improving undo motion navigation warehouse2
- Tuning warehouse3
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
- Fixing broken build

``Section_25``

Added:
- Docker improvements.
- Runtime dependency.
- Restored UR dependency.

Changed:
- Updated Galactic repositories.

Fixed:
- Fixed build.

Removed:
- None.

Other Changes:
- Merged additional changes.
- Master branch now rolling to Galactic backport.
- Tested Dance Bot demos.

Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco.

Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
