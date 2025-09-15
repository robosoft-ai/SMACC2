Changelog for package keyboard_client
======================================

Version 2.3.16 (2023-07-16)
---------------------------

### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2 <https://github.com/robosoft-ai/SMACC2>`_
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further work on buildfarm problem
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
---------------------------

### Added
- Pre-release
- Contributors: pabloinigoblasco

Version 1.22.1 (2022-11-09)
---------------------------

### Added
- Pre-release
- Contributors: pabloinigoblasco

### Changed
- Progress in humble SMACC2 deb generation
- Feature/fix mutex galactic (#319)
  - Bug fix for galactic mutex
  - Testing improvements for undo motion and action client
  - Important refactoring of smacc action client
  - Further progress and changes in action client
  - Added smaccServiceServer client to galactic
  - Updates and testing for husky robot
  - Finishing tests for cancel and undo behavior

### Removed
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco

```rst
0.3.0 (2022-04-04)
------------------

### Added
- More progress in humble SMACC2 deb generation.

### Fixed
- Bug fix in galactic mutex.
- Important refactoring in smacc action client.
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Ignored packages which should not be released.

### Changed
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Added smacc2_performance_tools.
- Performance tests improvements.
- Format cleanup in sm_respira_1.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Cleaned up sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Updated c_cpp_properties.json.

### Removed
- Do not execute clang-format on smacc2_sm_reference_library package.

### Contributors
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored by Denis Štogl <denis@stogl.de>.
- Co-authored by Denis Štogl <destogl@users.noreply.github.com>.
```

```rst
Section_3
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: now waits for `nav2` nodes to subscribe to the `/bond` topic and ensures they are alive. Nodes to wait for can be optionally selected.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.
- Removed `neo_simulation2` package.
- Source build enabled on PR for testing.
- Adjusted build packages of source CI.
- Diverse improvements in navigation and performance.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.
- Minor format improvements.
- Format fixes for gazebo to show the robot and the lidar.
- Cleaning and lidar show/hide option for `sm_dance_bot`.
- Progress in AWS navigation demo.
- Merge and progress in development.
- Minor hotfixes.

Fixed
-----

- Compile warnings removed.

Removed
-------

- Some compile warnings.

Contributors
------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_4
=========

Added
-----
- Diverse improvements in navigation and performance.

Changed
-------
- Minor format improvements.
- Moved method after the method it calls to prevent recursion (#126).
- Resolved compile warnings (#137).
- Added SM core test (#138).
- Added QOS durability to SmaccPublisherClient (#163).

Fixed
-----
- Removed merge markers from a Python file (#119).
- Fixed launch command in README.md.
- Fixed CI format for Python version (#148).

Removed
-------
- Removed node creation and created only a logger (#149).
- Removed parameters from SMACC (#147).

Other
-----
- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, Denis Štogl <denis@stogl.de>.
- Various minor improvements and bug fixes throughout the codebase.
```

```rst
Section 5
=========

Added
-----
- More Waypoints
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Feature/wharehouse2 dec 14 (#185)
  - warehouse2
  - minor changes
- Feature/sm warehouse 2 13 dec 2 (#186)
  - format changes
  - headless mode improvements
  - default values
- Feature/cb pure spinning (#188)
  - format changes
  - headless mode improvements
  - default values
- Feature/cb pure spinning (#189)
  - format changes
  - headless mode improvements
  - default values
- Feature/planner changes 16 12 (#191)
  - minor changes
  - more fixes
  - replanning for all examples
- Feature/replanning 16 dec (#193)
  - minor changes
  - replanning for all examples
- Feature/undo motion 20 12 (#196)
  - minor changes
  - replanning for all examples
  - improving undo motion navigation for warehouse2
- Feature/undo motion 20 12 (#198)
  - minor changes
  - replanning for all examples
  - improving undo motion navigation for warehouse2
- Feature/sync 21 12 (#199)
  - minor changes
  - replanning for all examples
  - format issue fixes
- Feature/warehouse2 22 12 (#200)
  - minor changes
  - replanning for all examples
  - format issue fixes
  - finishing warehouse2
- Feature/warehouse2 23 12 (#201)
  - minor changes
  - replanning for all examples
  - tuning and fixes
- Feature/minor tune (#203)
  - tuning and fixes
  - minor tune
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Add galactic CI build due to Navigation2 issues in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency for include resolution
- Disable ament_cpplint
- Disable some packages and update workflows
- Bump ccache version
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licenses
- Disable cpplint and cppcheck linters
- Correct formatters
- Enable cppcheck
- Correct formatting of python files
- Include necessary package and edit Threesome launch
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use docs/ as source and output directory for documentation
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference
- Update package name and package.xml for liter compliance
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Update description table
- Update table
- Copy initial docs
- Dockerfile with ROS distro as argument
  Use command: "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- Add setupTracing.sh
  Installs necessary packages and configures tracing group
- Removed manual installation of ros-rolling-ros2trace
  Now automated in setupTracing.sh
  Location of sh file assumed if user follows README.md under "Getting started"
- Create alternative ManualTracing
- Add new sm markdowns
- Add a Dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Update tracing/ManualTracing.md
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Change "smacc application" to "SMACC2 library"
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Update smacc_sm_reference_library/sm_atomic/README.md
  Edit from html to markdown syntax
  Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Reactivate smacc2 nav clients for rolling via submodules
- Rename tracing events
- Fix bug in smacc2 component
- Revert markdowns to html
- Add README tutorial for Dockerfile
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling
- Remove submodules and use .repos file
- Update mentions of SMACC/ROS to SMACC2/ROS2
- Progress on navigation rolling
- Rename folders, delete tracing.md, edit README.md

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- Wording "smacc application" to "SMACC2 library"
```

```rst
Section 6
=========

Added
-----

- Added smacc2_performance_tools.
- Added sm_respira_1 format cleanup.
- Added sm_respira_test_2.
- Added sm_atomic_24hr.
- Added sm_atomic_performance_trace_1.
- Added sm_reference_library reformatting.
- Added galactic CI setup and renamed rolling files. (#58)
- Added source CI fix and corrected README overview. (#62)
- Added doxygen links update (#70).
- Added more Readme updates (#72).
- Added more Readme updates (#74).
- Added new sm from sm_respira_1 (#76).
- Added base for the sm_aws_aarehouse navigation.
- Added progress in aws navigation.
- Added several core improvements during navigation testing.
- Added progress in aws navigation demo.
- Added more on navigation.
- Added sm_advanced_recovery_1 reworked (#83).
- Added fix for pre-commit.
- Added more sm_advanced_recovery_1 work (#85).
- Added sm_advanced_recovery_1 round 4 (#86).
- Added Brettpac branch (#87).
- Added sm_atomic_performance_test_a_2.
- Added sm_atomic_performance_test_a_1.
- Added sm_atomic_performance_test_c_1 (#88).
- Added modifying sm_atomic_performance_test_a_2 (#89).
- Added sm_multi_stage_1.
- Added fixing precommit.
- Added more sm_multi_stage_1 (#91).
- Added wait topic message client behavior (#81).
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new feature, cb_wait_topic_message: asynchronous client

```rst
Section_7
=========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`
- `cb_pause_slam` client behavior added
- `sm_dance_bot_lite` visualizing `turtlebot3`
- Cleaning and lidar show/hide option for `sm_dance_bot`
- Gazebo fixes to show the robot and the lidar for various dance bot versions
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components
- Introducing slam pausing/resuming functionality for `sm_dance_bot`
- Polishing `sm_dance_bot` and `s-pattern`
- First working version of `sm` template and template generator

Fixed
-----
- Remove some compile warnings
- Remove `neo_simulation2` package
- Correct formatting issues
- Adjust build packages of source CI
- Move method after the method it calls to prevent recursion
- Fix typo in `s-pattern`

Removed
-------
- `neo_simulation2` package

Other
-----
- Various formatting improvements
- Progress in AWS navigation demo
- Merge and progress in navigation testing
- Precommit cleanup run
- Enable source build on PR for testing
- Additional linting and formatting
- Remove merge markers from a Python file
- Minor tweaks and improvements
- Diverse improvements in navigation and performance
- Progress in testing `sm_dance_bot`
- Progress in `sm_multi_stage_1`
- Various stages of development in `sm_multi_stage_1`
- `smacc2::deep_history` syntax introduced
- Progress in `sm_dance_bot` and `slam` functionality
- More refinement in `sm_dance_bot`
- First working version of `sm` template and template generator
```

*pabloinigoblasco*

```rst
Section_8
=========

Added
-----

- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
- Progress in the sm_dance_bot tests (#135)
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
- Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Sm_pubsub_1 (#169)
- Sm_pubsub_1 part 2 (#170)
- Sm_advanced_recovery_1 renaming (#171)
- Sm_multi_stage_1 reworking (#172)
- Feature/aws navigation sm dance bot (#174)
- Minor changes (#175)
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Warehouse2 progress (#179)
- Format (#180)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- Finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)
- Pure spinning behavior missing files (#190)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)

Changed
-------

- Minor tuning to mitigate overshot issue cases
- Minor format issues (#134)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing compiling issues
- Update readme (#164)
- More readme updates
- Add reliability QOS config
- More on aws demo
- Fixing broken build
- More on aws navigation and some other refactorings on navigation clients and behaviors
- More testing on moveit
- Progress on moveit
- More testing on moveit behaviors
- Minor configuration
- Fixing pipeline error
- Fixing broken master build
- Finishing touches 1
- Redoing sm_dance_bot_warehouse_3 waypoints
- More waypoints
- Move trigger logic into headers
- Finetuning waypoints

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing some comments in the past

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_9
=========

Added
-----
- Feature/undo motion 20 12 (#196)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Added missing file from warehouse2 (#205)
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
- Included necessary package and edited Threesome launch
- Rename header files and correct format
- Add workflow for checking doc build
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Update changelogs
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- added setupTracing.sh
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Update smacc_sm_reference_library/sm_atomic/README.md
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- updated mentions of SMACC/ROS to SMACC2/ROS2
- some progress on navigation rolling
- added smacc2_performance_tools
- performance tests improvements
- more on performance and other issues
- sm_respira_1 format cleanup

Changed
-------
- Several fixes (#194)
- Minor changes (#195)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Feature/minor tune (#203)
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Change extension of imports
- Correct formatting of python file
- Reverting "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table
- Update table
- Reset all versions to 0.0.0
- changed wording "smacc application" to "SMACC2 library"
- reactivating smacc2 nav clients for rolling via submodules
- renamed tracing events after
- bug in smacc2 component
- reverted markdowns to html
- additional cleanup
- cleanup
- edited tracing.md to reflect new tracing event names
- renamed folders, deleted tracing.md, edited README.md

Removed
-------
- Weird moveit not downloaded repo
- Minor broken build
``` 

*pabloinigoblasco*

```rst
Section_10
==========

Added
-----

- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.

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

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

Authors
-------

- Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_11
==========

Added
-----
- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2, wait for nav2 nodes subscribing to the /bond topic and wait for them to be alive. Optional selection of nodes to wait.
- Base for the sm_aws_warehouse navigation.
- Gazebo fixes to show the robot and the lidar.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also smacc2::deep_history syntax.
- Introducing slam pausing/resuming functionality in testing sm_dance_bot.
- First working version of sm template and template generator.

Changed
-------
- Minor format improvements.
- Navigation parameters fixes on sm_dance_bot.
- Cleaning and lidar show/hide option.
- More fixes in sm_dance_bot.
- Polishing sm_dance_bot and s-pattern.
- Refinement in sm_dance_bot.

Fixed
----
- Remove some compile warnings.
- Remove neo_simulation2 package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.
- Noticed typo and corrected it.

Removed
-------
- Removed neo_simulation2 package.

Contributors
------------
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_12
==========

Added
-----

- Waypoints navigator bug (#133)
- Progress in the sm_dance_bot tests (#135)
- Feature/nav2z renaming (#144)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
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
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action msgs
- Formatting
- Removing parameters smacc
- Noticed launch command was incorrect in README.md
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Progress on move_it PR
- Improving dockerfile for building local tests
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools
- Pre-commit cleanup
- Fix: add a missing colon
- Refactor: remove line
- Fixing broken master build
- Format

Fixed
-----

- Fixing compiling issues
- Fixing pipeline error
- Fixing broken build
- Several fixes (#194)

Removed
-------

- Removing sm_dance_bot_msgs
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing some comments in the past
- Removing some build errors
- Removing test workaround
- Removing test from main moveit cmake
- Removing some comments in the past
- Removing some build errors
- Removing some linting warnings
- Removing some formatting issues
- Removing some broken build issues
- Removing some missing files

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section 13
-----------

Added
-----
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
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- Rename header files and correct format.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
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
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh

Changed
-------
- tuning warehouse3 (#197)
- improving undo motion navigation warehouse2
- finishing warehouse2
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements (#204)
- fixing docker for foxy and galactic
- some reordering fixes
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Remove example things from Foxy CI setup. (#214)
- Fix rolling builds (#222)
- do not merge yet - Feature/odom tracker improvements and retry motion (#223)
- odom tracker improvements
- adding forward behavior retry funcionality
- minor
- removing warnings (#213)
- changing wording "smacc application" to "SMACC2 library"

Fixed
-----
- minor broken build

Removed
-------
- weird moveit not downloaded repo

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>

Section_14
===========

Added
-----
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Renamed tracing events after.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed event generator library.

Fixed
-----
- Bug in smacc2 component.
- Correct trailing spaces.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Clean up of sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Correct all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.

Other
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Edited tracing.md to reflect new tracing event names.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting.
- Minor.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in aws navigation demo.
- Progress in aws navigation.
- Progressing in aws navigation.
- More on performance and other issues.
- More on navigation.
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
- Progress in aws navigation demo

```rst
Section_15
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo progress.
- `sm_multi_stage_1` improvements.
- Diverse improvements in navigation and performance.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components. Introduces `smacc2::deep_history` syntax.
- `slam` toggle and `smacc` deep history feature.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor format improvements.
- Format fixes for `gazebo` to show the robot and the `lidar`.

Fixed
----
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Remove merge markers from a Python file.

Removed
-------
- `neo_simulation2` package.

Authors
-------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```
Section_16
==========

Added
-----
- First working version of sm template and template generator. (#127)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Rolling Docker environment to be executed from any environment (#154)
- Added dependencies for husky in rolling and galactic for AWS navigation (#174)
- Waypoint Inputs (#178)

Changed
-------
- Moved method after the method it calls to prevent recursion (#126)
- Renamed state machines to smacc2_performance_tools (#166)
- Refactored SmaccPublisherClient to include QOS durability (#163)
- Reworked sm_multi_stage_1 with new sequences and modes (#172)
- Finetuned waypoints for sm_dance_bot_warehouse_3 (#187)

Fixed
-----
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148)
- Fixed CI format for Python version (#148)
- Fixed compiling issues in various components
- Fixed broken builds in master and AWS navigation (#174)
- Fixed some formatting and linting issues in SrConditional (#168)

Removed
-------
- Removed node creation and created only a logger in SM Atomic SM generator (#149)
- Removed parameters in smacc (#147)
- Removed test from main moveit cmake in moveit migration (#151)
- Removed sm_dance_bot_msgs in navigation 2 stack renaming (#144)

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_17
==========

Added
-----
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/retry behavior warehouse 1 (#226)
- docker build files for all versions
- dockerfiles (#225)
- Fix code generators (#221)
- Foxy backport (#206)
- Rename header files and correct format
- Add workflow for checking doc build
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- added setupTracing.sh
  Installs necessary packages and configures tracing group
- First ensure you have the necessary package installed
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Fixed
-----
- Fix other build issues
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
- branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Update description table
- Update table
- Copy initial docs
- Update name of package and package.xml to pass liter
- Execute on master update
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61

Removed
-------
- Use of node in the sm performance template
- Disable ament_cpplint
- Disable cpplint and cppcheck linters
- Disable some packages and update workflows
- Ignore further packages
- Disable disabled packages
``` 

*pabloinigoblasco*

```rst
Section_18
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
- Renamed tracing events after.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.

Fixed
----
- Bug in smacc2 component.
- Reverted markdowns to html.
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_reference_library reformatting.
- Corrected all linters and formatters.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh. Location of sh file assumed if user follows README.md under "Getting started".
- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.

Other
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Edited tracing.md to reflect new tracing event names.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Some progress on navigation rolling.
- Performance tests improvements.
- More on performance and other issues.
- Format cleanup for sm_respira_1.
- Format cleanup for sm_respira_1 pre-commit.
- More changes on performance tests.
- Updated smacc2_rta command across readmes.
- More on navigation.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- More on navigation.
- Progress in aws navigation.
- Progressing in aws navigation.
- More on navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in aws navigation.
- Progress in aws navigation demo.
- More on navigation.
- Progressing in

```rst
Section_19
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive, with optional node selection.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature.
- `sm_dance_bot_visualizing_turtlebot3` feature.
- `sm_multi_stage_1` feature.
- `sm_dance_bot_strikes_back` feature.
- AWS demo.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor hotfixes.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.

Fixed
-----
- Remove some compile warnings.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.

Removed
-------
- `neo_simulation2` package.

Contributors
------------
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_20
==========

Added
-----

- Feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing slam pausing/resuming functionality in sm_dance_bot
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
- Remove node creation and create only a logger. (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes so the robot can complete the course (#155)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Progressing in the moveit migration testing
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
  - Add reliability QoS config
- Feature/testing moveit behaviors (#167)
  - More testing on moveit behaviors
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)
  - Multistage modes, sequences, steps, and finishing touches
- Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and some other refactorings on navigation clients and behaviors
  - More on aws demo

Changed
-------

- Move method after the method it calls to prevent recursion (#126)
- Noticed typo "Finnaly" corrected to "Finally"
- Update package list. (#142)
- Fix CI: format fix python version (#148)
- Update readme (#164)
  - More readme updates

Fixed
-----

- Resolve compile warnings (#137)
- Minor navigation improvements (#141)
- Fixing broken master build
- Fixing pipeline error
- Fixing compiling issues
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Warehouse2 progress (#179)

Removed
-------

- Remove merge markers from a python file. (#119)
- Removing parameters smacc (#147)
- Removing sm_dance_bot_msgs
- Pending references
- Removing test from main moveit cmake
- Removing some comments in the past
- Removing parameters smacc
- Workflows update
- Workflow
- Docker refactoring
- Repos dependency
- Adding dependency to ur5 client
- Minor dockerfile test workaround
- Improving dockerfile for building local tests
- Removing parameters smacc
- Removing some comments in the past
- Removing sm_dance_bot_msgs
- Fixing some errors introduced on formatting
- Missing dependency
- Fixing some more linting warnings
- Removing parameters smacc
- Removing some comments in the past
- Removing sm_dance_bot_msgs
- Fixing some errors introduced on formatting
- Missing dependency
- Fixing some more linting warnings
- Removing parameters smacc
- Removing some comments in the past
- Removing sm_dance_bot_msgs
- Fixing some errors introduced on formatting
- Missing dependency
- Fixing some more linting warnings
```

```rst
Section_21
==========

Added
-----
- Feature/sm_dance_bot_warehouse_3 (#181)
- Feature/redoing sm_dance_bot_warehouse_3 waypoints (#184)
- Feature/finetuning waypoints (#187)
- Feature/pure spinning behavior missing files (#189)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- Feature/undo motion 20 12 (#196, #198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/fixing warehouse 3 problems and core improvements (#204)
- Feature/barrel demo (#227)
- Feature/improvements warehouse3 (#228)
- Foxy backport (#206)

Changed
-------
- SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- Feature/cb pure spinning (#188)
- Feature/undo motion 20 12 (#196, #198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/fixing warehouse 3 problems and core improvements (#204)
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)

Fixed
-----
- Fix broken source build (#227)
- Only rolling version should be pre-released on on master. (#230)
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
- Included necessary package and edited Threesome launch

Removed
-------
- Retry behavior warehouse 1
- Missing file
- Minor format fix
- Other minor changes

Co-Authored-By
--------------
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
Section_22
==========

Version 0.1.0 (Unreleased)
---------------------------

### Added
- Added `sm_three_some.launch` to launch `sm_three_some`.
- Renamed header files and corrected format.
- Added workflow for checking doc build.
- Updated `doxygen-check-build.yml`.
- Created `doxygen-deploy.yml`.
- Created workflow for testing prerelease builds.
- Used `docs/` as source folder and output directory for documentation.
- Renamed packages to `smacc2` and `smacc2_msgs`.
- Corrected GitHub branch reference.
- Updated package name and `package.xml` to pass liter.
- Executed master update.
- Reset all versions to 0.0.0.
- Ignored all packages except `smacc2` and `smacc2_msgs`.
- Updated changelogs.
- Reverted "Ignore all packages except `smacc2` and `smacc2_msgs`" (commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61).
- Updated description table.
- Updated table.
- Copied initial docs.
- Created Dockerfile with ROS distro as argument.
- Opened new folder for additional tracing contents.
- Deleted tracing directory.
- Moved `tracing.md` to tracing directory.
- Added `setupTracing.sh` to install necessary packages and configure tracing group.
- Removed manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`.
- Created alternative `ManualTracing`.
- Added new `sm` markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Updated `smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh` (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Updated `tracing/ManualTracing.md` (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Changed wording from "smacc application" to "SMACC2 library" (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Updated `smacc_sm_reference_library/sm_atomic/README.md` from html to markdown syntax (Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>).
- Reactivated `smacc2` nav clients for Rolling via submodules.
- Renamed tracing events.
- Fixed bug in `smacc2` component.
- Reverted markdowns to html.
- Added README tutorial for Dockerfile.
- Additional cleanup.
- Edited `tracing.md` to reflect new tracing event names.
- Enabled build of missing Rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed Galactic builds from master, kept only Rolling, removed submodules, and used `.repos` file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Made progress on navigation Rolling.
- Renamed folders, deleted `tracing.md`, edited `README.md`.
- Added `smacc2_performance_tools`.
- Made performance tests improvements.
- Made more performance and other issues changes.
- Cleaned up `sm_respira_1` format.
- Cleaned up `sm_respira_1` format pre-commit.
- Added `sm_respira_test_2`.
- Made more changes on performance tests.
- Skipped `clang-format` execution on `smacc2_sm_reference_library` package.
- Reformatted `sm_reference_library`.
- Corrected trailing spaces.
- Added galactic CI setup and renamed Rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69).
- Updated doxygen links (#70) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Made more README updates (#72) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Made more README updates (#74) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Created new `sm` from `sm_respira_1` (#76).
- Made feature/core and navigation fixes (#78).
- Based for the `sm_aws_aarehouse` navigation.
- Progressed in AWS navigation.
- Made several core improvements during navigation testing.
- Made formatting improvements.
- Progressed in AWS navigation demo.
- Made format improvements.
- Made more on navigation.
- Reworked `sm_advanced_recovery_1` (#83).
- Fixed pre-commit for `sm_advanced_recovery_1`.
- Tried to fix Pre-Commit (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Made more `sm_advanced_recovery_1` work (#85) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Made `sm_advanced_recovery_1` round 4 (#86) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Created `Brettpac` branch (#87).
- Added `sm_atomic_performance_test_a_2`.
- Added `sm_atomic_performance_test_a_1` (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Added `sm_atomic_performance_test_c_1` (#88) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Modified `sm_atomic_performance_test_a_2` (#89) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Added `sm_multi_stage_1`.
- Fixed precommit for `sm_multi_stage_1` (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Made more `sm_multi_stage_1` changes (#91) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>).
- Updated `README.md` with launch command.
- Waited topic message client behavior (#81).
```

```rst
Section_23
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `wait nav2 nodes` subscribes to the `/bond` topic and waits for them to be alive. You can optionally select the nodes to wait for.
- New client behavior: `cb_pause_slam` for pausing SLAM operations.
- New client behavior: `cb_pause_slam` for pausing SLAM operations.

Changed
-------
- Renamed doxygen deployment workflow (#100).
- Minor hotfix for `sm_dance_bot` visualizing TurtleBot3.
- Cleaning and lidar show/hide option for `sm_dance_bot` visualizing TurtleBot3.
- Gazebo fixes to show the robot and lidar for `sm_dance_bot` visualizing TurtleBot3.
- Doubling `sm_multi_stage_1`.

Fixed
-----
- Navigation parameters fixes on `sm_dance_bot`.
- Formatting fixes.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Format improvements.
- Merge and progress.
- Fix format.
- Cleaning files and making formatting work.
- More fixes.

Removed
-------
- Removed some compile warnings (#96).
``` 

*pabloinigoblasco*

Section 24
-----------

### Added
- Added AWS demo (#108).
- Added Brettpac branch (#110).
- Added a3 (#113).
- Added diverse improvements in navigation and performance (#116).
- Added feature to toggle SLAM and deep history in SMACC (#122).
- Added SM Atomic SM generator (#143).
- Added durability QoS to SmaccPublisherClient (#163).
- Added testing for MoveIt behaviors (#167).

### Changed
- Updated package list (#142).
- Renamed navigation 2 stack (#144).
- Refactored SM Dance Bot Strikes Back (#152).
- Updated READMEs with SVGs for atomic, dance_bot, and others (#140).
- Updated Docker environment for execution in any environment (#154).
- Improved Dockerfile for building local tests.

### Fixed
- Fixed formatting in neo_simulation2 package removal (#112).
- Fixed compilation warnings (#137).
- Fixed CI format for Python version (#148).
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed minor navigation improvements (#141).
- Fixed waypoint 4 and iterations for robot course completion (#155).

### Removed
- Removed neo_simulation2 package.
- Removed parameters in SMACC.
- Removed node creation in favor of logger.

### Miscellaneous
- Precommit cleanup run (#106).
- Minor format fixes.
- Progress in testing SM Dance Bot.
- Progress in MoveIt migration testing.
- Added missing dependencies.
- Updated formats and dependencies.
- Improved pipeline error handling.
- Moved reference library SMs to smacc2_performance_tools.
- Added reliability QoS configuration.
- Removed merge markers from a Python file (#119).
- Updated README with more details (#164).
- Added .reps dependencies and fixed build errors.
- Added dependency to UR5 client.
- Updated READMEs.
- Added timestamps for initial state machine transitions (#165).
- Added progress in testing MoveIt behaviors.
- Fixed linting warnings.
- Fixed some errors in formatting.
- Fixed compiling issues.

```rst
Section_25
==========

Added
-----

- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency and husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Updated dependencies for husky in rolling and galactic.
  - Progress on aws navigation and refactorings on navigation clients and behaviors.
  - More work on aws demo.

- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2 progress.
  - Minor changes.

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added more changes, headless mode, and merge improvements.
  - Set default values.
  - Fine-tuned waypoints.

- Feature/cb pure spinning (#188)
  - Added pure spinning behavior.
  - Fixed missing files.

- Feature/planner changes 16 12 (#191)
  - Made minor changes and more fixes.

- Feature/replanning 16 dec (#193)
  - Implemented replanning for all examples.
  - Made several fixes.

- Feature/undo motion 20 12 (#196)
  - Made minor changes.
  - Improved undo motion navigation in warehouse2.
  - Tuned warehouse3.

- Feature/sync 21 12 (#199)
  - Made minor changes.
  - Fixed format issues.

- Feature/warehouse2 22 12 (#200)
  - Made minor changes.
  - Fixed format issues.
  - Finished warehouse2.

- Feature/warehouse2 23 12 (#201)
  - Made minor changes.
  - Tuned and fixed issues.

- Feature/minor tune (#203)
  - Tuned and fixed minor issues.

- Feature/undo motion 20 12 (#198)
  - Tuned undo motion and fixed errors.
  - Fixed format issues.

Changed
-------

- Updated subscriber publisher components.
- Progress in autoware machine.
- Refinements in cp subscriber and cp publisher.
- Added more components to smacc core, mostly developed for autoware demo.
- Improved autoware demo.
- Docker files updated for different revisions.
- Warning removal and more navigation testing.
- Docker build files updated for foxy and galactic versions.
- Progress in barrel husky demo.
- Updated barrel search build and warehouse3.
- Fixed startup issues in warehouse3.
- Fixed format and minor issues.

Fixed
-----

- Fixed broken master build.
- Fixed broken build issues.
- Fixed broken build again.
- Fixed minor issues.
- Fixed docker for foxy and galactic.
- Fixed startup problems in warehouse3.
- Fixed broken build once more.
- Fixed runtime dependency issues.
- Restored ur dependency.

Removed
-------

- Removed some reordering fixes.
```

Section_26
==========

Version 0.1.0 (Backport/initial to galactic)
--------------------------------------------

### Added
- Build-status table
- Detailed install instructions ([source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme))
- `setupTracing.sh`: Installs necessary packages and configures tracing group.

### Changed
- Default build type set to `Release` for faster build and smaller executables.
- Updated examples section.

### Fixed
- Resolved missing dependency in `smacc_msgs` and reorganized for better overview.
- Fixed bug in `smacc2` component.
- Performance tests improvements and cleanup.
- Optimized dependencies in `move_base_z_planners_common`.
- Renamed event generator library.
- Corrected build-overview table.
- Updated and unified CI configurations.
- Used `tf_geometry_msgs.h` in galactic.
- Used galactic branches in `.repos-file`.

### Removed
- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`.

### Miscellaneous
- Reverted commit regarding package selection.
- Reorganized project structure.
- Updated README.md.
- Reverted markdowns to HTML.
- Added README tutorial for Dockerfile.
- Edited tracing.md to reflect new tracing event names.
- Do not execute clang-format on `smacc2_sm_reference_library`.
- Cleaned up `sm_atomic_24hr`.
- More cleanup on `sm_atomic_24hr`.
- Cleaned up `sm_respira_1` format.
- Cleaned up `sm_respira_test_2`.
- Reorganized `sm_reference_library`.
- Corrected trailing spaces.

Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco, reelrbtx, Declan Dury, DecDury, brettpac.
