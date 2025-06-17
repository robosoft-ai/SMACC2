Changelog for package sm_three_some
=======================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted to fix an issue with ros buildfarm
  - Addressed buildfarm issue
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
- Reverted "Ignore packages which should not be released."
- Contributors: Denis Štogl

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
### Added
- Reverted "Ignore packages which should not be released."
- Ignored packages not meant for release
- Galactic type walker (#264)
- Feature/master rolling to galactic backport (#236)
  - Updated references from SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Improved performance tests
  - Cleaned up sm_respira_1 format
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Updated CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Updated doxygen links
  - Added new feature, cb_wait_topic_message
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic
  - Corrected all linters and formatters
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Contributors: brettpac

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive; optional node selection
- Base for the `sm_aws_warehouse` navigation
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` visualizing TurtleBot3
- `dance_bot_launch_gz_lidar_choice`: cleaning and lidar show/hide option
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo
- `sm_multi_stage_1` doubling
- `sm_multi_stage_1` fixes and improvements
- Diverse improvements in navigation and performance

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`

Fixed
-----

- Remove some compile warnings
- Minor hotfix
- Correct formatting
- Enable source build on PR for testing
- Adjust build packages of source CI
- Minor fixes and improvements

Removed
-------

- `neo_simulation2` package

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_3

### Added
- Feature/diverse improvements navigation performance (#117)
  - Minor enhancements in navigation and performance.
  - Additional linting and formatting.
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
  - Introducing smacc2::deep_history syntax.
  - Testing sm_dance_bot with slam pausing/resuming functionality.
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Rolling Docker environment to be executed from any environment (#154)
- Add SM Atomic SM generator. (#143)
- Initial migration to smacc2 in Feature/migration moveit client (#151)
  - Fixing errors in formatting and dependencies.
  - Progress in moveit migration testing.
- Added QOS durability to SmaccPublisherClient (#163)
  - Configured QOS durability and reliability.
- Feature/aws navigation sm dance bot (#174)
  - Added dependencies for husky simulation.
  - Update dependencies for husky in rolling and galactic.

### Changed
- Move method after the method it calls in (#126)
  - Prevent recursion issues.
- Resolve compile warnings (#137)
- Minor navigation improvements (#141)
- Update package list. (#142)
- Update readme (#164)
- Minor changes in Feature/migration moveit client (#151)
- Update readme in Feature/aws navigation sm dance bot (#174)

### Fixed
- Fix CI: format fix python version (#148)
- Fixing compiling issues in Feature/migration moveit client (#151)
- Fixing broken master build in Feature/testing moveit behaviors (#167)
- Fixing broken build in Feature/aws navigation sm dance bot (#174)

### Removed
- Remove merge markers from a python file. (#119)
- Removing sm_dance_bot_msgs in Feature/nav2z renaming (#144)
- Removing parameters smacc in (#147)
- Remove node creation and create only a logger in (#149)

### Miscellaneous
- Noticed launch command was incorrect in README.md.
- Minor format issues (#134)
- Precommit cleanup.
- Pending references in Feature/nav2z renaming (#144)
- Warehouse2 progress (#179)
- Format (#180)
- Merge in Feature/sm warehouse 2 13 dec 2 (#182)
- Headless and other fixes in Feature/sm warehouse 2 13 dec 2 (#182)

```rst
Section 4
=========

Added
-----
- Added Brettpac branch (#184) with default values.
- Added redoing sm_dance_bot_warehouse_3 waypoints and more waypoints.
- Added SrConditional fixes and formatting (#168):
  - Fixed formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Linted the code.
- Added Feature/wharehouse2 dec 14 (#185) with minor changes.
- Added Feature/sm warehouse 2 13 dec 2 (#186) with format changes, headless merge, and other fixes.
- Added finetuning waypoints (#187).
- Added Feature/cb pure spinning (#188) with format changes, headless merge, and other fixes.
- Added Feature/cb pure spinning (#189) with format changes, headless merge, and other fixes.
- Added pure spinning behavior missing files and minor changes (#190).
- Added Feature/planner changes 16 12 (#191) with minor changes, more fixes, and replanning for all examples.
- Added Feature/replanning 16 dec (#193) with minor changes and replanning for all examples.
- Added several fixes (#194).
- Added minor changes (#195).
- Added Feature/undo motion 20 12 (#196) with minor changes, replanning for all examples, and improving undo motion navigation warehouse2.
- Added tuning warehouse3 (#197).
- Added Feature/undo motion 20 12 (#198) with minor changes, replanning for all examples, improving undo motion navigation warehouse2, and undo tuning and errors.
- Added Feature/sync 21 12 (#199) with minor changes, replanning for all examples, and format issues.
- Added Feature/warehouse2 22 12 (#200) with minor changes, replanning for all examples, and format issues.
- Added Feature/warehouse2 23 12 (#201) with minor changes, replanning for all examples, and tuning and fixes.
- Added Feature/minor tune (#203) with tuning and fixes, and minor tune.
- Added fixing warehouse 3 problems, and other core improvements (#204):
  - Fixed warehouse 3 problems to remove deadlocks and make continuous integration green.
  - Fixed weird moveit not downloaded repo.
  - Added missing file from warehouse2.
- Added Foxy backport (#206) with minor formatting fixes, fix for trailing spaces, codespell correction, python linters warnings correction, and other updates.
- Added galactic CI build because Navigation2 is broken in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added ament_lint_cmake satisfaction.
- Added missing licenses.
- Added necessary package and edited Threesome launch.
- Added workflow for checking doc build.
- Added doxygen-check-build.yml update.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Used docs/ as source folder for documentation and output directory.
- Renamed to smacc2 and smacc2_msgs.
- Corrected GitHub branch reference.
- Updated name of package and package.xml to pass liter.
- Executed on master update.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" commit.
- Updated description table.
- Updated table.
- Copied initial docs.
- Created Dockerfile w/ ROS distro as argument.
- Opened new folder for additional tracing contents.
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Reverted markdowns to html.
- Added README tutorial for Dockerfile.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling, removed submodules and used .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Renamed header files and corrected format.
- Updated doxygen-check-build.yml.
- Used manual deployment for now.
- Used docs/ as source folder for documentation and output directory.
- Corrected GitHub branch reference.
- Updated name of package and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Renamed tracing events after.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
``` 

*pabloinigoblasco*

```rst
Section 5
=========

Added
-----

- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files. (#58)
- Added more Readme updates. (#72)
- Added more Readme updates. (#74)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.

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
- Fixed pre-commit issues.
- Fixed navigation parameters on sm_dance_bot.

Removed
-------

- Do not execute clang-format on smacc2_sm_reference_library package.

Authors
-------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_6
=========

Added
-----

- Feature/sm aws warehouse (#94)
  - Implemented base for the sm_aws_warehouse navigation.
  - Made progress in AWS navigation.
  - Introduced several core improvements during navigation testing.
  - Improved formatting.
  - Added new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for nav2, which waits for nav2 nodes subscribing to the /bond topic and ensures they are alive.
  - Fixed navigation parameters on sm_dance_bot.

Changed
-------

- Feature/sm dance bot fixes (#95)
  - Improved AWS navigation.
  - Made core improvements during navigation testing.
  - Progressed in AWS navigation demo.
  - Enhanced formatting.
  - Fixed navigation parameters on sm_dance_bot.

- Feature/cb pause slam (#98)
  - Improved AWS navigation.
  - Made core improvements during navigation testing.
  - Progressed in AWS navigation demo.
  - Enhanced formatting.
  - Fixed navigation parameters on sm_dance_bot.
  - Added cb pause slam client behavior.

- sm_dance_bot_lite (#99)
  - Updated YAML configuration.

- Rename doxygen deployment workflow (#100)
  - Applied minor hotfix.

- sm_dance_bot visualizing turtlebot3 (#101)
  - Improved visualization of turtlebot3.

- Feature/dance bot launch gz lidar choice (#102)
  - Added cleaning and lidar show/hide option.

- Feature/sm dance bot lite gazebo fixes (#104)
  - Improved gazebo visualization for the robot and lidar.

- sm_multi_stage_1 doubling (#103)
  - Improved sm_multi_stage_1 functionality.

- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Improved gazebo visualization for the robot and lidar.
  - Fixed gazebo issues on sm_dance_bot_strikes_back.

- aws demo (#108)
  - Conducted AWS demo.

- Brettpac branch (#110, #111)
  - Improved functionality of sm_multi_stage_1.

- a3 (#113)
  - Implemented a3 feature.

- Remove neo_simulation2 package. (#112)
  - Removed neo_simulation2 package.
  - Corrected formatting.
  - Enabled source build on PR for testing.
  - Adjusted build packages of source CI.

- mm (#115)
  - Implemented mm feature.

- diverse improvements navigation and performance (#116)
  - Made diverse improvements in navigation and performance.

- Feature/diverse improvements navigation performance (#117)
  - Made diverse improvements in navigation and performance.
  - Conducted additional linting and formatting.

- Remove merge markers from a python file. (#119)
  - Removed merge markers from a Python file.

- Feature/slam toggle and smacc deep history (#122)
  - Made progress in navigation, slam toggle client behaviors, and slam_toolbox components.
  - Introduced smacc2::deep_history syntax.
  - Added slam pausing/resuming functionality to sm_dance_bot.

- Move method after the method it calls. Otherwise recursion could happen. (#126)

- Feature/dance bot s pattern (#128, #129)
  - Polished sm_dance_bot and s-pattern.
  - Corrected typo.
  - Refined sm_dance_bot functionality.
```

*pabloinigoblasco*

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
- fixing some formatting and templating on SrConditional
- move trigger logic into headers
- lint

Removed
-------
- removing sm_dance_bot_msgs
- removing parameters smacc
- removing test from main moveit cmake

Authors
-------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section 8
=========

Added
-----

- Feature/replanning 16 dec (#193)
  - Replanned examples
- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation in warehouse2
- Feature/warehouse2 22 12 (#200)
  - Finished warehouse2
- Feature/warehouse2 23 12 (#201)
  - Tuning and fixes (#202)
- Feature/minor tune (#203)
  - Fixed warehouse 3 problems and core improvements (#204)
- Merging code from backport foxy and updates about autoware (#208)
  - Backported to foxy
- Foxy backport (#206)
  - Added galactic CI build
  - Added partial changes for ament_cpplint
  - Added tf2_ros as dependency
  - Updated workflows
  - Bumped ccache version
  - Ignored further packages
  - Satisfied ament_lint_cmake
  - Added missing licenses
  - Disabled cpplint and cppcheck linters
  - Corrected formatters
  - Updated ci-build-source.yml
  - Changed extension of imports
  - Enabled cppcheck
  - Corrected formatting of python file
  - Included necessary package and edited Threesome launch
  - Renamed header files and corrected format
  - Added workflow for checking doc build
  - Created doxygen-deploy.yml
  - Created workflow for testing prerelease builds
  - Renamed to smacc2 and smacc2_msgs
  - Updated name of package and package.xml
  - Reset all versions to 0.0.0
  - Updated changelogs
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs"
  - Updated description table
  - Updated table
  - Copied initial docs
  - Opened new folder for tracing contents
  - Deleted tracing directory
  - Moved tracing.md to tracing directory
  - Added setupTracing.sh
  - Removed manual installation of ros-rolling-ros2trace
  - Created alternative ManualTracing
  - Added new sm markdowns
  - Added a dockerfile for Rolling and Galactic
  - Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
  - Updated tracing/ManualTracing.md
  - Changed wording "smacc application" to "SMACC2 library"
  - Updated smacc_sm_reference_library/sm_atomic/README.md
  - Reactivated smacc2 nav clients for rolling via submodules
  - Renamed tracing events
  - Fixed bug in smacc2 component
  - Added README tutorial for Dockerfile
  - Enabled build of missing rolling repositories
  - Enabled Navigation2 for semi-binary build
  - Removed galactic builds from master and kept only rolling
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Some progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements

Changed
-------

- Updated subscriber publisher components
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Updated cb_navigate_global_position.hpp
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Minor broken build

Fixed
-----

- Several fixes (#194)
- Tuning warehouse3 (#197)
- Undo tuning and errors
- Format issues
- Minor linking errors foxy
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Disable ament_cpplint
- Disable some packages and update workflows
- Ignore further packages
- Disable disabled packages
- Ignore all packages except smacc2 and smacc2_msgs
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Removed galactic builds from master and kept only rolling
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Some progress on navigation rolling
- Renamed folders, deleted tracing.md, edited README.md
- Added smacc2_performance_tools
``` 

*pabloinigoblasco*

## Section_9

### Added
- Added galactic CI setup and renamed rolling files. (#58)
- Added more Readme Updates (#72, #74)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait

### Changed
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Updated smacc2_rta command across readmes
- Updated doxygen links (#70)
- Updated README.md launch command

### Fixed
- Fixed source CI and corrected README overview. (#62)
- Fixed trailing spaces
- Corrected all linters and formatters

### Removed
- Do not execute clang-format on smacc2_sm_reference_library package

### Miscellaneous
- Minor formatting improvements
- Noticed a note that was not removed
- Attempted pre-commit fixes

### Collaborators
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_10
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive; optional node selection
- Base for the `sm_aws_aarehouse` navigation
- Gazebo fixes for showing the robot and the lidar
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo progress
- `sm_multi_stage_1` working progress
- `smacc2::deep_history` syntax for slam toggle client behaviors and slam_toolbox components
- `s-pattern` polishing in `sm_dance_bot`
- First working version of `sm` template and template generator

Changed
~~~~~~~
- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- `sm_dance_bot_lite` updates
- `sm_dance_bot` visualizing `turtlebot3` improvements
- `sm_multi_stage_1` enhancements
- `sm_dance_bot` refinement

Fixed
~~~~
- Remove some compile warnings
- Remove `neo_simulation2` package
- Correct formatting in various files
- Adjust build packages for source CI
- Linting and formatting improvements
- Remove merge markers from a Python file
- Move method after the method it calls to prevent recursion

Removed
~~~~~~~
- `neo_simulation2` package

Contributors
~~~~~~~~~~~~
- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_11

### Added
- Added SM core test (#138)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added .reps dependencies and fixed build errors (#151)
- Added dependency to ur5 client (#151)
- Added reliability QoS config to SmaccPublisherClient (#163)
- Added warehouse2 progress (#179)
- Added Waypoint Inputs (#178)
- Added more Waypoints to sm_dance_bot_warehouse_3 (#184)

### Changed
- Renamed Feature/nav2z to navigation 2 stack (#144)
- Renamed sm_advanced_recovery_1 to sm_pubsub_1 (#171)
- Renamed sm_multi_stage_1 to sm_multi_stage_1 reworking (#172)
- Renamed Feature/aws navigation sm dance bot to Feature/aws navigation sm dance bot (#174)
- Renamed Feature/sm warehouse 2 13 dec 2 to Feature/sm warehouse 2 13 dec 2 (#182)
- Renamed Feature/wharehouse2 dec 14 to Feature/wharehouse2 dec 14 (#185)
- Renamed Feature/cb pure spinning to Feature/cb pure spinning (#188)
- Renamed Feature/planner changes 16 12 to Feature/planner changes 16 12 (#191)
- Renamed Feature/replanning 16 dec to Feature/replanning 16 dec (#193)
- Renamed Feature/undo motion 20 12 to Feature/undo motion 20 12 (#196)

### Fixed
- Fixed launch command in README.md (#148)
- Fixed compile warnings (#137)
- Fixed formatting issues (#134)
- Fixed broken master build (#167)
- Fixed pipeline error (#167)
- Fixed broken build in aws navigation (#174)
- Fixed formatting in warehouse2 (#177)
- Fixed minor issues in several features

### Removed
- Removed sm_dance_bot_msgs (#144)
- Removed parameters smacc (#147)
- Removed node creation and created only a logger (#149)
- Removed test from main moveit cmake (#151)
- Removed some comments in README.md (#149)
- Removed some redundant files in pure spinning behavior (#189)

### Miscellaneous
- Co-authored commits by Ubuntu 20-04-02-amd64, Brett, DecDury, Denis Štogl
- Various minor improvements, progress updates, and format refinements across features

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
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licences.
- Disable cpplint and cppcheck linters.
- Correct formatters.
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
- Added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Improving undo motion navigation in warehouse2
- Tuning warehouse3 (#197)
- Undo tuning and errors
- Format issues
- Replanning for all our examples
- Finishing warehouse2
- Tuning and fixes (#202)
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Odom tracker improvements
- Adding forward behavior retry functionality
- Minor changes
- Minor broken build
- Minor formatting fixes
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Remove example things from Foxy CI setup. (#214)
- Fix rolling builds (#222)
- Do not merge yet - Feature/odom tracker improvements and retry motion (#223)
- Fixing docker for foxy and galactic
- Removing warnings (#213)
- Minor linking errors in foxy
- Changed wording "smacc application" to "SMACC2 library"
- Edit from html to markdown syntax

Removed
-------
- Weird moveit not downloaded repo

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
- Reactivated smacc2 nav clients for rolling via submodules.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You can optionally select the nodes to wait.

Changed
-------
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed smacc2_sm_reference_library package to sm_reference_library.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Fixed
-----
- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview.
- Attempted precommit fixes.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other
-----
- Additional cleanup.
- Edited tracing.md to reflect new tracing event names.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Some progress on navigation rolling.
- More on performance and other issues.
- Minor formatting improvements.
- Progress in aws navigation demo.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
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
Section_14
==========

Added
-----

- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Adding new client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and wait for them to be alive. Optionally select the nodes to wait.
- Base for the sm_aws_warehouse navigation.
- New client behavior, cb_pause_slam.
- sm_dance_bot_lite: visualizing turtlebot3.
- Feature/dance bot launch gz lidar choice: cleaning and lidar show/hide option.
- Feature/sm dance bot lite gazebo fixes: cleaning files, gazebo fixes to show the robot and lidar.
- Feature/sm dance bot strikes back gazebo fixes: cleaning files, gazebo fixes for sm_dance_bot_strikes_back.
- aws demo.
- Brettpac branch: got sm_multi_stage_1 working, gaining traction, progress in stages.
- Remove neo_simulation2 package: correct formatting, adjust build packages for source CI.
- diverse improvements navigation and performance.
- Feature/slam toggle and smacc deep history: progress in navigation, slam toggle client behaviors, and smacc2::deep_history syntax.

Changed
-------

- Navigation parameters fixes on sm_dance_bot.

Fixed
-----

- Remove some compile warnings.
- Remove merge markers from a python file.
- Additional linting and formatting.

Removed
-------

- Removed neo_simulation2 package.

Contributors
------------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_15
===========

Added
-----
- Introducing slam pausing/resuming functionality to sm_dance_bot (#124, #125, #126)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM core test (#138)
- Added dependencies for husky simulation in aws navigation (#174)
- Waypoint Inputs (#178)

Changed
-------
- Move method after the method it calls to prevent recursion (#126)
- Renamed state machines to smacc2_performance_tools (#166)
- Refactored SmaccPublisherClient to include QOS durability (#163)
- Progress on moveit migration testing (#151, #167)
- Progress on aws navigation and refactorings on navigation clients and behaviors (#174)
- Redoing sm_dance_bot_warehouse_3 waypoints (#184)

Fixed
-----
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148)
- Fixed CI format for python version (#148)
- Fixed compiling issues (#164)
- Fixed broken master build (#167, #174)
- Fixed warehouse2 formatting (#180)
- Fixed some linting warnings in moveit migration (#151)
- Fixed some errors introduced on formatting in moveit migration (#151)
- Fixed pipeline error in moveit migration (#167)

Removed
-------
- Removed node creation and create only a logger (#149)
- Removed parameters smacc (#147)
- Removed sm_dance_bot_msgs (#144)

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
- Added minor changes (#195)
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
- Added missing
- Added missing sm
- Added updating subscriber publisher components
- Added progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Added refining cp subscriber cp publisher
- Added improvements in smacc core adding more components mostly developed for autoware demo
- Added autoware demo
- Added missing
- Added foxy ci
- Added fix
- Added minor broken build
- Added some reordering fixes
- Added minor
- Added docker files for different revisions, warnings removval and more testing on navigation
- Added fixing docker for foxy and galactic
- Added docker build files for all versions
- Added dockerfiles (#225)
- Added Fix code generators (#221)
- Added Fix other build issues.
- Added Update SM template and make example code clearly visible.
- Added Remove use of node in the sm performance template.
- Added Updated templated to use Blackboard storage.
- Added Update template to resolve the global data correctly.
- Added Update sm_name.hpp
- Added Feature/retry behavior warehouse 1 (#226)
- Added Foxy backport (#206)
- Added minor formatting fixes
- Added Fix trailing spaces.
- Added Correct codespell.
- Added Correct python linters warnings.
- Added Add galactic CI build because Navigation2 is broken in rolling.
- Added Add partial changes for ament_cpplint.
- Added Add tf2_ros as dependency to find include.
- Added Disable ament_cpplint.
- Added Disable some packages and update workflows.
- Added Bump ccache version.
- Added Ignore further packages
- Added Satisfy ament_lint_cmake
- Added Add missing licences.
- Added Disable cpplint and cppcheck linters.
- Added Correct formatters.
- Added branching example
- Added Disable disabled packages
- Added Update ci-build-source.yml
- Added Change extension
- Added Change extension of imports.
- Added Enable cppcheck
- Added Correct formatting of python file.
- Added Included necessary package and edited Threesome launch

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Removed
-------
- Removed First ensure you have the necessary package installed.
- Removed ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Removed Then run this command.
- Removed Rename header files and correct format.
- Removed Add workflow for checking doc build.
- Removed Update doxygen-check-build.yml
- Removed Create doxygen-deploy.yml
- Removed Use manual deployment for now.
- Removed Create workflow for testing prerelease builds
- Removed Use docs/ as source folder for documentation
- Removed Use docs/ as output directory.
- Removed Rename to smacc2 and smacc2_msgs
- Removed Correct GitHub branch reference.
- Removed Update name of package and package.xml to pass liter.
- Removed Execute on master update
- Removed Reset all versions to 0.0.0
- Removed Ignore all packages except smacc2 and smacc2_msgs
- Removed Update changelogs
- Removed 0.1.0
- Removed Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Removed This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Removed Update description table.
- Removed Update table
- Removed Copy initial docs
- Removed Dockerfile w/ ROS distro as argument
- Removed use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Removed Opened new folder for additional tracing contents
- Removed Delete tracing directory

Co-authored-by
--------------
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
- Moved tracing.md to tracing directory
- Added setupTracing.sh to automate ros-rolling-ros2trace installation
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Added README tutorial for Dockerfile
- Added smacc2_performance_tools
- Performance tests improvements
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library
- Added galactic CI setup and renamed rolling files
- Fixed source CI and corrected README overview
- Updated c_cpp_properties.json
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
- Updated doxygen links
- More Readme Updates
- Created new sm from sm_respira_1
- Feature/core and navigation fixes
- Feature/aws demo progress
- Feature/wait nav2 nodes client behavior
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait

Changed
-------
- Changed wording "smacc application" to "SMACC2 library"
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Reactivated smacc2 nav clients for rolling via submodules
- Renamed tracing events
- Minor formatting changes
- Several core improvements during navigation testing
- Format improvements
- More on performance and other issues
- More changes on performance tests
- Cleaned up sm_reference_library
- Cleaned up sm_atomic_24hr
- Cleaned up sm_advanced_recovery_1
- Fixed pre-commit issues
- Attempted pre-commit fixes
- Corrected all linters and formatters

Removed
-------
- Manual installation of ros-rolling-ros2trace
- Galactic builds from master, keeping only rolling
- Submodules and use .repos file for dependencies
- Tracing.md file
- Trailing spaces
- Do not execute clang-format on smacc2_sm_reference_library package
```

*pabloinigoblasco*

```rst
Section_18
==========

Added
-----
- Feature/aws demo progress (#92)
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and waiting for them to be alive

Changed
-------
- Navigation parameters fixes on sm_dance_bot
- Minor format improvements
- Merge and progress
- Format fixes
- Minor hotfix
- Cleaning and lidar show/hide option
- Gazebo fixes to show the robot and the lidar
- Gazebo fixes for sm_dance_bot_strikes_back
- Precommit cleanup run

Fixed
-----
- Remove some compile warnings (#96)
- Correct formatting

Removed
-------
- Remove neo_simulation2 package (#112)

Contributors
------------
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```

*pabloinigoblasco*

Section_19
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
- Add SM Atomic SM generator.
- Add QOS durability to SmaccPublisherClient.
- Progress on moveit migration testing.
- Add dependencies for husky simulation.
- Update dependencies for husky in rolling and galactic.

Changed
~~~~~~~
- Move method after the method it calls to prevent recursion.
- Resolve compile warnings.
- Remove node creation and create only a logger.
- Moved reference library SMs to smacc2_performance_tools.
- Minor configuration changes.

Fixed
~~~~
- Remove merge markers from a python file.
- Fix CI: format fix python version.
- Noticed launch command was incorrect in README.md.
- Fixing broken master build.
- Fixing broken build.

Removed
~~~~~~~
- Removing sm_dance_bot_msgs.
- Removing parameters smacc.
- Removing test from main moveit cmake.

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

- Waypoint Inputs (#178)
- wharehouse2 progress (#179)
- sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- finetuning waypoints (#187)
- Feature/cb pure spinning (#188)
- pure spinning behavior missing files
- minor changes (#190)
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
- retry behavior warehouse 1
- fixing docker for foxy and galactic
- Update file for fake hardware simulation and add file for gazebo simulation. (#224)
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
- Feature/improvements warehouse3 (#228)
- Foxy backport (#206)

Changed
-------

- fix: some formatting and templating on SrConditional
- fix: move trigger logic into headers
- fix: lint
- format
- more changes and headless
- merge
- headless and other fixes
- default values
- minor
- replanning for all our examples
- format issues
- tuning and fixes
- minor tune
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- minor format fix
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
- Change extension of imports.
- Enable cppcheck

Removed
-------

- missing
- missing sm
- updating subscriber publisher components
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
- Update file for fake hardware simulation and add file for gazebo simulation.
- docker build files for all versions
- progress in barrel husky
- Only rolling version should be pre-released on on master. (#230)
- barrel demo
- barrel search updates
- making models local
- red picuup
- multiple controllable leds plugin
- progress in husky demo
- Update file for fake hardware simulation and add file for gazebo simulation.
- progressing in husky demo
- improving navigation behaviors
- branching example
- Disable disabled packages
- Update ci-build-source.yml
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
- Corrected formatting of python files.
- Renamed header files and corrected format.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Renamed to smacc2 and smacc2_msgs.
- Updated GitHub branch reference.
- Updated package name and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" (commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61).
- Updated description table.
- Updated table.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed galactic builds from master and kept only rolling, removing submodules and using .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
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
- Progress in aws navigation demo.
- Feature/aws demo progress (#80).
- Format improvements.
- More on navigation.
- Reworked sm_advanced_recovery_1 (#83).
- More sm_advanced_recovery_1 work (#85).
- Round 4 of sm_advanced_recovery_1 (#86).
- Brettpac branch (#87).
- Modified sm_atomic_performance_test_a_2 (#89).
- Added sm_multi_stage_1 (#90).
- Fixed precommit for sm_multi_stage_1.
- Updated README.md.

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Tracing directory.
- ManualTracing.md.
- Tracing.md.
- Additional tracing contents folder.
- README note that was not removed.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_22
==========

Added
-----

- New feature: `cb_wait_topic_message` is an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92, #93, #94, #95, #98)
  Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, Denis Štogl <destogl@users.noreply.github.com>, Denis Štogl <denis@stogl.de>

- New client behavior `cb_wait_topic_message` for `nav2`, allowing nodes to subscribe to the `/bond` topic and wait for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)

- Progress in AWS navigation demo (#92, #93, #94, #95, #98)

- Navigation parameters fixes on `sm_dance_bot` (#93, #95, #98)

- New feature: `cb_pause_slam` client behavior (#98)

- `sm_dance_bot_lite` updates (#99)

Changed
-------

- Corrected all linters and formatters (#82)

- Merge and progress (#94)

- Minor hotfix in `doxygen` deployment workflow (#100)

- Cleaning and lidar show/hide option in `sm_dance_bot` visualizing `turtlebot3` (#102, #104)

- Gazebo fixes to show the robot and lidar in `sm_dance_bot` visualizing `turtlebot3` (#104)

Removed
-------

- Removed some compile warnings (#96)

Fixed
-----

- Formatting improvements in various areas
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
  - Introducing slam pausing/resuming functionality for testing sm_dance_bot
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern
  - Noticed typo
- First working version of sm template and template generator (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
  - Build fix
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
  - Some more progress on markers cleanup
- Sm_dance_bot_lite (#136)
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140)
  - Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Remove parameters smacc (#147)
  - Workflows update
  - Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back, and removed some comments
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
- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Add QOS durability to SmaccPublisherClient (#163)
  - Add reliability QOS config

Removed
-------
- Remove neo_simulation2 package (#112)
  - Correct formatting
  - Enable source build on PR for testing
  - Adjust build packages of source CI
```

**Autoría:**
- Pablo Iñigo Blasco (pabloinigoblasco)

Section_24
-----------

Added
~~~~~
- Added `sm_pubsub_1` (#169) with minor configuration and pipeline error fixes.
- Added `sm_pubsub_1 part 2` with progress on moveit behaviors.
- Added `sm_advanced_recovery_1 renaming` (#171).
- Added `sm_multi_stage_1 reworking` with improvements on multistage modes, sequences, and steps.
- Added `sm_multi_stage_1 most` with finishing touches and a readme update.
- Added `Feature/aws navigation sm dance bot (#174)` with repo dependency updates and husky launch file in `sm_dance_bot`.
- Added `warehouse2 (#177)` and `Waypoint Inputs (#178)` with progress on warehouse2 and waypoint input format.
- Added `sm_dance_bot_warehouse_3 (#181)` with format changes and headless mode updates.
- Added `Feature/cb pure spinning (#188)` with format changes and headless mode updates.
- Added `Feature/planner changes 16 12 (#191)` with minor changes and fixes.
- Added `Feature/replanning 16 dec (#193)` with replanning updates for all examples.
- Added `Feature/undo motion 20 12 (#196)` with undo motion improvements and warehouse navigation updates.
- Added `Feature/sync 21 12 (#199)` with format fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/undo motion 20 12 (#198)` with undo motion tuning and error fixes.
- Added `Feature/sync 21 12 (#199)` with format fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes and warehouse2 improvements.
- Added `Feature/minor tune (#203)` with tuning and fixes.
- Added `Feature/warehouse2 22 12 (#200)` and `Feature/warehouse2 23 12 (#201)` with format fixes

```rst
Section_25
==========

Version 0.1.0 (Date: TBD)
-------------------------

### Added
- Added build-status table.
- Included detailed install instructions, adapted from [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme).

### Changed
- Set default build type as `Release` for faster performance and smaller executables.
- Updated examples section.

### Fixed
- Resolved missing dependency in smacc_msgs and reorganized for better overview.
- Fixed getLogger functionality.
- Addressed issues in sm_respira code.
- Fixed bug in smacc2 component.
- Cleaned up formatting in sm_respira_1.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Corrected build-overview table.
- Updated and unified CI configurations.
- Used tf_geometry_msgs.h in galactic.
- Updated to use galactic branches in .repos-file.

### Removed
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

### Other Changes
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" commit.
- Reorganized project structure.
- Updated README.md.
- Added setupTracing.sh for automated installation of necessary packages and configuration of tracing group.
- Updated tracing/ManualTracing.md.
- Reactivated smacc2 nav clients for rolling via submodules.
- Edited tracing.md to reflect new tracing event names.
- Improved performance tests.
- Made more changes on performance and other issues.
- Cleaned up sm_atomic_24hr.
- Made more cleanup in sm_atomic_24hr.
- Cleaned up sm_reference_library.
- Corrected trailing spaces.
- Removed execution of clang-format on smacc2_sm_reference_library package.

### Contributors
- Denis Štogl
- Pablo Iñigo Blasco

Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```
