Changelog for package forward_global_planner
===============================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` into humble.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for a weird issue with ros buildfarm.
  - Further work on the buildfarm issue.
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
### No changes listed.

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release.
- Contributors: pabloinigoblasco

### No changes listed.

Version 0.3.0 (2022-04-04)
--------------------------
### No changes listed.

Version 0.0.0 (2022-11-09)
---------------------------
### Added
- Publisher.
- Reverted "Ignore packages which should not be released."
- Ignored packages that should not be released.
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2.
  - Progress on navigation rolling.
  - Renamed folders, deleted tracing.md, edited README.md.
  - Added smacc2_performance_tools.
  - Performance tests improvements.
  - Format cleanup for sm_respira_1.
  - More format cleanup for sm_respira_1 pre-commit.
  - Added sm_respira_test_2.
  - More changes on performance tests.
  - Do not execute clang-format on smacc2_sm_reference_library package.
  - Reformatted sm_reference_library.
  - Corrected trailing spaces.
  - Added sm_atomic_24hr.
  - Added sm_atomic_performance_trace_1.
  - Updated smacc2_rta command across readmes.
  - Cleaned up sm_atomic_24hr.
  - More cleanup for sm_atomic_24hr.
  - Optimized dependencies in move_base_z_planners_common.
  - Renamed event generator library.
  - Minor formatting.
  - Added galactic CI setup and renamed rolling files (#58).
  - Fixed source CI and corrected README overview (#62).
  - Updated c_cpp_properties.json.
  - Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
  - Updated doxygen links (#70).
  - More Readme Updates (#72).
  - More Readme (#74).
  - Created new sm from sm_respira_1 (#76).
  - Feature/core and navigation fixes (#78).
  - Base for the sm_aws_aarehouse navigation.
  - Progress in aws navigation.
  - Several core improvements during navigation testing.
  - Formatting improvements.
  - Progress in aws navigation demo.
  - Feature/aws demo progress (#80).
  - More on navigation.
  - Reworked sm_advanced_recovery_1 (#83).
  - More work on sm_advanced_recovery_1 (#84).
  - More sm_advanced_recovery_1 work (#85).
  - Round 4 of sm_advanced_recovery_1 (#86).
  - Brettpac branch (#87).
  - Added sm_atomic_performance_test_a_2.
  - Added sm_atomic_performance_test_a_1.
  - Added sm_atomic_performance_test_c_1 (#88).
  - Modified sm_atomic_performance_test_a_2 (#89).
  - Added sm_multi_stage_1.
  - More sm_multi_stage_1 (#91).
  - Updated README.md.
  - Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.
  - Corrected all linters and formatters.
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>.
- Contributors: Denis Štogl, pabloinigoblasco

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive; optional node selection
- Base for the `sm_aws_warehouse` navigation
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` visualizing `turtlebot3`
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo
- `neo_simulation2` package removal
- Source build enabled on PR for testing
- Adjusted build packages for source CI

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`

Fixed
-----

- Minor format fixes
- Compile warnings removed
- Format fixes for gazebo to show the robot and lidar

Removed
-------

- `neo_simulation2` package

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_3
=========

Added
-----

- Diverse improvements in navigation and performance (#116)
- Additional linting and formatting
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Introducing slam pausing/resuming functionality in sm_dance_bot
- First working version of sm template and template generator (#127)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Rolling Docker environment to be executed from any environment (#154)
- Added SM Atomic SM generator (#143)
- Initial migration to smacc2 in moveit client (#151)
- Added QOS durability to SmaccPublisherClient (#163)
- Added reliability qos config to SmaccPublisherClient
- Moved reference library SMs to smacc2_performance_tools (#166)
- Added waypoint inputs (#178)

Changed
-------

- Move method after the method it calls to prevent recursion (#126)
- Fixed launch command in README.md for sm_dance_bot_strikes_back
- Updated package list (#142)
- Removed parameters smacc (#147)
- Updated format in README
- Updated format in move_it PR
- Improved Dockerfile for building local tests
- Added .reps dependencies and fixed build errors
- Updated readme (#164)
- Added missing colon in reliability qos config

Fixed
-----

- Minor tuning to mitigate overshot issue cases in waypoints navigator (#133)
- Fixed CI: format fix python version (#148)
- Fixed compiling issues

Removed
-------

- Removed merge markers from a python file (#119)
- Removed test from main moveit cmake

Collaborators
-------------

- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_4
=========

Added
-----

- Feature/sm warehouse 2 13 dec 2 (#182)
  - Implemented warehouse feature with new changes and headless mode.
  - Default values set.
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- Feature/Brettpac branch (#184)
  - Improved waypoints for sm_dance_bot_warehouse_3.
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- Feature/wharehouse2 dec 14 (#185)
  - Further enhancements to warehouse2 feature.

- Feature/cb pure spinning (#188, #189)
  - Implemented pure spinning behavior with various fixes.

- Feature/planner changes 16 12 (#191)
  - Updated planner with minor changes and fixes.

- Feature/replanning 16 dec (#193)
  - Improved replanning for all examples with several fixes.

- Feature/undo motion 20 12 (#196, #198)
  - Enhanced undo motion navigation for warehouse2.
  - Tuned warehouse3.

- Feature/sync 21 12 (#199)
  - Addressed format issues during sync.

- Feature/warehouse2 22 12 (#200)
  - Fixed format issues and finalized warehouse2.

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2.

- Feature/minor tune (#203)
  - Tuned and fixed minor issues.

- Feature/undo motion 20 12 (#198)
  - Tuned undo motion and fixed errors.

- Feature/sync 21 12 (#199)
  - Addressed format issues during sync.

- Feature/warehouse2 22 12 (#200)
  - Fixed format issues and finalized warehouse2.

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2.

- Feature/minor tune (#203)
  - Tuned and fixed minor issues.

Changed
-------

- Fixed trailing spaces, codespell, and python linters warnings.
- Added galactic CI build due to Navigation2 issues in rolling.
- Updated workflows and dependencies.
- Bumped ccache version.
- Ensured ament_lint_cmake satisfaction.
- Added missing licenses.
- Enabled cppcheck and corrected formatters.
- Updated ci-build-source.yml and extension imports.
- Updated python file formatting.
- Renamed header files and corrected formats.
- Updated doxygen-check-build.yml and created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Renamed to smacc2 and smacc2_msgs.
- Updated GitHub branch references and package names.
- Reset all versions to 0.0.0.
- Ignored packages except smacc2 and smacc2_msgs.
- Updated changelogs.

Removed
-------

- Manual installation of ros-rolling-ros2trace.
  - Now automated in setupTracing.sh.

Fixed
-----

- Fixed bug in smacc2 component.
- Reactivated smacc2 nav clients for rolling.
- Fixed missing rolling repositories build.
- Enabled Navigation2 for semi-binary build.

Contributors
------------

- Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

```rst
Section_5
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
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated c_cpp_properties.json.
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md with launch command.

Fixed
-----
- Fixed source CI and corrected README overview. (#62).
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Other
-----
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress

```rst
Section 6
=========

Added
-----

- New client behavior `cb_wait_topic_message` for nav2: Asynchronous behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: Waits for nav2 nodes to subscribe to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- `sm_multi_stage_1` doubling.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Changed
-------

- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor format improvements.
- Several core improvements during navigation testing.
- Cleaning and lidar show/hide option in `sm_dance_bot`.
- Format fixes for Gazebo to show the robot and lidar.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components.
- Introducing slam pausing/resuming functionality in `sm_dance_bot`.

Fixed
-----

- Remove some compile warnings.
- Correct formatting in removing `neo_simulation2` package.
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.

Removed
-------

- Remove `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
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
- Add SM core test (#138)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)
- Feature/cb pure spinning (#189)

Changed
-------

- Minor tweaks (#130)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Update package list. (#142)
- Update readme (#164)
- Finetuning waypoints (#187)

Fixed
-----

- Noticed typo "Finnaly" corrected to "Finally"
- Build fix
- Waypoints navigator bug (#133)
- Minor format issues (#134)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- Resolve compile warnings (#137)
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing compiling issues
- Fixing broken master build

Removed
-------

- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake

Other
-----

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> in various commits
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
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
- Update name of package and package.xml to pass liter
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
- Changed wording "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md

Changed
-------
- Rename header files and correct format
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Use manual deployment for now
- Correct GitHub branch reference
- Execute on master update
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build

Fixed
-----
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
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
- Change extension of imports
- Update ci-build-source.yml
- Change extension
- Fix minor broken build
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Weird moveit not downloaded repo
- Added missing file from warehouse2
- Update sm_three_some to sm_three_some.launch
- Update tracing event names
- Reactivating smacc2 nav clients for rolling via submodules
- Bug in smacc2 component
- Reverted markdowns to html
- Additional cleanup
- Cleanup
- Edited tracing.md to reflect new tracing event names

Removed
-------
- Merge
- Headless and other fixes
- Default values
- Minor
- Pure spinning behavior missing files
- More fixes
- Replanning for all our examples
- Minor tune
- Undo tuning and errors
- Format
- Minor linking errors foxy
- Missing
- Missing sm
- Updating subscriber publisher components
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Refining cp subscriber cp publisher
- Update cb_navigate_global_position.hpp
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Foxy ci
- Fix
- Branching example
- Ignore all packages except smacc2 and smacc2_msgs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
```

*pabloinigoblasco*

## Section_9

### Added
- Added `smacc2_performance_tools` for performance monitoring.
- Added new feature `cb_wait_topic_message` for asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for `nav2`, `wait nav2 nodes`, subscribing to the `/bond` topic and waiting for them to be alive. Optional selection of nodes to wait for.

### Changed
- Updated mentions of `SMACC/ROS` to `SMACC2/ROS2`.
- Renamed folders, deleted `tracing.md`, and edited `README.md`.
- Renamed `sm_reference_library` to `event generator library`.
- Renamed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.

### Fixed
- Corrected trailing spaces in various files.
- Fixed source CI setup and corrected README overview.
- Corrected all linters and formatters.

### Removed
- Removed galactic builds from master and kept only rolling builds.
- Removed submodules and used `.repos` file for dependencies.
- Removed execution of `clang-format` on `smacc2_sm_reference_library` package.

### Miscellaneous
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Minor formatting and cleanup across multiple files.
- Noted removal of a note that was not needed.

### Contributors
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored by Denis Štogl <destogl@users.noreply.github.com> and <denis@stogl.de>.

```rst
Section_10
==========

Added
-----
- New client behavior for nav2: waits for nav2 nodes to subscribe to the /bond topic and confirms they are alive. Optional node selection available.
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.

Fixed
----
- Removed some compile warnings.
- Minor hotfixes.
- Format fixes.

Removed
-------
- Removed neo_simulation2 package.

Other
-----
- Several core improvements during navigation testing.
- Formatting and minor improvements.
- Merge and progress in development.
- Precommit cleanup run.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

# Section 11

## Added
- First working version of sm template and template generator. (#127)
- Minor tweaks (#130)
- Added SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM core test (#138)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)

## Changed
- Renamed Feature/dance bot s pattern to Feature/sm dance bot refine (#131)
- Renamed Feature/sm dance bot refine 2 to Feature/sm dance bot refine (#132)
- Renamed Feature/sm dance bot strikes back refactoring to Feature/sm dance bot strikes back refactoring (#152)
- Renamed Feature/migration moveit client to Feature/migration moveit client (#151)
- Renamed Feature/testing moveit behaviors to Feature/testing moveit behaviors (#167)
- Renamed Feature/nav2z renaming to Feature/nav2z renaming (#144)
- Renamed Feature/wharehouse2 dec 14 to Feature/wharehouse2 dec 14 (#185)
- Renamed Feature/sm warehouse 2 13 dec 2 to Feature/sm warehouse 2 13 dec 2 (#186)
- Renamed Feature/cb pure spinning to Feature/cb pure spinning (#188)

## Fixed
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments in README.md (#148)
- Fix CI: format fix python version (#148)
- Fixed compiling issues (#164)
- Fixed broken master build (#167)
- Fixed pipeline error (#167)
- Fixed formatting in husky launch file in sm_dance_bot (#174)
- Fixed broken build in aws navigation (#174)
- Fixed some formatting and templating on SrConditional (#168)

## Removed
- Removed node creation and create only a logger (#149)
- Removed parameters smacc (#147)
- Removed sm_dance_bot_msgs (#144)

## Authors
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section 12
==========

Added
-----
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
- Merging to get backport working.
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
- Include necessary package and edit Threesome launch.

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Fixed
-----
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Fix rolling builds (#222)
- Fixing docker for foxy and galactic.
- Removing warnings (#213).

Removed
-------
- Remove example things from Foxy CI setup. (#214).

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
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed tracing events.
- Renamed folders.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Cleaned up sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Minor formatting improvements.
- Corrected trailing spaces.
- Updated smacc2_rta command across readmes.
- Corrected all linters and formatters.

Fixed
----
- Bug in smacc2 component.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other
-----
- Some progress on navigation rolling.
- More changes on performance tests.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- More on performance and other issues.
- Format improvements.
- Attempting pre-commit fixes.
- Fixing precommit.
``` 

*pabloinigoblasco*

```rst
Section_14
==========

Added
-----
- Introducing new feature, cb_wait_topic_message: an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Added new client behavior for nav2, allowing waiting for nav2 nodes subscribing to the /bond topic and ensuring they are alive. Node selection is optional.
- Progress made in AWS navigation demo.

Changed
-------
- Minor formatting improvements.

Fixed
-----
- Navigation parameters fixes on sm_dance_bot.
- Removed some compile warnings.

Removed
-------
- Removed neo_simulation2 package.

Other
-----
- Several core improvements during navigation testing.
- Merge and progress.
- Precommit cleanup run.
- Updates yaml.
- Corrected formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- Various fixes and improvements on sm_dance_bot visualizing turtlebot3.
- Gazebo fixes to show the robot and lidar.
- Gazebo fixes for sm_dance_bot_strikes_back.
- Progress on sm_multi_stage_1.
- Diverse improvements in navigation and performance.
- Ongoing work on sm_multi_stage_1 with multiple stages.
``` 

*pabloinigoblasco*

## Section_15

### Added
- Feature/diverse improvements in navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122)
- Feature/dance bot s pattern (#128)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/migration moveit client (#151)
- Added QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Added durability to SmaccPublisherClient
- Reworked sm_multi_stage_1 with multistage modes and sequences

### Fixed
- Minor tuning to mitigate overshot issue cases in waypoints navigator (#133)
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148)
- Fix CI: format fix python version (#148)
- Fixed compiling issues in docker environment
- Fixed broken master build during moveit testing (#167)
- Fixed formatting in husky launch file in sm_dance_bot (#174)
- Fixed broken build in aws navigation demo (#174)

### Removed
- Removed merge markers from a python file (#119)
- Removed parameters smacc (#147)
- Removed node creation and created only a logger (#149)
- Removed test from main moveit cmake
- Removed sm_dance_bot_msgs
- Removed some comments in the past from README.md

### Miscellaneous
- Noticed typo corrected to "Finally"
- Updated package list (#142)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Precommit cleanup
- Workflows update
- Pending references
- Repos dependency
- Added dependency to ur5 client
- Docker refactoring
- Update readme (#164)
- More readme updates
- Warehouse2 progress (#179)
- Format (#180)
- Sm_dance_bot_warehouse_3 (#181)
- Default values

Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>

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
- improving undo motion navigation warehouse2
- tuning warehouse3 (#197)
- Feature/undo motion 20 12 (#198)
- minor changes
- improving undo motion navigation warehouse2
- undo tuning and errors
- Feature/sync 21 12 (#199)
- format issues
- Feature/warehouse2 22 12 (#200)
- format issues
- finishing warehouse2
- Feature/warehouse2 23 12 (#201)
- tuning and fixes (#202)
- Feature/minor tune (#203)
- tuning and fixes
- minor tune
- fixing warehouse 3 problems, and other core improvements (#204)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- added missing file from warehouse2 (#205)
- docker build files for all versions
- dockerfiles (#225)
- Feature/retry behavior warehouse 1 (#226)
- backport to foxy
- Foxy backport (#206)
- minor formatting fixes
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
- Change extension
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- ros2 launch sm_three_some sm_three_some
- to
- ros2 launch sm_three_some sm_three_some.launch
- First ensure you have the necessary package installed
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
- Update SM template and make example code clearly visible
- Remove use of node in the sm performance template
- Updated templated to use Blackboard storage
- Update template to resolve the global data correctly
- Update sm_name.hpp
```

```rst
Section_17
==========

Added
-----
- Added setupTracing.sh script to install necessary packages and configure tracing group.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added galactic CI setup and renamed rolling files. (#58)
- Added source CI fix and corrected README overview. (#62)
- Added new sm from sm_respira_1 (#76)
- Added base for the sm_aws_aarehouse navigation.
- Added sm_multi_stage_1.
- Added sm_atomic_performance_test_c_1 (#88).
- Added sm_atomic_performance_test_a_1 and sm_atomic_performance_test_a_2.
- Added sm_atomic_performance_test_a_2 modification (#89).
- Added sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1.
- Added sm_atomic_performance_test_c_1 (#88).
- Added sm_advanced_recovery_1 reworked (#83), (#84), (#85), (#86).
- Added sm_advanced_recovery_1 round 4 (#86).
- Added sm_atomic_performance_trace_1.
- Added sm_atomic_24hr.
- Added sm_respira_test_2.
- Added sm_respira_1 format cleanup and pre-commit.
- Added sm_reference_library reformatting.
- Added Dockerfile with ROS distro as argument.
- Added alternative ManualTracing.
- Added sm markdowns.
- Added new smacc2 and smacc2_msgs packages.
- Added tracing event renaming.
- Added Navigation2 for semi-binary build.
- Added build of missing rolling repositories.
- Added reactivation of smacc2 nav clients for rolling via submodules.
- Added optimization of dependencies in move_base_z_planners_common.
- Added renaming of event generator library.
- Added more performance tests improvements.
- Added more changes on performance tests.
- Added more on performance and other issues.
- Added more on navigation.
- Added more on navigation demo progress.
- Added more on navigation testing.
- Added more on navigation formatting improvements.
- Added more on navigation format improvements.
- Added more on navigation progress.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format improvements.
- Added more on navigation demo format

```rst
Section_18
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_aarehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_pause_slam`.
- `sm_dance_bot_lite` visualization for TurtleBot3.
- Lidar show/hide option for `sm_dance_bot`.
- Gazebo fixes to show the robot and lidar.
- Doubling in `sm_multi_stage_1`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Changed
-------
- Corrected all linters and formatters.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor formatting improvements.
- Merge and progress in development.
- Hotfixes and minor improvements.

Fixed
----
- Removed some compile warnings.

Removed
-------
- Redundant formatting improvements.

Contributors
------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_19

### Added
- Added multistage modes to `sm_multi_stage_1`.
- Added sequences to `sm_multi_stage_1`.
- Added steps to `sm_multi_stage_1`.
- Added sequence d to `sm_multi_stage_1`.
- Added sequence c to `sm_multi_stage_1`.
- Added mode_5_sequence_b.
- Added mode_4_sequence_b.
- Added finishing touches 1 to `sm_multi_stage_1`.
- Added AWS navigation to `sm_dance_bot`.

### Changed
- Reworked `sm_multi_stage_1` with improvements in multistage modes, sequences, and steps.

### Fixed
- Fixed a recursion issue by moving a method after the method it calls.
- Fixed minor format issues.
- Fixed compile warnings.
- Fixed CI format for Python version.
- Fixed launch command in README.md for `sm_dance_bot_strikes_back`.
- Fixed some linting warnings.
- Fixed compiling issues.

### Removed
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs`.
- Removed parameters from `smacc`.

### Miscellaneous
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- Added SM core test.
- Added QOS durability to `SmaccPublisherClient`.
- Updated package list.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Updated README.
- Moved reference library SMs to `smacc2_performance_tools`.
- Refactored Docker environment for execution from any environment.
- Added SM Atomic SM generator.
- Added dependencies for husky simulation.
- Progressed in navigation, slam toggle client behaviors, and slam_toolbox components.
- Progressed in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Progressed in moveit migration testing.
- Progressed in moveit testing behaviors.
- Progressed in `sm_dance_bot_lite`.
- Progressed in `sm_pubsub_1`.
- Progressed in `sm_advanced_recovery_1` renaming.
- Progressed in `sm_multi_stage_1` reworking with sequences and modes.
- Progressed in `sm_multi_stage_1` with sequence d, sequence c, and finishing touches.
- Progressed in `sm_dance_bot` with s-pattern and refinements.
- Progressed in `sm_dance_bot_strikes_back` refactoring.
- Progressed in `sm_dance_bot` with refinements and improvements.
- Progressed in waypoint 4 and iterations changes for robot course completion.
- Progressed in navigation improvements.
- Progressed in moveit migration with fixes and dependencies.
- Progressed in pipeline error fixing.
- Progressed in broken master build fixing.
- Progressed in sm_pubsub_1 part 2.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence d and sequence c.
- Progressed in `sm_multi_stage_1` with most finishing touches.
- Progressed in `sm_multi_stage_1` with mode_5_sequence_b and mode_4_sequence_b.
- Progressed in `sm_multi_stage_1` with sequences and steps.
- Progressed in `sm_multi_stage_1` with multistage modes and sequences.
- Progressed in `sm_multi_stage_1` with sequence

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
- Improvements in smacc core adding more components.
- Autoware demo.
- Docker files for different revisions, warnings removal, and more testing on navigation.
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
- Merge changes.
- Headless and other fixes.
- Default values.
- Several fixes.
- Format issues.
- Tuning and fixes.
- Minor tune.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.
- Making models local.
- Red picuup.
- Foxy backport.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.

Fixed
-----
- Fixing broken build.
- Fixing broken source build.

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
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>

```rst
Section_21
==========

Added:
- First ensure you have the necessary package installed:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Rename to smacc2 and smacc2_msgs.
- Added setupTracing.sh:
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Changed wording "smacc application" to "SMACC2 library".
- Update smacc_sm_reference_library/sm_atomic/README.md: edit from html to markdown syntax.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Sm_respira_1 format cleanup.
- Sm_respira_test_2.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Sm_reference_library reformatting.
- Correct trailing spaces.
- Sm_atomic_24hr.
- Sm_atomic_performance_trace_1.
- Clean up of sm_atomic_24hr.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files (#58).
- Fix source CI and correct README overview (#62).
- Update c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
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
- More sm_advanced_recovery_1 (#84).
- More sm_advanced_recovery_1 work (#85).
- Sm_advanced_recovery_1 round 4 (#86).
- Brettpac branch (#87).
- Sm_atomic_performance_test_a_2.
- Sm_atomic_performance_test_a_1.
- Sm_atomic_performance_test_c_1 (#88).

Changed:
- Ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Change extension of imports.
- Correct formatting of python file.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Update changelogs.
- Update description table.
- Update table.
- Copy initial docs.
- Dockerfile w/ ROS distro as argument: use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/".
- Opened new folder for additional tracing contents.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Removed manual installation of ros-rolling-ros2trace: this is now automated in setupTracing.sh, location of sh file assumed if user follows README.md under "Getting started".
- Changed wording "smacc application" to "SMACC2 library".
- Update smacc2_sm_reference_library/sm_atomic/README.md: edit from html to markdown syntax.
- Reactivating smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Bug in smacc2 component.
- Reverted markdowns to html.
- Added README tutorial for Dockerfile.
- Additional cleanup.
- Cleanup.
- Edited tracing.md to reflect new tracing event names.
- Minor formatting.
- More on performance tests.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- More sm_atomic_24hr cleanup.
- Minor formatting.
- More on navigation.
- Format improvements.
- Format improvements.
- More on navigation.
- Fix pre-commit.
- More sm_advanced_recovery_1.
- More sm_advanced_recovery_1 work.
- Sm_advanced_recovery_1 round 4.
- Base for the sm_aws_aarehouse navigation.
- Progressing in AWS navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- Format improvements.
- Format improvements.
- More on navigation.
- Sm_atomic_performance_test_c_1.

Removed:
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licences.
- Correct formatters.
- Branching example.
- Update ci-build-source.yml.
- Change extension.
- Enable cppcheck.
- Correct formatting of python file.
- Included necessary package and edited Threesome launch.
- Use manual deployment for now.
- Create workflow for testing prerelease builds.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Ignore all packages except smacc2 and smacc2_msgs.
- Update changelogs.
- Revert "Ignore all packages except smacc2 and smacc2_msgs".
```


*pabloinigoblasco*

```rst
Section_22
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection.
- New client behavior: cb pause slam for pausing SLAM operations.
- New feature: sm_dance_bot visualizing turtlebot3.
- New feature: dance bot launch gz lidar choice.

Changed
-------

- Updated launch command.
- Corrected all linters and formatters.

Fixed
-----

- Fixed precommit issues.
- Fixed navigation parameters on sm_dance_bot.
- Removed some compile warnings.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

Section_23
==========

Added
-----
- Added visualization for turtlebot3 in sm_dance_bot.
- Added lidar show/hide option for cleaning.
- Added gazebo fixes for sm_dance_bot_lite (#104).
- Added gazebo fixes to display the robot and lidar.
- Added gazebo fixes for sm_dance_bot_strikes_back.
- Added AWS demo (#108).
- Added progress in sm_multi_stage_1 (#114).
- Added diverse improvements in navigation and performance (#116).
- Added slam toggle client behaviors and smacc2::deep_history syntax in Feature/slam toggle and smacc deep history (#122).
- Added progress in testing sm_dance_bot with slam pausing/resuming functionality.
- Added more changes in sm_dance_bot in Feature/dance bot s pattern (#128).
- Added first working version of sm template and template generator (#127).
- Added SM core test (#138).
- Added minor navigation improvements (#141).
- Added local action msgs usage.
- Added navigation 2 stack renaming in Feature/nav2z renaming (#144).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added rolling Docker environment to be executed from any environment (#154).
- Added slight changes to waypoint 4 and iterations for robot course completion (#155).
- Added initial migration to smacc2 in Feature/migration moveit client (#151).

Changed
-------
- Improved formatting in cleaning files.
- Enhanced formatting in gazebo fixes.
- Refined sm_dance_bot and s-pattern in Feature/dance bot s pattern (#129).
- Refactored sm_dance_bot strikes back (#152).
- Updated format in move_it PR.
- Improved Dockerfile for building local tests.

Fixed
-----
- Fixed format issues.
- Fixed source build on PR for testing.
- Fixed build packages of source CI.
- Fixed compiling issues.
- Fixed CI format for python version (#148).
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed errors introduced on formatting in migration to smacc2.
- Fixed linting warnings.
- Mitigated overshot issue cases in waypoints navigator.
- Removed neo_simulation2 package.
- Removed merge markers from a python file (#119).
- Removed parameters in smacc (#147).
- Removed node creation and created only a logger (#149).
- Removed test from main moveit cmake.
- Removed sm_dance_bot_msgs.
- Removed parameters in smacc.
- Removed some comments in README.md for sm_dance_bot_strikes_back.
- Removed test from main moveit cmake.

Removed
-------
- Removed neo_simulation2 package.

Authors
-------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Brett <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>

Section_24
==========

Added
-----
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Feature/wharehouse2 dec 14 (#185)
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
- Repo dependency
- Husky launch file in sm_dance_bot
- Update dependencies for husky in rolling and galactic
- Finetuning waypoints (#187)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Minor tune
- Improving undo motion navigation warehouse2
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green

Fixed
-----
- Add a missing colon
- Fixing pipeline error
- Fixing broken master build
- Fixing broken build
- Several fixes (#194)
- Format issues
- Minor format
- Fixing startup problems in warehouse 3

Removed
-------
- Pre-commit cleanup
- Remove line
- Minor configuration

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

pabloinigoblasco

```rst
Section_25
==========

Added
-----
- Docker improvements.
- Runtime dependency.
- Restoring UR dependency.
- Testing dance bot demos.
- Updating Galactic repositories.

Changed
-------
- Master rolling to Galactic backport.

Fixed
-----
- Fixing build.

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
