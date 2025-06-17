Changelog for package sr_conditional
=====================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from https://github.com/robosoft-ai/SMACC2 into humble
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted to fix weird issue with ros buildfarm
  - More details on the buildfarm issue
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
No changes listed.

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release
- Contributors: pabloinigoblasco

### Changed
- Progress made in humble SMACC2 deb generation
- Reverted "Ignore packages which should not be released."
- Contributors: Denis Štogl, pabloinigoblasco

Version 0.3.0 (2022-04-04)
--------------------------
No changes listed.

Version 0.0.0 (2022-11-09)
---------------------------
### Added
- More progress in humble SMACC2 deb generation
- Reverted "Ignore packages which should not be released."
- Ignored packages that should not be released
- Fixed: Initialized conditionFlag as false (#274) (#278)
  - Fixed: Initialized conditionFlag as false (#274)
  - Fixed precommit
  - Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
- Feature/master rolling to galactic backport (#236)
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
  - Reformatted sm_reference_library
  - Corrected trailing spaces
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Minor formatting
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
  - Feature/aws demo progress (#80)
  - More on navigation
  - Reworked sm_advanced_recovery_1 (#83)
  - More sm_advanced_recovery_1 work (#85)
  - Sm_advanced_recovery_1 round 4 (#86)
  - Brettpac branch (#87)
  - Added sm_atomic_performance_test_a_2
  - Added sm_atomic_performance_test_a_1
  - Added sm_atomic_performance_test_c_1 (#88)
  - Modified sm_atomic_performance_test_a_2 (#89)
  - Added sm_multi_stage_1
  - More sm_multi_stage_1 (#91)
  - Updated README.md
  - Wait topic message client behavior (#81)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Attempted precommit fixes
  - Feature/wait nav2 nodes client behavior (#82)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive; optionally, you can select the nodes to wait for
  - Corrected all linters and formatters
  - Co-authored-by: Denis Štogl <denis@stogl.de>
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message` (#92)
  - Asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: `cb_wait_nav2_nodes` (#92)
  - Waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive
- New feature: `cb_pause_slam` (#98)
  - Client behavior for pausing SLAM functionality
- New client behavior: `cb_pause_slam` (#98)
  - Pauses SLAM functionality
- New feature: `sm_dance_bot_lite gazebo fixes` (#104)
  - Fixes for visualizing TurtleBot3 in Gazebo
- New feature: `sm_dance_bot_strikes_back gazebo fixes` (#105)
  - Fixes for visualizing TurtleBot3 in Gazebo for `sm_dance_bot_strikes_back`
- New feature: `aws demo` (#108)
  - AWS demonstration improvements
- New feature: `sm_multi_stage_1 doubling` (#103)
  - Enhancements for `sm_multi_stage_1`

Changed
-------

- Improved core functionality during navigation testing
- Formatting enhancements for better readability
- Navigation parameters fixes for `sm_dance_bot`
- Minor formatting improvements
- Merge and progress updates
- Hotfix for minor issues
- Precommit cleanup run
- Corrected formatting for `neo_simulation2` package removal
- Enabled source build on PR for testing

Removed
-------

- Removed some compile warnings (#96)
- Removed `neo_simulation2` package
```

*pabloinigoblasco*

Section_3
==========

Added
-----
- Adjusted build packages of source CI.
- Added diverse improvements in navigation and performance.
- Introduced feature for diverse improvements in navigation performance.
- Implemented slam toggle and smacc deep history feature.
- Added dance bot s pattern feature.
- Added first working version of sm template and template generator.
- Added SM core test.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Added Waypoint Inputs.

Changed
-------
- Moved method after the method it calls to prevent recursion.
- Renamed sm_advanced_recovery_1.
- Reworked sm_multi_stage_1 with multistage modes and sequences.
- Moved reference library SMs to smacc2_performance_tools.
- Updated package list.
- Rolled Docker environment to be executed from any environment.
- Refactored sm dance bot strikes back.
- Migrated moveit client to smacc2.
- Updated READMEs with SVGs.
- Updated README with launch command correction.
- Updated README with more information.
- Added durability and reliability QoS configurations to SmaccPublisherClient.

Fixed
-----
- Removed merge markers from a python file.
- Fixed CI format for Python version.
- Removed node creation and created only a logger.
- Fixed launch command in README.md.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.

Removed
-------
- Removed merge markers from a python file.
- Removed parameters smacc.
- Removed test from main moveit cmake.
- Removed some comments from README.md.
- Removed sm_dance_bot_msgs.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_4
=========

Version 0.1.0 (2022-01-01)
---------------------------

Added
~~~~~

- First ensure you have the necessary package installed:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Rename header files and correct format.
- Add workflow for checking doc build.
- Update doxygen-check-build.yml.
- Create doxygen-deploy.yml.
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

Changed
~~~~~~~

- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Removed
~~~~~~~

- Revert "Ignore all packages except smacc2 and smacc2_msgs" (commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61).

Fixed
~~~~~

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
- Update description table.
- Update table.
- Copy initial docs.
- Dockerfile w/ ROS distro as argument: use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/".
- Opened new folder for additional tracing contents.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh: installs necessary packages and configures tracing group.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Version 0.1.0 (2022-01-01)
---------------------------

Added
~~~~~

- Feature/sm warehouse 2 13 dec 2 (#182): format, more changes and headless, merge, headless and other fixes, default values.
- Brettpac branch (#184): sm_dance_bot_warehouse_3, redoing sm_dance_bot_warehouse_3 waypoints, more waypoints.
- Feature/wharehouse2 dec 14 (#185): warehouse2, minor.
- Feature/sm warehouse 2 13 dec 2 (#186): format, more changes and headless, merge, headless and other fixes, default values, minor.
- Finetuning waypoints (#187).
- Feature/cb pure spinning (#188): format, more changes and headless, merge, headless and other fixes, default values, minor.
- Feature/cb pure spinning (#189): format, more changes and headless, merge, headless and other fixes, default values, minor, pure spinning behavior missing files, minor changes.
- Feature/planner changes 16 12 (#191): minor changes, more fixes, minor, minor, replanning for all our examples.
- Feature/replanning 16 dec (#193): minor changes, replanning for all our examples, several fixes.
- Feature/undo motion 20 12 (#196): minor changes, replanning for all our examples, improving undo motion navigation warehouse2, minor, tuning warehouse3.
- Feature/undo motion 20 12 (#198): minor changes, replanning for all our examples, improving undo motion navigation warehouse2, minor, undo tuning and errors, format.
- Feature/sync 21 12 (#199): minor changes, replanning for all our examples, format issues.
- Feature/warehouse2 22 12 (#200): minor changes, replanning for all our examples, format issues, finishing warehouse2.
- Feature/warehouse2 23 12 (#201): minor changes, replanning for all our examples, tuning and fixes.
- Feature/minor tune (#203): tuning and fixes, minor tune, fixing warehouse 3 problems, and other core improvements.
- Foxy backport (#206): backport to foxy, minor format, minor linking errors foxy.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_5
=========

Added
-----

- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------

- Edited tracing.md to reflect new tracing event names.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Renaming of event generator library.
- Update smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated launch command.
- Correct all linters and formaters.

Fixed
-----

- Correct trailing spaces.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Optimized dependencies in move_base_z_planners_common.
- Fix source CI and correct README overview.
- Several formatting improvements.
- Attempting precommit fixes.

Removed
-------

- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.

Other
-----

- Some progress on navigation rolling.
- More on performance and other issues.
- Minor formatting.
- Minor.
- Progressing in aws navigation.
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
- Progress in aws navigation demo.
- Progress in aws navigation demo.
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
Section_6
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: `add` for waiting for nav2 nodes subscribing to the `/bond` topic and ensuring they are alive; optional node selection
- Progress in AWS navigation demo
- Base for the `sm_aws_warehouse` navigation
- Several core improvements during navigation testing
- `cb_pause_slam` client behavior
- `sm_dance_bot_lite` visualizing TurtleBot3
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo
- Got `sm_multi_stage_1` working (barely)
- Gaining traction with `sm_multi_stage_1`
- Diverse improvements in navigation and performance
- Progress in navigation, `slam_toggle` client behaviors, and `slam_toolbox` components; `smacc2::deep_history` syntax
- Introducing slam pausing/resuming functionality in testing `sm_dance_bot`
- More changes in `sm_dance_bot`

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`
- Minor formatting improvements

Fixed
-----

- Removed some compile warnings
- Corrected formatting
- Enabled source build on PR for testing
- Adjusted build packages of source CI
- Additional linting and formatting
- Removed merge markers from a Python file

Removed
-------

- Removed `neo_simulation2` package

Contributors
------------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_7
=========

Added
-----
- First working version of sm template and template generator. (#127)
- Add SM core test (#138)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Waypoint Inputs (#178)
- SrConditional fixes and formatting (#168)

Changed
-------
- Move method after the method it calls to prevent recursion. (#126)
- Refactored husky launch file in sm_dance_bot for AWS navigation. (#174)
- Finetuned waypoints for better performance. (#187)

Fixed
-----
- Fix CI: format fix python version (#148)
- Resolve compile warnings (#137)
- Fixing broken master build for moveit testing. (#167)

Removed
-------
- Removed node creation and create only a logger. (#149)
- Removed parameters smacc for cleaner code. (#147)

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_8
=========

Added
-----
- Feature/cb pure spinning (#189): Introduces pure spinning behavior and fixes missing files.
- Feature/planner changes 16 12 (#191): Includes minor changes and fixes.
- Feature/replanning 16 dec (#193): Implements replanning for all examples and various fixes.
- Feature/undo motion 20 12 (#196): Enhances undo motion navigation in warehouse2.
- Feature/sync 21 12 (#199): Addresses format issues.
- Feature/warehouse2 22 12 (#200): Resolves format issues and finalizes warehouse2.
- Feature/warehouse2 23 12 (#201): Fine-tunes and fixes issues.
- Feature/minor tune (#203): Improves warehouse 3 functionality and makes core improvements.
- Merging code from backport foxy and updates about autoware (#208): Includes minor changes and backport to foxy.

Changed
-------
- Updated subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components.
- Corrected codespell and python linters warnings.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency.
- Disabled ament_cpplint, cpplint, and cppcheck linters.
- Bumped ccache version.
- Ignored further packages.
- Satisfied ament_lint_cmake.
- Added missing licenses.
- Updated ci-build-source.yml.
- Changed extension of imports.
- Enabled cppcheck.
- Corrected formatting of python file.
- Included necessary package and edited Threesome launch.
- Renamed header files and corrected format.
- Added workflow for checking doc build.
- Updated doxygen-check-build.yml and doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Renamed to smacc2 and smacc2_msgs.
- Updated name of package and package.xml.
- Reset all versions to 0.0.0.
- Updated changelogs.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Updated description table.
- Updated table.
- Copied initial docs.
- Opened new folder for additional tracing contents.
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh for automated installation.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Added README tutorial for Dockerfile.
- Additional cleanup.

Removed
-------
- Manual installation of ros-rolling-ros2trace.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
```
---

Este fragmento mejorado del changelog mantiene toda la información relevante, agrupando los cambios similares bajo las categorías de "Added", "Changed" y "Removed". Además, se ha conservado la autoría de Pablo Iñigo Blasco.

Section 9
----------

Added
~~~~~
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
~~~~~~~
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Renaming of event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.

Fixed
~~~~
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview.
- Fixed pre-commit issues.
- Attempted pre-commit fixes.

Removed
~~~~~~~
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other
~~~~
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progressing in AWS navigation.
- More on navigation.
- Format improvements.
- Minor format adjustments.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_10
==========

Added
-----
- New client behavior `cb_wait_topic_message` for asynchronous waiting and optional content check on a topic message.
- New client behavior for `nav2` to wait for nodes subscribing to `/bond` topic to become active, with optional node selection.
- Base for `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature with precommit updates.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- Lidar show/hide option in `sm_dance_bot` gazebo fixes.
- `sm_multi_stage_1` doubling and `sm_dance_bot_strikes_back` gazebo fixes.
- Progress in `aws` navigation demo.
- `smacc2::deep_history` syntax in `slam_toggle` client behaviors.
- `slam_toolbox` components integration.
- `dance_bot_s` pattern in `sm_dance_bot`.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Format improvements in various areas.
- Minor hotfixes and format corrections.
- Adjustments in source build packages for CI testing.
- Method reordering to prevent recursion in `sm_dance_bot`.

Fixed
-----
- Compile warnings removal.
- Merge markers cleanup from a Python file.

Removed
-------
- `neo_simulation2` package removal.

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pablo@ibrobotics.com>
```

## Section_11

### Added
- First working version of sm template and template generator. (#127)
- Added SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Add SM core test (#138)
- Add QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added Waypoint Inputs (#178)

### Changed
- Renamed SMs to smacc2_performance_tools (#166)
- Refactored SM dance bot strikes back (#152)
- Refactored SM warehouse 2 13 dec 2 (#182)
- Fine-tuned waypoints (#187)

### Fixed
- Fixed launch command for sm_dance_bot_strikes_back in README.md
- Fixed CI: format fix python version (#148)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error

### Removed
- Removed node creation and create only a logger. (#149)
- Removed parameters smacc (#147)
- Removed sm_dance_bot_msgs

### Miscellaneous
- Minor tweaks (#130)
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Minor configuration changes
- Precommit cleanup
- Workflows update
- Update package list. (#142)
- Update readme (#164)
- Update readme
- More readme updates
- More refinement in sm_dance_bot
- More progress on markers cleanup
- Progress in the sm_dance_bot tests (#135)
- Progress on moveit migration testing
- Progress on moveit
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot
- More refinement in sm_dance_bot

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
- Rename header files and correct format.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update
- Reset all versions to 0.0.0
- Update changelogs
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
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- weird moveit not downloaded repo
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- branching example
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Fixed
-----
- minor broken build
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- removing warnings (#213)
- minor linking errors foxy
- minor format issues

Removed
-------
- missing
- missing sm

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

- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
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
- Updated smacc2_rta command across readmes.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Minor formatting improvements.

Fixed
-----

- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct all linters and formatters.

Removed
-------

- Removed galactic builds from master and kept only rolling.
- Removed tracing.md.

Other
-----

- Reactivated smacc2 nav clients for rolling via submodules.
- Some progress on navigation rolling.
- More changes on performance tests.
- Performance tests improvements.
- More on performance and other issues.
- Format cleanup in sm_respira_1.
- Format cleanup in sm_respira_1 pre-commit.
- Format cleanup in sm_atomic_24hr.
- Format cleanup in sm_atomic_performance_trace_1.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_atomic_24hr more.
- Cleaned up sm_reference_library.
- Cleaned up sm_advanced_recovery_1.
- Cleaned up sm_advanced_recovery_1 more.
- Cleaned up sm_advanced_recovery_1 round 4.
- Modified sm_atomic_performance_test_a_2.
- Modified sm_atomic_performance_test_a_2 more.
- Modified sm_atomic_performance_test_c_1.
- Modified sm_multi_stage_1.
- Modified sm_multi_stage_1 more.
- Fixed source CI and corrected README overview.
- Attempted pre-commit fixes.
- Trying to fix Pre-Commit.

Commits
-------

- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```
*pabloinigoblasco*

```rst
Section_14
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: `add` for waiting for nav2 nodes subscribing to the `/bond` topic and ensuring they are alive; optional node selection
- Progress in AWS navigation demo
- Base for the `sm_aws_warehouse` navigation
- Navigation parameters fixes on `sm_dance_bot`
- New client behavior: `cb_pause_slam`
- `sm_dance_bot_lite` visualizing TurtleBot3
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo
- Got `sm_multi_stage_1` working (barely)
- Brettpac branch
- Remove `neo_simulation2` package
- Diverse improvements in navigation and performance

Changed
-------

- Minor formatting improvements

Fixed
-----

- Remove some compile warnings
- Minor hotfix
- Format fixes
- Precommit cleanup run
- Correct formatting
- Enable source build on PR for testing
- Adjust build packages of source CI

Removed
-------

- Removed `neo_simulation2` package

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

# Changelog

## Version 1.0.0 (2022-01-01)

### Added
- Feature/diverse improvements in navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122)
- First working version of sm template and template generator (#127)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Moved reference library SMs to smacc2_performance_tools (#166)

### Fixed
- Resolve compile warnings (#137)
- Fix CI: format fix python version (#148)
- Fixing broken master build (#167)
- Fix formatting in Feature/aws navigation sm dance bot (#174)

### Removed
- Remove merge markers from a python file (#119)
- Removing sm_dance_bot_msgs (#144)
- Removing parameters smacc (#147)
- Remove node creation and create only a logger (#149)

## Version 1.1.0 (2022-02-01)

### Added
- Added remaining SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)

### Changed
- Update package list (#142)
- Update readme (#164)

### Fixed
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back (#164)

### Removed
- Remove test from main moveit cmake (#151)

## Version 1.2.0 (2022-03-01)

### Added
- Added dependency to ur5 client (#151)

### Changed
- Minor changes in sm_dance_bot_lite (#136)

### Fixed
- Minor tuning to mitigate overshot issue cases in waypoints navigator bug (#133)
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Minor configuration in Feature/testing moveit behaviors (#167)
- Format (#180)

### Removed
- Removing test from main moveit cmake (#151)

## Version 1.3.0 (2022-04-01)

### Added
- Added .reps dependencies and fixed some build errors (#151)

### Changed
- Progress in the moveit migration testing (#151)

### Fixed
- Fixing compiling issues (#151)

### Removed
- Removing parameters smacc (#147)

## Version 1.4.0 (2022-05-01)

### Added
- Added reliability qos config to SmaccPublisherClient (#163)

### Changed
- Progress on aws navigation and some other refactorings on navigation clients and behaviors (#174)

### Fixed
- Fixing broken build in Feature/aws navigation sm dance bot (#174)

### Removed
- Remove test from main moveit cmake (#151)

## Version 1.5.0 (2022-06-01)

### Added
- Warehouse2 progress (#179)
- Waypoint Inputs (#178)

### Changed
- Minor changes in sm_dance_bot_warehouse_3 (#181)

### Fixed
- Fix formatting in Feature/sm warehouse 2 13 dec 2 (#182)

### Removed
- Default values in Feature/sm warehouse 2 13 dec 2 (#182)

## Version 1.6.0 (2022-07-01)

### Added
- More changes in sm_dance_bot (#125)
- More changes in sm_dance_bot (#128)
- More changes in sm_dance_bot (#129)
- More changes in sm_dance_bot (#131)
- More changes in sm_dance_bot (#132)
- More changes in sm_dance_bot (#151)
- More changes in sm_dance_bot (#152)
- More changes in sm_dance_bot (#174)
- More changes in sm_dance_bot (#175)
- More changes in sm_dance_bot (#177)
- More changes in sm_dance_bot (#179)
- More changes in sm_dance_bot (#181)
- More changes in sm_dance_bot (#182)

### Changed
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components (#122)
- Progress in testing sm_dance_bot introducing slam pausing/resuming functionality (#122)
- Progress on moveit (#151)
- Progress on moveit migration testing (#151)
- Progress on aws demo (#174)
- Progress on aws navigation and some other refactorings on navigation clients and behaviors (#174)

### Fixed
- Noticed typo in Feature/slam toggle and smacc deep history (#122)
- Minor format in Feature/more_sm_dance_bot_fixes (#122)
- Minor format in Feature/sm dance bot refine (#131)
- Minor format in Feature/sm dance bot refine 2 (#132)
- Build fix in Feature/sm dance bot refine 2 (#132)
- Minor format in Feature/nav2z renaming (#144)
- Formatting in Feature/sm warehouse 2 13 dec 2 (#182)

### Removed
- Remove merge markers from a python file (#119)
- Removing sm_dance_bot_msgs (#144)
- Removing parameters smacc (#147)
- Remove node creation and create only a logger (#149)
- Default values in Feature/sm warehouse 2 13 dec 2 (#182)

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
- Fix code generators (#221)
- Feature/retry behavior warehouse 1 (#226)
- Foxy backport (#206)

Changed
-------
- Update SM template and make example code clearly visible.
- Remove use of node in the sm performance template.
- Updated templated to use Blackboard storage.
- Update template to resolve the global data correctly.
- Update sm_name.hpp
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
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
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
- 0.1.0

Removed
-------
- Some reordering fixes
- Minor broken build
- Docker files for different revisions, warnings removal and more testing on navigation
- Fixing docker for foxy and galactic
``` 

**Autor:** Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_17
==========

Added
-----

- Added setupTracing.sh script for installing necessary packages and configuring tracing group.
- Added README tutorial for Dockerfile.
- Added new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added sm_multi_stage_1 state machine.
- Added sm_atomic_performance_test_c_1 state machine.
- Added sm_atomic_performance_test_a_1 and sm_atomic_performance_test_a_2 state machines.
- Added sm_atomic_performance_test_a_2 state machine.
- Added sm_atomic_performance_test_c_1 state machine.
- Added sm_atomic_performance_test_a_2 state machine.
- Added sm_atomic_performance_trace_1 state machine.
- Added sm_atomic_24hr state machine.
- Added sm_advanced_recovery_1 state machine.
- Added sm_respira_test_2 state machine.
- Added sm_respira_1 format cleanup state machine.
- Added sm_respira_1 format cleanup pre-commit state machine.
- Added sm_respira_1 state machine.
- Added sm_reference_library reformatting.
- Added smacc2_performance_tools.
- Added smacc2_sm_reference_library/sm_atomic/README.md in markdown syntax.
- Added Dockerfile with ROS distro as argument.
- Added new sm markdowns.
- Added new sm from sm_respira_1.
- Added tracing events renaming.
- Added tracing.md editing to reflect new tracing event names.
- Added alternative ManualTracing.
- Added smacc2_rta command updates across readmes.
- Added galactic CI setup and renamed rolling files.
- Added Navigation2 for semi-binary build.
- Added build of missing rolling repositories.
- Added smacc2 nav clients reactivation for rolling via submodules.
- Added new folder for additional tracing contents.

Changed
-------

- Changed wording from "smacc application" to "SMACC2 library".
- Changed launch command to 'ros2 launch sm_respira_1 sm_respira_1.launch'.
- Changed description table.
- Changed table format.
- Changed launch command to 'ros2 launch sm_respira_1 sm_respira_1.launch'.
- Changed wording from "SMACC/ROS" to "SMACC2/ROS2".
- Changed wording "smacc application" to "SMACC2 library".

Fixed
-----

- Fixed bug in smacc2 component.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Fixed pre-commit issues.
- Fixed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Fixed formatting in sm_atomic_24hr.
- Fixed formatting in sm_advanced_recovery_1.
- Fixed formatting in sm_multi_stage_1.
- Fixed formatting in sm_respira_1.
- Fixed formatting in tracing.md.
- Fixed formatting in README.md.
- Fixed formatting in sm_atomic_performance_test_a_2.
- Fixed formatting in sm_atomic_performance_test_c_1.
- Fixed formatting in sm_advanced_recovery_1.
- Fixed formatting in sm_multi_stage_1.
- Fixed formatting in sm_respira_1.

Removed
-------

- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed galactic builds from master and kept only rolling, removed submodules and used .repos file.
- Removed tracing directory.
- Removed tracing.md file.
- Removed smacc2 and smacc2_msgs package ignore.
- Removed tracing events renaming.
- Removed tracing.md editing to reflect new tracing event names.
- Removed tracing events renaming.
```

```rst
Section_18
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` behavior waits for nav2 nodes to subscribe to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_aarehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_pause_slam`.
- `sm_dance_bot_lite` added.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- Lidar show/hide option in `sm_dance_bot`.
- Gazebo fixes for robot and lidar visualization.
- Doubling in `sm_multi_stage_1`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Changed
~~~~~~~
- Corrected all linters and formatters.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor formatting improvements.
- Cleanup in precommit stage.
- Hotfix for minor issues.

Fixed
~~~~
- Removed some compile warnings.

Removed
~~~~~~~
- Redundant formatting improvements.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
``` 

*pabloinigoblasco*

```
Section_19
==========

Added
-----
- Added multistage modes and sequences to sm_multi_stage_1 (#172).
- Added finishing touches and updated README for sm_multi_stage_1 (#172).
- Added AWS navigation for sm_dance_bot (#174), including repository dependencies and husky simulation launch files.

Changed
-------
- Renamed sm_advanced_recovery_1 (#171).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Updated QOS durability for SmaccPublisherClient (#163).
- Refactored reliability QOS configuration for SmaccPublisherClient (#163).
- Updated format and dependencies for moveit migration testing (#151).
- Improved Dockerfile for local test building (#151).
- Updated README files with SVGs for atomic, dance_bot, and others (#140, #145).
- Updated package list (#142).

Fixed
-----
- Fixed waypoint and iteration changes for robot course completion (#155).
- Fixed compilation warnings and errors.
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148).
- Fixed Python version formatting in CI (#148).
- Fixed broken master build and pipeline errors in moveit testing (#167).

Removed
-------
- Removed neo_simulation2 package (#112).

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

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
- Sm_dance_bot_warehouse_3.
- Feature/sm warehouse 2 13 dec 2.
- Finetuning waypoints.
- Feature/cb pure spinning.
- Feature/planner changes 16 12.
- Feature/replanning 16 dec.
- Several fixes.
- Feature/undo motion 20 12.
- Improving undo motion navigation warehouse2.
- Tuning warehouse3.
- Feature/sync 21 12.
- Feature/warehouse2 22 12.
- Finishing warehouse2.
- Feature/warehouse2 23 12.
- Feature/minor tune.
- Fixing warehouse 3 problems and other core improvements.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components.
- Autoware demo.
- Docker files for different revisions.
- Retry behavior warehouse 1.
- Update file for fake hardware simulation and add file for gazebo simulation.
- Multiple controllable leds plugin.
- Progress in husky demo.
- Add ignition file and update repos files.
- Improving navigation behaviors.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.

Changed
-------

- Only rolling version should be pre-released on master.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.
- Making models local.

Fixed
-----

- Fix formatting.
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
- Missing file.
- Minor broken build.
- Some reordering fixes.
- Minor format fix.
- Other minor changes.
```

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
- Use docs/ as source folder and output directory.
- Added setupTracing.sh:
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Added README tutorial for Dockerfile.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Add galactic CI setup and rename rolling files (#58).
- Fix source CI and correct README overview (#62).
- Update c_cpp_properties.json.
- Update doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Feature/aws demo progress (#80).
- More on navigation.
- Sm_advanced_recovery_1 reworked (#83).
- More sm_advanced_recovery_1 work (#85).
- Sm_advanced_recovery_1 round 4 (#86).
- Brettpac branch (#87).
- Sm_atomic_performance_test_c_1 (#88).

Changed
-------
- Rename header files and correct format.
- Update doxygen-check-build.yml.
- Change extension of imports.
- Update ci-build-source.yml.
- Rename to smacc2 and smacc2_msgs.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Update description table.
- Update table.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Format improvements.
- More on performance and other issues.
- More changes on performance tests.
- Sm_reference_library reformatting.
- Correct trailing spaces.
- Update changelogs.

Fixed
-----
- Correct formatters.
- Correct formatting of python file.
- Revert "Ignore all packages except smacc2 and smacc2_msgs" (#69).
- Fix pre-commit.
- Trying to fix Pre-Commit.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Disable further packages.
- Ignore all packages except smacc2 and smacc2_msgs.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh.

Co-Authored-By
--------------
- Denis Štogl <destogl@users.noreply.github.com>.
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
```

```rst
Section_22
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection.
- New feature: cb pause slam client behavior.
- New feature: sm_dance_bot_lite.

Changed
-------
- Updated launch command.
- Corrected all linters and formatters.

Fixed
-----
- Fixed precommit issues.
- Fixed navigation parameters on sm_dance_bot.
- Removed some compile warnings.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

Commits
-------
- (#89) Modifying sm_atomic_performance_test_a_2.
- (#90) Fixed precommit issues in sm_multi_stage_1.
- (#91) Added more features to sm_multi_stage_1.
- (#81) Worked on wait topic message client behavior.
- (#82) Implemented wait nav2 nodes client behavior.
- (#92) Progressed in aws navigation demo.
- (#93) Fixed navigation parameters on sm_dance_bot.
- (#94) Merged and progressed in aws navigation.
- (#95) Fixed navigation parameters on sm_dance_bot.
- (#96) Removed some compile warnings.
- (#98) Implemented cb pause slam client behavior.
- (#99) Worked on sm_dance_bot_lite.
- (#100) Renamed doxygen deployment workflow.
- (#101) Visualized turtlebot3 in sm_dance_bot.
- (#102) Added choice for gz lidar launch in dance bot.

pabloinigoblasco
```

Section_23
==========

Added
-----
- Added sm_dance_bot visualizing turtlebot3.
- Added lidar show/hide option for cleaning.
- Added formatting improvements.
- Added gazebo fixes to show the robot and lidar.
- Added gazebo fixes for sm_dance_bot_strikes_back.
- Added AWS demo.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added smacc2::deep_history syntax.
- Added slam pausing/resuming functionality for testing sm_dance_bot.
- Added waypoints navigator bug fix.
- Added SM core test.
- Added local action msgs usage.
- Added navigation 2 stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added rolling Docker environment to be executed from any environment.
- Added slight waypoint 4 and iterations changes for robot course completion.
- Added initial migration to smacc2.
- Added missing dependencies and fixed linting warnings for migration to smacc2.
- Added .reps dependencies and fixed build errors for migration to smacc2.
- Added repos dependency for migration to smacc2.
- Added dependency to ur5 client for migration to smacc2.
- Added docker refactoring for migration to smacc2.
- Added progress on move_it PR.
- Added improvements to dockerfile for building local tests.
- Added readme updates.

Changed
-------
- Changed "Finnaly" to "Finally" for correction.

Fixed
-----
- Fixed recursion issue by moving method after the method it calls.
- Fixed format issues.
- Fixed compile warnings.
- Fixed CI format for Python version.
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed some errors introduced on formatting during migration to smacc2.
- Fixed some linting warnings during migration to smacc2.
- Fixed compiling issues.

Removed
-------
- Removed neo_simulation2 package.
- Removed parameters from smacc.
- Removed node creation and created only a logger.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_24
==========

Added
-----
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Waypoint Inputs (#178)
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
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
- Moved reference library SMs to smacc2_performance_tools
- Add reliability qos config
- Repo dependency
- Husky launch file in sm_dance_bot
- Update dependencies for husky in rolling and galactic
- Finishing touches 1
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints
- Improving undo motion navigation warehouse2
- Tuning and fixes
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Improvements in smacc core adding more components mostly developed for autoware demo
- Refining cp subscriber cp publisher
- Progress in autoware machine
- Progress in autoware demo
- Progress in barrel husky
- Progress in barrel demo
- Progress in barrel search build fix and warehouse3
- Progress in warehouse2
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- More testing on moveit
- More testing on moveit behaviors
- Minor configuration
- Minor changes
- Minor format
- Minor linking errors foxy
- Minor tune
- Minor broken build
- Minor reordering fixes
- Minor broken build
- Minor
- Docker files for different revisions, warnings removal and more testing on navigation
- Fixing docker for foxy and galactic
- Docker build files for all versions
- Fixing startup problems in warehouse 3
- Fixing format and minor
- Fixing broken master build
- Fixing pipeline error
- Fixing broken build
- Fixing broken build

Removed
-------
- Sm_pubsub_1 part 2
- Sm_advanced_recovery_1 renaming
- Sm_multi_stage_1 reworking
- Sm_multi_stage sequences
- Sm_multi_state_1 steps
- Sm_multi_stage_1 sequence d
- Sm_multi_stage_1 c sequence
- Mode_5_sequence_b
- Mode_4_sequence_b
- Sm_multi_stage_1 most
- Missing sm
- Updating subscriber publisher components
- Weird moveit not downloaded repo
- Added missing file from warehouse2
- Backport to foxy
- Pure spinning behavior missing files
- Undo tuning and errors
- Format issues
- Finishing warehouse2
- Tuning and fixes
```

```rst
Section_25
==========

Version 0.1.0 (Date: TBD)
-------------------------

### Added
- Build-status table
- Detailed install instructions ([source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme))
- `setupTracing.sh`: Installs necessary packages and configures tracing group

### Changed
- Default build type set to `Release` for faster performance and smaller executables
- Updated examples section

### Fixed
- Resolved missing dependency in `smacc_msgs` and reorganized for better overview
- Fixed bug in `smacc2` component
- Cleaned up formatting in `sm_respira_1` and `sm_atomic_24hr`
- Optimized dependencies in `move_base_z_planners_common`
- Renamed event generator library
- Corrected build-overview table
- Updated and unified CI configurations
- Used `tf_geometry_msgs.h` in Galactic
- Switched to Galactic branches in `.repos-file`

### Removed
- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`

### Miscellaneous
- More merge
- Docker improvements
- Master rolling to Galactic backport
- Fixing build
- Testing dance bot demos
- Updating Galactic repos
- Runtime dependency
- Restoring UR dependency
- Reverted markdowns to HTML
- Added README tutorial for Dockerfile
- Edited `tracing.md` to reflect new tracing event names
- Performance tests improvements
- More on performance and other issues
- Do not execute `clang-format` on `smacc2_sm_reference_library`
- Cleaned up `sm_atomic_24hr`
- More cleanup on `sm_atomic_24hr`
- Use of `galactic` branches in `.repos-file`

Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco
```
