Changelog for package sm_cl_keyboard_unit_test_1
==================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fixes for ros buildfarm issue.

### Contributors
- brettpac
- pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
No significant changes.

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release.

### Contributors
- pabloinigoblasco

Version 0.3.0 (2022-04-04)
--------------------------
No significant changes.

Version 0.0.0 (2022-11-09)
---------------------------
### Added
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Ignored packages not to be released.
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2.
  - Progress on navigation rolling.
  - Renamed folders, deleted tracing.md, edited README.md.
  - Added smacc2_performance_tools.
  - Performance tests improvements.
  - Format cleanup for sm_respira_1.
  - Optimized dependencies in move_base_z_planners_common.
  - Renamed event generator library.
  - Added galactic CI setup and renamed rolling files.
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
  - Created new sm from sm_respira_1.
  - Several core improvements during navigation testing.
  - New feature, cb_wait_topic_message: asynchronous client behavior.
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic.
  - Corrected all linters and formatters.

### Contributors
- pabloinigoblasco
- brettpac
- Denis Štogl

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for `nav2`: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive; optional node selection available
- Progress in AWS navigation demo
- Base for the `sm_aws_warehouse` navigation
- Navigation parameters fixes on `sm_dance_bot`
- New client behavior: `cb_pause_slam`
- `sm_dance_bot_lite` visualizing TurtleBot3
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo improvements
- `sm_multi_stage_1` enhancements
- Diverse improvements in navigation and performance

Changed
-------

- Minor formatting improvements
- Merge and progress in development
- Hotfix for minor issues
- Cleaning and lidar show/hide option in `sm_dance_bot`
- Gazebo fixes to show the robot and lidar

Removed
-------

- Removed some compile warnings
- Removed `neo_simulation2` package

Fixed
-----

- Corrected formatting issues
- Adjusted build packages for source CI
- Fixed various issues in navigation testing

Collaborators
-------------

- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

## Section_3

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

### Removed
- Remove merge markers from a python file (#119)
- Remove node creation and create only a logger (#149)
- Removing parameters smacc (#147)
- Removing sm_dance_bot_msgs

### Miscellaneous
- Minor linting and formatting improvements
- Minor format tweaks
- Minor navigation improvements
- Minor tuning to mitigate overshot issue cases
- Update package list (#142)
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back and removed some comments
- Update READMEs with added SVGs for atomic, dance_bot, and others
- Warehouse2 progress (#179)
- Format improvements
- Headless and other fixes

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_4
=========

Added
-----
- Default values for various features.
- BrettPac branch (#184).
- Redoing waypoints for sm_dance_bot_warehouse_3.
- More waypoints added.
- SrConditional fixes and formatting (#168).
- Warehouse2 feature added on Dec 14 (#185).
- Feature for sm warehouse 2 on Dec 13 (#186).
- Finetuning waypoints (#187).
- Pure spinning behavior with missing files.
- Minor changes for planner on Dec 16 (#191).
- Undo motion feature on Dec 20 (#196, #198).
- Sync feature on Dec 21 (#199).
- Warehouse2 features on Dec 22 (#200) and Dec 23 (#201).
- Minor tune feature (#203).
- Fixes and improvements for warehouse 3 (#204).
- Backport to Foxy (#206).

Changed
-------
- Various formatting fixes.
- Trailing spaces fixed.
- Codespell corrections.
- Python linters warnings corrected.
- CI build added for Galactic due to Navigation2 issues in Rolling.
- Partial changes for ament_cpplint added.
- Dependency tf2_ros included for finding includes.
- Several linters disabled/enabled and updated.
- ccache version bumped.
- Formatters corrected.
- Branching example added.
- Extension changes for imports.
- cppcheck enabled.
- Python file formatting corrected.
- Necessary package included and Threesome launch edited.

Removed
-------
- Manual installation of ros-rolling-ros2trace.
- Tracing directory deleted.
- ManualTracing alternative removed.

Fixed
-----
- Deadlock issues in warehouse 3 resolved.
- Moveit repository download issue fixed.
- Missing file added for warehouse2.

Authors
-------
- Pablo Iñigo Blasco (@pabloinigoblasco)
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section 5
=========

Added
-----

- Added smacc2_performance_tools for performance testing.
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------

- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated c_cpp_properties.json.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Corrected all linters and formatters.

Fixed
-----

- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview. (#62).
- Fixed pre-commit issues in various packages.

Removed
-------

- Removed redundant note in the documentation.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>
```

Section_6
==========

Added
-----

- Feature/sm aws warehouse (#94)
  - Base for the sm_aws_warehouse navigation
  - Progress in AWS navigation
  - Several core improvements during navigation testing
  - Formatting improvements
  - Progress in AWS navigation demo
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Adding new client behavior for Nav2, waiting for Nav2 nodes subscribing to the /bond topic and ensuring they are alive
  - Navigation parameters fixes on sm_dance_bot

- Feature/sm dance bot fixes (#95)
  - Navigation parameters fixes on sm_dance_bot

- Feature/cb pause slam (#98)
  - Navigation parameters fixes on sm_dance_bot
  - CB pause slam client behavior

- sm_dance_bot_lite (#99)
  - Precommit
  - Updates yaml

- Rename doxygen deployment workflow (#100)

- sm_dance_bot visualizing turtlebot3 (#101)
  - Cleaning and lidar show/hide option
  - Cleaning files and fixing formatting

- Feature/dance bot launch gz lidar choice (#102)
  - Cleaning and lidar show/hide option
  - Cleaning files and fixing formatting

- Feature/sm dance bot lite gazebo fixes (#104)
  - Cleaning and lidar show/hide option
  - Cleaning files and fixing formatting
  - Gazebo fixes to show the robot and the lidar

- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Cleaning and lidar show/hide option
  - Cleaning files and fixing formatting
  - Gazebo fixes to show the robot and the lidar
  - Gazebo fixes for sm_dance_bot_strikes_back

- Precommit cleanup run (#106)

- AWS demo (#108)

- Brettpac branch (#110)
  - Got sm_multi_stage_1 working
  - Gaining traction sm_multi_stage_1

- Brettpac branch (#111)
  - Got sm_multi_stage_1 working
  - Gaining traction sm_multi_stage_1
  - More stages

- a3 (#113)

- Remove neo_simulation2 package. (#112)
  - Correct formatting
  - Enable source build on PR for testing
  - Adjust build packages of source CI

- mm (#115)

- Diverse improvements navigation and performance (#116)
  - Additional linting and formatting

- Feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting

- Remove merge markers from a Python file. (#119)

- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing slam pausing/resuming functionality in sm_dance_bot
  - Feature/more_sm_dance_bot_fixes

- Move method after the method it calls. Otherwise, recursion could happen. (#126)

- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern
  - Noticed typo
  - Finally > Finally

- Feature/dance bot s pattern (#129)
  - Polishing sm_dance_bot and s-pattern
  - More refinement in sm_dance_bot

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
- Feature/sm dance bot strikes back refactoring (#152)
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
- fixing some errors introduced on formatting
- fixing some more linting warnings
- fixing compiling issues
- update readme (#164)
- moved reference library SMs to smacc2_performance_tools (#166)
- Add reliability qos config
- fixing pipeline error
- fixing broken master build
- fixing broken build

Fixed
-----
- minor tuning to mitigate overshot issue cases
- fixing some build errors
- fixing some formatting and templating on SrConditional
- move trigger logic into headers
- lint
- pure spinning behavior missing files

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

```rst
Section_8
=========

Version 0.1.0 (2022-01-01)
---------------------------

Added
-----

- Feature/replanning 16 dec (#193): Replanning for all examples.
- Feature/undo motion 20 12 (#196): Improving undo motion navigation in warehouse2.
- Feature/warehouse2 22 12 (#200): Finishing warehouse2.
- Feature/warehouse2 23 12 (#201): Tuning and fixes (#202).
- Feature/minor tune (#203): Fixing warehouse 3 problems and other core improvements (#204).
- Merging code from backport foxy and updates about autoware (#208): Backport to foxy.

Changed
-------

- Foxy backport (#206): Minor formatting fixes, trailing spaces, codespell corrections, python linters warnings, and more.
- Update cb_navigate_global_position.hpp: Improvements in smacc core, adding more components mostly developed for autoware demo.
- Update ci-build-source.yml: Change extension of imports, enable cppcheck, correct formatting of python file.
- Update tracing/ManualTracing.md: Changed wording "smacc application" to "SMACC2 library".

Fixed
-----

- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Some progress on navigation rolling.
- Added smacc2_performance_tools: Performance tests improvements.

Removed
-------

- Deleted tracing directory.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Co-authored-by: Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>, Declan Dury <44791484+DecDury@users.noreply.github.com>, DecDury <declandury@gmail.com>, reelrbtx <brett2@reelrobotics.com>, brettpac <brett@robosoft.ai>, David Revay <MrBlenny@users.noreply.github.com>
```

```rst
Section_9
=========

Added
-----
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success. Also, added a new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for. (#82, #92)
- Added navigation parameters fixes on sm_dance_bot. (#93)

Changed
-------
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Updated doxygen links (#70)
- Updated c_cpp_properties.json
- Updated README.md with launch command

Fixed
-----
- Fixed source CI and corrected README overview. (#62)
- Corrected trailing spaces.
- Corrected all linters and formatters.

Removed
-------
- Removed execution of clang-format on smacc2_sm_reference_library package.

Other
-----
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Minor formatting improvements.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_10
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive; optional node selection
- Base for the `sm_aws_warehouse` navigation
- Gazebo fixes to show the robot and lidar
- `sm_multi_stage_1` doubling
- `sm_dance_bot_strikes_back` gazebo fixes
- AWS demo progress
- Source build enabled on PR for testing
- `slam_toggle` client behaviors and `slam_toolbox` components; `smacc2::deep_history` syntax
- First working version of `sm` template and template generator

Changed
-------
- Progress in AWS navigation demo
- Navigation parameters fixes on `sm_dance_bot`
- Minor format improvements
- Cleaning and lidar show/hide option
- Several core improvements during navigation testing
- Formatting enhancements
- `sm_dance_bot` visualizing Turtlebot3
- Polishing `sm_dance_bot` and S-pattern
- More refinement in `sm_dance_bot`
- Minor tweaks

Fixed
-----
- Remove some compile warnings
- Remove `neo_simulation2` package
- Correct formatting
- Adjust build packages of source CI
- Move method after the method it calls to prevent recursion
- Typo correction ("Finnaly" to "Finally")

Removed
-------
- `neo_simulation2` package

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pablo@ibrobotics.com>
```

Section 11
===========

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
- Format fix python version (#148)
- Fixing some errors introduced on formatting
- Progressing in the moveit migration testing
- Improving dockerfile for building local tests
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----
- Build fix
- Resolve compile warnings (#137)
- Fix CI: format fix python version (#148)
- Fixing broken master build
- Fixing pipeline error
- Fixing broken build
- Warehouse2 progress (#179)
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

Section_12
-----------

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
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Create doxygen-deploy.yml
- Added setupTracing.sh
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Improving undo motion navigation warehouse2
- Undo tuning and errors
- Tuning and fixes (#202)
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Updating subscriber publisher components
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Odom tracker improvements
- Adding forward behavior retry functionality
- Minor formatting fixes
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Rename header files and correct format.
- Correct formatters.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Reset all versions to 0.0.0
- Update description table.
- Update table
- Copy initial docs
- Changed "ros2 launch sm_three_some sm_three_some" to "ros2 launch sm_three_some sm_three_some.launch"
- Changed wording "smacc application" to "SMACC2 library"
- Edit from html to markdown syntax

Fixed
-----
- Minor broken build
- Fix rolling builds (#222)
- Remove example things from Foxy CI setup. (#214)
- Fixing docker for foxy and galactic
- Removing warnings (#213)
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages
- Enable cppcheck
- Docker files for different revisions, warnings removal and more testing on navigation
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licenses.

Removed
-------
- Weird moveit not downloaded repo
- Delete tracing directory
- Moved tracing.md to tracing directory
- Removed manual installation of ros-rolling-ros2trace
- This is now automated in setupTracing.sh
- Location of sh file assumed if user follows README.md under "Getting started"
- Created alternative ManualTracing

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
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed smacc2_sm_reference_library package to sm_reference_library.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.

Fixed
-----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview.
- Fixed pre-commit issues.
- Fixed formatting issues.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.

Collaborators
-------------
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_14
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Nodes to wait for can be optionally selected.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Cleaning and lidar show/hide option for `sm_dance_bot`.
- Gazebo fixes to show the robot and the lidar.
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components. Also `smacc2::deep_history` syntax.

Fixed
----
- Minor format improvements.
- Remove some compile warnings.
- Correct formatting.
- Additional linting and formatting.
- Remove merge markers from a Python file.

Removed
-------
- Removed `neo_simulation2` package.

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
- Waypoint Inputs (#178)
- More Waypoints to sm_dance_bot_warehouse_3 (#181)
- Redoing sm_dance_bot_warehouse_3 waypoints (#184)

Changed
-------
- Move method after the method it calls to avoid recursion (#126)
- Renamed reference library SMs to smacc2_performance_tools (#166)
- Minor navigation improvements (#141)
- Updated package list (#142)
- Update readme (#164)
- Fixed launch command in README.md (#147)
- Fix CI: format fix python version (#148)
- Progress on move_it PR (#164)
- Improved Dockerfile for building local tests
- Added .reps dependencies and fixed build errors
- Updated format in moveit migration testing
- Added reliability qos config to SmaccPublisherClient
- Minor configuration changes in testing moveit behaviors

Fixed
-----
- Resolved compile warnings (#137)
- Fixed compiling issues in various parts of the project
- Fixed pipeline error and broken master build
- Fixed broken build in aws navigation
- Fixed formatting issues in various parts of the project
- Fixed some linting warnings
- Fixed some errors introduced on formatting in moveit migration
- Fixed some more linting warnings in moveit migration
- Fixed some formatting and templating on SrConditional
- Moved trigger logic into headers in SrConditional
- Fixed some linting issues in SrConditional

Removed
-------
- Removed node creation and created only a logger in SM Atomic SM generator (#149)
- Removed parameters smacc (#147)
- Removed test from main moveit cmake
- Removed sm_dance_bot_msgs
- Removed some comments in the past from launch command for sm_dance_bot_strikes_back

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> in multiple commits
- Co-authored-by: DecDury <declandury@gmail.com> in Feature/sm dance bot strikes back refactoring (#152)
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com> in Feature/sm dance bot strikes back refactoring (#152) and Feature/aws navigation sm dance bot (#174)
- Co-authored-by: Denis Štogl <denis@stogl.de> in Feature/aws navigation sm dance bot (#174)

```rst
Section_16
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
- dockerfiles (#225)
- Fix code generators (#221)
- Foxy backport (#206)

Changed
-------
- Finetuning waypoints (#187)
- Improving undo motion navigation warehouse2
- Tuning warehouse3 (#197)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Fix other build issues.
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
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch

Fixed
-----
- Several fixes (#194)
- Fix trailing spaces.
- Minor formatting fixes

Removed
-------
- Delete tracing directory

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
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
```

```rst
Section_17
==========

Added
-----
- Moved tracing.md to tracing directory
- Added setupTracing.sh script to automate ros-rolling-ros2trace installation and configuration
- Created alternative ManualTracing
- Added new sm markdowns
- Added a Dockerfile for Rolling and Galactic
- Added README tutorial for Dockerfile
- Added smacc2_performance_tools
- Added new feature: cb_wait_topic_message for asynchronous client behavior

Changed
-------
- Renamed "smacc application" to "SMACC2 library"
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Renamed tracing events
- Renamed folders, deleted tracing.md, edited README.md
- Renamed event generator library

Fixed
-----
- Bug in smacc2 component
- Reverted markdowns to HTML
- Optimized dependencies in move_base_z_planners_common
- Corrected trailing spaces
- Fixed source CI and corrected README overview
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
- Updated doxygen links
- Corrected all linters and formatters

Removed
-------
- Manual installation of ros-rolling-ros2trace

Other
-----
- Reactivated smacc2 nav clients for Rolling via submodules
- Enabled build of missing Rolling repositories
- Enabled Navigation2 for semi-binary build
- Removed Galactic builds from master and kept only Rolling, removed submodules and used .repos file
- Some progress on navigation Rolling
- More changes on performance tests
- Reformatted sm_reference_library
- Cleaned up sm_cl_keyboard_unit_test_1_24hr
- Cleaned up sm_advanced_recovery_1
- More work on sm_advanced_recovery_1
- Modified sm_cl_keyboard_unit_test_1_performance_test_a_2
- Created sm_cl_keyboard_unit_test_1_performance_test_c_1
- Modified sm_multi_stage_1
- More work on sm_multi_stage_1
- More work on sm_cl_keyboard_unit_test_1_performance_test_a_2
- Created sm_cl_keyboard_unit_test_1_performance_test_a_1
- Created sm_cl_keyboard_unit_test_1_performance_test_c_1
- Created sm_multi_stage_1
- More work on sm_multi_stage_1
- More work on navigation
- Attempted precommit fixes
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait
``` 

*pabloinigoblasco*

```rst
Section_18
==========

Added
-----
- Feature/aws demo progress (#92)
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait
- Navigation parameters fixes on sm_dance_bot
- cb pause slam client behavior
- sm_dance_bot_lite (#99)
- Rename doxygen deployment workflow (#100)
- sm_dance_bot visualizing turtlebot3 (#101)
- Feature/dance bot launch gz lidar choice (#102)
- gazebo fixes, to show the robot and the lidar
- sm_multi_stage_1 doubling (#103)
- gazebo fixes for sm_dance_bot_strikes_back
- aws demo (#108)
- Brettpac branch (#110)
- a3 (#113)
- Remove neo_simulation2 package. (#112)

Changed
-------
- Minor hotfix
- Cleaning and lidar show/hide option
- Cleaning files and making formatting work
- Format fixes
- Got sm_multi_stage_1 working (barely)
- Gaining traction sm_multi_stage_1
- Making progress
- Keep hammering
- Two stages
- 3 part
- 4th stage
- 5th stage
- Correct formatting

Fixed
-----
- Remove some compile warnings. (#96)
- Precommit cleanup run (#106)
``` 

*pabloinigoblasco*

Section 19
-----------

### Added
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Added diverse improvements in navigation and performance.
- Added linting and formatting.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Introduced slam pausing/resuming functionality in sm_dance_bot testing.
- Added first working version of sm template and template generator.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM core test.
- Added SM Atomic SM generator.
- Rolling Docker environment to be executed from any environment.
- Added initial state machine transition timestamp.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added repo dependency for AWS navigation sm dance bot.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Updated dependencies for husky in rolling and galactic.

### Changed
- Moved method after the method it calls to prevent recursion.
- Renamed sm_advanced_recovery_1.
- Reworked sm_multi_stage_1 with multistage modes, sequences, steps, and finishing touches.
- Moved reference library SMs to smacc2_performance_tools.

### Fixed
- Removed merge markers from a python file.
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed CI format for python version.
- Removed node creation and create only a logger.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed formatting issues.
- Fixed warehouse2.

### Removed
- Removed parameters smacc.
- Removed sm_dance_bot_msgs.

### Miscellaneous
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>.
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.

### Commits
- (#114): more sm_multi_stage_1.
- (#115): mm.
- (#116): diverse improvements in navigation and performance.
- (#119): Remove merge markers from a python file.
- (#122): Feature/slam toggle and smacc deep history.
- (#124): minor.
- (#125): more changes in sm_dance_bot.
- (#126): Move method after the method it calls.
- (#127): First working version of sm template and template generator.
- (#130): minor tweaks.
- (#131): Feature/sm dance bot refine.
- (#132): Feature/sm dance bot refine 2.
- (#133): waypoints navigator bug.
- (#134): minor format issues.
- (#135): progress in the sm_dance_bot tests.
- (#136): sm_dance_bot_lite.
- (#137): Resolve compile warnings.
- (#138): Add SM core test.
- (#139): using local action msgs.
- (#140): added SVGs to READMEs of atomic, dance_bot, and others.
- (#141): minor navigation improvements.
- (#142): Update package list.
- (#143): Add SM Atomic SM generator.
- (#144): Feature/nav2z renaming.
- (#145): added remaining SVGs to READMEs.
- (#147): removing parameters smacc.
- (#148): Fix CI: format fix python version.
- (#149): Remove node creation and create only a logger.
- (#151): Feature/migration moveit client.
- (#152): Feature/sm dance bot strikes back refactoring.
- (#163): Add QOS durability to SmaccPublisherClient.
- (#164): update readme.
- (#165): initial state machine transition timestamp.
- (#166): moved reference library SMs to smacc2_performance_tools.
- (#167): Feature/testing moveit behaviors.
- (#169): sm_pubsub_1.
- (#170): sm_pubsub_1 part 2.
- (#171): sm_advanced_recovery_1 renaming.
- (#172): sm_multi_stage_1 reworking.
- (#174): Feature/aws navigation sm dance bot.
- (#175): minor changes.
- (#177): warehouse2.

```rst
Section_20
==========

Added
-----
- Waypoint Inputs (#178) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- wharehouse2 progress (#179)
- sm_dance_bot_warehouse_3 (#181) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Feature/sm warehouse 2 13 dec 2 (#182)
- Brettpac branch (#184)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints (#186)
- finetuning waypoints (#187) by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Feature/cb pure spinning (#188)
- pure spinning behavior missing files
- minor changes (#190)
- Feature/planner changes 16 12 (#191)
- Feature/replanning 16 dec (#193)
- several fixes (#194)
- minor changes (#195)
- Feature/undo motion 20 12 (#196)
- improving undo motion navigation warehouse2
- tuning warehouse3 (#197)
- undo tuning and errors
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- finishing warehouse2
- Feature/warehouse2 23 12 (#201)
- tuning and fixes (#202)
- Feature/minor tune (#203)
- fixing warehouse 3 problems, and other core improvements (#204) by Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>, Declan Dury <44791484+DecDury@users.noreply.github.com>, DecDury <declandury@gmail.com>, reelrbtx <brett2@reelrobotics.com>, brettpac <brett@robosoft.ai>, David Revay <MrBlenny@users.noreply.github.com>, pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- added missing file from warehouse2 (#205)
- fixing docker for foxy and galactic
- Update file for fake hardware simulation and add file for gazebo simulation.
- retry behavior warehouse 1
- fixing format and minor
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- foxy ci
- fixing docker for foxy and galactic
- Update file for fake hardware simulation and add file for gazebo simulation.
- progressing in husky demo
- improving navigation behaviors
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
- branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck

Changed
-------
- fix: some formatting and templating on SrConditional
- fix: move trigger logic into headers
- fix: lint
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor
- minor

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
- Updated name of package and package.xml to pass liter.
- Renamed to smacc2 and smacc2_msgs.
- Corrected GitHub branch reference.
- Updated description table.
- Updated table.
- Renamed tracing events.
- Changed wording "smacc application" to "SMACC2 library".
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed folders and edited README.md.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.

Fixed
-----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library.
- Corrected trailing spaces.

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Galatic builds from master, keeping only rolling.
- Submodules, using .repos file instead.

Other Changes
-------------
- Added workflow for checking doc build.
- Created workflow for testing prerelease builds.
- Created workflow for Galatic CI setup and renamed rolling files.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Cleanup and formatting improvements.
- Progress in AWS navigation demo.
- Several core improvements during navigation testing.
- More on navigation.
- Reworked sm_advanced_recovery_1.
- Progress in AWS navigation.
- Base for the sm_aws_aarehouse navigation.
- Base for the sm_multi_stage_1.
- Fixing precommit issues.

Commits
-------
- f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61: Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- #58: Added galactic CI setup and renamed rolling files.
- #62: Fixed source CI and corrected README overview.
- #69: Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- #70: Updated doxygen links.
- #72: More README updates.
- #74: More README updates.
- #76: Created new sm from sm_respira_1.
- #78: Feature/core and navigation fixes.
- #80: Feature/aws demo progress.
- #83: Reworked sm_advanced_recovery_1.
- #84: More sm_advanced_recovery_1 work.
- #85: More sm_advanced_recovery_1 work.
- #86: Reworked sm_advanced_recovery_1.
- #87: Brettpac branch.
- #88: Added sm_cl_keyboard_unit_test_1_performance_test_c_1.
- #89: Modified sm_cl_keyboard_unit_test_1_performance_test_a_2.
- #90: Added sm_multi_stage_1.
- #91: More sm_multi_stage_1.

Co-Authored-By
--------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

```rst
Section_22
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81)
- New feature: cb_wait_nav2_nodes, a client behavior for nav2 that subscribes to the /bond topic and waits for nodes to be alive, with optional node selection (#82)
- New feature: cb_pause_slam, a client behavior for pausing SLAM (#98)
- New feature: sm_dance_bot_lite gazebo fixes, including showing the robot and lidar (#104)

Changed
-------
- Corrected all linters and formatters for consistency (#82)

Fixed
----
- Fixed navigation parameters on sm_dance_bot (#93)
- Removed some compile warnings (#96)

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
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
- Move method after the method it calls to prevent recursion (#126)
- First working version of sm template and template generator (#127)
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases
- Progress in the sm_dance_bot tests (#135)
  - Some more progress on markers cleanup
- Minor navigation improvements (#141)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Rolling Docker environment to be executed from any environment (#154)
- Slight waypoint 4 and iterations changes so robot can complete course (#155)
- Initial migration moveit client (#151)
  - Fixing errors introduced on formatting
  - Missing dependency
  - Fixing linting warnings
  - Removing test from main moveit cmake
  - Progressing in the moveit migration testing
  - Adding .reps dependencies and fixing build errors
  - Adding dependency to ur5 client
  - Docker refactoring
- Update readme (#164)
  - More readme updates

Changed
-------
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Feature/sm dance bot strikes back refactoring (#152)
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Pre-commit cleanup
- Add QOS durability to SmaccPublisherClient (#163)
  - Add reliability QoS config

Removed
-------
- Remove neo_simulation2 package (#112)
  - Pending references
- Removing parameters smacc (#147)
  - Workflows update
- Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments

Fixed
-----
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Feature/testing moveit behaviors (#167)
```

Section_24
==========

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
- Added `readme` with contributions from Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Added `Feature/aws navigation sm dance bot` (#174).
- Added `repo dependency`.
- Added `husky launch file in sm_dance_bot`.
- Added dependencies for husky simulation.
- Added `Fix formatting`.
- Added `Update dependencies for husky in rolling and galactic`.
- Added `progress on aws navigation and some other refactorings on navigation clients and behaviors`.
- Added `more on aws demo`.
- Added `Feature/wharehouse2 dec 14` (#185).
- Added `warehouse2`.
- Added `minor`.
- Added `Feature/sm warehouse 2 13 dec 2` (#186).
- Added `more changes and headless`.
- Added `merge`.
- Added `headless and other fixes`.
- Added `default values`.
- Added `finetuning waypoints` (#187) with contributions from Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Added `Feature/cb pure spinning` (#188).
- Added `Feature/cb pure spinning` (#189).
- Added `pure spinning behavior missing files`.
- Added `Feature/planner changes 16 12` (#191).
- Added `more fixes`.
- Added `Feature/replanning 16 dec` (#193).
- Added `replanning for all our examples`.
- Added `several fixes` (#194).
- Added `Feature/undo motion 20 12` (#196).
- Added `improving undo motion navigation warehouse2`.
- Added `tuning warehouse3` (#197).
- Added `Feature/undo motion 20 12` (#198).
- Added `undo tuning and errors`.
- Added `format`.
- Added `Feature/sync 21 12` (#199).
- Added `format issues`.
- Added `Feature/warehouse2 22 12` (#200).
- Added `format issues`.
- Added `finishing warehouse2`.
- Added `Feature/warehouse2 23 12` (#201).
- Added `tuning and fixes` (#202).
- Added `Feature/minor tune` (#203).
- Added `tuning and fixes`.
- Added `fixing warehouse 3 problems, and other core improvements` (#204).
- Added `weird moveit not downloaded repo`.
- Added `added missing file from warehouse2` (#205).
- Added `backport to foxy`.
- Added `minor format`.
- Added `minor linking errors foxy`.
- Added `missing`.
- Added `missing sm`.
- Added `updating subscriber publisher components`.
- Added `progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine`.
- Added `refining cp subscriber cp publisher`.
- Added `improvements in smacc core adding more components mostly developed for autoware demo`.
- Added `autoware demo`.
- Added `foxy ci`.
- Added `fix`.
- Added `some reordering fixes`.
- Added `docker files for different revisions, warnings removval and more testing on navigation`.
- Added `fixing docker for foxy and galactic`.
- Added `docker build files for all versions`.
- Added `barrel demo`.
- Added `barrel search build fix and warehouse3`.
- Added `fixing startup problems in warehouse 3`.
- Added `fixing format and minor`.
- Added `progress in barrel husky`.
- Added `barrel demo`.
- Added `progress`.
- Added `testing dance bot demos`.
- Added `updating galactic repos`.
- Added `runtime dependency`.
- Added `restoring ur dependency`.

Changed
-------
- Changed `minor configuration`.
- Changed `progress on moveit`.
- Changed `more testing on moveit`.
- Changed `more testing on moveit behaviors`.
- Changed `finishing touches 1`.
- Changed `more merge`.
- Changed `docker improvements`.
- Changed `master rolling to galactic backport`.
- Changed `fixing build`.

Fixed
-----
- Fixed `pipeline error`.
- Fixed `fixing broken master build`.
- Fixed `fixing broken build`.
- Fixed `fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green`.

Removed
-------
- Removed `sm_pubsub_1 part 2`.
- Removed `sm_pubsub_1 part 2`.
- Removed `sm_multi_stage_1 most`.
- Removed `default values`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `minor`.
- Removed `

```rst
Section_25
==========

Version 0.1.0 (Date: TBD)
-------------------------

Added
-----
- Build-status table
- Detailed install instructions ([source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme))
- `setupTracing.sh` script for installing necessary packages and configuring tracing group

Changed
-------
- Default build type set to `Release` for faster build and smaller executables
- Updated examples section

Fixed
-----
- Resolved missing dependency in `smacc_msgs` and reorganized for better overview
- Fixed bug in `smacc2` component
- Cleaned up formatting in `sm_respira_1` and `sm_cl_keyboard_unit_test_1_24hr`
- Optimized dependencies in `move_base_z_planners_common`
- Renamed event generator library
- Corrected build-overview table
- Updated and unified CI configurations
- Used `tf_geometry_msgs.h` in Galactic
- Used Galactic branches in `.repos-file`

Removed
-------
- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`

Other Changes
-------------
- Reorganized project structure
- Removed test phase from CMake and dependencies from package.xml
- Compiled with navigation and slam_toolbox
- Enabled all packages to compile
- Refactored `getLogger` functionality
- Reverted changes in commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Ignored all packages except `smacc2` and `smacc2_msgs`
- Reactivated `smacc2` nav clients for Rolling via submodules
- Edited tracing.md to reflect new tracing event names
- Performance tests improvements and other related changes
- Do not execute clang-format on `smacc2_sm_reference_library`
- Cleaned up `sm_cl_keyboard_unit_test_1_24hr`
- More cleanup on `sm_cl_keyboard_unit_test_1_24hr`

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco
```
