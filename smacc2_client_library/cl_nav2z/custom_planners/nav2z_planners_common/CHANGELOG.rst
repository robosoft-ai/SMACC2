Changelog for package nav2z_planners_common
===========================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from robosoft-ai/SMACC2
- Brettpac branch:
  - Attempted fix for ros buildfarm issue
  - Further work on buildfarm issue
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
- Pre-release
- Migration progress to humble
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Ignored packages not to be released
- Feature/master rolling to galactic backport (#236):
  - Updated SMACC/ROS mentions to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup for sm_respira_1
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Added galactic CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Updated doxygen links
  - New feature, cb_wait_topic_message: asynchronous client behavior
### Contributors
- Denis Štogl, brettpac, pabloinigoblasco

Section_2
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive. Optionally select the nodes to wait.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Got `sm_multi_stage_1` working (barely).
- Brettpac branch: gaining traction with `sm_multi_stage_1`.

Fixed
-----
- Remove some compile warnings.
- Minor hotfix.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.

Removed
-------
- Removed `neo_simulation2` package.

Collaborators
-------------
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.

Section_3
==========

Version 0.1.0 (2022-01-01)
---------------------------

### Added
- Feature/diverse improvements navigation performance (#117)
- additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
- progress in navigation, slam toggle client behaviors, and slam_toolbox components
- going forward in testing sm_dance_bot introducing slam pausing/resuming functionality
- feature/more_sm_dance_bot_fixes
- more changes in sm_dance_bot (#125)
- Move method after the method it calls to prevent recursion (#126)
- Feature/dance bot s pattern (#128)
- polishing sm_dance_bot and s-pattern
- First working version of sm template and template generator (#127)
- minor tweaks (#130)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- build fix
- waypoints navigator bug (#133)
- minor tuning to mitigate overshot issue cases
- progress in the sm_dance_bot tests (#135)
- some more progress on markers cleanup
- sm_dance_bot_lite (#136)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- minor navigation improvements (#141)
- using local action messages
- Feature/nav2z renaming (#144)
- navigation 2 stack renaming
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
- Remove node creation and create only a logger (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- slight waypoint 4 and iterations changes so robot can complete the course (#155)
- Feature/migration moveit client (#151)
- initial migration to smacc2
- fixing some errors introduced in formatting
- missing dependency
- fixing some more linting warnings
- removing test from the main moveit cmake
- test ur5
- progressing in the moveit migration testing
- updating format
- adding .reps dependencies and fixing some build errors
- repos dependency
- adding dependency to ur5 client
- docker refactoring
- progress on move_it PR
- minor dockerfile test workaround
- improving dockerfile for building local tests
- fixing compiling issues
- update readme (#164)
- more readme updates

### Changed
- Moved reference library SMs to smacc2_performance_tools (#166)
- Add QOS durability to SmaccPublisherClient (#163)

### Fixed
- Fix CI: format fix python version (#148)

### Removed
- removing parameters smacc (#147)

Version 0.2.0 (2022-02-01)
---------------------------

### Added
- initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- more testing on moveit
- progress on moveit
- more testing on moveit behaviors
- fixing pipeline error
- fixing broken master build
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)
- husky launch file in sm_dance_bot
- Add dependencies for husky simulation
- Update dependencies for husky in rolling and galactic

### Changed
- slight waypoint 4 and iterations changes so robot can complete the course (#155)
- minor changes (#175)

### Removed
- warehouse2 (#177)
- Waypoint Inputs (#178)

```rst
Section_4
=========

Added
-----
- Implemented progress in warehouse2 (#179).
- Added feature for sm_dance_bot_warehouse_3 (#181).
- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Introduced new branch Brettpac (#184).
- Redesigned waypoints for sm_dance_bot_warehouse_3.
- Added new spinning behavior for cb pure spinning (#188).
- Integrated planner changes on 16th December (#191).
- Implemented undo motion on 20th December (#196).
- Improved undo motion navigation for warehouse2.
- Tuned warehouse3 (#197).
- Added sync feature on 21st December (#199).
- Completed warehouse2 on 22nd December (#200).
- Fine-tuned and fixed on 23rd December (#201).
- Minor tune on 23rd December (#203).
- Fixed warehouse 3 problems and core improvements (#204).
- Backported to foxy (#206).
- Updated CI build for Galactic due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency.
- Updated workflows and bumped ccache version.
- Satisfied ament_lint_cmake.
- Added missing licenses.
- Enabled cppcheck and corrected formatters.
- Updated ci-build-source.yml.
- Changed extension of imports.
- Corrected formatting of python files.
- Included necessary package and edited Threesome launch.
- Renamed header files and corrected format.
- Added workflow for checking doc build.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Renamed to smacc2 and smacc2_msgs.
- Updated GitHub branch reference.
- Updated package name and package.xml.
- Reset all versions to 0.0.0.
- Ignored all packages except smacc2 and smacc2_msgs.
- Updated changelogs.
- Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Updated description table.
- Updated table.
- Copied initial docs.
- Opened new folder for additional tracing contents.
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh.
- Removed manual installation of ros-rolling-ros2trace.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Updated buildGalactic.sh.
- Updated ManualTracing.md.
- Changed wording from "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Reverted markdowns to html.
- Added README tutorial for Dockerfile.
- Additional cleanup.
```

*pabloinigoblasco*

```rst
Section_5
=========

Added
-----

- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added galactic CI setup and rename rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------

- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated c_cpp_properties.json.
- Updated README.md with launch command.

Fixed
-----

- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview. (#62).
- Fixed pre-commit issues.
- Corrected all linters and formatters.

Removed
-------

- Removed galactic builds from master and kept only rolling. Removed submodules and use .repos file.

Contributors
------------

- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_6
=========

Added
-----

- New client behavior `cb_wait_topic_message` added for nav2, allowing nodes to subscribe to the `/bond` topic and wait until they are active. Nodes selection is optional.
- New client behavior for nav2 added, enabling waiting for nodes subscribing to the `/bond` topic to become active. Node selection is possible.
- New feature `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause_slam` added for pausing SLAM functionality.
- New client behavior `cb_pause_slam` introduced for pausing SLAM operations.
- New client behavior `cb_pause

Section_7
=========

Added
-----

- First working version of sm template and template generator. (#127)
- Feature/dance bot s pattern (#129)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- sm_dance_bot_lite (#136)
- Add SM core test (#138)
- using local action msgs (#139)
- Feature/nav2z renaming (#144)
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- initial migration to smacc2
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
- Feature/wharehouse2 dec 14 (#185)
- Feature/cb pure spinning (#188)

Changed
-------

- Noticed typo "Finnaly" corrected to "Finally"
- minor tweaks (#130)
- minor navigation improvements (#141)
- Resolve compile warnings (#137)
- Add QOS durability to SmaccPublisherClient (#163)
- slight waypoint 4 and iterations changes for course completion (#155)
- moved reference library SMs to smacc2_performance_tools (#166)
- finetuning waypoints (#187)
- fixing some errors introduced on formatting during migration
- progress on moveit migration testing
- improving dockerfile for building local tests
- fixing compiling issues
- update readme (#164)
- fixing broken master build
- fixing pipeline error
- fixing broken build

Fixed
-----

- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger. (#149)
- fixing some more linting warnings
- fixing some formatting and templating on SrConditional
- move trigger logic into headers
- linting fixes

Removed
-------

- removing sm_dance_bot_msgs
- removing parameters smacc
- removing test from main moveit cmake
- removing parameters smacc
- removing some comments from README.md

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section 8
=========

Added
-----
- Feature/planner changes on 16th December (#191)
- Feature/replanning on 16th December (#193)
- Feature/undo motion on 20th December (#196)
- Feature/undo motion on 20th December (#198)
- Feature/sync on 21st December (#199)
- Feature/warehouse2 on 22nd December (#200)
- Feature/warehouse2 on 23rd December (#201)
- Feature/minor tune (#203)
- Merging code from backport foxy and updates about autoware (#208)
- Foxy backport (#206)
- Add galactic CI build because Navigation2 is broken in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency to find include
- Add workflow for checking doc build
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Dockerfile w/ ROS distro as argument
- First ensure you have the necessary package installed
- Include necessary package and edit Threesome launch
- Open new folder for additional tracing contents
- Rename to smacc2 and smacc2_msgs
- Update changelogs
- Update description table
- Update table

Changed
-------
- Branching example
- Change extension of imports
- Change wording "smacc application" to "SMACC2 library"
- Reactivate smacc2 nav clients for rolling via submodules
- Rename tracing events
- Revert markdowns to HTML
- Update GitHub branch reference
- Update name of package and package.xml to pass liter
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Update smacc_sm_reference_library/sm_atomic/README.md
- Update tracing/ManualTracing.md
- Use docs/ as output directory
- Use docs/ as source folder for documentation

Fixed
-----
- Fix trailing spaces
- Fix minor broken build
- Fixing warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green
- Several fixes (#194)
- Updating subscriber publisher components
- Weird moveit not downloaded repo

Removed
-------
- Disable ament_cpplint
- Disable cpplint and cppcheck linters
- Disable some packages and update workflows
- Disable disabled packages
- Ignore further packages
- Ignore all packages except smacc2 and smacc2_msgs
- Satisfy ament_lint_cmake

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_9
=========

Added
-----

- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------

- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated smacc2_rta command across readmes.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Corrected trailing spaces.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md.

Fixed
-----

- Fixed source CI and corrected README overview. (#62).
- Fixed trailing spaces.
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
- Minor formatting.
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
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in

```rst
Section_10
==========

Added
-----
- New client behavior `cb_wait_topic_message` for asynchronous waiting and optional content check on a topic message.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the `/bond` topic and confirm they are active, with optional node selection.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature for visualizing TurtleBot3.
- Gazebo fixes to show the robot and lidar in various dance bot features.
- Progress in AWS navigation demo.
- Progress in navigation testing with core improvements.
- Progress in testing `sm_dance_bot` with navigation parameter fixes.
- Progress in testing `sm_multi_stage_1`.
- Progress in testing `sm_dance_bot_strikes_back`.
- Progress in testing `sm_dance_bot_s_pattern`.
- Progress in testing `sm_dance_bot` with lidar show/hide option.
- Progress in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Progress in testing `smacc2::deep_history` syntax.

Changed
-------
- Minor formatting improvements.

Fixed
-----
- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting issues.
- Adjusted build packages for source CI.
- Removed merge markers from a Python file.
- Moved method to prevent recursion in `sm_dance_bot` changes.

Removed
-------
- Removed `neo_simulation2` package.

Contributors
------------
- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_11

### Added
- First working version of sm template and template generator. (#127)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Rolling Docker environment to be executed from any environment (#154)
- Husky launch file in sm_dance_bot
- Waypoint Inputs (#178)
- More Waypoints

### Changed
- Renamed reference library SMs to smacc2_performance_tools (#166)
- Renamed navigation 2 stack to navigation 2
- Renamed sm_advanced_recovery_1 to sm_advanced_recovery
- Reworked sm_multi_stage_1 (#172)
- Refactored aws navigation and navigation clients and behaviors
- Finetuned waypoints (#187)

### Fixed
- Fixed launch command for sm_dance_bot_strikes_back in README.md
- Fixed CI: format fix python version (#148)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error

### Removed
- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed node creation and create only a logger

### Miscellaneous
- Minor tweaks (#130)
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Minor configuration
- Minor changes (#175)
- Format (#180)
- Format
- Merge
- Headless and other fixes
- Default values
- Finishing touches 1
- Readme
- Precommit cleanup

### Contributors
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
- branching example
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
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

Changed
-------
- minor changes (#190)
- several fixes (#194)
- tuning warehouse3 (#197)
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements (#204)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green

Fixed
-----
- minor broken build
- removing warnings (#213)
- minor linking errors foxy
- minor format
- minor linking errors foxy

Removed
-------
- merge
- headless and other fixes
- default values
- minor
- pure spinning behavior missing files
- more fixes
- replanning for all our examples
- improving undo motion navigation warehouse2
- undo tuning and errors
- format
- minor
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
``` 

*Autoría de Pablo Iñigo Blasco (pabloinigoblasco)*

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
- Edited tracing.md to reflect new tracing event names.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Renamed event generator library.

Fixed
----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Cleaned up sm_atomic_24hr.
- Fixed source CI and corrected README overview.
- Attempted pre-commit fixes.
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other
-----
- Several core improvements during navigation testing.
- More changes on performance tests.
- More on performance and other issues.
- Progress in aws navigation demo.
- More on navigation.
- Formatting improvements.
- Minor formatting.
- Additional cleanup.
- Cleanup.
- Edited tracing.md to reflect new tracing event names.
- Some progress on navigation rolling.
- More on navigation.
- Progress in aws navigation demo.
- More on navigation.
- More on navigation.
- More on navigation.

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
- Introducing new feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Added new client behavior for `nav2`, allowing waiting for nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.

Changed
-------
- Minor formatting improvements.

Fixed
-----
- Navigation parameters fixes on `sm_dance_bot`.
- Removed some compile warnings.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Base for the `sm_aws_warehouse` navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Merge and progress.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite`.
- Updates yaml.
- `sm_dance_bot` visualizing `turtlebot3`.
- Cleaning and lidar show/hide option.
- Cleaning files and fixing formatting.
- More fixes.
- Gazebo fixes to show the robot and the lidar.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Precommit cleanup run.
- AWS demo.
- Got `sm_multi_stage_1` working (barely).
- Brettpac branch.
- Gaining traction with `sm_multi_stage_1`.
- More progress.
- Keep hammering.
- Two stages.
- 3 part.
- 4th stage.
- 5th stage.
- A3.
- MM.
- Diverse improvements in navigation and performance.
``` 

*pabloinigoblasco*

Section_15
===========

Added
-----

- Feature/diverse improvements navigation performance (#117)
  - Various enhancements to navigation and performance.
  - Additional linting and formatting.
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
  - Introducing smacc2::deep_history syntax.
  - Testing sm_dance_bot with slam pausing/resuming functionality.
- Feature/more sm_dance_bot fixes
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern.
  - Typo correction.
- First working version of sm template and template generator. (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
  - More changes in sm_dance_bot.
  - Build fix.
- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issue cases.
- Progress in the sm_dance_bot tests (#135)
  - Progress on markers cleanup.
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming.
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140).
  - Added remaining SVGs to READMEs (#145).
- Update package list. (#142)
- Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes for course completion.
- Feature/migration moveit client (#151)
  - Initial migration to smacc2.
  - Fixing errors on formatting.
  - Adding missing dependency.
  - Progressing in moveit migration testing.
  - Updating format.
  - Adding .reps dependencies and fixing build errors.
  - Adding dependency to ur5 client.
- Rolling Docker environment to be executed from any environment (#154).
- Initial state machine transition timestamp (#165).
- Add QOS durability to SmaccPublisherClient (#163).
  - Added QOS durability to SmaccPublisherClient.
  - Fixed missing colon.
  - Removed line.
  - Added reliability QOS config.
- Feature/testing moveit behaviors (#167)
  - More testing on moveit.
  - Progress on moveit.
  - More testing on moveit behaviors.
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)
  - Multistage modes and sequences.
  - Steps and sequences for sm_multi_stage_1.
  - Finishing touches and readme updates.
- Feature/aws navigation sm dance bot (#174)
  - Repository dependency.
  - Husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Update dependencies for husky in rolling and galactic.
  - Progress on aws navigation and refactorings on navigation clients and behaviors.
- Waypoint Inputs (#178)
- Warehouse2 (#177)
- Warehouse2 progress (#179)
- Sm_dance_bot_warehouse_3 (#181)
- Feature/sm warehouse 2 13 dec 2 (#182)
  - More changes and headless.
  - Merge with default values.

Changed
-------

- Move method after the method it calls. Otherwise, recursion could happen. (#126)
- Resolve compile warnings (#137).
- Minor navigation improvements (#141).
- Using local action messages (#139).
- Removing sm_dance_bot_msgs.
- Removing parameters smacc (#147).
- Update readme (#164).
- Fix CI: format fix python version (#148).
- Add SM Atomic SM generator. (#143).
- Remove node creation and create only a logger. (#149).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Minor changes (#175).
- Format (#180).

Removed
-------

- Remove merge markers from a Python file. (#119).
- Noticed launch command was incorrect in README.md.
- Removing test from main moveit CMake.
- Removing parameters smacc.
- Workflows update.
- Workflow.
- Removing sm_dance_bot_msgs.
- Pending references.
- Default values.

Co-Authored-By
--------------

- pabloinigoblasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>

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
- Feature/warehouse2 22 12 (#200, #201)
- Finishing warehouse2
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
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command
``` 

*pabloinigoblasco*
```

```rst
Section_17
==========

Added
-----
- Added Dockerfile with ROS distro as argument for easier setup.
- Added setupTracing.sh script for automated installation of necessary packages and configuration of tracing group.
- Added alternative ManualTracing method.
- Added new SM markdowns.
- Added README tutorial for Dockerfile setup.
- Added smacc2_performance_tools for performance testing improvements.
- Added new feature cb_wait_topic_message for asynchronous client behavior.
- Added new feature wait nav2 nodes client behavior.

Changed
-------
- Changed wording from "smacc application" to "SMACC2 library" for clarity.
- Updated mentions of SMACC/ROS to SMACC2/ROS2 for consistency.
- Renamed tracing events for better organization.
- Renamed folders and files for improved structure.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch for accuracy.
- Updated smacc2_rta command across readmes for consistency.
- Cleaned up formatting in various files for better readability.
- Optimized dependencies in move_base_z_planners_common package.
- Renamed event generator library for better identification.
- Enabled build of missing rolling repositories and Navigation2 for semi-binary build.
- Removed galactic builds from master, keeping only rolling, and using .repos file for submodule management.

Fixed
-----
- Fixed bug in smacc2 component for smoother operation.
- Fixed source CI setup and corrected README overview for accuracy.
- Fixed trailing spaces for cleaner code.
- Fixed formatting issues in sm_reference_library for better presentation.
- Fixed pre-commit issues in various files for consistency.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh script.
- Deleted tracing directory for better organization.

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_18
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection.
- Base for the `sm_aws_warehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_pause_slam`.
- `sm_dance_bot_lite` visualization for TurtleBot3.
- Lidar show/hide option for `sm_dance_bot`.
- Gazebo fixes to show the robot and lidar for various dance bot versions.
- `sm_multi_stage_1` doubling.
- AWS demo.
- `sm_multi_stage_1` working progress.
- `sm_multi_stage_1` development in Brettpac branch.

Changed
-------
- Corrected all linters and formatters.
- Formatting improvements.

Fixed
----
- Navigation parameters fixes on `sm_dance_bot`.
- Removed some compile warnings.

Removed
-------
- Minor format fixes.
- Merge and progress entries.
- `doxygen` deployment workflow renamed.

Contributors
------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

# Changelog

## [Unreleased]

### Added
- Added `sm_multi_stage_1` with multistage modes, sequences, steps, and finishing touches. Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.

### Changed
- Renamed `sm_advanced_recovery_1`.
- Reworked `sm_multi_stage_1` for improved functionality.

### Fixed
- Fixed recursion issue in method calls.
- Resolved compile warnings.
- Fixed CI formatting for Python version.

### Removed
- Removed `neo_simulation2` package.
- Removed unnecessary node creation in favor of logger.
- Removed `sm_dance_bot_msgs` and parameters from `smacc`.

## [1.0.0] - 2022-01-01

### Added
- Initial release with various improvements in navigation and performance.
- Added `sm_dance_bot` features and refinements.
- Added `SM Atomic SM generator`.
- Added QOS durability to `SmaccPublisherClient`.
- Added AWS navigation for `sm_dance_bot`.

### Changed
- Migrated `moveit` client to `smacc2`.
- Updated format and dependencies for `moveit` migration.
- Moved reference library SMs to `smacc2_performance_tools`.

### Fixed
- Fixed course completion issues with slight waypoint adjustments.
- Fixed launch command in `README.md`.
- Fixed broken master build and pipeline errors.

### Removed
- Removed test from main `moveit` CMake.
- Removed unnecessary comments and test from `moveit` migration.

### Contributors
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>

## [0.1.0] - 2021-12-01

### Added
- Initial state machine transition timestamp.
- Added `sm_pubsub_1` and `sm_pubsub_1 part 2`.
- Added `sm_dance_bot_strikes_back` refactoring.
- Added `sm_dance_bot_lite`.

### Changed
- Renamed `sm_multi_stage_1` sequences and modes.
- Updated navigation 2 stack naming.

### Fixed
- Fixed overshot issues in `waypoints` navigator.
- Fixed format issues in various files.
- Fixed compiling issues in Docker environment.

### Removed
- Removed unnecessary parameters in `smacc`.
- Removed `sm_dance_bot_msgs` and unnecessary dependencies.

### Contributors
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

[pabloinigoblasco]

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
- Sm_dance_bot_warehouse_3.
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
- Tuning and fixes.
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
- Progress in barrel husky.
- Only rolling version should be pre-released on master.
- Barrel search build fix and warehouse3.
- Fixing startup problems in warehouse 3.
- Multiple controllable leds plugin.
- Progress in husky demo.
- Update file for fake hardware simulation and add file for gazebo simulation.
- Add ignition file and update repos files.
- Progressing in husky demo.
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
- Format issues.
- Tuning and fixes.
- Minor tune.
- Minor format fix.
- Other minor changes.
- Fix broken source build.
- Correct Focal-Rolling builds by fixing the version of rosdep yaml.
- Foxy backport.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.

Removed
-------
- Some reordering fixes.
- Warnings removal and more testing on navigation.
- Fixing docker for foxy and galactic.
- Red picuup.

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
Section_21
==========

Added:
------
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

Changed:
--------
- Rename header files and correct format.
- Change extension of imports.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Update description table.
- Update table.
- Copy initial docs.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and keep only rolling.
- Update mentions of SMACC/ROS to SMACC2/ROS2.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.

Fixed:
------
- Correct formatting of python file.
- Correct formatters.
- Correct trailing spaces.

Removed:
--------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Disable further packages.
- Ignore all packages except smacc2 and smacc2_msgs.
- Ignore further packages.
- Revert "Ignore all packages except smacc2 and smacc2_msgs".

Authors:
--------
- Pablo Iñigo Blasco
```

```rst
Section_22
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection.
- New feature: cb pause slam client behavior.
- New feature: sm_dance_bot_lite.

Changed
-------
- Updated launch command.
- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor hotfixes.
- Progress in aws navigation demo.
- Merge and progress.
- Fix formatting.
- Updates yaml.

Fixed
-----
- Fixing precommit.
- Remove some compile warnings.

Removed
-------
- Minor format improvements.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

Section_23
==========

Added
-----
- Added `sm_dance_bot` visualization for `turtlebot3`.
- Added lidar show/hide option for cleaning.
- Added formatting improvements.

Changed
-------
- Improved `sm_dance_bot` Lite Gazebo functionality (#104).
- Enhanced Gazebo visualization for the robot and lidar.
- Doubled functionality for `sm_multi_stage_1` (#103).
- Improved Gazebo functionality for `sm_dance_bot_strikes_back`.
- Enhanced AWS demo functionality (#108).
- Improved functionality for `sm_multi_stage_1` in various stages.
- Enhanced navigation and performance in diverse areas (#116).
- Progressed in navigation, slam toggle client behaviors, and slam_toolbox components for `sm_dance_bot`.
- Improved `sm_dance_bot` S-pattern functionality.
- Refined `sm_dance_bot` and S-pattern.
- Resolved compile warnings.
- Added SM core test.
- Made minor navigation improvements.
- Updated package list.
- Refactored `sm_dance_bot_strikes_back` (#152).
- Made slight changes to waypoints and iterations for course completion (#155).
- Migrated MoveIt client to `smacc2`.
- Updated README files with SVGs.
- Removed unnecessary parameters from `smacc`.
- Fixed CI formatting for Python version.
- Added SM Atomic SM generator.
- Updated Docker environment for execution in any environment.

Fixed
-----
- Fixed recursion possibility by moving method calls.
- Fixed launch command in README.md for `sm_dance_bot_strikes_back`.
- Fixed CI formatting for Python version.

Removed
-------
- Removed `neo_simulation2` package.
- Removed unnecessary test from main MoveIt CMake.
- Removed test from main MoveIt CMake.
- Removed `sm_dance_bot_msgs` references.
- Removed parameters from `smacc`.

Authors
-------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Brett <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_24
==========

Added
-----
- Initial state machine transition timestamp (#165)
- Added QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Feature/aws navigation sm dance bot (#174)
- Added dependencies for husky simulation
- Added Waypoint Inputs (#178)
- Added SrConditional fixes and formatting (#168)
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
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
- Added reliability QOS config
- Progress on moveit
- More testing on moveit behaviors
- Reworked sm_multi_stage_1
- Finishing touches on sm_multi_stage_1
- Redoing sm_dance_bot_warehouse_3 waypoints
- Improving undo motion navigation warehouse2
- Tuning warehouse3
- Improvements in smacc core
- Refining cp subscriber cp publisher
- Progress in autoware machine
- Finishing warehouse2
- Tuning and fixes
- Improvements in barrel husky
- Progress in barrel demo
- Reordering fixes
- Docker build files for all versions
- Barrel search build fix and warehouse3
- Fixing startup problems in warehouse 3

Fixed
-----
- Added a missing colon
- Fixed pipeline error
- Fixed broken master build
- Fixed formatting
- Fixed broken build
- Fixed lint
- Fixed errors in pure spinning behavior
- Fixed format issues
- Fixed warehouse 3 problems
- Fixed dead lock in warehouse 3
- Fixed missing files in warehouse2
- Fixed linking errors for foxy
- Fixed broken build in barrel demo
- Fixed warnings removal
- Fixed docker build for foxy and galactic

Removed
-------
- Removed a line
- Removed weird moveit not downloaded repo
- Removed warnings
```

```rst
Section_25
==========

Added
-----
- Docker improvements.
- Runtime dependency.
- Restoring UR dependency.

Changed
-------
- Master rolling to galactic backport.
- Updating galactic repos.

Fixed
-----
- Fixing build.

Other
-----
- More merge.
- Testing dance bot demos.

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
