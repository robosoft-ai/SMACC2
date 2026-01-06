Changelog for package backward_global_planner
=============================================

Version 2.3.16 (2023-07-16)
----------------------------
### Added
- Merged branch 'humble' from https://github.com/robosoft-ai/SMACC2 into humble
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further details on the buildfarm problem
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
- Improved navigation client behaviors and husky barrel demo (#311)
  - Enhanced navigation client behaviors and husky barrel demo
  - Numerous enhancements in action client and cb sequence for husky barrel search
  - Enhanced navigation behaviors for husky barrel search demo
  - Functionality improvements in navigation and warehouse 3
  - Format enhancements in navigation and warehouse 3 for husky
- Reverted "Ignore packages which should not be released."
  - Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Contributors: Denis Štogl, Pablo Iñigo Blasco

0.3.0 (2022-04-04)
------------------

### Added
- Improvements in navigation client behaviors and husky barrel demo (#311)
  - Many enhancements in action client and cb sequence for husky barrel search
  - Enhanced navigation behaviors on husky barrel search demo
  - Functionality improvements in navigation and warehouse 3
  - Format enhancements in warehouse 3 and husky

### Changed
- Revert "Ignore packages which should not be released."
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Renamed folders, deleted tracing.md, edited README.md
- Added smacc2_performance_tools
- Performance tests improvements
- Renaming of event generator library
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Corrected all linters and formatters

### Fixed
- Do not execute clang-format on smacc2_sm_reference_library package
- Correct trailing spaces

### Removed
- Ignored packages which should not be released

### Miscellaneous
- Various minor formatting and cleanup tasks

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_3
=========

Added
-----

- New client behavior for nav2: Now waits for nav2 nodes to subscribe to the /bond topic and ensures they are alive. Optional node selection available.
- Progress in AWS navigation demo.
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Merge and progress.
- New client behavior for nav2: Waits for nav2 nodes to subscribe to the /bond topic and ensures they are alive. Optional node selection available.
- Navigation parameters fixes on sm_dance_bot.
- New feature: cb_pause_slam client behavior.
- Rename doxygen deployment workflow.
- sm_dance_bot visualizing turtlebot3.
- Feature: dance bot launch gz lidar choice.
- Feature: sm dance bot lite gazebo fixes.
- Feature: sm dance bot strikes back gazebo fixes.
- AWS demo.
- Got sm_multi_stage_1 working (barely).
- Brettpac branch.
- Remove neo_simulation2 package.
- More sm_multi_stage_1.
- Diverse improvements in navigation and performance.

Changed
-------

- Minor formatting improvements.

Fixed
-----

- Remove some compile warnings.

Removed
-------

- Removed neo_simulation2 package.
```

Section 4
=========

Added
-----

- Additional linting and formatting.
- Feature to toggle slam and smacc deep history (#122).
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also introduced smacc2::deep_history syntax.
- Introducing slam pausing/resuming functionality in testing sm_dance_bot.
- More fixes in sm_dance_bot (#125).
- First working version of sm template and template generator (#127).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Rolling Docker environment to be executed from any environment (#154).
- Added SM Atomic SM generator (#143).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Added QOS durability to SmaccPublisherClient (#163).
- Feature to test moveit behaviors (#167).
- Progress on aws navigation and some other refactorings on navigation clients and behaviors.
- More on aws demo.
- Waypoint inputs (#178).
- Redoing sm_dance_bot_warehouse_3 waypoints.

Changed
-------

- Move method after the method it calls to prevent recursion (#126).
- Fixed launch command in README.md for sm_dance_bot_strikes_back and removed some comments.
- Fixed CI: format fix python version (#148).
- Removed node creation and created only a logger (#149).
- Renamed sm_advanced_recovery_1 (#171).
- Reworked sm_multi_stage_1 with multistage modes, sequences, steps, and finishing touches (#172).
- Updated package list (#142).
- Removed parameters smacc (#147).
- Added .reps dependencies and fixed some build errors.
- Updated readme (#164).

Fixed
-----

- Minor format issues (#134).
- Minor tweaks (#130).
- Minor changes (#175).
- Minor format (#124).
- Minor navigation improvements (#141).
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.

Removed
-------

- Removed merge markers from a python file (#119).
- Removed test from main moveit cmake.
- Removed sm_dance_bot_msgs.
- Removed some comments in the past.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section 5
=========

Added
-----
- Added Feature/wharehouse2 dec 14 (#185) with warehouse2 changes and minor updates.
- Added Feature/sm warehouse 2 13 dec 2 (#186) with format changes, headless improvements, and default values.
- Added Feature/cb pure spinning (#188) with format changes, headless improvements, and default values.
- Added Feature/cb pure spinning (#189) with format changes, headless improvements, and default values.
- Added Feature/planner changes 16 12 (#191) with minor changes, fixes, and replanning.
- Added Feature/replanning 16 dec (#193) with minor changes and replanning.
- Added several fixes (#194) and minor changes (#195).
- Added Feature/undo motion 20 12 (#196) with minor changes, undo motion improvements, and warehouse2 navigation tuning.
- Added Feature/undo motion 20 12 (#198) with minor changes, undo motion improvements, and warehouse2 navigation tuning.
- Added Feature/sync 21 12 (#199) with minor changes, replanning, and format fixes.
- Added Feature/warehouse2 22 12 (#200) with minor changes, replanning, and format fixes.
- Added Feature/warehouse2 23 12 (#201) with minor changes, replanning, tuning, and fixes.
- Added Feature/minor tune (#203) with tuning, fixes, and minor adjustments.
- Added fixing warehouse 3 problems and core improvements (#204) to remove deadlocks and improve continuous integration.
- Added Foxy backport (#206) with formatting fixes, trailing spaces correction, and various updates.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Added
-----
- Added instructions to ensure the necessary package is installed before running commands.

Changed
-------
- Changed wording from "smacc application" to "SMACC2 library" for clarity.

Fixed
-----
- Fixed bug in smacc2 component.
- Fixed formatting and linting issues.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_6
=========

Added
-----
- Performance tests improvements.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files. (#58)
- Fixed source CI and corrected README overview. (#62)
- Updated c_cpp_properties.json.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.
- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.

Changed
-------
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md with launch command.
- Progress in aws navigation demo.

Fixed
-----
- Corrected trailing spaces.
- Attempted pre-commit fixes.
- Fixed pre-commit.
- Fixed pre-commit.
- Fixed pre-commit.

Removed
-------
- Removed redundant formatting entries.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_7
=========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- `sm_dance_bot_lite` gazebo visualization.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `sm_multi_stage_1` doubling.
- `smacc2::deep_history` syntax for `slam_toolbox` components.

Changed
-------
- Progress in AWS navigation demo.
- Formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Minor format adjustments.
- `sm_multi_stage_1` improvements.
- Polishing `sm_dance_bot` and `s-pattern`.

Fixed
-----
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting issues.
- Adjust build packages for source CI.
- Move method after the method it calls to prevent recursion.

Removed
-------
- `neo_simulation2` package.

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pablo@ibrobotics.com>
```

Section 8
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
  - Progress made in markers cleanup.
- sm_dance_bot_lite (#136)
  - Lightweight version of sm_dance_bot.
- Add SM core test (#138)
  - Addition of SM core test.
- Feature/nav2z renaming (#144)
  - Renaming navigation 2 stack, added SVGs to READMEs.
- Update package list. (#142)
  - Package list update.
- Add SM Atomic SM generator. (#143)
  - Addition of SM Atomic SM generator.
- Rolling Docker environment to be executed from any environment (#154)
  - Docker environment now executable from any environment.
- Add QOS durability to SmaccPublisherClient (#163)
  - QOS durability feature added to SmaccPublisherClient.
- Feature/migration moveit client (#151)
  - Migration to smacc2, dependency updates, linting fixes.
- Feature/aws navigation sm dance bot (#174)
  - Progress on AWS navigation, refactorings on navigation clients and behaviors.
- Waypoint Inputs (#178)
  - Addition of waypoint inputs.
- Feature/planner changes 16 12 (#191)
  - Minor changes and fixes in planner functionality.
- Feature/replanning 16 dec (#193)
  - Replanning for all examples.

Changed
-------
- Minor navigation improvements (#141)
  - Minor improvements in navigation.
- Using local action msgs (#139)
  - Transition to using local action messages.
- Resolved compile warnings (#137)
  - Compilation warnings resolved.
- Fix CI: format fix python version (#148)
  - CI fix for Python version.
- Noticed launch command was incorrect in README.md
  - Corrected launch command in README.md.
- Initial state machine transition timestamp (#165)
  - Timestamp added to initial state machine transition.
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Reference library SMs moved to smacc2_performance_tools.
- Finetuning waypoints (#187)
  - Fine-tuning of waypoints.

Fixed
-----
- Warehouse2 progress (#179)
  - Progress in Warehouse2 functionality.
- SrConditional fixes and formatting (#168)
  - Fixes and formatting in SrConditional.
- Several fixes (#194)
  - Various fixes applied.

Removed
-------
- Removing parameters smacc (#147)
  - Parameters related to smacc removed.
- Removing node creation and create only a logger. (#149)
  - Node creation removed, only logger created.
- Removing test from main moveit cmake
  - Test removed from main moveit cmake.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing sm_dance_bot_msgs
  - Removed sm_dance_bot_msgs.
- Removing sm_dance_bot_msgs
  - Removed sm_dance_bot_msgs.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.
- Removing parameters smacc
  - Parameters related to smacc removed.

```rst
Section_9
=========

Version 20.12
-------------

Added:
- Feature/undo motion 20 12 (#196)
- Improving undo motion navigation warehouse2
- Tuning warehouse3 (#197)

Changed:
- Minor changes
- Replanning for all examples

Version 21.12
-------------

Added:
- Feature/sync 21 12 (#199)

Changed:
- Minor changes
- Replanning for all examples
- Format issues

Version 22.12
-------------

Added:
- Feature/warehouse2 22 12 (#200)

Changed:
- Minor changes
- Replanning for all examples
- Format issues
- Finishing warehouse2

Version 23.12
-------------

Added:
- Feature/warehouse2 23 12 (#201)

Changed:
- Minor changes
- Replanning for all examples
- Tuning and fixes (#202)

Added:
- Feature/minor tune (#203)

Changed:
- Tuning and fixes
- Minor tune
- Fixing warehouse 3 problems, and other core improvements (#204)
- Added missing file from warehouse2 (#205)
- Merging code from backport foxy and updates about autoware (#208)

Removed:
- Weird moveit not downloaded repo

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>

Version 0.1.0
-------------

Added:
- First ensure you have the necessary package installed.
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

Changed:
- Ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
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
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Some progress on navigation rolling
- Added smacc2_performance_tools
- Performance tests improvements
- More on performance and other issues
- Sm_respira_1 format cleanup
- Sm_respira_1 format cleanup pre-commit
- Sm_respira_test_2

Removed:
- Trailing spaces
- Manual installation of ros-rolling-ros2trace (now automated in setupTracing.sh)
- Tracing directory
- Moved tracing.md to tracing directory
- Added setupTracing.sh (installs necessary packages and configures tracing group)
- Created alternative ManualTracing
- Added new sm markdowns
- Added a Dockerfile for Rolling and Galactic
- Updated tracing/ManualTracing.md
- Changed wording "smacc application" to "SMACC2 library"
- Reactivating smacc2 nav clients for rolling via submodules
- Renamed tracing events after
- Bug in smacc2 component
- Reverted markdowns to HTML
- Added README tutorial for Dockerfile
- Additional cleanup
- Edited tracing.md to reflect new tracing event names

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```
*pabloinigoblasco*

```rst
Section_10
==========

Added
-----
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.

Changed
-------
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Updated doxygen links (#70)
- Updated README.md launch command.
- Corrected all linters and formatters.

Fixed
----
- Fixed source CI and corrected README overview. (#62)
- Fixed trailing spaces.
- Fixed pre-commit issues.

Removed
-------
- Removed execution of clang-format on smacc2_sm_reference_library package.

Other
-----
- Updated smacc2_rta command across readmes.
- Renamed event generator library.
- Cleaned up sm_atomic_24hr.
- Optimized dependencies in move_base_z_planners_common.
- Minor formatting improvements.
- Progressed in aws navigation demo.
- Fixed navigation parameters on sm_dance_bot.
- Several core improvements during navigation testing.
- More on navigation.
- Several rounds of work on sm_advanced_recovery_1.
- More readme updates.
- Created new state machine from sm_respira_1.
- Base for the sm_aws_aarehouse navigation.
- Progressed in aws navigation.
- Formatting improvements.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
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
Section_11
==========

Added
-----

- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and wait for them to be alive, with optional node selection
- Base for the sm_aws_warehouse navigation
- New client behavior, cb_pause_slam
- Sm_dance_bot_lite gazebo fixes
- Sm_dance_bot_strikes_back gazebo fixes
- Aws demo progress
- Slam toggle client behaviors and slam_toolbox components
- Smacc2::deep_history syntax
- Dance bot s-pattern refinement
- First working version of sm template and template generator

Changed
-------

- Navigation parameters fixes on sm_dance_bot
- Minor format improvements
- Format fixes for gazebo to show the robot and lidar
- Minor tweaks

Fixed
-----

- Remove some compile warnings
- Remove neo_simulation2 package
- Correct formatting
- Enable source build on PR for testing
- Adjust build packages of source CI
- Remove merge markers from a python file
- Move method after the method it calls to prevent recursion
- Fix typo "Finnaly" to "Finally"
- Build fix
- Waypoints navigator bug

Removed
-------

- Remove neo_simulation2 package

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_12
==========

Added
-----
- Progress in the sm_dance_bot tests (#135)
- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added Waypoint Inputs (#178)
- Added more Waypoints to sm_dance_bot_warehouse_3 (#184)
- Finetuned waypoints (#187)
- Added pure spinning behavior missing files (#189)
- Added replanning for all examples (#193)
- Improved undo motion navigation in warehouse2 (#196)

Changed
-------
- Minor tuning to mitigate overshot issue cases
- Some progress on markers cleanup
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action messages
- Feature/nav2z renaming (#144)
- Resolved compile warnings (#137)
- Updated package list (#142)
- Removed parameters smacc (#147)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Pre-commit cleanup
- Fixed launch command for sm_dance_bot_strikes_back in README.md
- Updated readme (#164)
- Updated format
- Fixed CI: format fix python version (#148)
- Removed node creation and created only a logger (#149)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed formatting
- Fixed some linting warnings
- Fixed some errors introduced on formatting
- Fixed some build errors
- Fixed some formatting and templating on SrConditional
- Moved trigger logic into headers

Fixed
-----
- Noticed launch command was incorrect in README.md

Removed
-------
- Removed sm_dance_bot_msgs
- Removed test from main moveit cmake
- Removed test from main moveit cmake
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
- Removed some comments in the past
-

Section_13
==========

Added
-----
- Feature/sync 21 12 (#199): Sync feature added.
- Feature/warehouse2 22 12 (#200): Warehouse2 feature added.
- Feature/warehouse2 23 12 (#201): Warehouse2 feature finished.
- Feature/minor tune (#203): Minor tuning and fixes added.
- Fixing warehouse 3 problems, and other core improvements (#204): Warehouse 3 issues fixed, core improvements made.
- Added missing file from warehouse2 (#205): Missing file added.
- Add mergify rules file: Mergify rules file added.
- Add Autoware Auto Msgs into not-released dependencies (#220): Autoware Auto Msgs added to dependencies.
- Fix rolling builds (#222): Rolling builds fixed.
- Add galactic CI build because Navigation2 is broken in rolling: Galactic CI build added due to Navigation2 issues.
- Add partial changes for ament_cpplint: Partial changes for ament_cpplint added.
- Add tf2_ros as dependency to find include: tf2_ros dependency added.
- Disable ament_cpplint: ament_cpplint disabled.
- Disable some packages and update workflows: Some packages disabled, workflows updated.
- Bump ccache version: ccache version bumped.
- Ignore further packages: Further packages ignored.
- Satisfy ament_lint_cmake: ament_lint_cmake satisfied.
- Add missing licences: Missing licenses added.
- Disable cpplint and cppcheck linters: cpplint and cppcheck linters disabled.
- Correct formatters: Formatters corrected.
- Enable cppcheck: cppcheck enabled.
- Correct formatting of python file: Python file formatting corrected.
- Included necessary package and edited Threesome launch: Necessary package included, Threesome launch edited.
- Add workflow for checking doc build: Workflow added for checking documentation build.
- Create doxygen-deploy.yml: doxygen-deploy.yml created.
- Create workflow for testing prerelease builds: Workflow created for testing prerelease builds.
- Rename to smacc2 and smacc2_msgs: Renamed to smacc2 and smacc2_msgs.
- Update name of package and package.xml to pass liter: Package name and package.xml updated.
- Update changelogs: Changelogs updated.
- Revert "Ignore all packages except smacc2 and smacc2_msgs": Reverted commit.
- Update description table: Description table updated.
- Update table: Table updated.
- Dockerfile w/ ROS distro as argument: Dockerfile with ROS distro as argument added.
- Opened new folder for additional tracing contents: New folder for tracing contents opened.
- Delete tracing directory: Tracing directory deleted.
- Moved tracing.md to tracing directory: tracing.md moved to tracing directory.
- added setupTracing.sh: setupTracing.sh added.
- Removed manual installation of ros-rolling-ros2trace: Manual installation of ros-rolling-ros2trace removed.
- Created alternative ManualTracing: Alternative ManualTracing created.
- added new sm markdowns: New sm markdowns added.
- added a dockerfile for Rolling and Galactic: Dockerfile for Rolling and Galactic added.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh: buildGalactic.sh updated.
- Update tracing/ManualTracing.md: ManualTracing.md updated.
- changed wording "smacc application" to "SMACC2 library": Wording changed from "smacc application" to "SMACC2 library".
- Update smacc_sm_reference_library/sm_atomic/README.md: README.md updated.

Changed
-------
- ros2 launch sm_three_some sm_three_some: Changed launch command.
- First ensure you have the necessary package installed: Instructions added for package installation.
- Rename header files and correct format: Header files renamed and format corrected.

Fixed
-----
- Remove example things from Foxy CI setup (#214): Removed example things from Foxy CI setup.
- Fix trailing spaces: Trailing spaces fixed.
- Correct codespell: Codespell corrected.
- Correct python linters warnings: Python linters warnings corrected.
- Fix minor broken build: Minor broken build fixed.
- Fixing docker for foxy and galactic: Docker issues fixed for foxy and galactic.
- removing warnings (#213): Warnings removed.

Removed
-------
- some reordering fixes: Reordering fixes removed.
- branching example: Branching example removed.
- Disable disabled packages: Disabled packages disabled.
- Update ci-build-source.yml: ci-build-source.yml updated.
- Change extension: Extension changed.
- Change extension of imports: Extension of imports changed.
- Execute on master update: Execution on master update removed.
- Reset all versions to 0.0.0: Versions reset to 0.0.0.
- Ignore all packages except smacc2 and smacc2_msgs: Ignored all packages except smacc2 and smacc2_msgs.

Section_14
-----------

Added
-----
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.

Changed
-------
- Reverted markdowns to html.
- Edited tracing.md to reflect new tracing event names.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Renamed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links.
- Updated c_cpp_properties.json.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Updated README.md launch command.
- Corrected all linters and formatters.

Fixed
----
- Fixed source CI and corrected README overview (#62).
- Fixed pre-commit issues.
- Fixed Pre-Commit issues.
- Attempted pre-commit fixes.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Other
-----
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting improvements.
- Minor improvements during navigation testing.
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
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress in AWS navigation demo.
- Progress

```rst
Section_15
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` visualization for TurtleBot3.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo improvements.
- `sm_multi_stage_1` enhancements.
- Diverse improvements in navigation and performance.
- Progress in navigation, `slam_toggle` client behaviors, and `slam_toolbox` components. Introducing `smacc2::deep_history` syntax.
- Introducing `slam` pausing/resuming functionality in `sm_dance_bot`.
- More changes in `sm_dance_bot`.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor formatting improvements.
- Format fixes for gazebo to show the robot and lidar.

Fixed
----
- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.
- Removed merge markers from a Python file.

Removed
-------
- Removed `neo_simulation2` package.

Contributors
------------
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_16
==========

Added
-----
- First working version of sm template and template generator. (#127)
- Add SM core test (#138)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Waypoint Inputs (#178)
- finetuning waypoints (#187)

Changed
-------
- Move method after the method it calls to prevent recursion (#126)
- Rename "Finnaly" to "Finally"
- Update package list. (#142)
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Move trigger logic into headers on SrConditional

Fixed
-----
- Fix CI: format fix python version (#148)
- Fix compiling issues
- Fix broken master build
- Fix formatting on warehouse2 (#177)

Removed
-------
- Remove node creation and create only a logger. (#149)
- Remove parameters smacc (#147)
- Remove test from main moveit cmake
- Remove sm_dance_bot_msgs
- Remove parameters smacc

Other
-----
- Minor tweaks (#130)
- Minor navigation improvements (#141)
- Minor format issues (#134)
- Minor configuration
- Minor changes (#175)
- Minor format (#180)
- Minor changes and headless on Feature/sm warehouse 2 13 dec 2 (#186)
- Format Feature/cb pure spinning (#188)

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
- Feature/cb pure spinning (#189): Implemented pure spinning behavior with missing files.
- Feature/planner changes 16 12 (#191): Introduced planner changes.
- Feature/replanning 16 dec (#193): Added replanning for all examples and fixed several issues.
- Feature/undo motion 20 12 (#196): Improved undo motion navigation in warehouse2.
- Feature/sync 21 12 (#199): Fixed format issues.
- Feature/warehouse2 22 12 (#200): Resolved format issues and completed warehouse2.
- Feature/warehouse2 23 12 (#201): Tuned and fixed issues.
- Feature/minor tune (#203): Tuned and fixed warehouse 3 problems, improving core functionality.
- Feature/retry behavior warehouse 1 (#226): Backported changes to foxy with minor formatting fixes.

Changed
-------
- Fix code generators (#221): Resolved build issues and updated SM template.
- Foxy backport (#206): Fixed formatting, trailing spaces, codespell, and linter warnings. Added galactic CI build.
- Update SM template and make example code clearly visible.
- Update template to resolve the global data correctly.
- Update sm_name.hpp.

Removed
-------
- Removed use of node in the sm performance template.
- Disabled ament_cpplint, cpplint, cppcheck linters, and some packages.
- Ignored further packages and disabled disabled packages.
- Disable ament_lint_cmake.
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.

Fixed
-----
- Fixed docker files for foxy and galactic.
- Fixed broken build, linking errors in foxy, and some reordering issues.
- Fixed docker for different revisions, warnings removal, and navigation testing.
- Fixed warehouse 3 problems, deadlocks, and continuous integration issues.
- Fixed formatting of python files and corrected codespell.
- Fixed formatters and corrected python linters warnings.
- Fixed extension of imports and corrected formatting.
- Fixed branching example and updated ci-build-source.yml.

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
- Optimized dependencies in move_base_z_planners_common.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected trailing spaces.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Minor formatting improvements.
- Corrected all linters and formatters.

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

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_19
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive. Optionally select the nodes to wait.

Changed
-------
- Minor format improvements.
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and lidar.
- `sm_multi_stage_1` doubling.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Got `sm_multi_stage_1` working (barely).
- Gaining traction with `sm_multi_stage_1`.

Fixed
----
- Remove some compile warnings.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Merge and progress.
- Precommit cleanup run.
- Updates `yaml`.
- Several core improvements during navigation testing.
- Keep hammering on multiple stages.
```

Section_20
==========

Added
-----

- Diverse improvements in navigation and performance (#116)
- Additional linting and formatting
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Introducing slam pausing/resuming functionality in testing sm_dance_bot
- First working version of sm template and template generator (#127)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs
- Rolling Docker environment to be executed from any environment (#154)
- Initial migration to smacc2 in moveit client
- Added QOS durability to SmaccPublisherClient
- Moved reference library SMs to smacc2_performance_tools
- Added reliability qos config to SmaccPublisherClient
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- Waypoint Inputs (#178)

Changed
-------

- Move method after the method it calls to prevent recursion (#126)
- Noticed typo "Finnaly" corrected to "Finally"
- Resolved compile warnings (#137)
- Fixed launch command in README.md for sm_dance_bot_strikes_back
- Fix CI: format fix python version (#148)
- Removed node creation and create only a logger (#149)
- Minor navigation improvements (#141)
- Using local action messages instead of sm_dance_bot_msgs
- Navigation 2 stack renaming
- Update package list (#142)
- Removing parameters smacc
- Noticed launch command was incorrect in README.md
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools
- Pre-commit cleanup
- Fixing broken master build
- Progress on moveit migration testing
- Minor configuration changes
- Fixing pipeline error
- Fixing compiling issues
- Format (#180)

Fixed
-----

- Minor format issues (#134)
- Minor tweaks (#130)
- Minor format in feature/more_sm_dance_bot_fixes
- Minor format in feature/dance bot s pattern
- Minor in feature/sm dance bot refine
- Minor in feature/sm dance bot refine 2
- Build fix
- Minor in sm_dance_bot_lite
- Minor in sm_pubsub_1
- Minor in sm_pubsub_1 part 2
- Minor in sm_advanced_recovery_1 renaming
- Minor in sm_multi_stage_1 reworking
- Minor changes (#175)

Removed
-------

- Remove merge markers from a python file (#119)
- Removing sm_dance_bot_msgs
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing parameters smacc
- Workflows update
- Workflow
- Repos dependency
- Adding dependency to ur5 client
- Docker refactoring
- Removing test from main moveit cmake
- Minor dockerfile test workaround
- Improving dockerfile for building local tests
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing parameters smacc
- Removing

```rst
Section_21
==========

Added
-----

- **Feature/sm warehouse 2 13 dec 2 (#182)**
  - Implemented warehouse 2 features on December 13th.
  - Added default values.
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.

- **Feature/Brettpac branch (#184)**
  - Introduced Brett's branch with improvements.
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.

- **Feature/finetuning waypoints (#187)**
  - Refined waypoints for better navigation.
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.

- **Feature/cb pure spinning (#188)**
  - Added pure spinning behavior enhancements.

- **Feature/planner changes 16 12 (#191)**
  - Implemented minor planner adjustments.

- **Feature/replanning 16 dec (#193)**
  - Enhanced replanning functionality for all examples.

- **Feature/undo motion 20 12 (#196)**
  - Improved undo motion navigation for warehouse 2.

- **Feature/sync 21 12 (#199)**
  - Addressed format issues for better synchronization.

- **Feature/warehouse2 22 12 (#200)**
  - Resolved format issues and finalized warehouse 2.

- **Feature/warehouse2 23 12 (#201)**
  - Tuned and fixed warehouse 2 components.

- **Feature/minor tune (#203)**
  - Fine-tuned minor aspects for better performance.

- **Feature/fixing warehouse 3 problems (#204)**
  - Resolved issues in warehouse 3 to enhance core functionality.
  - Co-authored by multiple contributors.

- **Feature/barrel demo**
  - Introduced barrel demo features.

- **Feature/progress in barrel husky**
  - Made progress in husky demo development.

- **Feature/progress in husky demo**
  - Continued improvements in husky demo navigation.

- **Feature/improvements warehouse3 (#228)**
  - Implemented enhancements for warehouse 3.
  - Backported to foxy for compatibility.

- **Feature/Foxy backport (#206)**
  - Backported changes to Foxy release.
  - Addressed formatting issues and trailing spaces.
  - Corrected codespell and Python linters warnings.
  - Added Galactic CI build due to Navigation2 issues.
  - Made necessary adjustments for CI workflows.
  - Updated dependencies and licenses.

Changed
-------

- **Feature/undo motion 20 12 (#198)**
  - Improved undo motion navigation for warehouse 2.
  - Tuned undo behavior and fixed errors.

- **Feature/warehouse2 23 12 (#201)**
  - Tuned and fixed warehouse 2 components.

- **Feature/minor tune (#203)**
  - Fine-tuned minor aspects for better performance.

Removed
-------

- **Feature/undo motion 20 12 (#196)**
  - Removed duplicate entry.

- **Feature/sync 21 12 (#199)**
  - Removed duplicate entry.

- **Feature/warehouse2 22 12 (#200)**
  - Removed duplicate entry.

- **Feature/cb pure spinning (#189)**
  - Removed duplicate entry.

- **Feature/sm warehouse 2 13 dec 2 (#186)**
  - Removed duplicate entry.

- **Feature/sm warehouse 2 13 dec 2 (#185)**
  - Removed duplicate entry.

- **Feature/sm warehouse 2 13 dec 2 (#182)**
  - Removed duplicate entry.

- **Feature/wharehouse2 dec 14 (#185)**
  - Removed duplicate entry.

Fixed
-----

- **Feature/undo motion 20 12 (#196)**
  - Fixed undo motion navigation issues.

- **Feature/warehouse2 23 12 (#201)**
  - Fixed tuning and other minor issues.

- **Feature/minor tune (#203)**
  - Fixed minor tune adjustments.

- **Feature/Foxy backport (#206)**
  - Fixed broken source build.
  - Corrected formatting and minor issues.

- **Feature/improvements warehouse3 (#228)**
  - Fixed minor format and linking errors for Foxy.

- **Feature/fixing warehouse 3 problems (#204)**
  - Fixed warehouse 3 problems and core improvements.
  - Addressed deadlocks and CI integration issues.

Removed
-------

- **Feature/undo motion 20 12 (#196)**
  - Removed duplicate entry.

- **Feature/sync 21 12 (#199)**
  - Removed duplicate entry.

- **Feature/warehouse2 22 12 (#200)**
  - Removed duplicate entry.

- **Feature/cb pure spinning (#189)**
  - Removed duplicate entry.

- **Feature/sm warehouse 2 13 dec 2 (#186)**
  - Removed duplicate entry.

- **Feature/sm warehouse 2 13 dec 2 (#185)**
  - Removed duplicate entry.

- **Feature/sm warehouse 2 13 dec 2 (#182)**
  - Removed duplicate entry.

- **Feature/wharehouse2 dec 14 (#185)**
  - Removed duplicate entry.
```

Section_22
==========

Added
-----
- Renamed header files and corrected format.
- Added workflow for checking doc build.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Created setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Added smacc2_performance_tools.
- Performance tests improvements.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Updated c_cpp_properties.json.
- Updated launch command to `ros2 launch sm_respira_1 sm_respira_1.launch` (#69).
- Updated doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Feature/aws demo progress (#80).
- More on navigation.
- Sm_advanced_recovery_1 reworked (#83).
- More sm_advanced_recovery_1 work (#85).
- Sm_atomic_performance_test_c_1 (#88).
- Sm_multi_stage_1 (#90).
- Wait topic message client behavior (#81).

Changed
-------
- Renamed to smacc2 and smacc2_msgs.
- Updated name of package and package.xml to pass liter.
- Changed wording "smacc application" to "SMACC2 library".
- Reactivated smacc2 nav clients for rolling via submodules.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Minor formatting changes.

Fixed
-----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Corrected trailing spaces.
- Fixed pre-commit issues.
- Fixed several core improvements during navigation testing.

Removed
-------
- Ignored all packages except smacc2 and smacc2_msgs.
- Removed manual installation of ros-rolling-ros2trace (now automated in setupTracing.sh).
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```rst
Section_23
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `wait nav2 nodes subscribing to the /bond topic and waiting they are alive`, with optional node selection
- New client behavior: `cb pause slam` for pausing SLAM operations
- New client behavior: `sm_dance_bot_lite` for lite dance bot operations
- New client behavior: `sm_dance_bot visualizing turtlebot3` for visualizing Turtlebot3 in dance bot operations
- New client behavior: `sm_multi_stage_1 doubling` for multi-stage operations

Changed
-------
- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Gazebo fixes to show the robot and the lidar
- Cleaning and lidar show/hide option for dance bot operations
- Updates in YAML files
- Hotfix for minor issues

Fixed
----
- Corrected all linters and formatters
- Removed some compile warnings

Removed
-------
- None

Commits
-------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

Contributors
------------
- Brett <brett@robosoft.ai>
- Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

Section_24
===========

Added
-----
- Added AWS demo (#108).
- Added Brettpac branch (#110).
- Added a3 (#113).
- Added diverse improvements navigation and performance (#116).
- Added Feature/diverse improvements navigation performance (#117).
- Added Feature/slam toggle and smacc deep history (#122).
- Added Feature/dance bot s pattern (#128).
- Added Feature/dance bot s pattern (#129).
- Added Feature/sm dance bot refine (#131).
- Added Feature/sm dance bot refine 2 (#132).
- Added waypoints navigator bug (#133).
- Added Resolve compile warnings (#137).
- Added Add SM core test (#138).
- Added minor navigation improvements (#141).
- Added Feature/nav2z renaming (#144).
- Added Update package list (#142).
- Added Add SM Atomic SM generator (#143).
- Added Rolling Docker environment to be executed from any environment (#154).
- Added slight waypoint 4 and iterations changes so robot can complete course (#155).
- Added Feature/migration moveit client (#151).
- Added initial state machine transition timestamp (#165).
- Added Feature/testing moveit behaviors (#167).

Changed
-------
- Changed progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax (#122).
- Changed progress in the sm_dance_bot tests (#135).
- Changed progress on moveit migration testing (#151).
- Changed moved reference library SMs to smacc2_performance_tools (#166).
- Changed feat: add qos durability to SmaccPublisherClient (#163).

Fixed
-----
- Fixed gazebo fixes to show the robot and the lidar.
- Fixed format issues.
- Fixed minor format issues (#134).
- Fixed minor tuning to mitigate overshot issue cases.
- Fixed some more linting warnings.
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments in README.md.
- Fixed CI: format fix python version (#148).
- Fixed missing dependency.
- Fixed compiling issues.

Removed
-------
- Removed neo_simulation2 package (#112).
- Removed node creation and create only a logger (#149).
- Removed parameters smacc (#147).
- Removed test from main moveit cmake.
- Removed sm_dance_bot_msgs.
- Removed merge markers from a python file (#119).

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai> (multiple entries).
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>.
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.

```rst
Section_25
==========

Added
-----

- Added sm_pubsub_1 part 2 (#170)
- Added sm_advanced_recovery_1 renaming (#171)
- Added sm_multi_stage_1 reworking (#172)
- Added Feature/aws navigation sm dance bot (#174)
- Added minor changes (#175)
- Added warehouse2 (#177)
- Added Waypoint Inputs (#178)
- Added wharehouse2 progress (#179)
- Added format (#180)
- Added sm_dance_bot_warehouse_3 (#181)
- Added Feature/sm warehouse 2 13 dec 2 (#182)
- Added finetuning waypoints (#187)
- Added Feature/cb pure spinning (#188)
- Added pure spinning behavior missing files
- Added minor changes (#190)
- Added Feature/planner changes 16 12 (#191)
- Added Feature/replanning 16 dec (#193)
- Added several fixes (#194)
- Added Feature/undo motion 20 12 (#196)
- Added tuning warehouse3 (#197)
- Added Feature/sync 21 12 (#199)
- Added Feature/warehouse2 22 12 (#200)
- Added Feature/warehouse2 23 12 (#201)
- Added Feature/minor tune (#203)
- Added fixing warehouse 3 problems, and other core improvements (#204)
- Added added missing file from warehouse2 (#205)
- Added missing sm
- Added updating subscriber publisher components
- Added progress in autoware machine
- Added refining cp subscriber cp publisher
- Added improvements in smacc core adding more components mostly developed for autoware demo
- Added autoware demo
- Added foxy ci
- Added fix
- Added some reordering fixes
- Added docker files for different revisions, warnings removal and more testing on navigation
- Added fixing docker for foxy and galactic
- Added docker build files for all versions
- Added barrel demo
- Added barrel search build fix and warehouse3
- Added fixing startup problems in warehouse 3
- Added fixing format and minor
- Added progress in barrel husky
- Added barrel demo
- Added progress
- Added fixing broken build
- Added more merge
- Added docker improvements
- Added master rolling to galactic backport
- Added fixing build
- Added testing dance bot demos
- Added updating galactic repos
- Added runtime dependency
- Added restoring ur dependency

Changed
-------

- Changed multistage modes
- Changed sm_multi_stage sequences
- Changed sm_multi_state_1 steps
- Changed sm_multi_stage_1 sequence d
- Changed sm_multi_stage_1 c sequence
- Changed mode_5_sequence_b
- Changed mode_4_sequence_b
- Changed sm_multi_stage_1 most
- Changed finishing touches 1
- Changed readme
- Changed progress on aws navigation and some other refactorings on navigation clients and behaviors
- Changed more on aws demo
- Changed fixing broken build
- Changed headless and other fixes
- Changed default values
- Changed minor tune
- Changed improving undo motion navigation warehouse2
- Changed tuning and fixes

Removed
-------

- Removed weird moveit not downloaded repo
- Removed minor broken build
- Removed minor linking errors foxy
```

Section_26
===========

Version 1.2.0 (2023-01-15)
---------------------------

### Added
- New feature for enhanced user authentication.
- Support for custom themes in the UI.

### Changed
- Improved performance in data processing algorithms.
- Updated third-party libraries to latest versions.

### Fixed
- Resolved issue causing crashes on certain devices.
- Fixed incorrect display of timestamps in logs.

### Removed
- Deprecated legacy API endpoints.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
* Contributors: Denis Štogl, Pablo Iñigo Blasco
