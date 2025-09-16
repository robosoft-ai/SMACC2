Changelog for package nav2z_client
===================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from robosoft-ai/SMACC2
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further work on buildfarm issue
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
### No changes

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release
- Contributors: pabloinigoblasco

### Changed
- More progress in humble SMACC2 deb generation
- Humble check
- Publisher
- Progress in migration to humble
- Feature/fix mutex galactic (#319)
  - Bug fix for galactic mutex
  - Testing undo motion and improving action client
  - Refactored smacc action client
  - Fix and progress in smacc action client
  - Progress in smacc action client fork based on signals
  - More changes and testing
  - More testing on abort
  - Added smaccServiceerver client to galactic
  - Updates and testing for husky robot
  - Progress and testing for husky demo
  - Testing abort forward and undo
  - Finishing cancel and undo behavior tests
- Undo motion in stEvasion after detecting enemy - in testing (#315)
  - Undo motion in stEvasion after detecting enemy - in testing
  - Minor format changes
  - Green SMACC2
- Feature/husky barrel improvements (#314)
  - Improvements in navigation client behaviors and husky barrel demo
  - Many improvements in action client and cb sequence for husky barrel search
  - More navigation behaviors on husky barrel search demo
  - Functionality improvements in navigation and warehouse 3
- Husky_improvements (#299)
  - Different planners profiles for navigation
  - Changes from galactic branch
  - Planner switcher and fixes
- Feature/barrel husky improvements (#293)
  - Renamed to smacc2 and smacc2_msgs
  - Updated package and package.xml
  - Dockerfile with ROS distro argument
  - Added setupTracing.sh for tracing group configuration
  - Removed manual installation of ros-rolling-ros2trace
  - Created alternative ManualTracing
  - Added new sm markdowns
  - Added Dockerfile for Rolling and Galactic
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Some progress on navigation rolling
  - Performance tests improvements
  - Format cleanup and performance tests

### Removed
- Tracing directory
- Manual installation steps
- Unused tracing events
- Galactic builds from master
- Submodules for rolling repositories
- Trailing spaces in sm_reference_library
- Unused smacc2_sm_reference_library package
- Clang-format execution on smacc2_sm_reference_library
- Unused smacc2_rta command across readmes
- Cleanup of sm_atomic_24hr

```rst
Section_2
=========

Added
-----
- Added galactic CI setup and renamed rolling files. (#58)
- Added more README updates (#72, #74)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.
- Added navigation parameters fixes on sm_dance_bot.

Changed
-------
- Changed launch command to 'ros2 launch sm_respira_1 sm_respira_1.launch' (#69).
- Updated doxygen links.
- Updated README.md launch command.
- Corrected all linters and formatters.

Fixed
-----
- Fixed source CI and corrected README overview. (#62)
- Fixed pre-commit issues.
- Fixed formatting.

Removed
-------
- Removed note that was not removed.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_3
=========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and wait for them to be alive, with optional node selection.
- New feature: sm_dance_bot visualizing turtlebot3.
- New feature: dance bot launch gz lidar choice, with cleaning and lidar show/hide option.
- New feature: gazebo fixes for sm_dance_bot_strikes_back.
- New feature: diverse improvements in navigation and performance.
- New feature: slam toggle client behaviors and slam_toolbox components, introducing smacc2::deep_history syntax.
- New feature: dance bot s pattern, polishing sm_dance_bot and s-pattern.
- First working version of sm template and template generator.

Changed
-------

- Progress in AWS navigation demo.
- Progress in navigation testing.
- Progress in sm_dance_bot tests.
- Progress in markers cleanup.
- Minor navigation improvements.
- Minor tuning to mitigate overshot issue cases.

Fixed
-----

- Navigation parameters fixes on sm_dance_bot.
- Remove some compile warnings.
- Remove neo_simulation2 package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.
- Resolve compile warnings.

Removed
-------

- Remove merge markers from a Python file.

Contributors
------------

- Ubuntu 20-04-02-amd64 (brett@robosoft.ai)
- Pablo Iñigo Blasco (pablo@ibrobotics.com)
```

```rst
Section_4
=========

Added
-----

- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)

Changed
-------

- Renamed Feature/nav2z to navigation 2 stack (#144)
- Updated package list (#142)
- Refactored Feature/sm dance bot strikes back (#152)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Reworked sm_multi_stage_1 (#172)
- Progressed in moveit migration testing
- Updated readme (#164)
- Finetuned waypoints (#187)

Fixed
-----

- Fixed launch command in README.md
- Fixed CI: format fix python version (#148)
- Fixed node creation in SM Atomic SM generator (#149)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed broken build in aws navigation
- Fixed formatting in warehouse2 (#177)
- Fixed SrConditional formatting (#168)
- Fixed several issues in replanning (#194)
- Fixed several minor issues

Removed
-------

- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed test from main moveit cmake
- Removed test ur5
- Removed node creation and create only a logger

Other Changes
-------------

- Updated navigation 2 stack naming
- Added .reps dependencies and fixed build errors
- Added dependency to ur5 client
- Updated format in moveit migration
- Updated dockerfile for building local tests
- Updated docker refactoring
- Updated progress on move_it PR
- Updated husky launch file in sm_dance_bot
- Updated dependencies for husky in rolling and galactic
- Updated progress on aws navigation and refactorings
- Updated warehouse3 tuning
- Updated undo motion navigation warehouse2
- Updated undo tuning and errors
- Updated format issues
- Updated default values
- Updated headless and other fixes
- Updated merge
- Updated mode_5_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b
- Updated mode_4_sequence_b

```rst
Section 5
=========

Added
-----
- Feature/warehouse2 23 12 (#201)
- Added missing file from warehouse2 (#205)
- Merging code from backport foxy and updates about autoware (#208)
- Foxy backport (#206)
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Rename to smacc2 and smacc2_msgs
- Added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- Changed wording "smacc application" to "SMACC2 library"
- Updated name of package and package.xml to pass liter.
- Reset all versions to 0.0.0
- Updated description table
- Updated table
- Renamed tracing events after
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Updated smacc2_rta command across readmes
- Cleaned up sm_atomic_24hr
- Optimized deps in move_base_z_planners_common
- Renamed event generator library
- Renamed folders, deleted tracing.md, edited README.md
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Updated tracing/ManualTracing.md
- Updated smacc_sm_reference_library/sm_atomic/README.md
- Updated sm_respira_1 sm_respira_1.launch launch command (#69)
- Updated doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)

Fixed
-----
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Correct formatters
- Enable cppcheck
- Correct formatting of python file
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Removed manual installation of ros-rolling-ros2trace
- Fixed source CI and corrected README overview
- Several core improvements during navigation testing

Removed
-------
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cpplint and cppcheck linters
- Disable disabled packages
- Ignore further packages
- Ignore all packages except smacc2 and smacc2_msgs

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

- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait

Changed
-------

- Progress in aws navigation demo
- Navigation parameters fixes on sm_dance_bot
- Several core improvements during navigation testing
- Formatting improvements
- Corrected all linters and formatters

Fixed
-----

- Fix pre-commit issues
- Remove some compile warnings

Removed
-------

- Minor changes

Contributors
------------

- Pablo Iñigo Blasco
- Brett <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
```

## Section_7

### Added
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior `add` for nav2: waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- First working version of sm template and template generator.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.

### Changed
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Going forward in testing `sm_dance_bot` introducing slam pausing/resuming functionality.
- Polishing `sm_dance_bot` and `s-pattern`.
- Minor tuning to mitigate overshot issue cases.
- Minor navigation improvements.
- Renaming in navigation 2 stack.
- Rolling Docker environment to be executed from any environment.

### Fixed
- Format fixes.
- Corrected formatting.
- Adjusted build packages of source CI.
- Fixed launch command for `sm_dance_bot_strikes_back`.
- Removed merge markers from a python file.
- Removed `neo_simulation2` package.
- Removed parameters from `smacc`.

### Removed
- Removed `sm_dance_bot_msgs`.

### Miscellaneous
- Several core improvements during navigation testing.
- Formatting improvements.
- Minor format adjustments.
- Minor hotfixes.
- Precommit cleanup.
- Workflows update.
- Noticed launch command was incorrect in README.md.

### Contributors
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored by pabloinigoblasco <pablo@ibrobotics.com>.

```rst
Section_8
=========

Added
-----

- Feature/sm dance bot strikes back refactoring (#152)
  - Refactored dance bot strikes back feature.
  - Co-authored-by: DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>

- Feature/migration moveit client (#151)
  - Initial migration to smacc2.
  - Fixed errors introduced on formatting.
  - Added missing dependency.
  - Fixed linting warnings.
  - Added .reps dependencies and fixed build errors.
  - Added repos dependency.
  - Added dependency to ur5 client.
  - Docker refactoring.
  - Progress on move_it PR.
  - Improved dockerfile for building local tests.
  - Fixed compiling issues.

Changed
-------

- Update readme (#164)
  - Updated readme content.
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- Add QOS durability to SmaccPublisherClient (#163)
  - Added QOS durability feature to SmaccPublisherClient.

Fixed
-----

- Fixed broken master build.
- Fixed pipeline error.

Removed
-------

- Removed test from main moveit cmake.

Other
-----

- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Added durability to SmaccPublisherClient.
- SrConditional fixes and formatting (#168)
- Warehouse2 progress (#179)
- Waypoint Inputs (#178)
- Finetuning waypoints (#187)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Minor tune (#203)
- Fixed warehouse 3 problems and other core improvements (#204)
- Added missing file from warehouse2 (#205)

Authors
-------

- Pablo Iñigo Blasco
```

```rst
Section 9
=========

Added
-----
- Add mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Enable cppcheck
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
- Include necessary package and edit Threesome launch
- Open new folder for additional tracing contents
- Use manual deployment for now.
- Rename header files and correct format.
- Rename to smacc2 and smacc2_msgs
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Update name of package and package.xml to pass liter.
- Update table
- Update tracing/ManualTracing.md
- Update description table.
- Use docs/ as source folder for documentation
- Use docs/ as output directory.

Changed
-------
- Change extension of imports.
- Change extension
- Change wording "smacc application" to "SMACC2 library"
- Correct codespell.
- Correct formatting of python file.
- Correct formatters.
- Correct python linters warnings.
- Correct trailing spaces.
- Disable ament_cpplint.
- Disable cpplint and cppcheck linters.
- Disable disabled packages and update workflows.
- Disable some packages and update workflows.
- Ignore further packages
- Ignore all packages except smacc2 and smacc2_msgs
- Optimized deps in move_base_z_planners_common.
- Refine cp subscriber cp publisher
- Reactivate smacc2 nav clients for rolling via submodules
- Reformat sm_reference_library
- Remove example things from Foxy CI setup. (#214)
- Remove manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
- Reset all versions to 0.0.0
- Rename tracing events after
- Rename folders, delete tracing.md, edit README.md
- Rename smacc2_rta command across readmes
- Satisfy ament_lint_cmake
- Update changelogs
- Update GitHub branch reference.
- Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update smacc2_rta command across readmes
- Update tracing/ManualTracing.md
- Update tracing.md to reflect new tracing event names
- Update to master update
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs
- Update to smacc2 and smacc2_msgs

Fixed
-----
- Fix rolling builds (#222)
- Fix source CI and correct README overview. (#62)
- Minor broken build
- Minor changes
- Minor format
- Minor formatting fixes
- Minor linking errors foxy
- Remove tracing directory
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory
- Remove tracing directory

```rst
Section_10
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
- Sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Sm_multi_stage_1 (#90)
- More sm_multi_stage_1 (#91)
- Wait topic message client behavior (#81)
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Feature/wait nav2 nodes client behavior (#82)
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait
- Correct all linters and formaters (#82)
- Progress in AWS navigation demo
- Navigation parameters fixes on sm_dance_bot

Changed
-------
- Update README.md with updated launch command

Fixed
-----
- Attempting precommit fixes
- Fix pre-commit
- Trying to fix Pre-Commit

Removed
-------
- Minor format
- Merge and progress
- Fix format
```

```rst
Section_11
==========

Added
-----
- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2, waiting for nav2 nodes to subscribe to the /bond topic and ensuring they are alive. Optional selection of nodes to wait for.
- Base for the sm_aws_warehouse navigation.
- Gazebo fixes to show the robot and lidar.
- First working version of sm template and template generator.
- Waypoints navigator bug minor tuning to mitigate overshot issue cases.
- SM core test.

Changed
-------
- Progress in AWS navigation demo.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Progress in testing sm_dance_bot introducing slam pausing/resuming functionality.
- Polishing sm_dance_bot and s-pattern.
- More refinement in sm_dance_bot.

Fixed
-----
- Navigation parameters fixes on sm_dance_bot.
- Minor format issues.
- Minor hotfix.
- Minor navigation improvements.

Removed
-------
- Removed neo_simulation2 package.
- Removed sm_dance_bot_msgs.

Other
-----
- Several core improvements during navigation testing.
- Additional linting and formatting.
- Adjust build packages of source CI.
- Precommit cleanup run.
- Enable source build on PR for testing.
- Format improvements.
- Minor format adjustments.
- Minor tweaks.
- Format fixes.
- Pending references.
- Formatting.
- Cleaning files and making formatting work.
- More fixes.
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>.
```

```rst
Section_12
==========

Added
-----

- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SM warehouse 2 13 dec 2 (#182)
- Added CB pure spinning (#188)
- Added planner changes 16 12 (#191)
- Added replanning 16 dec (#193)
- Added undo motion 20 12 (#196)
- Added sync 21 12 (#199)
- Added warehouse2 22 12 (#200)
- Added warehouse2 23 12 (#201)
- Added minor tune (#203)

Changed
-------

- Updated package list (#142)
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments (#152)
- Fixed CI: format fix python version (#148)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Redoing sm_dance_bot_warehouse_3 waypoints (#184)
- Finetuning waypoints (#187)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)

Fixed
-----

- Noticed launch command was incorrect in README.md (#152)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed broken build
- Fixed formatting
- Fixed some linting warnings
- Fixed some errors introduced on formatting
- Fixed warehouse 3 problems and other core improvements (#204)
- Several fixes (#194)

Removed
-------

- Removed parameters smacc (#147)
- Removed node creation and create only a logger (#149)
- Removed test from main moveit cmake

Other
-----

- Precommit cleanup
- Workflows update
- Docker refactoring
- Progress on move_it PR
- Progress on moveit migration testing
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- More testing on moveit
- More testing on moveit behaviors
- More readme updates
- More changes and headless
- Merge
- Default values
- Pure spinning behavior missing files
- Warehouse2 progress
- Format
- Minor changes
- Minor configuration
- Minor dockerfile test workaround
- Minor tune
- Improving dockerfile for building local tests
- Undo tuning and errors
- Format issues
- Finishing warehouse2

Collaborators
-------------

- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_13
==========

Added
-----
- Added missing file from warehouse2 (#205).
- Added dockerfiles (#225).
- Added Feature/retry behavior warehouse 1 (#226).
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh to automate installation of ros-rolling-ros2trace.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------
- Updated SM template and made example code clearly visible.
- Updated templated to use Blackboard storage.
- Updated template to resolve the global data correctly.
- Updated sm_name.hpp.
- Changed wording "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax.
- Renamed tracing events after.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.

Fixed
-----
- Fixed code generators (#221).
- Fixed other build issues.
- Fixed trailing spaces.
- Corrected codespell.
- Corrected python linters warnings.
- Corrected formatters.
- Fixed bug in smacc2 component.

Removed
-------
- Removed use of node in the sm performance template.
- Removed manual installation of ros-rolling-ros2trace.

Other Changes
-------------
- Backported changes to foxy.
- Reordered and refined components for autoware demo.
- Improved smacc core by adding more components.
- Progress in autoware machine.
- Reactivated smacc2 nav clients for rolling via submodules.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Updated ci-build-source.yml.
- Updated tracing/ManualTracing.md.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Updated README tutorial for Dockerfile.
- Updated description table.
- Updated table.
- Copied initial docs.
- Moved tracing.md to tracing directory.
- Edited tracing.md to reflect new tracing event names.
- Created alternative ManualTracing.
- Opened new folder for additional tracing contents.
- Deleted tracing directory.
- Renamed folders, deleted tracing.md, edited README.md.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.

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
Section 14
==========

Added
-----

- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.

Changed
-------

- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Updated doxygen links (#70)
- Updated README.md launch command
- Corrected all linters and formatters.

Fixed
-----

- Fixed source CI and corrected README overview. (#62)
- Fixed trailing spaces.
- Fixed pre-commit issues.

Removed
-------

- Removed execution of clang-format on smacc2_sm_reference_library package.

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_15
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes subscribing to the /bond topic and wait for them to be alive. Optionally select the nodes to wait.

Changed
-------
- Navigation parameters fixes on sm_dance_bot.
- Gazebo fixes to show the robot and the lidar.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also smacc2::deep_history syntax.
- Going forward in testing sm_dance_bot introducing slam pausing/resuming functionality.
- Polishing sm_dance_bot and s-pattern.
- More refinement in sm_dance_bot.

Fixed
----
- Remove some compile warnings.
- Remove neo_simulation2 package.
- Correct formatting.
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.

Removed
-------
- Remove neo_simulation2 package.

Other
-----
- Several core improvements during navigation testing.
- Format improvements.
- Merge and progress.
- Precommit cleanup run.
- Minor hotfix.
- Minor tweaks.
- Additional linting and formatting.
- Remove merge markers from a python file.
- First working version of sm template and template generator.
- Progress in aws navigation demo.
- Base for the sm_aws_aarehouse navigation.
- Progressing in aws navigation.
- Navigation parameters fixes on sm_dance_bot.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
- Progress in aws navigation demo.
```

Section 16
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
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action msgs (#139)
- Resolve compile warnings (#137)
- Fix CI: format fix python version (#148)
- Fixing pipeline error
- Fixing broken master build
- Finetuning waypoints (#187)

Fixed
-----
- Fixing compiling issues
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Fixing some formatting and templating on SrConditional
- Several fixes (#194)

Removed
-------
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_17
==========

Added
-----
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Added missing file from warehouse2 (#205)
- Update file for fake hardware simulation and add file for gazebo simulation. (#224)
- Add ignition file and update repos files.
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
- Update ci-build-source.yml
- Change extension of imports.
- Enable cppcheck
- Included necessary package and edited Threesome launch
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml
- Use manual deployment for now.
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory.
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter.
- Execute on master update
- Reset all versions to 0.0.0
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Update description table.
- Update table
- Copy initial docs
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

Changed
-------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- Correct Focal-Rolling builds by fixing the version of rosdep yaml (#234)
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Rename header files and correct format.
- Minor formatting fixes
- Change extension
- Correct GitHub branch reference.
- Minor broken build
- Some reordering fixes
- Minor format fix
- Other minor changes
- Changed wording "smacc application" to "SMACC2 library"

Fixed
-----
- Fix broken source build (#227)
- Only rolling version should be pre-released on on master. (#230)
- Retry behavior warehouse 1
- Minor format
- Minor linking errors foxy
- Minor format
- Minor linking errors foxy

Removed
-------
- Disable disabled packages
- Disable some packages and update workflows.
- Ignore further packages
- Disable ament_cpplint.
- Disable cpplint and cppcheck linters.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Ignore further packages
- Disable ament_cpplint.
- Disable cpplint and cppcheck linters.
- Disable disabled packages
```


*pabloinigoblasco*

```rst
Section_18
==========

Added
-----

- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax.
- Reactivated smacc2 nav clients for rolling via submodules.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Optimized dependencies in move_base_z_planners_common.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- More Readme Updates (#72).
- Created new sm from sm_respira_1 (#76).
- Feature/core and navigation fixes (#78).
- Feature/aws demo progress (#80).
- Feature/wait nav2 nodes client behavior (#82).
- Feature/aws demo progress (#92).

Changed
-------

- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Corrected trailing spaces.
- Corrected all linters and formatters.

Fixed
-----

- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Minor formatting fixes.
- Several core improvements during navigation testing.
- Attempted precommit fixes.

Removed
-------

- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
```

```rst
Section_19
==========

Added
-----

- New feature: cb_wait_topic_message
  - Asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Added client behavior for nav2 to wait for nodes subscribing to the /bond topic and ensure they are alive

Changed
-------

- Navigation parameters fixes on sm_dance_bot
- Minor formatting improvements
- Progress in AWS navigation demo
- Merge and progress
- Formatting improvements for cb_pause_slam client behavior
- Cleaning and lidar show/hide option for sm_dance_bot visualizing turtlebot3
- Gazebo fixes to show the robot and lidar for various dance bot versions
- Got sm_multi_stage_1 working (barely) and other related improvements

Fixed
-----

- Removed some compile warnings
- Removed neo_simulation2 package
- Corrected formatting and enabled source build on PR for testing
- Removed merge markers from a Python file

Removed
-------

- Removed neo_simulation2 package

Collaborators
-------------

- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by pabloinigoblasco <pablo@ibrobotics.com>
```

Section 20
-----------

### Added
- Introduced `smacc2::deep_history` syntax.
- Added slam pausing/resuming functionality to `sm_dance_bot`.
- First working version of `sm` template and template generator.
- Added SM Atomic SM generator.
- Rolling Docker environment to be executed from any environment.
- Added QOS durability to `SmaccPublisherClient`.
- Added AWS navigation to `sm_dance_bot`.

### Changed
- Moved method after the method it calls to prevent recursion.
- Renamed `sm_advanced_recovery_1`.
- Reworked `sm_multi_stage_1` with multistage modes and sequences.
- Moved reference library SMs to `smacc2_performance_tools`.

### Fixed
- Resolved compile warnings.
- Fixed CI format for Python version.
- Fixed launch command in README.md.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed some formatting errors.

### Removed
- Removed `sm_dance_bot_msgs`.
- Removed parameters from `smacc`.
- Removed node creation and created only a logger.
- Removed test from main MoveIt CMake.

### Miscellaneous
- Minor format tweaks and improvements.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Updated package list.
- Updated README files.
- Precommit cleanup.
- Workflow updates.
- Added remaining SVGs to READMEs.
- Added dependencies for husky simulation.
- More progress on AWS navigation and other refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress.
- Waypoint inputs progress.
- More waypoints added to `sm_dance_bot_warehouse_3`.
- Redid `sm_dance_bot_warehouse_3` waypoints.
- More changes and headless updates to `sm_warehouse_2_13_dec_2`.
- Merged changes in `sm_warehouse_2_13_dec_2`.
- Default values added.
- SrConditional fixes and formatting.

Section_21
===========

Added
-----
- Feature/wharehouse2 dec 14 (#185): Added warehouse2 minor changes.
- Feature/sm warehouse 2 13 dec 2 (#186): Added format changes, more changes, headless merge, headless and other fixes, and default values.
- Feature/cb pure spinning (#188): Added format changes, more changes, headless merge, headless and other fixes, default values, and minor changes.
- Feature/cb pure spinning (#189): Added format changes, more changes, headless merge, headless and other fixes, default values, and minor changes.
- Feature/planner changes 16 12 (#191): Added minor changes, more fixes.
- Feature/replanning 16 dec (#193): Added minor changes, replanning for all examples, and several fixes.
- Feature/undo motion 20 12 (#196): Added minor changes, replanning for all examples, improving undo motion navigation in warehouse2, and tuning warehouse3.
- Feature/undo motion 20 12 (#198): Added minor changes, replanning for all examples, improving undo motion navigation in warehouse2, and undo tuning and errors.
- Feature/sync 21 12 (#199): Added minor changes, replanning for all examples, and format issues.
- Feature/warehouse2 22 12 (#200): Added minor changes, replanning for all examples, and format issues, finishing warehouse2.
- Feature/warehouse2 23 12 (#201): Added minor changes, replanning for all examples, tuning and fixes.
- Feature/minor tune (#203): Added tuning and fixes, and minor tune, fixing warehouse 3 problems, and other core improvements.
- Feature/barrel - do not merge yet (#233): Added minor changes, replanning for all examples, backport to foxy, minor format, and minor linking errors foxy.

Changed
-------
- Foxy backport (#206): Updated formatting, fixed trailing spaces, corrected codespell and python linters warnings, added galactic CI build due to Navigation2 issues in rolling, made partial changes for ament_cpplint, added tf2_ros as dependency, disabled ament_cpplint and some packages, bumped ccache version, satisfied ament_lint_cmake, added missing licenses, disabled cpplint and cppcheck linters, corrected formatters, updated ci-build-source.yml, changed extension of imports, enabled cppcheck, corrected formatting of python file, included necessary package and edited Threesome launch, renamed header files, added workflow for checking doc build, updated doxygen-check-build.yml, created doxygen-deploy.yml, created workflow for testing prerelease builds, updated package names, executed on master update, reset all versions to 0.0.0, ignored all packages except smacc2 and smacc2_msgs, updated changelogs, reverted previous commit, updated description table, updated table, copied initial docs, created Dockerfile with ROS distro as argument, opened new folder for additional tracing contents, deleted tracing directory, moved tracing.md to tracing directory, added setupTracing.sh.

Removed
-------
- Removed weird moveit not downloaded repo.

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

```rst
Section_22
==========

Added
-----
- Automated installation of ros-rolling-ros2trace in setupTracing.sh
- Alternative ManualTracing method
- New SM markdowns
- Dockerfile for Rolling and Galactic
- README tutorial for Dockerfile
- smacc2_performance_tools
- Performance tests improvements
- Updates in SMACC2/ROS2 mentions
- New feature: cb_wait_topic_message for asynchronous client behavior
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic

Changed
-------
- Wording changed from "smacc application" to "SMACC2 library"
- Renamed tracing events
- Renamed folders and files
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
- Cleaned up sm_reference_library and sm_atomic_24hr
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library

Fixed
----
- Bug in smacc2 component
- Trailing spaces corrected
- Build of missing rolling repositories enabled
- Navigation2 enabled for semi-binary build
- Corrected README overview
- All linters and formatters corrected

Removed
-------
- Manual installation of ros-rolling-ros2trace
- Galactic builds from master, keeping only rolling
- Submodules usage, now using .repos file
- Tracing.md file deleted

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_23
==========

Added
-----
- New feature: cb_wait_topic_message (#92)
  Asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: cb_wait_topic_message_add (#94)
  Waits for nav2 nodes subscribing to the /bond topic and ensures they are alive, with optional node selection.
- New client behavior: cb_pause_slam (#98)
  Introduces cb_pause_slam client behavior.
- New feature: sm_dance_bot visualizing turtlebot3 (#101)
  Includes lidar show/hide option and file cleaning.
- New feature: gazebo fixes for sm_dance_bot_strikes_back (#105)
  Enhancements for showing the robot and lidar in Gazebo.

Changed
-------
- Navigation parameters fixes on sm_dance_bot (#95)
  Corrected navigation parameters on sm_dance_bot.
- Minor hotfix in doxygen deployment workflow (#100)
  Fixed a minor issue in the doxygen deployment workflow.

Fixed
-----
- Removed some compile warnings (#96)
  Eliminated certain compile warnings.
- Removed neo_simulation2 package (#112)
  Deleted the neo_simulation2 package.
  Corrected formatting and enabled source build on PR for testing.

Collaborators
-------------
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai> in multiple entries.
```

Section_24
===========

Added
-----
- Adjusted build packages of source CI.
- Diverse improvements in navigation and performance.
- Added linting and formatting.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM Atomic SM generator.
- Rolling Docker environment to be executed from any environment.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added dependencies for husky simulation.
- Added dependencies for husky in rolling and galactic.
- Waypoint Inputs.

Changed
-------
- Moved reference library SMs to smacc2_performance_tools.
- Renamed husky launch file in sm_dance_bot.
- Updated dependencies for husky in rolling and galactic.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Progress in testing sm_dance_bot introducing slam pausing/resuming functionality.
- Progress in moveit migration testing.
- Progress in moveit.
- Progress in moveit behaviors testing.
- Progress on aws navigation and some other refactorings on navigation clients and behaviors.
- More on aws demo.
- Slight waypoint 4 and iterations changes so robot can complete the course.
- First working version of sm template and template generator.

Fixed
-----
- Remove merge markers from a python file.
- Move method after the method it calls to prevent recursion.
- Minor tuning to mitigate overshot issue cases.
- Fixed launch command for sm_dance_bot_strikes_back and removed some comments.
- Fix CI: format fix python version.
- Fixing broken master build.

Removed
-------
- Removed parameters smacc.
- Removed node creation and create only a logger.
- Removed sm_dance_bot_msgs.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored by: DecDury <declandury@gmail.com>
- Co-authored by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored by: Denis Štogl <denis@stogl.de>

```rst
Section_25
==========

Added
-----
- Added progress in warehouse2 (#179)
- Added feature for warehouse2 on December 13th (#182)
- Added Brettpac branch (#184)
- Added redoing waypoints for sm_dance_bot_warehouse_3
- Added more waypoints for sm_dance_bot_warehouse_3
- Added SrConditional fixes and formatting (#168)
- Added feature for warehouse2 on December 14th (#185)
- Added feature for warehouse2 on December 13th (#186)
- Added finetuning waypoints (#187)
- Added feature for cb pure spinning (#188)
- Added feature for cb pure spinning (#189)
- Added pure spinning behavior and minor changes (#190)
- Added feature for planner changes on December 16th (#191)
- Added feature for replanning on December 16th (#193)
- Added several fixes (#194)
- Added minor changes (#195)
- Added feature for undo motion on December 20th (#196)
- Added improving undo motion navigation for warehouse2
- Added tuning for warehouse3 (#197)
- Added feature for undo motion on December 20th (#198)
- Added improving undo motion navigation for warehouse2
- Added undo tuning and errors
- Added format for sync on December 21st (#199)
- Added format for warehouse2 on December 22nd (#200)
- Added finishing warehouse2
- Added format for warehouse2 on December 23rd (#201)
- Added minor tune for feature (#203)
- Added fixing warehouse 3 problems and other core improvements (#204)
- Added backport to foxy
- Added minor format and linking errors for foxy
- Added feature for docker improvements in March 2022 (#235)
- Added Foxy backport (#206)
- Added minor formatting fixes
- Added fix for trailing spaces, codespell, and python linters warnings
- Added galactic CI build due to Navigation2 issues in rolling
- Added partial changes for ament_cpplint
- Added tf2_ros as dependency
- Disabled ament_cpplint, some packages, and updated workflows
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

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch

Fixed
-----
- Fixed progress in autoware machine
- Fixed refining cp subscriber cp publisher
- Fixed improvements in smacc core
- Fixed autoware demo
- Fixed foxy CI
- Fixed minor broken build
- Fixed some reordering issues
- Fixed docker files for different revisions, warnings removal, and more testing on navigation
- Fixed docker for foxy and galactic
- Fixed barrel search build and warehouse3 startup problems
- Fixed format and minor issues
- Fixed progress in barrel husky
- Fixed barrel search updates
- Fixed making models local
- Fixed red pickup
- Fixed multiple controllable leds plugin
- Fixed progress in husky demo
- Fixed improving navigation behaviors
- Fixed more merge

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
Section_26
==========

Added
-----

- Workflow for testing prerelease builds.
- Use `docs/` as source and output directory.
- Renamed to `smacc2` and `smacc2_msgs`.
- Updated package name and `package.xml` for liter compatibility.
- Initial documentation setup.
- Dockerfile with ROS distro as argument.
- New folder for tracing contents.
- Added `setupTracing.sh` for automated installation.
- Created alternative `ManualTracing`.
- Added new SM markdowns.
- Added Dockerfile for Rolling and Galactic.
- Enable build of missing Rolling repositories.
- Enable Navigation2 for semi-binary build.
- Renamed folders, deleted `tracing.md`, edited `README.md`.
- Added `smacc2_performance_tools`.
- Performance tests improvements.
- Optimized dependencies in `move_base_z_planners_common`.
- Renamed event generator library.
- Added galactic CI setup and renamed Rolling files.
- Fixed source CI and corrected README overview.
- Updated `c_cpp_properties.json`.
- Changed launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.
- Updated doxygen links.
- More README updates.
- Created new SM from `sm_respira_1`.
- Progress in AWS navigation demo.
- Several core improvements during navigation testing.
- Format improvements.
- Feature `aws demo progress`.
- Reworked `sm_advanced_recovery_1`.
- Fixing pre-commit issues.
- More work on `sm_advanced_recovery_1`.
- Branch `Brettpac`.
- Added `sm_atomic_performance_test_a_2` and `sm_atomic_performance_test_a_1`.
- Added `sm_atomic_performance_test_c_1`.
- Modifying `sm_atomic_performance_test_a_2`.
- Added `sm_multi_stage_1`.
- Fixing precommit issues.
- Updated launch command in `README.md`.
- New feature `cb_wait_topic_message`: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Attempting precommit fixes.

Changed
-------

- Wording from "smacc application" to "SMACC2 library".
- Updated description table.
- Updated table.
- Reactivated `smacc2` nav clients for Rolling via submodules.
- Bug fix in `smacc2` component.
- Reverted markdowns to HTML.
- Edited `tracing.md` to reflect new tracing event names.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation Rolling.
- More changes on performance tests.
- `sm_reference_library` reformatting.
- Corrected trailing spaces.
- Cleaned up `sm_atomic_24hr`.
- More cleanup on `sm_atomic_24hr`.
- Minor formatting changes.

Removed
-------

- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`.

Fixed
-----

- Corrected GitHub branch reference.
- Do not execute `clang-format` on `smacc2_sm_reference_library` package.
```

```rst
Section_27
==========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: `wait nav2 nodes` subscribes to the `/bond` topic and waits for them to be alive, with optional node selection.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` updates.
- `sm_dance_bot` visualizes turtlebot3.
- Lidar show/hide option for `sm_dance_bot`.
- Gazebo fixes to show the robot and lidar for `sm_dance_bot` variations.
- `sm_multi_stage_1` doubling.
- `aws demo`.

Changed
-------

- Corrected all linters and formatters.
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Merge and progress in various features.
- Precommit cleanup run.
- Renamed doxygen deployment workflow.

Fixed
-----

- Removed some compile warnings.

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_28
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
- Added Fix CI: format fix python version (#148).
- Added Add SM Atomic SM generator. (#143).
- Added Remove node creation and create only a logger. (#149).
- Added Rolling Docker environment to be executed from any environment (#154).
- Added slight waypoint 4 and iterations changes so robot can complete course (#155).
- Added Feature/migration moveit client (#151).
- Added update readme (#164).
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
- Fixed typo in Feature/dance bot s pattern (#129).
- Refactored sm dance bot strikes back in Feature/sm dance bot strikes back refactoring (#152).
- Moved reference library SMs to smacc2_performance_tools in initial state machine transition timestamp (#165).
- Added QOS durability to SmaccPublisherClient in Add QOS durability to SmaccPublisherClient (#163).

Fixed
-----

- Fixed launch command for sm_dance_bot_strikes_back and removed some comments in README.md.
- Fixed compiling issues in Feature/migration moveit client (#151).
- Fixed broken master build in Feature/testing moveit behaviors (#167).

Removed
-------

- Removed neo_simulation2 package.
- Removed parameters smacc in removing parameters smacc.
- Removed sm_dance_bot_msgs in using local action msgs.

Authors
-------

- Pablo Iñigo Blasco (pabloinigoblasco)
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

Section_29
===========

Added
-----

- Introduce multistage modes and sequences:
  - sm_multi_stage sequences
  - sm_multi_state_1 steps
  - sm_multi_stage_1 sequence d
  - sm_multi_stage_1 c sequence
  - mode_5_sequence_b
  - mode_4_sequence_b
  - sm_multi_stage_1 most
  - finishing touches 1
  - readme

Changed
-------

- Enhance AWS navigation for sm_dance_bot (#174):
  - Add repo dependency
  - Include husky launch file in sm_dance_bot
  - Add dependencies for husky simulation
  - Update dependencies for husky in rolling and galactic
  - Progress on AWS navigation and refactorings on navigation clients and behaviors
  - More on AWS demo
  - Fix broken build

Fixed
-----

- Resolve minor issues in warehouse2 (#177)
- Address waypoint inputs (#178)
- Improve warehouse2 progress (#179)
- Refine formatting (#180)
- Develop sm_dance_bot_warehouse_3 (#181)
- Update sm warehouse 2 on 13th Dec (#182)
- Fine-tune waypoints (#187)
- Fix pure spinning behavior and missing files (#188)
- Implement planner changes on 16th Dec (#191)
- Enhance replanning for all examples (#193)
- Fix errors and tune undo motion on 20th Dec (#196)
- Tune warehouse3 (#197)
- Fix sync issues on 21st Dec (#199)
- Address format issues on warehouse2 on 22nd Dec (#200)
- Finish warehouse2 on 23rd Dec (#201)
- Minor tune on feature (#203)
- Fix warehouse 3 problems and core improvements (#204)
- Add missing file from warehouse2 (#205)
- Backport to foxy and fix minor format issues

Removed
-------

- Remove redundant entries and reordering fixes

Contributors
------------

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

```rst
Section_30
==========

Added
-----
- Added cache matrix rolling and source build package.
- Added significant update in Getting Started Instructions.

Changed
-------
- Updated foxy-source-build.yml.
- Improved husky project build on rolling.
- Updated type string walker threesome demo.

Fixed
----
- Fixed building issue.
- Fixed broken build.
- Fixed checkout branches for scheduled builds.
- Fixed initializing conditionFlag as false.
- Fixed precommit issue.

Removed
-------
- Removed trailing spaces.

Other
-----
- Restored workflow files (#252).
- Restored files (#253).
- Restored files.
- Progress on the sm_husky_barrel.
- More on husky demo for galactic.
- Ignored packages which should not be released (reverted).

Contributors
------------
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

0.3.0 (2022-04-04)
------------------

### Added
- More progress in humble SMACC2 deb generation
- Humble check
- Publisher

### Changed
- Progress in migration to humble

### Fixed
- Bug fix galactic mutex
- Testing undo motion and improving action client
- Important refactoring smacc action client
- Fix
- Progress in smacc action client
- Progress in the smacc action client fork based on signals
- More changes
- More testing
- More testing on abort
- Minor
- Adding smaccServiceerver client to galactic
- Update cb_default_keyboard_behavior.hpp
- Testing more husky robot
- Progress in tests husky demo
- Testing abort forward and undo
- Finishing cancel and undo behavior tests
- Undo motion in stEvasion after detecting enemy - in testing
- Minor format
- Putting in green SMACC2
- Many improvements in action client and cb sequence for husky barrel search
- More and better navigation behaviors on husky barrel search demo
- Functionality improvements in navigation and improvements of warehouse 3
- Warehouse 3 improvements
- Merge galactic
- Merge fix
- Minor
- Dead branch for husky barrel sm and opencv functionalities
- Final s-pattern and final attack
- Refining final attack state and also retry states for s-pattern
- Fixing PR green
- Minor changes
- Different planners profiles for navigation
- Getting changes from galactic
- Planner switcher
- Using galactic branch files
- Fixing breaking changes
- Minor fix
- Removing nav from source files
- Merge
- Rename to smacc2 and smacc2_msgs
- Correct GitHub branch reference
- Update name of package and package.xml to pass liter
- Execute on master update
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Installs necessary packages and configures tracing group
- Removed manual installation of ros-rolling-ros2trace
- This is now automated in setupTracing.sh
- Location of sh file assumed if user follows README.md under "Getting started"
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Update tracing/ManualTracing.md
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Changed wording "smacc application" to "SMACC2 library"
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Update smacc_sm_reference_library/sm_atomic/README.md
- Edit from html to markdown syntax
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Reactivating smacc2 nav clients for rolling via submodules
- Renamed tracing events after
- Bug in smacc2 component
- Reverted markdowns to html
- Added README tutorial for Dockerfile
- Additional cleanup
- Cleanup
- Edited tracing.md to reflect new tracing event names
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling
- Remove submodules and use .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Some progress on navigation rolling
- Renamed folders, deleted tracing.md, edited README.md
- Added smacc2_performance_tools
- Performance tests improvements
- More on performance and other issues
- Sm_respira_1 format cleanup
- Sm_respira_1 format cleanup pre-commit
- Sm_respira_test_2
- More changes on performance tests
- Do not execute clang-format on smacc2_sm_reference_library package
- Sm_reference_library reformatting
- Correct trailing spaces
- Sm_atomic_24hr
- Sm_atomic_performance_trace_1
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- More sm_atomic_24hr cleanup
- Optimized deps in move_base_z_planners_common
- Renaming of event generator library
- Minor formatting
- Add galactic CI setup and rename rolling files
- Fix source CI and correct README overview
- Update c_cpp_properties.json
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch
- Also noticed a note I had made while producing these that was not removed
- Update doxygen links
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More Readme Updates
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More Readme

### Removed
- Removed manual installation of ros-rolling-ros2trace (now automated in setupTracing.sh)

```rst
Section_32
==========

Added
-----

- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 work (#85)
- sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- sm_atomic_performance_test_a_2
- sm_atomic_performance_test_a_1
- sm_atomic_performance_test_c_1 (#88)
- modifying sm_atomic_performance_test_a_2 (#89)
- sm_multi_stage_1 (#90)
- Update README.md with updated launch command
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Changed
-------

- Several core improvements during navigation testing
- Progress in aws navigation
- Formatting improvements
- Progress in aws navigation demo
- Corrected all linters and formatters

Fixed
-----

- Fix pre-commit
- Trying to fix Pre-Commit
- Navigation parameters fixes on sm_dance_bot

Removed
-------

- Minor format

Collaborators
-------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_33
==========

Added
-----
- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are alive. Optional node selection available.
- New feature: cb_wait_topic_message - Asynchronous client behavior that waits for a topic message and optionally checks its contents for success.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Base for the sm_aws_warehouse navigation.
- Several core improvements during navigation testing.
- Gazebo fixes to show the robot and the lidar.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Smacc2::deep_history syntax introduced.

Fixed
-----
- Remove some compile warnings. (#96)
- Minor hotfix.
- Minor navigation improvements.
- Minor tuning to mitigate overshot issue cases.
- Minor format issues.

Removed
-------
- Remove neo_simulation2 package.
- Removing sm_dance_bot_msgs.
- Removing parameters smacc.

Other
-----
- Format improvements.
- Precommit cleanup.
- Updates yaml.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Move method after the method it calls to prevent recursion.
- First working version of sm template and template generator.
- Pending references resolved.
``` 

*pabloinigoblasco*

```rst
Section_34
==========

Added
-----
- Add SM Atomic SM generator. (#143)
- Add QOS durability to SmaccPublisherClient (#163)

Changed
-------
- Update dependencies for husky in rolling and galactic.
- Progress on moveit migration testing.
- Finishing warehouse2.
- Several fixes on replanning for all our examples.

Fixed
-----
- Fix launch command for sm_dance_bot_strikes_back in README.md.
- Fix CI: format fix python version (#148).
- Fix compiling issues.
- Fix broken master build.
- Fix formatting in various files.
- Fix some linting warnings.
- Fix pipeline error.
- Fix broken build.
- Fix some formatting and templating on SrConditional.
- Fix some errors introduced on formatting.
- Fix some build errors.
- Fix warehouse 3 problems and other core improvements to remove dead lock.

Removed
-------
- Remove node creation and create only a logger. (#149)
- Remove test from main moveit cmake.
- Remove parameters smacc.
- Remove some comments in the past.

Other Changes
-------------
- Workflows update.
- Rolling Docker environment to be executed from any environment (#154).
- Docker refactoring.
- Pre-commit cleanup.
- Minor configuration changes.
- Minor dockerfile test workaround.
- Improving dockerfile for building local tests.
- Update readme (#164).
- Update readme.
- More readme updates.
- Initial state machine transition timestamp (#165).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Add reliability qos config.
- More testing on moveit behaviors.
- More on aws demo.
- More waypoints added.
- Redoing sm_dance_bot_warehouse_3 waypoints.
- More changes and headless.
- Default values adjustments.
- Pure spinning behavior missing files.
- Tuning warehouse3.
- Undo tuning and errors.
- Format issues adjustments.
- Weird moveit not downloaded repo.
- Added missing file from warehouse2 (#205).
- Update cb_navigate_global_position.hpp.

Collaborators
-------------
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
``` 

*pabloinigoblasco*

```rst
Section_35
==========

Added
-----
- Merging code from backport foxy and updates about autoware (#208)
- Replanning for all examples
- Add galactic CI build because Navigation2 is broken in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency to find include
- Add missing licences
- Add workflow for checking doc build
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use manual deployment for now
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- added setupTracing.sh
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- updated mentions of SMACC/ROS to SMACC2/ROS2
- added smacc2_performance_tools
- performance tests improvements
- more on performance and other issues
- sm_respira_1 format cleanup
- sm_respira_test_2
- Do not execute clang-format on smacc2_sm_reference_library package
- sm_reference_library reformatting
- Optimized deps in move_base_z_planners_common
- Renaming of event generator library
- Add galactic CI setup and rename rolling files (#58)
- Fix source CI and correct README overview (#62)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- sm_advanced_recovery_1 reworked (#83)

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- First ensure you have the necessary package installed
- Rename header files and correct format
- Change extension of imports
- Correct GitHub branch reference
- Update name of package and package.xml to pass liter
- Reset all versions to 0.0.0
- Update description table
- Update table
- Copy initial docs
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- Update c_cpp_properties.json
- changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- update doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)
- created new sm from sm_respira_1 (#76)
- base for the sm_aws_aarehouse navigation
- several core improvements during navigation testing
- progress in aws navigation demo
- more on navigation
- sm_advanced_recovery_1 reworked

Fixed
-----
- minor formatting fixes
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- minor linking errors foxy
- minor format
- Correct formatters
- bug in smacc2 component
- reverted markdowns to html
- cleanup

Removed
-------
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cpplint and cppcheck linters
- Disable disabled packages
- Ignore further packages
- Ignore all packages except smacc2 and smacc2_msgs
- Delete tracing directory
- Removed manual installation of ros-rolling-ros2trace
```

*pabloinigoblasco*

```rst
Section_36
==========

Added
-----
- More work on sm_advanced_recovery_1 (#85, #86, #87)
- Added sm_atomic_performance_test_a_1 and sm_atomic_performance_test_a_2
- Added sm_atomic_performance_test_c_1 (#88)
- Added sm_multi_stage_1 (#90, #91)
- New feature: cb_wait_topic_message for asynchronous client behavior (#81)
- Added new client behavior for nav2: wait nav2 nodes subscribing to the /bond topic (#82)
- Added new feature for sm_aws_aarehouse navigation (#92, #94)
- Added navigation parameters fixes on sm_dance_bot (#93, #95)
- Added new feature: cb_pause_slam (#98)

Changed
-------
- Updated launch command in README.md
- Corrected all linters and formatters

Fixed
-----
- Fixed compile warnings (#96)
```

Section_37
==========

Added
-----
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Added gazebo fixes to show the robot and the lidar.
- Added sm_multi_stage_1 doubling feature.
- Added gazebo fixes for sm_dance_bot_strikes_back.
- Added AWS demo feature.
- Added Brettpac branch feature.
- Added a3 feature.
- Added diverse improvements in navigation and performance.
- Added slam toggle and smacc deep history feature.
- Added slam pausing/resuming functionality to sm_dance_bot.
- Added s-pattern feature to sm_dance_bot.
- Added first working version of sm template and template generator.
- Added waypoints navigator bug fix.
- Added SM core test.
- Added minor navigation improvements.
- Added rolling Docker environment to be executed from any environment.
- Added slight waypoint 4 and iterations changes for robot course completion.
- Added initial migration to smacc2 for moveit client.

Changed
-------
- Renamed doxygen deployment workflow.
- Updated yaml.
- Corrected formatting.
- Adjusted build packages of source CI.
- Enabled source build on PR for testing.
- Removed neo_simulation2 package.
- Removed merge markers from a python file.
- Removed parameters smacc.
- Fixed launch command for sm_dance_bot_strikes_back in README.md.

Fixed
-----
- Minor format fixes.
- Format fixes.
- Precommit cleanup.
- Fixed recursion possibility by moving method after the method it calls.
- Mitigated overshot issue cases in minor tuning.
- Fixed minor format issues.

Removed
-------
- Removed sm_dance_bot_msgs.
- Removed test from main moveit cmake.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_38
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
- Added redoing sm_dance_bot_warehouse_3 waypoints.
- Added more Waypoints.
- Added finetuning waypoints.
- Added pure spinning behavior missing files.
- Added replanning for all examples.
- Added improving undo motion navigation warehouse2.
- Added tuning and fixes.
- Added fixing warehouse 3 problems and other core improvements to remove deadlocks, also making continuous integration green.
- Added backport to foxy.

Changed
-------
- Updated format in various places.
- Refactored docker.
- Improved dockerfile for building local tests.
- Updated readme.
- Moved reference library SMs to smacc2_performance_tools.
- Reworked sm_multi_stage_1.
- Renamed sm_advanced_recovery_1.
- Updated dependencies for husky in rolling and galactic.
- Updated subscriber publisher components.
- Refining cp subscriber cp publisher.
- Improved smacc core by adding more components mostly developed for autoware demo.
- Updated autoware demo.
- Updated foxy ci.

Fixed
----
- Fixed compiling issues.
- Fixed pipeline error.
- Fixed broken master build.
- Fixed formatting.
- Fixed broken build.
- Fixed some formatting and templating on SrConditional.
- Fixed lint.
- Fixed move trigger logic into headers.
- Fixed minor linking errors in foxy.

Removed
------
- Removed a missing colon.
- Removed a line.

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

Section_39
==========

Added
-----
- Add mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies.
- Add galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Add missing licences.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Create workflow for testing prerelease builds.
- First ensure you have the necessary package installed.
- Include necessary package and edited Threesome launch.
- Open new folder for additional tracing contents.
- Rename header files and correct format.
- Update doxygen-check-build.yml.
- Update name of package and package.xml to pass liter.
- Update table.
- Update ci-build-source.yml.
- Update changelogs.
- Update description table.
- Update tracing/ManualTracing.md.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Use manual deployment for now.

Changed
-------
- Change extension of imports.
- Change extension.
- Changed wording "smacc application" to "SMACC2 library".
- Reactivating smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Renaming of event generator library.
- Reset all versions to 0.0.0.
- Several core improvements during navigation testing.

Fixed
-----
- Fix rolling builds.
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Correct formatting of python file.
- Correct formatting.
- Correct trailing spaces.
- Enable cppcheck.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Optimized deps in move_base_z_planners_common.
- Satisfy ament_lint_cmake.

Removed
-------
- Remove example things from Foxy CI setup.
- Remove manual installation of ros-rolling-ros2trace.
- Disable ament_cpplint.
- Disable cpplint and cppcheck linters.
- Disable some packages and update workflows.
- Disable disabled packages.
- Ignore further packages.
- Ignore all packages except smacc2 and smacc2_msgs.

Co-Authored-By
--------------
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>

```rst
Section_40
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Nodes to wait can be optionally selected.
- Merge and progress.

Changed
-------
- Navigation parameters fixes on sm_dance_bot.

Fixed
-----
- Fix pre-commit.
- Correct all linters and formatters.
- Remove some compile warnings.

Removed
-------
- Trying to fix Pre-Commit.

Contributors
------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

```rst
Section_41
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior `add` for nav2: waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive; optional node selection

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Introducing slam pausing/resuming functionality in testing `sm_dance_bot`
- Polishing `sm_dance_bot` and `s-pattern`
- Minor tuning to mitigate overshot issue cases in `waypoints navigator`
- Minor navigation improvements
- Using local action messages
- Renaming navigation 2 stack to `nav2z`
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others
- Removed node creation and created only a logger

Fixed
-----
- Move method after the method it calls to prevent recursion
- Format fix for Python version in CI
- Fixed launch command for `sm_dance_bot_strikes_back` in README.md

Removed
-------
- Removed `neo_simulation2` package
- Removed `sm_dance_bot_msgs` package
- Removed parameters in `smacc`

Other
-----
- Various core improvements during navigation testing
- Formatting improvements
- Precommit cleanup
- Adjusted build packages of source CI
- Noticed merge markers from a Python file and removed them
- Noticed launch command was incorrect in README.md and fixed it
- Cleanup of precommit
- Update package list
- Workflow updates
- Added remaining SVGs to READMEs
- Removed some comments in the past
``` 

**Autor:** Pablo Iñigo Blasco (pabloinigoblasco)

Section_42
===========

Added
-----
- Rolling Docker environment to be executed from any environment (#154)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing errors introduced on formatting
  - Adding missing dependency
  - Fixing linting warnings
  - Removing test from main moveit cmake
  - Progressing in the moveit migration testing
  - Updating format
  - Adding .reps dependencies and fixing build errors
  - Adding dependency to ur5 client
- Feature/aws navigation sm dance bot (#174)
  - Husky launch file in sm_dance_bot
  - Adding dependencies for husky simulation
  - Updating dependencies for husky in rolling and galactic
  - Progress on aws navigation and refactorings on navigation clients and behaviors
  - More on aws demo
- Feature/warehouse2 22 12 (#200)
  - Replanning for all examples
  - Fixing format issues
  - Finishing warehouse2

Changed
-------
- Moved reference library SMs to smacc2_performance_tools (#166)
- Add QOS durability to SmaccPublisherClient (#163)
  - Adding QOS durability to SmaccPublisherClient
  - Fixing missing colon
  - Removing line
  - Adding reliability QOS config
- Finetuning waypoints (#187)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Minor tune (#203)

Fixed
-----
- Update readme (#164)
  - More readme updates
- Fixing compiling issues
- Fixing pipeline error
- Fixing broken master build
- Fixing broken build
- Several fixes (#194)
- Undo tuning and errors
- Weird moveit not downloaded repo
- Added missing file from warehouse2 (#205)
- Backport to foxy
- Minor format
- Minor linking errors foxy

Removed
-------
- Removing test from main moveit cmake

Authors
-------
- Pablo Iñigo Blasco
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>

```rst
Section_43
==========

Added:
-------
- Feature/retry behavior warehouse 1 (#226)
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
- 0.1.0
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Update description table.
- Update table
- Copy initial docs
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
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh

Changed:
--------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- Changed wording "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax
- updated mentions of SMACC/ROS to SMACC2/ROS2

Fixed:
------
- Fix code generators (#221)
- Fix other build issues.
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
- Do not execute clang-format on smacc2_sm_reference_library package.
- sm_reference_library reformatting
- Correct trailing spaces.
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- more sm_atomic_24hr cleanup
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library
- minor formatting

Removed:
--------
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
- fixing docker for foxy and galactic
- renamed tracing events after
- bug in smacc2 component
- reverted markdowns to html
- added README tutorial for Dockerfile
- additional cleanup
- cleanup
- cleanup
- edited tracing.md to reflect new tracing event names
- more on performance and other issues
- sm_respira_1 format cleanup
- sm_respira_1 format cleanup pre-commit
- sm_respira_test_2
- sm_respira_test_2
- more changes on performance tests
- sm_atomic_24hr
- sm_atomic_performance_trace_1
- more sm_atomic_24hr cleanup
- minor formatting

Authors:
--------
- David Revay <MrBlenny@users.noreply.github.com>
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
```

```rst
Section_44
==========

Added
-----
- Galactic CI setup and renamed rolling files. (#58)
- More README updates (#72)
- More README updates (#74)
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Wait topic message client behavior (#81)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)

Changed
-------
- Updated c_cpp_properties.json launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Updated README.md launch command

Fixed
-----
- Fixed source CI and corrected README overview. (#62)
- Update doxygen links (#70)
- Fixed pre-commit
- Corrected all linters and formatters

Removed
-------
- Removed note not removed while producing changes

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_45
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components.
- Introducing `smacc2::deep_history` syntax in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Polishing `sm_dance_bot` and `s-pattern`.
- Minor tuning to mitigate overshot issue cases in `waypoints navigator`.

Fixed
-----
- Remove some compile warnings. (#96)
- Correct formatting in removing `neo_simulation2` package.
- Adjust build packages of source CI.
- Resolve compile warnings.
- Minor format issues.

Removed
-------
- Removing `sm_dance_bot_msgs`.

Other
-----
- Precommit cleanup run.
- Updates `yaml`.
- Enable source build on PR for testing.
- More on `sm_multi_stage_1`.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Progress in AWS navigation.
- Progress in navigation testing.
- Formatting improvements.
- More on navigation.
- Formatting improvements.
- More fixes in `sm_dance_bot`.
- Cleaning and lidar show/hide option.
- Cleaning files and making formatting work.
- More fixes in gazebo to show the robot and the lidar.
- Format fixes.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Progress in `sm_multi_stage_1`.
- Progress in AWS navigation demo.
- Progress in AWS navigation.
- Progress in navigation testing.
- Formatting improvements.
- More on navigation.
- Formatting improvements.
- More fixes in `sm_dance_bot`.
- Cleaning and lidar show/hide option.
- Cleaning files and making formatting work.
- More fixes in gazebo to show the robot and the lidar.
- Format fixes.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components.
- Introducing `smacc2::deep_history` syntax in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Polishing `sm_dance_bot` and `s-pattern`.
- Noticed typo.
- First working version of `sm` template and template generator.
- Minor tweaks.
- Build fix.
- Some more progress on markers cleanup.
- Minor format issues.
- Minor navigation improvements.
- Using local action messages.
- Using local action messages.
```

```rst
Section_46
==========

Added
-----
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Add QOS durability to SmaccPublisherClient (#163)

Changed
-------
- Renamed Feature/nav2z to navigation 2 stack (#144)
- Updated package list (#142)
- Refactored Feature/sm dance bot strikes back (#152)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Reworked sm_multi_stage_1 (#172)
- Progress on AWS navigation and refactorings on navigation clients and behaviors (#174)
- Finetuned waypoints (#187)

Fixed
-----
- Fixed launch command in README.md (#148)
- Fixed CI format for python version (#148)
- Fixed compiling issues (#164)
- Fixed broken master build (#174)
- Fixed pipeline error (#174)
- Fixed broken build in AWS demo (#174)
- Fixed formatting and templating on SrConditional (#168)
- Fixed some formatting and templating on SrConditional (#168)
- Fixed linting issues (#168)

Removed
-------
- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed test from main moveit cmake

Other Changes
-------------
- Updated Docker environment to be executed from any environment (#154)
- Added missing dependency to ur5 client
- Improved Dockerfile for building local tests
- Added .reps dependencies and fixed build errors
- Updated format in moveit migration testing
- Added reliability qos config
- Added mode_5_sequence_b, mode_4_sequence_b, and other sequences in sm_multi_stage_1
- Added husky launch file in sm_dance_bot
- Updated dependencies for husky in rolling and galactic
- Redid waypoints in sm_dance_bot_warehouse_3
- Added more waypoints in sm_dance_bot_warehouse_3
- Moved trigger logic into headers
- Added pure spinning behavior missing files
- Improved undo motion navigation in warehouse2
- Tuned warehouse3
- Fixed several issues in replanning for all examples
- Finished warehouse2
``` 

*pabloinigoblasco*

Section_47
==========

Added
-----
- Added missing file from warehouse2 (#205)
- Added ignition file and update repos files
- Added setupTracing.sh to install necessary packages and configure tracing group
- Added a README tutorial for Dockerfile

Changed
-------
- Changed wording "smacc application" to "SMACC2 library"
- Changed...
  ```
  ros2 launch sm_three_some sm_three_some
  ```
  to
  ```
  ros2 launch sm_three_some sm_three_some.launch
  ```
- Renamed tracing events after
- Reactivated smacc2 nav clients for rolling via submodules

Fixed
-----
- Fixed warehouse 3 problems and other core improvements to remove deadlocks, also making continuous integration green
- Fixed broken source build (#227)
- Fixed trailing spaces
- Fixed codespell
- Fixed python linters warnings
- Fixed docker for foxy and galactic
- Fixed minor broken build
- Fixed some reordering fixes
- Fixed minor format
- Fixed minor format fix
- Fixed minor linking errors in foxy
- Fixed missing file
- Fixed minor broken build
- Fixed minor format fix
- Fixed other minor changes
- Fixed missing rolling repositories

Removed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh

Other
-----
- Tuning and fixes (#202)
- Feature/minor tune (#203)
- Minor tune
- Weird moveit not downloaded repo
- Updating subscriber publisher components
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Foxy CI
- Some reordering fixes
- Docker files for different revisions, warnings removal, and more testing on navigation
- Update file for fake hardware simulation and add file for gazebo simulation
- Docker build files for all versions
- Retry behavior warehouse 1
- Progress in autoware machine
- Branching example
- Update ci-build-source.yml
- Change extension of imports
- Enable cppcheck
- Correct formatting of python file
- Included necessary package and edited Threesome launch
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
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
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
  Use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Update tracing/ManualTracing.md
- Reverted markdowns to html
- Additional cleanup
- Cleanup

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
Section_48
==========

Added
-----

- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added galactic CI setup and rename rolling files. (#58)
- More Readme Updates (#72) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- More Readme (#74) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. you optionally can select the nodes to wait

Changed
-------

- updated mentions of SMACC/ROS to SMACC2/ROS2
- Update smacc2_rta command across readmes
- changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Update README.md
- updated launch command
- Correct all linters and formaters.

Fixed
-----

- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Fix source CI and correct README overview. (#62)
- Attempting precommit fixes

Removed
-------

- deleted tracing.md
- removed sm_respira_1 format cleanup
- removed sm_respira_1 format cleanup pre-commit
- removed more changes on performance tests
- removed sm_reference_library reformatting
- removed Clean up of sm_atomic_24hr
- removed more sm_atomic_24hr cleanup
- removed Renaming of event generator library
- removed minor formatting
- removed Optimized deps in move_base_z_planners_common
- removed several core improvements during navigation testing
- removed formatting improvements
- removed progress in aws navigation demo
- removed minor format
```

Section_49
===========

Added
-----
- New client behavior `cb_wait_topic_message` added for nav2. It allows waiting for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Users can optionally select the nodes to wait for.
- New feature: `cb_pause_slam` client behavior added.
- New feature: `sm_dance_bot_lite` introduced for visualizing TurtleBot3.
- New feature: `sm_multi_stage_1` doubling functionality added.
- New feature: `sm_dance_bot_strikes_back` gazebo fixes implemented.
- AWS demo progress made.

Changed
-------
- Navigation parameters fixes applied to `sm_dance_bot`.
- Formatting improvements made throughout the codebase.
- Gazebo fixes implemented to show the robot and lidar.

Fixed
-----
- Compile warnings removed.
- Recursion issue addressed by moving a method after the one it calls.

Removed
-------
- `neo_simulation2` package removed.

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_50
==========

Added
-----

- First working version of sm template and template generator. (#127)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)
- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added dependencies for husky in rolling and galactic in AWS navigation (#174)
- Added Waypoint Inputs (#178)

Changed
-------

- Polished sm_dance_bot and s-pattern
- Refactored sm_dance_bot strikes back (#152)
- Reworked sm_multi_stage_1 (#172)
- Renamed sm_advanced_recovery_1 (#171)
- Renamed sm_multi_stage_1 (#172)
- Renamed sm_dance_bot_warehouse_3 (#181)
- Renamed warehouse2 (#177)
- Renamed sm_pubsub_1 (#169)
- Renamed sm_pubsub_1 part 2 (#170)
- Renamed sm_multi_stage_1 (#172)
- Renamed sm_dance_bot_warehouse_3 (#181)
- Renamed SrConditional (#168)
- Renamed warehouse2 (#177)
- Renamed sm_dance_bot_warehouse_3 (#181)

Fixed
-----

- Fixed a typo (Finnaly > Finally)
- Fixed launch command in README.md
- Fixed CI format for Python version (#148)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error

Removed
-------

- Removed sm_dance_bot_msgs
- Removed parameters smacc
- Removed node creation and create only a logger (#149)
- Removed test from main moveit cmake
- Removed some comments in the past

Other
-----

- More changes in sm_dance_bot
- More refinement in sm_dance_bot
- More progress in the sm_dance_bot tests (#135)
- More progress on markers cleanup
- More testing on moveit behaviors (#167)
- More testing on moveit
- More on AWS demo
- More readme updates
- More waypoints in sm_dance_bot_warehouse_3
- More changes and headless in various features
- Minor tweaks (#130)
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Minor configuration
- Minor tuning to mitigate overshot issue cases
- Minor dockerfile test workaround
- Minor improvements in various features
- Minor changes in multiple features
- Progress on move_it PR
- Progress on moveit migration testing
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- Progress on moveit
- Progress on moveit behaviors
- Progress on sm_dance_bot_warehouse_3
- Progress on warehouse2
- Progress on sm_dance_bot tests
- Progress on sm_dance_bot strikes back
- Progress on sm_multi_stage_1
- Progress on sm_pubsub_1
- Progress on sm_advanced_recovery_1
- Progress on cb pure spinning
- Progress on various features
- Pending references
- Pre-commit cleanup
- Rolling Docker environment to be executed from any environment (#154)
- Update package list (#142)
- Update readme (#164)
- Update readme
- Update format
- Update dependencies for husky in rolling and galactic
- Update format
- Update package list
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
- Update format
- Update readme
-

```rst
Section_51
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
- Feature/barrel - do not merge yet (#233)
- Foxy backport (#206)
- Added galactic CI build due to Navigation2 issues in rolling
- Added partial changes for ament_cpplint
- Added tf2_ros as dependency
- Added workflow for checking doc build
- Created doxygen-deploy.yml
- Created workflow for testing prerelease builds
- Use docs/ as source and output directory
- Renamed to smacc2 and smacc2_msgs
- Updated name of package and package.xml
- Updated changelogs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Added setupTracing.sh to automate ros-rolling-ros2trace installation
- Created alternative ManualTracing
- Added dockerfile for Rolling and Galactic
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh

Changed
-------
- Updated subscriber publisher components
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components
- Progress in barrel husky

Fixed
-----
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Fixing broken build

Removed
-------
- Removed manual installation of ros-rolling-ros2trace

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
Section_52
==========

Added
-----
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.

Fixed
----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Correct all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.

Other
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Some progress on navigation rolling.
- Performance tests improvements.
- More on performance and other issues.
- SM_RESPIRA_1 format cleanup.
- SM_RESPIRA_TEST_2.
- More changes on performance tests.
- SM_REFERENCE_LIBRARY reformatting.
- SM_ATOMIC_24HR.
- SM_ATOMIC_PERFORMANCE_TRACE_1.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- More sm_atomic_24hr cleanup.
- Minor formatting.
- Minor improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- Format improvements.
- More on navigation.
- Several core improvements during navigation testing.
- Attempting pre-commit fixes.
- Fixing precommit.
- Correct README overview.
- Updated launch command.
- Update doxygen links.
- Update c_cpp_properties.json.
- Update README.md.
``` 

*pabloinigoblasco*

```rst
Section_53
==========

Added
-----
- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and confirm they are active. Optional node selection available.
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Merge and progress.
- Minor hotfix.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.
- Format fixes.

Fixed
-----
- Remove some compile warnings.
- Format fixes.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Several core improvements during navigation testing.
- Formatting improvements.
- Precommit cleanup run.
- Updates yaml.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Diverse improvements in navigation and performance.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_54
==========

Added
-----
- Additional linting and formatting.
- Introduced slam toggle and smacc deep history features (#122).
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Testing progress for sm_dance_bot with slam pausing/resuming functionality.
- Introduced dance bot s pattern feature (#128).
- First working version of sm template and template generator (#127).
- Added SM Atomic SM generator (#143).
- Rolling Docker environment to be executed from any environment (#154).
- Added QOS durability to SmaccPublisherClient (#163).
- Introduced husky launch file in sm_dance_bot for AWS navigation (#174).
- Added Waypoint Inputs (#178).

Changed
-------
- Moved method after the method it calls to prevent recursion (#126).
- Renamed reference library SMs to smacc2_performance_tools (#166).
- Refactored sm dance bot strikes back (#152).
- Renamed sm_advanced_recovery_1 (#171).
- Reworked sm_multi_stage_1 with multistage modes and sequences.
- Updated dependencies for husky in rolling and galactic for AWS navigation (#174).

Fixed
-----
- Resolved compile warnings (#137).
- Fixed CI format for Python version (#148).
- Fixed launch command in README.md.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.

Removed
-------
- Removed merge markers from a Python file (#119).
- Removed node creation and created only a logger (#149).
- Removed parameters smacc (#147).
- Removed test from main moveit CMake.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_55
==========

Added
-----
- Feature/wharehouse2 dec 14 (#185)
  - Added warehouse2 feature with minor changes.

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added warehouse2 feature with formatting improvements and default values.

- Feature/cb pure spinning (#188)
  - Implemented pure spinning behavior with format changes and default values.

- Feature/replanning 16 dec (#193)
  - Implemented replanning for all examples with several fixes.

- Feature/undo motion 20 12 (#196)
  - Improved undo motion navigation in warehouse2 with minor changes.

- Feature/sync 21 12 (#199)
  - Implemented sync feature with replanning and format fixes.

- Feature/warehouse2 22 12 (#200)
  - Implemented warehouse2 feature with replanning and format fixes.

- Feature/warehouse2 23 12 (#201)
  - Implemented warehouse2 feature with tuning and fixes.

- Feature/minor tune (#203)
  - Implemented minor tune with fixes for warehouse3 and core improvements.

- Feature/docker improvements march 2022 (#235)
  - Implemented docker improvements with backport to foxy and format fixes.

Changed
-------
- finetuning waypoints (#187)
  - Made minor changes for finetuning waypoints.

- tuning warehouse3 (#197)
  - Tuned warehouse3 with minor changes.

- fixing warehouse 3 problems, and other core improvements (#204)
  - Fixed warehouse3 problems, removed deadlocks, and improved continuous integration.

- Foxy backport (#206)
  - Backported changes to Foxy with formatting fixes and various corrections.

Fixed
-----
- Several fixes (#194)
  - Implemented several fixes.

- minor changes (#195)
  - Made minor changes.

Removed
-------
- Removed some reordering fixes.

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

Section_56
==========

Added
-----
- Opened new folder for additional tracing contents.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added README tutorial for Dockerfile.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Corrected trailing spaces.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- Updated c_cpp_properties.json.
- Renamed rolling files and added Galactic CI setup.
- Fixed source CI and corrected README overview.
- Attempted precommit fixes.

Fixed
-----
- Bug in smacc2 component.

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Galactic builds from master, keeping only rolling. Removed submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Author
------
- Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_57
==========

Added
-----
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and confirm they are alive. Optional node selection available.
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Navigation parameters fixes on sm_dance_bot.
- cb_pause_slam client behavior.
- sm_dance_bot_lite gazebo fixes: cleaning, lidar show/hide option, and formatting improvements.
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
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_58

### Added
- Added source build on PR for testing.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added smacc2::deep_history syntax.
- Added progress in testing sm_dance_bot introducing slam pausing/resuming functionality.
- Added First working version of sm template and template generator.
- Added SM core test.
- Added local action msgs usage.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Added reliability qos config.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.

### Changed
- Changed method order to prevent recursion in sm_dance_bot.
- Changed "Finnaly" to "Finally" for correct spelling.
- Changed launch command in README.md for sm_dance_bot_strikes_back.
- Changed format to fix python version in CI.
- Changed node creation to create only a logger.
- Changed Docker environment to be executed from any environment.
- Changed slight waypoint 4 and iterations for the robot to complete the course.
- Changed library SMs to smacc2_performance_tools.
- Changed state machine transition timestamp to initial state machine transition timestamp.

### Fixed
- Fixed minor format issues.
- Fixed launch command in README.md.
- Fixed CI pipeline error.
- Fixed broken master build.

### Removed
- Removed neo_simulation2 package.
- Removed merge markers from a python file.
- Removed node creation and create only a logger.
- Removed parameters smacc.
- Removed sm_dance_bot_msgs.
- Removed test from main moveit cmake.

### Miscellaneous
- Diverse improvements in navigation and performance.
- Minor adjustments in formatting.
- Minor format tweaks.
- Minor tuning to mitigate overshot issue cases.
- Pending references.
- Formatting improvements.
- Precommit cleanup.
- Workflow updates.
- Noticed launch command was incorrect in README.md.
- Noticed typo.
- More refinement in sm_dance_bot.
- More progress on markers cleanup.
- More progress on moveit migration testing.
- More readme updates.
- More testing on moveit behaviors.
- Finishing touches on sm_multi_stage_1.
- Readme updates.
- Mode_5_sequence_b.
- Mode_4_sequence_b.
- Mode_5_sequence_b.
- Mode_4_sequence_b.
- Mode_5_sequence_b.
- Mode_4_sequence_b.
- Mode_5_sequence_b.
- Mode_4_sequence_b.

```rst
Section_59
==========

Added
-----
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 (#177).
- Waypoint Inputs (#178).
- Warehouse2 progress (#179).
- Format (#180).
- sm_dance_bot_warehouse_3 (#181).
- Feature/sm warehouse 2 13 dec 2 (#182).
- Brettpac branch (#184).
- Feature/wharehouse2 dec 14 (#185).
- Feature/cb pure spinning (#188).
- Feature/planner changes 16 12 (#191).
- Feature/replanning 16 dec (#193).
- Feature/undo motion 20 12 (#196).
- Feature/sync 21 12 (#199).
- Feature/warehouse2 22 12 (#200).
- Feature/warehouse2 23 12 (#201).
- Feature/minor tune (#203).
- Feature/warehouse2 23 12 (#201).
- Use correct upstream .repos files for source builds (#243).
- Correct mergify branch names (#246).
- Update galactic source build job name (#250).
- Galactic source build: update .repos file, bump action version and use correct version of upstream packages (backport #241) (#248).
- Restoring workflow files (#252).
- Restoring files (#253).
- Fix checkout branches for scheduled builds (#254).
- Update foxy-source-build.yml.

Changed
-------
- Minor changes (#175).
- Several fixes (#194).
- Tuning and fixes (#202).
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green.
- Improving undo motion navigation warehouse2.
- Improvements in SMACC core adding more components mostly developed for Autoware demo.
- Progress in Autoware machine.
- Refining CP subscriber CP publisher.
- Improving navigation behaviors.
- Docker improvements.

Fixed
-----
- Fixing broken build.
- Fixing to focal by the moment.
- Fixing rolling build.
- Trying to fix dependencies.
- Fixing building issue.
- Typo.
- Correct checkout branch on scheduled build.
- Fix checkout branches for scheduled builds.

Removed
-------
- Pure spinning behavior missing files.
- Weird MoveIt not downloaded repo.
- Added missing file from Warehouse2.
- Missing SM.
- Missing.
- Missing repo.
- Missing deps.
- Missing.
- Missing repo.
- Missing.
- Minor broken build.
- Some reordering fixes.
- Minor.
- Minor format.
- Minor linking errors for Foxy.
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
- Minor.

```rst
Section_60
==========

Added
-----

- Feature/fixing husky build rolling (#257)
  - Restored files and made husky project build on rolling.

- Feature/fixing husky build rolling (#258)
  - Restored files and made husky project build on rolling.
  - Made progress on husky.

- Feature/fixing ur demos (#261)
  - Restored files and made fixes.

- Feature/fixing type string walker (#263)
  - Restored files and fixed type string walker threesome demo.

- Significant update in Getting Started Instructions (#269)
  - Removed trailing spaces.
  - Co-authored by: Denis Štogl <denis@stogl.de>

- Fix: initialise conditionFlag as false (#274)

- Precommit fix (#280)
  - Merged in red for focal-rolling due to minor update of the precommit.

Changed
-------

- Updated mentions of SMACC/ROS to SMACC2/ROS2 in Feature/master rolling to galactic backport (#236).

- Renamed folders, deleted tracing.md, and edited README.md in Feature/master rolling to galactic backport (#236).

- Added smacc2_performance_tools in Feature/master rolling to galactic backport (#236).

- Improved performance tests in Feature/master rolling to galactic backport (#236).

- Cleaned up sm_respira_1 format in Feature/master rolling to galactic backport (#236).

- Reformatted sm_reference_library in Feature/master rolling to galactic backport (#236).

- Corrected trailing spaces in Feature/master rolling to galactic backport (#236).

- Optimized dependencies in move_base_z_planners_common in Feature/master rolling to galactic backport (#236).

- Renamed event generator library in Feature/master rolling to galactic backport (#236).

- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch in Feature/master rolling to galactic backport (#236).
  - Also updated doxygen links.
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- More Readme Updates (#72)
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- More Readme (#74)
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

- Created new sm from sm_respira_1 (#76)

- Base for the sm_aws_aarehouse navigation in Feature/core and navigation fixes (#78).

- Progressed in aws navigation in Feature/core and navigation fixes (#78).

- Made several core improvements during navigation testing in Feature/core and navigation fixes (#78).

- Progressed in aws navigation demo in Feature/core and navigation fixes (#78).

- Reworked sm_advanced_recovery_1 in sm_advanced_recovery_1 reworked (#83).

- More work on sm_advanced_recovery_1 in More sm_advanced_recovery_1 work (#85).

- Round 4 of sm_advanced_recovery_1 in sm_advanced_recovery_1 round 4 (#86).

- Worked on sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1 in Brettpac branch (#87).

- Worked on sm_atomic_performance_test_c_1 in sm_atomic_performance_test_c_1 (#88).

- Modified sm_atomic_performance_test_a_2 in modifying sm_atomic_performance_test_a_2 (#89).

- Worked on sm_multi_stage_1 in sm_multi_stage_1 (#90).

- Updated launch command in sm_multi_stage_1 (#90).

- Wait topic message client behavior (#81)
  - Introduced new feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
  - Attempted precommit fixes.
  - Co-authored by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  - Co-authored by: Denis Štogl <destogl@users.noreply.github.com>

- Feature/wait nav2 nodes client behavior (#82)
  - Progressed in aws navigation demo.
  - Made several core improvements during navigation testing.
  - Formatted improvements.
  - More on navigation.
```

```rst
Section_61
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection.
- Base for the `sm_aws_aarehouse` navigation.
- Progress in AWS navigation demo.
- New client behavior: `cb_pause_slam`.
- `sm_dance_bot_lite` gazebo fixes.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo.
- Got `sm_multi_stage_1` working.

Changed
-------
- Corrected all linters and formatters.
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- `sm_dance_bot` visualizing `turtlebot3`.
- Cleaning and lidar show/hide option.
- Cleaning files and formatting work.
- Gazebo fixes to show the robot and the lidar.

Fixed
----
- Removed some compile warnings.

Removed
-------
- Nothing.

Contributors
------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

*pabloinigoblasco*

## Section_62

### Added
- Added multistage modes to `sm_multi_stage_1`.
- Added sequences to `sm_multi_stage_1`.
- Added steps to `sm_multi_stage_1`.
- Added sequence d to `sm_multi_stage_1`.
- Added sequence c to `sm_multi_stage_1`.
- Added mode_5_sequence_b.
- Added mode_4_sequence_b.
- Added finishing touches 1 to `sm_multi_stage_1`.
- Added husky launch file in `sm_dance_bot` for AWS navigation.

### Changed
- Reworked `sm_multi_stage_1` with multistage modes and sequences.

### Fixed
- Fixed minor format issues.
- Fixed launch command in README.md for `sm_dance_bot_strikes_back`.
- Fixed CI format for Python version.

### Removed
- Removed `neo_simulation2` package.
- Removed merge markers from a Python file.
- Removed node creation and created only a logger.

### Miscellaneous
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>.
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.

---

Recuerda que la autoría de este changelog mejorado sigue siendo de Pablo Iñigo Blasco (pabloinigoblasco).

```rst
Section_63
==========

Added
-----
- Update dependencies for husky in rolling and galactic.
- Progress on AWS navigation and refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress.
- Waypoint Inputs.
- Warehouse2 progress.
- Format.
- Sm_dance_bot_warehouse_3.
- Brettpac branch.
- Redoing sm_dance_bot_warehouse_3 waypoints.
- More Waypoints.
- SrConditional fixes and formatting.
- Move trigger logic into headers.
- Lint.
- Several fixes.
- Replanning for all examples.
- Improving undo motion navigation warehouse2.
- Tuning warehouse3.
- Tuning and fixes.
- Fixing warehouse 3 problems and other core improvements to remove dead lock, also making continuous integration green.
- Added missing file from warehouse2.
- Bump ccache version.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Satisfy ament_lint_cmake.
- Add missing licenses.
- Update ci-build-source.yml.
- Enable cppcheck.
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
- Update changelogs.
- Update description table.
- Update table.
- Copy initial docs.
- Dockerfile w/ ROS distro as argument.
- Opened new folder for additional tracing contents.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh.
- Removed manual installation of ros-rolling-ros2trace.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.

Changed
-------
- Fix formatting.
- First ensure you have the necessary package installed.
- Rename header files and correct format.
- Change extension of imports.
- Update doxygen-deploy.yml.
- Use manual deployment for now.
- Ignore further packages.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Disable disabled packages.
- Change extension.
- Correct formatting of python file.
- Use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/".

Fixed
-----
- Fixing broken build.

Removed
-------
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Ignore further packages.
- Disable cpplint and cppcheck linters.
- Disable disabled packages.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_64
==========

Added
-----
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Changed wording "smacc application" to "SMACC2 library".
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Renamed event generator library.

Fixed
-----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Correct trailing spaces.
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
- Performance tests improvements.
- More on performance and other issues.
- Format cleanup for sm_respira_1.
- Format cleanup for sm_respira_test_2.
- More changes on performance tests.
- sm_respira_1 format cleanup pre-commit.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- More sm_atomic_24hr cleanup.
- Minor formatting.
- Minor improvements during navigation testing.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Format improvements.
- Attempting pre-commit fixes.
- Base for the sm_aws_aarehouse navigation.
- Progressing in AWS navigation.
- More on navigation.
- Fixing pre-commit.

Commits
-------
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_65
==========

Added
-----

- New client behavior for nav2: now waits for nav2 nodes to subscribe to the /bond topic and confirms they are active. Optional node selection available.
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- `cb_pause_slam` client behavior added.
- `sm_dance_bot_lite` feature added.
- Visualizing turtlebot3 in `sm_dance_bot`.
- Lidar show/hide option added.
- Gazebo fixes to display the robot and lidar.
- `sm_multi_stage_1` doubling feature included.
- `sm_dance_bot_strikes_back` gazebo fixes implemented.

Changed
-------

- Navigation parameters fixes on `sm_dance_bot`.
- Minor formatting improvements.
- Merge and progress updates.
- Hotfix for minor issues.
- Cleaning and formatting work on various files.
- Precommit cleanup run.

Fixed
-----

- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting issues.
- Enabled source build on PR for testing.
- Adjusted build packages of source CI.

Removed
-------

- `neo_simulation2` package removed.

Collaborators
-------------

- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
```

```rst
Section_66
==========

Added
-----
- Additional linting and formatting.
- Feature to toggle slam and smacc deep history (#122).
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also added smacc2::deep_history syntax.
- Introducing slam pausing/resuming functionality in testing sm_dance_bot.
- More fixes in sm_dance_bot (#125).
- First working version of sm template and template generator (#127).
- Added SM Atomic SM generator (#143).
- Rolling Docker environment to be executed from any environment (#154).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Update package list (#142).
- Add SM core test (#138).
- Add QOS durability to SmaccPublisherClient (#163).
- Waypoint Inputs (#178).

Changed
-------
- Move method after the method it calls to prevent recursion (#126).
- Renamed reference library SMs to smacc2_performance_tools (#166).
- Refactored to remove line and add reliability qos config in SmaccPublisherClient.
- Minor navigation improvements (#141).
- Using local action msgs instead of sm_dance_bot_msgs.
- Renamed navigation 2 stack.
- Removed parameters smacc.
- Fixed launch command in README.md.
- Fix CI: format fix python version (#148).
- Fixing some errors introduced on formatting in migration to smacc2.
- Progress on moveit migration testing.
- Improved Dockerfile for building local tests.
- Fixed compiling issues.
- Updated README (#164).
- Minor changes (#175).
- Warehouse2 progress (#179).
- Format (#180).
- Redoing sm_dance_bot_warehouse_3 waypoints.
- More Waypoints.

Fixed
-----
- Minor format issues (#134).
- Waypoints navigator bug (#133).
- Mitigated overshot issue cases in navigation.
- Fixed broken master build.
- Fixed pipeline error.

Removed
-------
- Removed merge markers from a python file (#119).
- Removed node creation and create only a logger (#149).
- Removed test from main moveit cmake.
- Removed some comments in the past from launch command for sm_dance_bot_strikes_back.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_67
==========

Added
-----
- Feature/wharehouse2 dec 14 (#185)
  - Implemented warehouse2 feature with minor changes

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Updated format and made headless changes for warehouse2

- Feature/cb pure spinning (#188)
  - Introduced pure spinning behavior with format changes

- Feature/replanning 16 dec (#193)
  - Improved replanning for all examples with fixes

- Feature/undo motion 20 12 (#196)
  - Enhanced undo motion navigation for warehouse2

- Feature/sync 21 12 (#199)
  - Addressed format issues for synchronization

- Feature/warehouse2 22 12 (#200)
  - Fixed format issues and completed warehouse2

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2

- Feature/minor tune (#203)
  - Made minor tune-ups and fixes

- Feature/undo motion 20 12 (#198)
  - Improved undo motion navigation for warehouse2

- Feature/sync 21 12 (#199)
  - Addressed format issues for synchronization

- Feature/warehouse2 22 12 (#200)
  - Fixed format issues and completed warehouse2

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed warehouse2

Changed
-------
- Finetuned waypoints (#187)
  - Improved waypoint functionality

- Fixing warehouse 3 problems, and other core improvements (#204)
  - Resolved warehouse 3 issues and core improvements

- Merging code from backport foxy and updates about autoware (#208)
  - Integrated code from backport foxy and autoware updates

- Foxy backport (#206)
  - Backported changes to foxy

Fixed
-----
- Several fixes (#194)
  - Addressed various issues

- Minor broken build (#207)
  - Fixed minor issues causing build failures

Removed
-------
- Removed manual installation of ros-rolling-ros2trace
  - Automation now handles ros-rolling-ros2trace installation

- Deleted tracing directory
  - Removed unnecessary tracing directory

- Ignore all packages except smacc2 and smacc2_msgs
  - Focused on smacc2 and smacc2_msgs packages

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

```rst
Section_68
==========

Added
-----
- Update smacc_sm_reference_library/sm_atomic/README.md:
  - Edit from HTML to Markdown syntax.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More changes on performance tests.
- Optimized dependencies in move_base_z_planners_common.
- Added galactic CI setup and renamed rolling files. (#58)
- Fix source CI and correct README overview. (#62)
- More Readme Updates (#72)
- More Readme (#74)
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Update doxygen links (#70)
- More on navigation.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Adding new client behavior add for Nav2, wait Nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.
- Progress in AWS navigation demo.

Changed
-------
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Update smacc2_rta command across READMEs.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update README.md with updated launch command.
- Correct trailing spaces.
- Renaming of event generator library.
- Minor formatting improvements.
- Correct all linters and formatters.

Removed
-------
- Remove galactic builds from master and keep only rolling.
- Remove submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Removed tracing.md file.

Fixed
-----
- Bug in smacc2 component.
- Reverted Markdowns to HTML.
- Minor format fixes.
- Several core improvements during navigation testing.
- Formatting improvements.
- Attempting pre-commit fixes.
- Fixing pre-commit.
- Trying to fix Pre-Commit.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Denis Štogl <denis@stogl.de>
```

```rst
Section_69
==========

Added
-----

- New feature: `cb_wait_topic_message` - asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#93)
- New client behavior for `nav2`: `cb_wait_nav2_nodes` - waits for nodes subscribing to the `/bond` topic to become alive, with optional node selection (#93)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause_slam` (#98)
- New client behavior: `cb_pause

```rst
Section_70
==========

Added
-----
- Progress in navigation, SLAM toggle client behaviors, and slam_toolbox components.
- Introducing smacc2::deep_history syntax.
- Testing sm_dance_bot with slam pausing/resuming functionality.
- First working version of sm template and template generator.
- Added SM Atomic SM generator.
- Rolling Docker environment to be executed from any environment.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Add SM core test.
- Add QOS durability to SmaccPublisherClient.
- Add dependencies for husky simulation.
- Waypoint Inputs.

Changed
-------
- Move method after the method it calls to prevent recursion.
- Renamed reference library SMs to smacc2_performance_tools.
- Renamed sm_advanced_recovery_1.
- Reworked sm_multi_stage_1 with multistage modes and sequences.
- Renamed sm_multi_stage_1.
- Renamed sm_dance_bot_warehouse_3.
- Redoing sm_dance_bot_warehouse_3 waypoints.

Fixed
-----
- Minor format issues.
- Fixed launch command for sm_dance_bot_strikes_back.
- Fixed CI format for Python version.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed broken build.
- Fixed some formatting and linting issues on SrConditional.
- Fixed some errors introduced on formatting.
- Fixed some more linting warnings.

Removed
-------
- Removed node creation and create only a logger.
- Removed parameters smacc.
- Removed test from main moveit cmake.
- Removed sm_dance_bot_msgs.

Authors
-------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_71
==========

Added
-----
- Feature/wharehouse2 dec 14 (#185): Added warehouse2 minor changes.
- Feature/sm warehouse 2 13 dec 2 (#186): Added format changes, headless merge, and default values.
- Feature/cb pure spinning (#188): Added pure spinning behavior changes.
- Feature/planner changes 16 12 (#191): Added minor changes and fixes.
- Feature/replanning 16 dec (#193): Added replanning for all examples and several fixes.
- Feature/undo motion 20 12 (#196): Added improvements in undo motion navigation for warehouse2.
- Feature/sync 21 12 (#199): Added format fixes.
- Feature/warehouse2 22 12 (#200): Added format fixes and finished warehouse2.
- Feature/warehouse2 23 12 (#201): Added tuning and fixes.
- Feature/minor tune (#203): Added tuning and fixes, minor tune, and core improvements for warehouse 3.
- Added missing file from warehouse2 (#205): Added missing files and backported to foxy.
- Add mergify rules file (#209): Added mergify rules file.
- Add Autoware Auto Msgs into not-released dependencies (#220): Added Autoware Auto Msgs into dependencies.
- Add galactic CI build because Navigation2 is broken in rolling (#222): Added galactic CI build.
- Add partial changes for ament_cpplint: Added partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include: Added tf2_ros as dependency.
- Bump ccache version: Updated ccache version.
- Add missing licences: Added missing licenses.
- Satisfy ament_lint_cmake: Satisfied ament_lint_cmake.
- Update ci-build-source.yml: Updated ci-build-source.yml.
- Change extension of imports: Changed extension of imports.
- Enable cppcheck: Enabled cppcheck.
- Create workflow for testing prerelease builds: Created workflow for testing prerelease builds.
- Use docs/ as source folder for documentation: Used docs/ as source folder.
- Use docs/ as output directory: Used docs/ as output directory.
- Rename to smacc2 and smacc2_msgs: Renamed to smacc2 and smacc2_msgs.
- Update name of package and package.xml to pass liter: Updated package name and package.xml.
- Update changelogs: Updated changelogs.
- Update description table: Updated description table.
- Update table: Updated table.
- Copy initial docs: Copied initial docs.
- Dockerfile w/ ROS distro as argument: Added Dockerfile with ROS distro as argument.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Renamed header files and corrected format.

Fixed
-----
- Fix trailing spaces.
- Correct codespell.
- Correct python linters warnings.
- Remove example things from Foxy CI setup (#214): Removed example things from Foxy CI setup.
- Fix rolling builds (#222): Fixed rolling builds.
- Fixing docker for foxy and galactic: Fixed docker for foxy and galactic.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green (#204): Fixed warehouse 3 problems and other core improvements to remove deadlock.
- Fixing warehouse 3 problems, and other

Section_72
==========

Added
-----
- Opened new folder for additional tracing contents.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Moved tracing.md to tracing directory.
- Renamed tracing events after.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
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
- Updated c_cpp_properties.json.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh.
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Collaborators
-------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Author
------
- Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_73
==========

Version 1.0.0 (2022-01-01)
---------------------------

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive.

Changed
-------

- Minor formatting improvements during navigation testing.

Fixed
----

- Navigation parameters fixes on `sm_dance_bot`.
- Remove some compile warnings.

Version 1.1.0 (2022-02-01)
---------------------------

Added
-----

- New feature: `cb_pause_slam` client behavior.

Changed
-------

- Minor formatting improvements during navigation testing.

Fixed
----

- Fix format on `sm_dance_bot_lite`.
- Fix gazebo issues to show the robot and the lidar.

Version 1.2.0 (2022-03-01)
---------------------------

Added
-----

- New feature: `sm_multi_stage_1` doubling.

Changed
-------

- Minor formatting improvements during navigation testing.

Fixed
----

- Fix gazebo issues for `sm_dance_bot_strikes_back`.

Version 1.3.0 (2022-04-01)
---------------------------

Added
-----

- AWS demo progress.

Changed
-------

- Got `sm_multi_stage_1` working.

Fixed
----

- Precommit cleanup run.

Contributors
------------

- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_74

### Added
- Enable source build on PR for testing. (#112)
- Added SM core test. (#138)
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient. (#163)
- Added AWS navigation for SM Dance Bot. (#174)

### Changed
- Adjusted build packages of source CI.
- Improved navigation and performance.
- Progress in navigation, SLAM toggle client behaviors, and SLAM_toolbox components.
- Refactored SM Dance Bot Strikes Back.
- Migration to SMACC2.
- Moved reference library SMs to smacc2_performance_tools.
- Reworked SM Multi-Stage 1.

### Fixed
- Corrected formatting.
- Resolved compile warnings.
- Fixed CI format for Python version.
- Fixed launch command in README.md.
- Fixed compiling issues.

### Removed
- Removed neo_simulation2 package.
- Removed merge markers from a Python file.
- Removed node creation and created only a logger.
- Removed parameters SMACC.
- Removed test from main MoveIt CMake.

### Miscellaneous
- Co-authored commits by Ubuntu 20-04-02-amd64, Brett (brett@robosoft.ai), Pablo Iñigo Blasco (pablo@ibrobotics.com), DecDury (declandury@gmail.com), and Denis Štogl (destogl@users.noreply.github.com) were included in various changes.

```rst
Section_75
==========

Added
-----
- Added warehouse2 (#177).
- Added Waypoint Inputs (#178).
- Added wharehouse2 progress (#179).
- Added format (#180).
- Added sm_dance_bot_warehouse_3 (#181).
- Added Feature/sm warehouse 2 13 dec 2 (#182).
- Added more changes and headless.
- Added merge.
- Added headless and other fixes.
- Added default values.
- Added Brettpac branch (#184).
- Added Redoing sm_dance_bot_warehouse_3 waypoints.
- Added More Waypoints.
- Added SrConditional fixes and formatting (#168).
- Added fix: some formatting and templating on SrConditional.
- Added fix: move trigger logic into headers.
- Added fix: lint.
- Added Feature/wharehouse2 dec 14 (#185).
- Added Feature/sm warehouse 2 13 dec 2 (#186).
- Added finetuning waypoints (#187).
- Added Feature/cb pure spinning (#188).
- Added pure spinning behavior missing files.
- Added minor changes (#190).
- Added Feature/planner changes 16 12 (#191).
- Added more fixes.
- Added Feature/replanning 16 dec (#193).
- Added replanning for all our examples.
- Added several fixes (#194).
- Added Feature/undo motion 20 12 (#196).
- Added improving undo motion navigation warehouse2.
- Added tuning warehouse3 (#197).
- Added Feature/undo motion 20 12 (#198).
- Added undo tuning and errors.
- Added format issues.
- Added Feature/sync 21 12 (#199).
- Added format issues.
- Added Feature/warehouse2 22 12 (#200).
- Added finishing warehouse2.
- Added Feature/warehouse2 23 12 (#201).
- Added tuning and fixes (#202).
- Added Feature/minor tune (#203).
- Added fixing warehouse 3 problems, and other core improvements (#204).
- Added fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green.
- Added weird moveit not downloaded repo.
- Added added missing file from warehouse2 (#205).
- Added backport to foxy.
- Added minor format.
- Added minor linking errors foxy.
- Added docker build files for all versions.
- Added dockerfiles (#225).
- Added Fix code generators (#221).
- Added Fix other build issues.
- Added Update SM template and make example code clearly visible.
- Added Remove use of node in the sm performance template.
- Added Updated templated to use Blackboard storage.
- Added Update template to resolve the global data correctly.
- Added Update sm_name.hpp.
- Added Foxy backport (#206).
- Added minor formatting fixes.
- Added Fix trailing spaces.
- Added Correct codespell.
- Added Correct python linters warnings.
- Added Add galactic CI build because Navigation2 is broken in rolling.
- Added Add partial changes for ament_cpplint.
- Added Add tf2_ros as dependency to find include.
- Added Disable ament_cpplint.
- Added Disable some packages and update workflows.
- Added Bump ccache version.
- Added Ignore further packages.
- Added Satisfy ament_lint_cmake.
- Added Add missing licences.
- Added Disable cpplint and cppcheck linters.
- Added Correct formatters.
- Added branching example.
- Added Disable disabled packages.
- Added Update ci-build-source.yml.
- Added Change extension.
- Added Change extension of imports.
- Added Enable cppcheck.
- Added Correct formatting of python file.
- Added Included necessary package and edited Threesome launch.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.

Removed
-------
- Removed First ensure you have the necessary package installed.
- Removed ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
- Removed Rename header files and correct format.
- Removed Add workflow for checking doc build.

Authors
-------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
```

Section_76
==========

Added
-----

- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Update description table
- Update table
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
- Add galactic CI setup and rename rolling files (#58)
- Fix source CI and correct README overview (#62)
- Update c_cpp_properties.json
- Update doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Feature/aws demo progress (#80)
- Sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_c_1 (#88)
- Sm_multi_stage_1 (#90)
- Wait topic message client behavior (#81)

Changed
-------

- Use manual deployment for now
- Correct GitHub branch reference
- Execute on master update
- Ignore all packages except smacc2 and smacc2_msgs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Changed wording "smacc application" to "SMACC2 library"
- Reactivating smacc2 nav clients for rolling via submodules
- Renamed tracing events after
- Bug in smacc2 component
- Reverted markdowns to html
- Edited tracing.md to reflect new tracing event names
- Update smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- Correct trailing spaces
- Sm_atomic_performance_trace_1
- Sm_atomic_24hr
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Modifying sm_atomic_performance_test_a_2

Removed
-------

- Delete tracing directory
- Removed manual installation of ros-rolling-ros2trace
- This is now automated in setupTracing.sh
- Location of sh file assumed if user follows README.md under "Getting started"
- Reformatting of sm_reference_library
- Do not execute clang-format on smacc2_sm_reference_library package
- Minor formatting
- Several core improvements during navigation testing
- Formatting improvements
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on navigation
- More on

```rst
Section_77
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- Added new client behavior for nav2: `wait nav2 nodes` subscribing to the `/bond` topic and waiting for them to be alive. You can optionally select the nodes to wait for
- Added `cb_pause_slam` client behavior

Changed
-------
- Corrected all linters and formatters
- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Merge and progress
- Fix format
- Cleaning and lidar show/hide option
- Updates yaml
- Gazebo fixes to show the robot and the lidar
- Format fixes

Fixed
----
- Removed some compile warnings

Removed
-------
- Removed redundant entries

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_78
===========

Added
-----
- Added AWS demo (#108, #110).
- Added Brettpac branch (#110, #111).
- Added feature to toggle SLAM and deep history for SM Dance Bot (#122).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient (#163).

Changed
-------
- Improved gazebo fixes for SM Dance Bot Strikes Back.
- Improved functionality of SM Multi Stage 1 (#109, #114).
- Improved navigation and performance (#116).
- Improved dance bot's pattern (#128, #129).
- Refactored SM Dance Bot Strikes Back (#152).
- Updated package list (#142).
- Renamed navigation 2 stack (#144).
- Updated READMEs with SVGs (#140, #145).
- Updated Docker environment for execution in any environment (#154).
- Updated format for CI (#148).
- Updated README (#164).
- Updated state machine transition timestamp (#165).

Fixed
-----
- Fixed waypoint issues for robot course completion (#155).
- Fixed migration errors for MoveIt client (#151).
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.

Removed
-------
- Removed neo_simulation2 package (#112).
- Removed parameters from SMACC (#147).
- Removed node creation, creating only a logger (#149).
- Removed SM Dance Bot messages.
- Removed test from main MoveIt CMake.

Other
-----
- Cleaned up pre-commit (#106, #152, #163).
- Minor formatting and linting improvements.
- Addressed minor issues and bugs.
- Made progress on various features and functionalities.
- Continued testing and improvements on MoveIt behaviors.
- Added dependencies and fixed build errors.
- Made progress on MoveIt migration testing.
- Updated Dockerfile for building local tests.
- Added reliability QOS config.
- Added missing colon.
- Removed unnecessary line.
- Added references to commits and collaborators.

Co-authored-by
--------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- DecDury <declandury@gmail.com>.
- Denis Štogl <destogl@users.noreply.github.com>.
- Pablo Iñigo Blasco <pablo@ibrobotics.com>.

```rst
Section_79
==========

Added
-----

- Added `sm_advanced_recovery_1` renaming (#171).
- Added `sm_multi_stage_1` reworking (#172) with:
  - Multistage modes.
  - `sm_multi_stage` sequences.
  - `sm_multi_state_1` steps.
  - `sm_multi_stage_1` sequence d.
  - `sm_multi_stage_1` c sequence.
  - `mode_5_sequence_b`.
  - `mode_4_sequence_b`.
  - `sm_multi_stage_1` most.
  - Finishing touches 1.
  - Readme.

Changed
-------

- Changed `Feature/aws navigation sm dance bot` (#174) with:
  - Repo dependency.
  - Husky launch file in `sm_dance_bot`.
  - Added dependencies for Husky simulation.
  - Fixed formatting.
  - Updated dependencies for Husky in rolling and galactic.
  - Progress on AWS navigation and refactorings on navigation clients and behaviors.
  - More on AWS demo.
  - Fixed broken build.

Fixed
-----

- Fixed `minor changes` (#175).
- Fixed `warehouse2` (#177).
- Fixed `Waypoint Inputs` (#178).
- Fixed `wharehouse2` progress (#179).
- Fixed `format` (#180).
- Fixed `sm_dance_bot_warehouse_3` (#181).
- Fixed `Feature/sm warehouse 2 13 dec 2` (#182) with:
  - More changes and headless.
  - Merge.
  - Headless and other fixes.
  - Default values.
- Fixed `SrConditional` (#168) with:
  - Some formatting and templating on `SrConditional`.
  - Moved trigger logic into headers.
  - Lint.

Removed
-------

- Removed `pure spinning behavior missing files`.
- Removed `weird moveit not downloaded repo`.
- Removed `retry behavior warehouse 1`.
- Removed `missing file`.

Authors
-------

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
Section_80
==========

Added
-----
- Added ignition file and updated repos files.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Added alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------
- Progressed in husky demo.
- Improved navigation behaviors.
- Replanned for all examples.
- Backported to foxy.
- Renamed header files and corrected format.
- Renamed to smacc2 and smacc2_msgs.
- Updated name of package and package.xml.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Updated ci-build-source.yml.
- Changed extension of imports.
- Enabled cppcheck.
- Corrected formatting of python file.
- Reactivated smacc2 nav clients for rolling via submodules.
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, edited README.md.
- Optimized deps in move_base_z_planners_common.
- Renamed event generator library.
- Cleaned up sm_atomic_24hr.
- Updated smacc2_rta command across readmes.
- Cleaned up of sm_atomic_24hr.
- Renamed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More readme updates.
- Created new sm from sm_respira_1.
- Several core improvements during navigation testing.
- Progressed in aws navigation demo.
- Formatted improvements.
- Reworked sm_advanced_recovery_1.

Fixed
-----
- Fixed trailing spaces.
- Corrected codespell.
- Corrected python linters warnings.
- Corrected formatters.
- Fixed source CI and corrected README overview.

Removed
-------
- Disabled ament_cpplint.
- Disabled some packages and updated workflows.
- Disabled cpplint and cppcheck linters.
- Disabled disabled packages.
- Ignored further packages.
- Ignored all packages except smacc2 and smacc2_msgs.
- Do not execute clang-format on smacc2_sm_reference_library package.
```

*pabloinigoblasco*

```rst
Section_81
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Added new client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection.

Changed
-------

- Updated launch command in README.md.
- Corrected all linters and formatters.

Fixed
-----

- Resolved pre-commit issues.
- Fixed formatting in various sections.
- Fixed navigation parameters on sm_dance_bot.
- Removed some compile warnings.

Removed
-------

- None.

Contributors
------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

## Section_82

### Added
- Added new client behavior for nav2, now waits for nav2 nodes to subscribe to the /bond topic and ensures they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.

### Changed
- Navigation parameters fixes on sm_dance_bot.

### Fixed
- Fixed cb pause slam client behavior.
- Fixed formatting issues in various parts of the code.
- Fixed gazebo issues to show the robot and lidar correctly.

### Removed
- Removed neo_simulation2 package.
- Removed unnecessary parameters from smacc.

### Miscellaneous
- Various minor improvements and fixes throughout the codebase.
- Co-authored commits with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, and Denis Štogl <destogl@users.noreply.github.com>.
- Precommit cleanup and workflow updates.
- Updated package list and launch commands in README.md.
- Added SVGs to READMEs for atomic, dance_bot, and other components.
- Rolling Docker environment to be executed from any environment.
- Initial migration to smacc2 for moveit client, fixing formatting errors and missing dependencies.
- Slight changes in waypoints and iterations to ensure the robot can complete the course successfully.

```rst
Section_83
==========

Added
-----
- Added .reps dependencies and fixed build errors (#commit_ref)
- Added dependency to ur5 client
- Added QOS durability to SmaccPublisherClient (#163) (#commit_ref)
- Added reliability QOS configuration

Changed
-------
- Improved dockerfile for building local tests
- Refactored docker
- Progressed in move_it PR
- Reworked sm_multi_stage_1 with multistage modes, sequences, and steps
- Updated dependencies for husky in rolling and galactic
- Finetuned waypoints (#187) (#commit_ref)
- Improved undo motion navigation in warehouse2
- Tuned warehouse3 (#197)
- Tuned and fixed warehouse2 (#202)
- Tuned and fixed minor issues (#203)
- Fixed warehouse 3 problems to remove deadlock and make continuous integration green (#204)

Fixed
-----
- Fixed compiling issues
- Fixed pipeline error
- Fixed broken master build
- Fixed formatting issues
- Fixed broken build
- Fixed some formatting and templating on SrConditional
- Fixed linting
- Fixed missing files in pure spinning behavior
- Fixed minor issues
- Fixed linking errors in foxy

Removed
-------
- Removed test from main moveit cmake
- Removed some linting warnings
- Removed line in refactor

Other
-----
- Progressed in moveit migration testing
- Progressed in moveit testing
- Progressed in moveit behaviors testing
- Progressed in AWS navigation and refactorings on navigation clients and behaviors
- Progressed in AWS demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in replanning for all examples
- Progressed in undo motion
- Progressed in warehouse2
- Progressed in warehouse3
- Progressed in sm_dance_bot
- Progressed in sm_pubsub_1
- Progressed in sm_advanced_recovery_1 renaming
- Progressed in sm_multi_stage_1 reworking
- Progressed in warehouse2 (#179)
- Progressed in warehouse2 (#185)
- Progressed in warehouse2 (#186)
- Progressed in warehouse2 (#200)
- Progressed in warehouse2 (#201)
- Progressed in autoware demo
- Progressed in foxy CI
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in smacc core adding more components mostly developed for autoware demo
- Progressed in autoware demo
- Progressed in foxy CI
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo
- Progressed in autoware machine
- Progressed in autoware demo

Authors
-------
- Pablo Iñigo Blasco (@pabloinigoblasco)
```

```rst
Section_84
==========

Added
-----
- Docker build files for different revisions
- Docker build files for all versions
- Barrel demo progress
- Progress in Barrel Husky
- More merge improvements
- Master rolling to Galactic backport
- Testing dance bot demos
- Updating Galactic repositories
- Runtime dependency restoration

Changed
-------
- Improved Docker files
- Fixed Docker for Foxy and Galactic
- Fixed Docker build for Barrel search and Warehouse3
- Fixed startup problems in Warehouse 3
- Fixed broken build format and minor issues

Removed
-------
- Warnings removal and more testing on navigation

Contributors
------------
- Denis Štogl
- Pablo Iñigo Blasco
- pabloinigoblasco
- Declan Dury
- reelrbtx
- brettpac
- David Revay
```
