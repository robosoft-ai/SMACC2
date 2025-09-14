Changelog for package cl_ros2_timer
=======================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2`
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further work on buildfarm issue
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
- Progress in humble SMACC2 deb generation
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Contributors: Denis Štogl, pabloinigoblasco

Version 0.3.0 (2022-04-04)
---------------------------
### No changes

Version 0.0.0 (2022-11-09)
---------------------------
### Added
- Progress in humble SMACC2 deb generation
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Ignored packages not to be released
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup in sm_respira_1
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Added galactic CI setup and renamed rolling files (#58)
  - Fixed source CI and corrected README overview (#62)
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  - Updated doxygen links (#70)
  - More Readme Updates (#72)
  - More Readme (#74)
  - Created new sm from sm_respira_1 (#76)
  - Feature/core and navigation fixes (#78)
  - Feature/aws demo progress (#80)
  - Feature/wait nav2 nodes client behavior (#82)
  - Feature/aws demo progress (#92)
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
  - Co-authored-by: Denis Štogl <denis@stogl.de>
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- New client behavior: `cb_pause_slam`.

Changed
-------

- Improved core functionality during navigation testing.
- Formatting enhancements throughout.
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Gazebo fixes for `sm_dance_bot_visualizing_turtlebot3`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Fixed
-----

- Removed some compile warnings.
- Removed `neo_simulation2` package.
- Corrected formatting issues.
- Enabled source build on PR for testing.
- Adjusted build packages for source CI.

Removed
-------

- `neo_simulation2` package.

Other
-----

- Various minor format improvements.
- Merge and progress updates.
- Precommit cleanup run.
- Updates to YAML files.
- `sm_dance_bot_lite` improvements.
- `sm_multi_stage_1` doubling.
- `sm_multi_stage_1` progress updates.
- `sm_multi_stage_1` additional work.
- `sm_multi_stage_1` further progress.
- `sm_multi_stage_1` reaching fifth stage.
- `a3` updates.
- `mm` updates.
- `Brettpac` branch updates.
```

*pabloinigoblasco*

Section_3
=========

Version 0.1.0 (2022-01-01)
--------------------------

### Added
- Diverse improvements in navigation and performance (#116)
- Additional linting and formatting
- Feature to toggle SLAM and deep history in SMACC (#122)
- Progress in navigation, SLAM toggle client behaviors, and SLAM toolbox components
- Introducing slam pausing/resuming functionality in testing sm_dance_bot
- More fixes for sm_dance_bot (#125)
- Method moved after the method it calls to prevent recursion (#126)
- First working version of SM template and template generator (#127)
- SM Atomic SM generator added (#143)
- SM core test added (#138)
- SVGs added to READMEs of atomic, dance_bot, and others (#140)
- Remaining SVGs added to READMEs (#145)
- QOS durability added to SmaccPublisherClient (#163)
- Initial state machine transition timestamp (#165)
- Reference library SMs moved to smacc2_performance_tools
- Waypoint Inputs (#178)

### Changed
- Renaming of sm_dance_bot to s-pattern (#128)
- Refactoring of sm_dance_bot strikes back (#152)
- Migration of moveit client to smacc2
- Renaming of sm_advanced_recovery_1 (#171)
- Reworking of sm_multi_stage_1 (#172)

### Fixed
- Waypoint 4 and iterations changes for robot course completion (#155)
- Formatting fixes
- Compilation warnings resolved (#137)
- CI format fix for Python version (#148)
- Node creation removed to create only a logger (#149)
- Docker environment now executable from any environment (#154)
- Pipeline errors fixed
- Broken master build fixed

### Removed
- Parameters from smacc (#147)
- Test from main moveit CMake
- sm_dance_bot_msgs removed

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_4
=========

Added
-----

- New feature "sm warehouse 2" (#182)
- New feature "SrConditional fixes and formatting" (#168)
- New feature "cb pure spinning" (#188)
- New feature "planner changes 16 12" (#191)
- New feature "replanning 16 dec" (#193)
- New feature "undo motion 20 12" (#196)
- New feature "sync 21 12" (#199)
- New feature "warehouse2 22 12" (#200)
- New feature "warehouse2 23 12" (#201)
- New feature "minor tune" (#203)
- Fixed trailing spaces, codespell, and python linters warnings
- Added galactic CI build due to Navigation2 issues in rolling
- Added partial changes for ament_cpplint
- Added tf2_ros as dependency
- Disabled ament_cpplint, cpplint, and cppcheck linters
- Updated workflows and packages
- Bumped ccache version
- Ignored further packages
- Satisfied ament_lint_cmake
- Added missing licenses
- Corrected formatters
- Enabled cppcheck
- Included necessary package and edited Threesome launch
- Renamed header files and corrected format
- Added workflow for checking doc build
- Updated doxygen-check-build.yml and doxygen-deploy.yml
- Created workflow for testing prerelease builds
- Renamed to "smacc2" and "smacc2_msgs"
- Updated GitHub branch reference
- Updated package name and package.xml
- Reset all versions to 0.0.0
- Ignored all packages except "smacc2" and "smacc2_msgs"
- Updated changelogs
- Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Updated description table
- Updated table
- Copied initial docs
- Dockerfile with ROS distro as argument
- Opened new folder for additional tracing contents
- Deleted tracing directory
- Moved tracing.md to tracing directory
- Added setupTracing.sh to automate installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Changed wording "smacc application" to "SMACC2 library"
- Updated smacc_sm_reference_library/sm_atomic/README.md
- Reactivated smacc2 nav clients for rolling via submodules
- Renamed tracing events
- Fixed bug in smacc2 component
- Reverted markdowns to html
- Added README tutorial for Dockerfile
- Enabled build of missing rolling repositories

Removed
-------

- Manual installation of ros-rolling-ros2trace
- Manual installation steps now automated in setupTracing.sh
- Tracing directory assumed in location if user follows README.md under "Getting started"

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

Section 5
----------

### Added
- Enable Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Added galactic CI setup and rename rolling files. (#58)
- More Readme Updates (#72)
- More Readme (#74)
- created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)

### Changed
- Update mentions of SMACC/ROS to SMACC2/ROS2.
- Update smacc2_rta command across readmes.
- Change launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Update README.md with launch command.
- Update README.md.

### Fixed
- Correct trailing spaces.
- Fix source CI and correct README overview. (#62).
- Fix pre-commit in various commits.

### Removed
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

### Miscellaneous
- Minor formatting improvements.
- Optimized dependencies in move_base_z_planners_common.
- Renamed folders, deleted tracing.md, edited README.md.
- Correct all linters and formatters.
- Noticed a note that was not removed.
- Attempting pre-commit fixes.

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_6
=========

Added
-----

- New client behavior `cb_wait_topic_message` added for nav2, allowing nodes to subscribe to the `/bond` topic and wait until they are active. Users can select specific nodes to wait for.
- New feature `cb_pause_slam` introduced.
- New feature `sm_dance_bot_lite` added for visualizing Turtlebot3 in Gazebo.
- New feature `sm_multi_stage_1` doubling functionality.
- New feature `sm_dance_bot_strikes_back` for Gazebo fixes.
- AWS demo progress.
- Progress in navigation with slam toggle client behaviors and slam_toolbox components. Also includes `smacc2::deep_history` syntax.
- Progress in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Progress in `sm_dance_bot` with S pattern changes.

Changed
-------

- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting in package removal.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Move method after the method it calls to prevent recursion.

Fixed
-----

- Various core improvements during navigation testing.

Removed
--------

- `neo_simulation2` package.

Contributors
------------

- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_7
=========

Added
-----

- Added First working version of sm template and template generator. (#127)
- Added minor tweaks (#130)
- Added Feature/sm dance bot refine (#131)
- Added Feature/sm dance bot refine 2 (#132)
- Added waypoints navigator bug (#133)
- Added progress in the sm_dance_bot tests (#135)
- Added sm_dance_bot_lite (#136)
- Added Resolve compile warnings (#137)
- Added Add SM core test (#138)
- Added minor navigation improvements (#141)
- Added using local action msgs (#139)
- Added Feature/nav2z renaming (#144)
- Added formatting
- Added added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added added remaining SVGs to READMEs (#145)
- Added Update package list. (#142)
- Added Add SM Atomic SM generator. (#143)
- Added Rolling Docker environment to be executed from any environment (#154)
- Added initial migration to smacc2
- Added Add QOS durability to SmaccPublisherClient (#163)
- Added Feature/testing moveit behaviors (#167)
- Added sm_pubsub_1 (#169)
- Added sm_pubsub_1 part 2 (#170)
- Added sm_advanced_recovery_1 renaming (#171)
- Added sm_multi_stage_1 reworking (#172)
- Added Feature/aws navigation sm dance bot (#174)
- Added warehouse2 (#177)
- Added Waypoint Inputs (#178)
- Added wharehouse2 progress (#179)
- Added sm_dance_bot_warehouse_3 (#181)
- Added Feature/sm warehouse 2 13 dec 2 (#182)
- Added Brettpac branch (#184)
- Added SrConditional fixes and formatting (#168)
- Added Feature/wharehouse2 dec 14 (#185)
- Added Feature/cb pure spinning (#188)
- Added Feature/cb pure spinning (#189)

Changed
-------

- Changed Finnaly to Finally

Fixed
-----

- Fixed launch command for sm_dance_bot_strikes_back and removed some comments in README.md
- Fixed CI: format fix python version (#148)
- Fixed Remove node creation and create only a logger. (#149)
- Fixed moved reference library SMs to smacc2_performance_tools (#166)
- Fixed fixing some errors introduced on formatting
- Fixed fixing some more linting warnings
- Fixed fixing compiling issues
- Fixed fixing broken master build
- Fixed fixing pipeline error

Removed
-------

- Removed removing sm_dance_bot_msgs
- Removed removing parameters smacc

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
- Enable Navigation2 for semi-binary build

Changed
-------
- Renamed "smacc application" to "SMACC2 library"

Fixed
-----
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Tuning and fixes (#202)
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Disable ament_cpplint
- Disable cpplint and cppcheck linters
- Correct formatters
- Enable cppcheck
- Correct formatting of python file
- Enable build of missing rolling repositories

Removed
-------
- Weird moveit not downloaded repo
- Disable disabled packages
- Ignore further packages
- Ignore all packages except smacc2 and smacc2_msgs

Co-Authored-By
--------------
- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>

```

**Autor:** Pablo Iñigo Blasco (pabloinigoblasco)

```rst
Section_9
=========

Added
-----
- Added smacc2_performance_tools.
- Added galactic CI setup and renamed rolling files (#58).
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.
- Added new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait.

Changed
-------
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated README.md.
- Corrected all linters and formatters.

Fixed
-----
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview (#62).
- Fixed pre-commit.
- Attempted pre-commit fixes.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Removed tracing.md.

Other
-----
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting.
- Minor.
- Progress in aws navigation demo.
- Progressing in aws navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- More on performance and other issues.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
- More on navigation.
-

```rst
Section_10
==========

Added
-----
- New client behavior `cb_wait_topic_message` for asynchronous waiting and optional content check on a topic message.
- New client behavior for `nav2` to wait for nodes subscribing to `/bond` topic to become active, with optional node selection.
- Base for `sm_aws_warehouse` navigation.
- `cb_pause_slam` client behavior.
- `sm_dance_bot_lite` feature.
- Visualizing `turtlebot3` in `sm_dance_bot`.
- Lidar show/hide option in `sm_dance_bot`.
- Gazebo fixes for robot and lidar visualization.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo progress.
- `sm_multi_stage_1` improvements.
- `smacc2::deep_history` syntax in `slam_toolbox` components.
- `sm_dance_bot` improvements.
- `dance_bot_s` pattern feature.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- Minor format improvements.
- Format fixes.

Fixed
-----
- Compile warnings removed.
- Recursion prevention by moving method calls after the methods they invoke.

Removed
-------
- `neo_simulation2` package.

Contributors
------------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section 11
-----------

### Added
- First working version of sm template and template generator. (#127)
- Add SM core test (#138)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)

### Changed
- Renamed Feature/dance bot s pattern to Feature/sm dance bot refine (#131)
- Renamed Feature/sm dance bot refine 2 to Feature/sm dance bot refine 2 (#132)
- Renamed Feature/sm dance bot strikes back refactoring to Feature/sm dance bot strikes back refactoring (#152)
- Renamed Feature/migration moveit client to Feature/migration moveit client (#151)
- Moved reference library SMs to smacc2_performance_tools (#166)

### Fixed
- Fix CI: format fix python version (#148)
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back and removed some comments (#149)
- Fixing pipeline error in Feature/testing moveit behaviors (#167)
- Fixing broken master build in Feature/testing moveit behaviors (#167)
- Fixing broken build in Feature/aws navigation sm dance bot (#174)

### Removed
- Removed node creation and create only a logger in Add SM Atomic SM generator. (#149)
- Removed parameters smacc in Update package list. (#142)
- Removed test from main moveit cmake in Feature/migration moveit client (#151)
- Removed sm_dance_bot_msgs in Feature/nav2z renaming (#144)
- Removed parameters smacc in Feature/aws navigation sm dance bot (#174)

### Miscellaneous
- Various minor tweaks and refinements in multiple features
- Co-authored commits with Brett <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, and Denis Štogl <denis@stogl.de>

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
- Foxy backport (#206)
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
- Remove example things from Foxy CI setup. (#214)
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
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
- Update description table.
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Delete tracing directory
- Moved tracing.md to tracing directory
- Added setupTracing.sh
- Removed manual installation of ros-rolling-ros2trace
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Several fixes (#194)
- Tuning warehouse3 (#197)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Minor broken build
- Some reordering fixes
- Format issues
- Minor formatting fixes

Fixed
-----
- Headless and other fixes
- Default values
- Pure spinning behavior missing files
- More fixes
- Tuning and fixes (#202)
- Fix
- Minor linking errors foxy
- Minor format
- Minor linking errors foxy

Removed
-------
- Merge
- Minor
- Minor changes
- Minor tune
- Weird moveit not downloaded repo
- Missing
- Missing sm
- Updating subscriber publisher components
- Progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Autoware demo
- Foxy ci
- Minor
- Minor changes
- Replanning for all our examples
- Backport to foxy
- Minor format
- Minor linking errors foxy
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports
- Disable some packages and update workflows
- Ignore further packages
- Disable cpplint and cppcheck linters
- Correct formatters
- Branching example
- Disable disabled packages
``` 

*pabloinigoblasco*

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
----
- Bug in smacc2 component.
- Reverted markdowns to html.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed source CI and corrected README overview.
- Fixed pre-commit issues.
- Corrected all linters and formatters.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.
- Deleted tracing.md.

Other
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Some progress on navigation rolling.
- More changes on performance tests.
- Performance tests improvements.
- More on performance and other issues.
- Format cleanup in sm_respira_1.
- Format cleanup pre-commit in sm_respira_1.
- Format cleanup in sm_atomic_24hr.
- Format cleanup in sm_atomic_performance_trace_1.
- Cleaned up sm_atomic_24hr.
- More cleanup in sm_atomic_24hr.
- Cleaned up sm_reference_library.
- Cleaned up sm_advanced_recovery_1.
- More cleanup in sm_advanced_recovery_1.
- Reworked sm_advanced_recovery_1.
- More work on sm_advanced_recovery_1.
- Work on sm_atomic_performance_test_a_1.
- Work on sm_atomic_performance_test_a_2.
- Work on sm_atomic_performance_test_c_1.
- Work on sm_multi_stage_1.
- More work on sm_multi_stage_1.
- Progress in aws navigation demo.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in aws navigation.
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
- New feature: `cb_wait_topic_message`: Asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: Waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.
- Base for the `sm_aws_warehouse` navigation.
- Navigation parameters fixes on `sm_dance_bot`.
- New client behavior `cb_pause_slam`.
- `sm_dance_bot_lite` visualizing TurtleBot3.
- `sm_multi_stage_1` doubling.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Changed
-------
- Minor formatting improvements.
- Merge and progress.
- Minor hotfix.
- Cleaning and lidar show/hide option.
- Cleaning files and formatting work.
- Format fixes.
- Enable source build on PR for testing.
- Adjust build packages of source CI.

Fixed
-----
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting.

Removed
-------
- Removed `neo_simulation2` package.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

# Section 15

## Added
- Feature/diverse improvements in navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122): Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Introducing smacc2::deep_history syntax and testing slam pausing/resuming functionality.
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added remaining SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added QOS durability to SmaccPublisherClient (#163)
- Feature/aws navigation sm dance bot (#174): Progress on aws navigation, refactorings on navigation clients and behaviors, and husky simulation dependencies.

## Changed
- Move method after the method it calls to prevent recursion (#126)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Renamed sm_multi_stage_1 (#172) and sm_advanced_recovery_1 (#171)
- Renamed navigation 2 stack to nav2z (#144)
- Removed node creation and created only a logger (#149)
- Updated package list (#142)
- Removed parameters smacc (#147)
- Noticed launch command was incorrect in README.md and fixed it
- Fixed CI: format fix python version (#148)
- Minor navigation improvements (#141)
- Using local action messages instead of sm_dance_bot_msgs (#139)
- Fixed compiling issues
- Updated README (#164)
- More readme updates

## Fixed
- Minor tuning to mitigate overshot issue cases in waypoints navigator (#133)
- Fixed broken master build
- Fixed pipeline error

## Removed
- Removed merge markers from a python file (#119)
- Removed sm_dance_bot_msgs

## Contributors
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
- minor format
- minor linking errors foxy
- Foxy backport (#206)

Changed
-------
- SrConditional fixes and formatting (#168)
- fix: some formatting and templating on SrConditional
- fix: move trigger logic into headers
- fix: lint
- Fix code generators (#221)
- Fix other build issues.
- Update SM template and make example code clearly visible.
- Remove use of node in the sm performance template.
- Updated templated to use Blackboard storage.
- Update template to resolve the global data correctly.
- Update sm_name.hpp
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
- ros2 launch sm_three_some sm_three_some
- to
- ros2 launch sm_three_some sm_three_some.launch
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
```

```rst
Section_17
==========

Added
-----
- Dockerfile now accepts ROS distro as argument for easier setup.
- New setupTracing.sh script automates installation of necessary packages and configures tracing group.
- Added alternative ManualTracing option.
- New sm markdowns added.
- Performance tests improvements and cleanup.
- Added smacc2_performance_tools.
- Optimized dependencies in move_base_z_planners_common.
- New feature: cb_wait_topic_message for asynchronous client behavior.
- Feature/wait nav2 nodes client behavior for navigation improvements.

Changed
-------
- Renamed "smacc application" to "SMACC2 library" for clarity.
- Updated mentions of SMACC/ROS to SMACC2/ROS2 for consistency.
- Updated launch commands for various components.

Fixed
-----
- Reverted markdowns to html format for consistency.
- Fixed bug in smacc2 component.
- Corrected trailing spaces in code.
- Fixed source CI setup and corrected README overview.

Removed
-------
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Removed galactic builds, keeping only rolling. Submodules replaced with .repos file.
- Do not execute clang-format on smacc2_sm_reference_library package.

Authors
-------
- Pablo Iñigo Blasco
- Denis Štogl
- Ubuntu 20-04-02-amd64
```

```rst
Section_18
==========

Added
-----
- Introduce new feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for `nav2`, allowing waiting for nav2 nodes subscribing to the `/bond` topic and ensuring they are alive. Node selection is optional.
- Add base for the `sm_aws_aarehouse` navigation.
- Introduce `cb_pause_slam` client behavior.

Changed
-------
- Correct all linters and formatters.
- Fix navigation parameters on `sm_dance_bot`.
- Minor format improvements.
- Update yaml files.
- Rename doxygen deployment workflow.

Fixed
----
- Resolve compile warnings.

Removed
-------
- Eliminate some compile warnings.

Collaborators
-------------
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

Section_19
===========

Added
-----
- Added multistage modes to `sm_multi_stage_1`.
- Added sequences to `sm_multi_stage_1`.
- Added steps to `sm_multi_stage_1`.
- Added sequence d to `sm_multi_stage_1`.
- Added sequence c to `sm_multi_stage_1`.
- Added mode_5_sequence_b.
- Added mode_4_sequence_b.
- Added finishing touches to `sm_multi_stage_1`.
- Added AWS navigation to `sm_dance_bot`.

Changed
-------
- Renamed `sm_advanced_recovery_1`.
- Reworked `sm_multi_stage_1` with new sequences and steps.

Fixed
-----
- Fixed waypoint 4 and iterations for robot course completion.
- Fixed CI formatting for Python version.
- Fixed launch command in README.md for `sm_dance_bot_strikes_back`.
- Fixed compiling issues.

Removed
-------
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs`.
- Removed parameters from `smacc`.

Other
-----
- Co-authored with Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>, and pabloinigoblasco <pablo@ibrobotics.com>.
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others.
- Updated package list.
- Added SM Atomic SM generator.
- Rolled Docker environment for execution from any environment.
- Added QOS durability to `SmaccPublisherClient`.
- Added reliability QOS config to `SmaccPublisherClient`.
- Added durability QOS to `SmaccPublisherClient`.
- Added husky launch file in `sm_dance_bot`.
- Added dependencies for husky simulation.
- Progressed in navigation, slam toggle client behaviors, and slam_toolbox components.
- Progressed in testing `sm_dance_bot` with slam pausing/resuming functionality.
- Progressed in moveit migration testing.
- Progressed in `sm_dance_bot` refinement.
- Progressed in `sm_dance_bot_strikes_back` refactoring.
- Progressed in `sm_pubsub_1`.
- Progressed in `sm_advanced_recovery_1` renaming.
- Progressed in `sm_multi_stage_1` reworking.
- Progressed in testing moveit behaviors.
- Progressed in `sm_multi_stage_1` most.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` c sequence.
- Progressed in `sm_multi_stage_1` sequence b.
- Progressed in `sm_multi_stage_1` sequence a.
- Progressed in `sm_multi_stage_1` sequence z.
- Progressed in `sm_multi_stage_1` sequence y.
- Progressed in `sm_multi_stage_1` sequence x.
- Progressed in `sm_multi_stage_1` sequence w.
- Progressed in `sm_multi_stage_1` sequence v.
- Progressed in `sm_multi_stage_1` sequence u.
- Progressed in `sm_multi_stage_1` sequence t.
- Progressed in `sm_multi_stage_1` sequence s.
- Progressed in `sm_multi_stage_1` sequence r.
- Progressed in `sm_multi_stage_1` sequence q.
- Progressed in `sm_multi_stage_1` sequence p.
- Progressed in `sm_multi_stage_1` sequence o.
- Progressed in `sm_multi_stage_1` sequence n.
- Progressed in `sm_multi_stage_1` sequence m.
- Progressed in `sm_multi_stage_1` sequence l.
- Progressed in `sm_multi_stage_1` sequence k.
- Progressed in `sm_multi_stage_1` sequence j.
- Progressed in `sm_multi_stage_1` sequence i.
- Progressed in `sm_multi_stage_1` sequence h.
- Progressed in `sm_multi_stage_1` sequence g.
- Progressed in `sm_multi_stage_1` sequence f.
- Progressed in `sm_multi_stage_1` sequence e.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` sequence c.
- Progressed in `sm_multi_stage_1` sequence b.
- Progressed in `sm_multi_stage_1` sequence a.
- Progressed in `sm_multi_stage_1` sequence z.
- Progressed in `sm_multi_stage_1` sequence y.
- Progressed in `sm_multi_stage_1` sequence x.
- Progressed in `sm_multi_stage_1` sequence w.
- Progressed in `sm_multi_stage_1` sequence v.
- Progressed in `sm_multi_stage_1` sequence u.
- Progressed in `sm_multi_stage_1` sequence t.
- Progressed in `sm_multi_stage_1` sequence s.
- Progressed in `sm_multi_stage_1` sequence r.
- Progressed in `sm_multi_stage_1` sequence q.
- Progressed in `sm_multi_stage_1` sequence p.
- Progressed in `sm_multi_stage_1` sequence o.
- Progressed in `sm_multi_stage_1` sequence n.
- Progressed in `sm_multi_stage_1` sequence m.
- Progressed in `sm_multi_stage_1` sequence l.
- Progressed in `sm_multi_stage_1` sequence k.
- Progressed in `sm_multi_stage_1` sequence j.
- Progressed in `sm_multi_stage_1` sequence i.
- Progressed in `sm_multi_stage_1` sequence h.
- Progressed in `sm_multi_stage_1` sequence g.
- Progressed in `sm_multi_stage_1` sequence f.
- Progressed in `sm_multi_stage_1` sequence e.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` sequence c.
- Progressed in `sm_multi_stage_1` sequence b.
- Progressed in `sm_multi_stage_1` sequence a.
- Progressed in `sm_multi_stage_1` sequence z.
- Progressed in `sm_multi_stage_1` sequence y.
- Progressed in `sm_multi_stage_1` sequence x.
- Progressed in `sm_multi_stage_1` sequence w.
- Progressed in `sm_multi_stage_1` sequence v.
- Progressed in `sm_multi_stage_1` sequence u.
- Progressed in `sm_multi_stage_1` sequence t.
- Progressed in `sm_multi_stage_1` sequence s.
- Progressed in `sm_multi_stage_1` sequence r.
- Progressed in `sm_multi_stage_1` sequence q.
- Progressed in `sm_multi_stage_1` sequence p.
- Progressed in `sm_multi_stage_1` sequence o.
- Progressed in `sm_multi_stage_1` sequence n.
- Progressed in `sm_multi_stage_1` sequence m.
- Progressed in `sm_multi_stage_1` sequence l.
- Progressed in `sm_multi_stage_1` sequence k.
- Progressed in `sm_multi_stage_1` sequence j.
- Progressed in `sm_multi_stage_1` sequence i.
- Progressed in `sm_multi_stage_1` sequence h.
- Progressed in `sm_multi_stage_1` sequence g.
- Progressed in `sm_multi_stage_1` sequence f.
- Progressed in `sm_multi_stage_1` sequence e.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` sequence c.
- Progressed in `sm_multi_stage_1` sequence b.
- Progressed in `sm_multi_stage_1` sequence a.
- Progressed in `sm_multi_stage_1` sequence z.
- Progressed in `sm_multi_stage_1` sequence y.
- Progressed in `sm_multi_stage_1` sequence x.
- Progressed in `sm_multi_stage_1` sequence w.
- Progressed in `sm_multi_stage_1` sequence v.
- Progressed in `sm_multi_stage_1` sequence u.
- Progressed in `sm_multi_stage_1` sequence t.
- Progressed in `sm_multi_stage_1` sequence s.
- Progressed in `sm_multi_stage_1` sequence r.
- Progressed in `sm_multi_stage_1` sequence q.
- Progressed in `sm_multi_stage_1` sequence p.
- Progressed in `sm_multi_stage_1` sequence o.
- Progressed in `sm_multi_stage_1` sequence n.
- Progressed in `sm_multi_stage_1` sequence m.
- Progressed in `sm_multi_stage_1` sequence l.
- Progressed in `sm_multi_stage_1` sequence k.
- Progressed in `sm_multi_stage_1` sequence j.
- Progressed in `sm_multi_stage_1` sequence i.
- Progressed in `sm_multi_stage_1` sequence h.
- Progressed in `sm_multi_stage_1` sequence g.
- Progressed in `sm_multi_stage_1` sequence f.
- Progressed in `sm_multi_stage_1` sequence e.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` sequence c.
- Progressed in `sm_multi_stage_1` sequence b.
- Progressed in `sm_multi_stage_1` sequence a.
- Progressed in `sm_multi_stage_1` sequence z.
- Progressed in `sm_multi_stage_1` sequence y.
- Progressed in `sm_multi_stage_1` sequence x.
- Progressed in `sm_multi_stage_1` sequence w.
- Progressed in `sm_multi_stage_1` sequence v.
- Progressed in `sm_multi_stage_1` sequence u.
- Progressed in `sm_multi_stage_1` sequence t.
- Progressed in `sm_multi_stage_1` sequence s.
- Progressed in `sm_multi_stage_1` sequence r.
- Progressed in `sm_multi_stage_1` sequence q.
- Progressed in `sm_multi_stage_1` sequence p.
- Progressed in `sm_multi_stage_1` sequence o.
- Progressed in `sm_multi_stage_1` sequence n.
- Progressed in `sm_multi_stage_1` sequence m.
- Progressed in `sm_multi_stage_1` sequence l.
- Progressed in `sm_multi_stage_1` sequence k.
- Progressed in `sm_multi_stage_1` sequence j.
- Progressed in `sm_multi_stage_1` sequence i.
- Progressed in `sm_multi_stage_1` sequence h.
- Progressed in `sm_multi_stage_1` sequence g.
- Progressed in `sm_multi_stage_1` sequence f.
- Progressed in `sm_multi_stage_1` sequence e.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` sequence c.
- Progressed in `sm_multi_stage_1` sequence b.
- Progressed in `sm_multi_stage_1` sequence a.
- Progressed in `sm_multi_stage_1` sequence z.
- Progressed in `sm_multi_stage_1` sequence y.
- Progressed in `sm_multi_stage_1` sequence x.
- Progressed in `sm_multi_stage_1` sequence w.
- Progressed in `sm_multi_stage_1` sequence v.
- Progressed in `sm_multi_stage_1` sequence u.
- Progressed in `sm_multi_stage_1` sequence t.
- Progressed in `sm_multi_stage_1` sequence s.
- Progressed in `sm_multi_stage_1` sequence r.
- Progressed in `sm_multi_stage_1` sequence q.
- Progressed in `sm_multi_stage_1` sequence p.
- Progressed in `sm_multi_stage_1` sequence o.
- Progressed in `sm_multi_stage_1` sequence n.
- Progressed in `sm_multi_stage_1` sequence m.
- Progressed in `sm_multi_stage_1` sequence l.
- Progressed in `sm_multi_stage_1` sequence k.
- Progressed in `sm_multi_stage_1` sequence j.
- Progressed in `sm_multi_stage_1` sequence i.
- Progressed in `sm_multi_stage_1` sequence h.
- Progressed in `sm_multi_stage_1` sequence g.
- Progressed in `sm_multi_stage_1` sequence f.
- Progressed in `sm_multi_stage_1` sequence e.
- Progressed in `sm_multi_stage_1` sequence d.
- Progressed in `sm_multi_stage_1` sequence c.
- Progressed in `sm_multi_stage_1` sequence b.
- Progressed in `sm_multi_stage_1` sequence a.
- Progressed in `sm_multi_stage_1` sequence z.
- Progressed in `sm_multi_stage_1` sequence y.
- Progressed in `sm_multi_stage_1` sequence x.
- Progressed in `sm_multi_stage_1` sequence w.
- Progressed in `sm_multi_stage_1` sequence v.
- Progressed in `sm_multi_stage_1` sequence u.
- Progressed in `sm_multi_stage_1` sequence t.
- Progressed in `sm_multi_stage_1` sequence s.
- Progressed in `sm_multi_stage_1` sequence r.
- Progressed in `sm_multi_stage_1` sequence q.
- Progressed in `sm_multi_stage_1` sequence p.
- Progressed in `sm_multi_stage_1` sequence o.
- Progressed in `sm_multi_stage_1` sequence n.
- Progressed in `sm_multi_stage_1` sequence m.
- Progressed in `sm_multi_stage_1` sequence l.
- Progressed in `sm_multi_stage_1` sequence k.
- Progressed in `sm_multi_stage_1` sequence j.
- Progressed in `sm_multi_stage_1

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
- Redoing sm_dance_bot_warehouse_3 waypoints.
- More Waypoints.
- Several fixes.
- Improving undo motion navigation warehouse2.
- Finetuning waypoints.
- Tuning and fixes.
- Finishing warehouse2.
- Tuning and fixes.
- Fixing warehouse 3 problems and other core improvements to remove dead lock, also making continuous integration green.
- Added missing file from warehouse2.
- Updating subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components mostly developed for autoware demo.
- Autoware demo.
- Retry behavior warehouse 1.
- Update file for fake hardware simulation and add file for gazebo simulation.
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

- Default values.
- Several fixes.
- Format issues.
- Minor tune.
- Minor format fix.
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

- Only rolling version should be pre-released on on master.

Co-authored-by
-------------

- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```rst
Section_21
==========

Added
-----
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Added setupTracing.sh.
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Update smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Update tracing/ManualTracing.md.
- Update smacc_sm_reference_library/sm_atomic/README.md.
  Edit from html to markdown syntax.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Update mentions of SMACC/ROS to SMACC2/ROS2.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Added sm_atomic_performance_trace_1.
- Update smacc2_rta command across readmes.
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
- Progressing in aws navigation.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Feature/aws demo progress (#80).
- More on navigation.
- Sm_advanced_recovery_1 reworked (#83).
- Fix pre-commit.
- More sm_advanced_recovery_1 (#84).
- More sm_advanced_recovery_1 work (#85).
- Sm_advanced_recovery_1 round 4 (#86).
- Brettpac branch (#87).
- Sm_atomic_performance_test_a_2.
- Sm_atomic_performance_test_a_1.
- Sm_atomic_performance_test_c_1 (#88).

Changed
-------
- Ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Rename header files and correct format.
- Update doxygen-check-build.yml.
- Change extension of imports.
- Update ci-build-source.yml.
- Correct GitHub branch reference.
- Update name of package and package.xml to pass liter.
- Execute on master update.
- Reset all versions to 0.0.0.
- Update changelogs.
- Changed wording "smacc application" to "SMACC2 library".
- Reactivating smacc2 nav clients for rolling via submodules.
- Renamed tracing events after.
- Bug in smacc2 component.
- Added README tutorial for Dockerfile.
- Additional cleanup.
- Cleanup.
- Edited tracing.md to reflect new tracing event names.
- Minor formatting.
- Correct trailing spaces.
- Clean up of sm_atomic_24hr.
- More sm_atomic_24hr cleanup.
- Minor formatting improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements.
- Format improvements

```rst
Section_22
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #82, #92, #93, #94, #95, #98)
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- New feature: cb pause slam client behavior (#98)
- New feature: sm_dance_bot_lite (#99)
- New feature: sm_dance_bot visualizing turtlebot3 (#101)
- New feature: dance bot launch gz lidar choice (#102)

Changed
-------

- Updated launch command
- Corrected all linters and formatters
- Navigation parameters fixes on sm_dance_bot
- Minor formatting improvements
- Merge and progress
- Fix format
- Minor hotfix
- Updates yaml

Fixed
-----

- Several core improvements during navigation testing
- Progress in aws navigation demo
- Formatting improvements
- Fixed precommit
- Removed some compile warnings

Authors
-------

- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_23
==========

Added
-----
- Added `sm_dance_bot` visualizing `turtlebot3`.
- Added lidar show/hide option for cleaning.
- Added formatting improvements to cleaning files.
- Added gazebo fixes for `sm_dance_bot_lite` (#104).
- Added gazebo fixes to show the robot and lidar.
- Added gazebo fixes for `sm_dance_bot_strikes_back`.
- Added AWS demo (#108).
- Added progress in `sm_multi_stage_1` (#109).
- Added `Brettpac` branch (#110).
- Added progress in `sm_multi_stage_1`, including multiple stages (#111).
- Added `a3` (#113).
- Added diverse improvements in navigation and performance (#116).
- Added `slam toggle` and `smacc deep history` features (#122).
- Added progress in navigation, `slam toggle` client behaviors, and `slam_toolbox` components.
- Added `smacc2::deep_history` syntax.
- Added `dance bot s pattern` feature (#128).
- Added polishing to `sm_dance_bot` and `s-pattern`.
- Added `dance bot s pattern` feature (#129).
- Added more refinement to `sm_dance_bot`.
- Added first working version of `sm template` and `template generator` (#127).
- Added `sm dance bot refine` feature (#131).
- Added `sm dance bot refine 2` feature (#132).
- Added build fix.
- Added `waypoints navigator` bug fix (#133).
- Added progress in `sm_dance_bot` tests (#135).
- Added minor navigation improvements (#141).
- Added `SM core test` (#138).
- Added `nav2z` renaming (#144).
- Added navigation 2 stack renaming.
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added rolling Docker environment execution from any environment (#154).
- Added refactoring to `sm dance bot strikes back` (#152).
- Added slight changes to waypoint 4 and iterations for robot course completion (#155).
- Added `migration moveit client` feature (#151).
- Added initial migration to `smacc2`.
- Added fixing errors introduced on formatting.
- Added missing dependency.
- Added fixing linting warnings.
- Added removing test from main `moveit` CMake.
- Added progressing in `moveit` migration testing.
- Added updating format.
- Added `.reps` dependencies and fixing build errors.
- Added repos dependency.
- Added adding dependency to `ur5` client.
- Added docker refactoring.
- Added progress on `move_it` PR.
- Added improving dockerfile for building local tests.
- Added fixing compiling issues.
- Added updating README (#164).

Changed
-------
- Changed `neo_simulation2` package removal (#112).
- Changed method order to prevent recursion (#126).
- Changed `neo_simulation2` package removal.
- Changed source build on PR for testing.
- Changed build packages of source CI.
- Changed launch command in `README.md` for `sm_dance_bot_strikes_back`.
- Changed CI format fix for Python version (#148).
- Changed node creation to create only a logger (#149).

Removed
-------
- Removed `neo_simulation2` package.
- Removed `sm_dance_bot_msgs`.
- Removed parameters from `smacc`.
- Removed merge markers from a Python file (#119).

Fixed
-----
- Fixed format issues.
- Fixed overshot issue cases in `waypoints navigator`.
- Fixed compile warnings.
- Fixed CI workflows.
- Fixed launch command in `README.md` for `sm_dance_bot_strikes_back`.
- Fixed some errors introduced on formatting.
- Fixed some linting warnings.

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>.
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.

Author
------
- Pablo Iñigo Blasco (pabloinigoblasco).

```rst
Section_24
==========

Added
-----
- Initial state machine transition timestamp (#165)
- QOS durability to SmaccPublisherClient (#163)
- Feature: testing moveit behaviors (#167)
- Feature: AWS navigation sm dance bot (#174)
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- SrConditional fixes and formatting (#168)
- Feature: warehouse2 dec 14 (#185)
- Feature: sm warehouse 2 13 dec 2 (#186)
- Finetuning waypoints (#187)
- Feature: cb pure spinning (#188)
- Feature: planner changes 16 12 (#191)
- Feature: replanning 16 dec (#193)
- Several fixes (#194)
- Feature: undo motion 20 12 (#196)
- Feature: sync 21 12 (#199)
- Feature: warehouse2 22 12 (#200)
- Feature: warehouse2 23 12 (#201)
- Feature: minor tune (#203)
- Fixing warehouse 3 problems, and other core improvements (#204)
- Added missing file from warehouse2 (#205)

Changed
-------
- Moved reference library SMs to smacc2_performance_tools
- Added reliability QOS config
- Progress on moveit
- More testing on moveit behaviors
- More on AWS demo
- Improving undo motion navigation warehouse2
- Tuning warehouse3
- Improvements in SMACC core adding more components mostly developed for autoware demo
- Refining CP subscriber CP publisher
- Progress in autoware machine
- Docker files for different revisions, warnings removal, and more testing on navigation
- Fixing Docker for Foxy and Galactic
- Barrel search build fix and warehouse3
- Fixing startup problems in warehouse 3
- Progress in barrel husky
- Progress in barrel demo

Fixed
-----
- Add a missing colon
- Fixing pipeline error
- Fixing broken master build
- Fixing broken build
- Fixing warehouse 3 problems, and other core improvements to remove deadlock, also making continuous integration green
- Weird moveit not downloaded repo
- Foxy CI
- Minor broken build
- Some reordering fixes
- Fixing format and minor
- Fixing broken build
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
- Performance tests improvements
- Cleanup and optimization in various packages

### Removed
- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`

### Miscellaneous
- Reverted changes in commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Reset all versions to 0.0.0
- Ignored all packages except `smacc2` and `smacc2_msgs`
- Updated changelogs

### Docker
- Docker improvements
- Added `README` tutorial for Dockerfile
- Command for building Docker image: `sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/`

### Dependencies
- Updated runtime dependencies
- Restored `ur` dependency
- Optimized dependencies in `move_base_z_planners_common`

### Branching
- Master rolling to galactic backport
- Reactivated `smacc2` nav clients for rolling via submodules
- Use `galactic` branches in `.repos-file`

### Code Quality
- Refactored code for better readability and maintainability
- Corrected trailing spaces
- Reformatting and cleanup in various packages
- Renamed event generator library
- Unified CI configurations
- Used `tf_geometry_msgs.h` in `galactic`

Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco
```
