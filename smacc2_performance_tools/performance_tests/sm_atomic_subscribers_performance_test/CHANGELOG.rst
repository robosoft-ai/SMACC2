Changelog for package sm_atomic_subscribers_performance_test
===============================================================

Version 2.3.16 (2023-07-16)
---------------------------
- Merged branch 'humble' from https://github.com/robosoft-ai/SMACC2 into humble
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for a strange issue with ros buildfarm
  - Further work on the buildfarm issue
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
No significant changes.

Version 1.22.1 (2022-11-09)
---------------------------
- Pre-release
- Contributors: pabloinigoblasco

- Feature/galactic rolling merge (#288)
  - Various updates and improvements related to rolling and Galactic versions
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Wait topic message client behavior (#81)
  - Base for the sm_aws_aarehouse navigation
  - Progress in AWS navigation
  - Several core improvements during navigation testing
  - Formatting improvements

- Other minor changes and improvements related to performance tests, cleanup, and navigation demos.

```rst
Section_2
=========

Added
-----

- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `wait nav2 nodes` subscribing to the `/bond` topic and waiting for them to be alive, with optional node selection
- New client behavior: `cb pause slam` for pausing SLAM operations

Changed
-------

- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Gazebo fixes to show the robot and lidar
- Cleaning and lidar show/hide option for `sm_dance_bot` visualizing TurtleBot3
- Several core improvements during navigation testing

Fixed
-----

- Corrected all linters and formatters
- Removed some compile warnings

Removed
-------

- None

Contributors
------------

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

Commits
-------

- Feature/wait nav2 nodes client behavior (#82)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)
- Remove some compile warnings. (#96)
- Feature/cb pause slam (#98)
- Rename doxygen deployment workflow (#100)
- sm_dance_bot_lite (#99)
- sm_dance_bot visualizing turtlebot3 (#101)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- sm_multi_stage_1 doubling (#103)
- Feature/sm dance bot strikes back gazebo fixes (#105)
```

```rst
Section_3
=========

Added
-----
- Added gazebo fixes to display the robot and lidar.
- Added AWS demo (#108).
- Added Brettpac branch (#110).
- Added progress in sm_multi_stage_1 (#114).
- Added diverse improvements in navigation and performance (#116).
- Added feature to toggle slam and deep history in smacc2 (#122).
- Added first working version of sm template and template generator (#127).
- Added SM core test (#138).
- Added minor navigation improvements (#141).
- Added SVGs to READMEs of atomic, dance_bot, and others (#140).
- Added remaining SVGs to READMEs (#145).
- Added SM Atomic SM generator (#143).
- Added rolling Docker environment for execution in any environment (#154).
- Added initial state machine transition timestamp (#165).
- Added QOS durability to SmaccPublisherClient (#163).
- Added feature for testing moveit behaviors (#167).

Changed
-------
- Updated package list (#142).
- Renamed navigation 2 stack.
- Refactored sm dance bot strikes back (#152).
- Updated readme (#164).
- Moved reference library SMs to smacc2_performance_tools (#166).

Fixed
-----
- Fixed formatting issues.
- Fixed launch command in README.md.
- Fixed CI format for Python version (#148).
- Fixed node creation to create only a logger (#149).
- Fixed compile warnings (#137).
- Fixed minor format issues (#134).
- Fixed minor tuning to mitigate overshot issue cases in waypoints navigator (#133).
- Fixed minor format issues.
- Fixed missing dependency in migration moveit client (#151).
- Fixed errors introduced on formatting in migration moveit client.
- Fixed linting warnings in migration moveit client.
- Fixed compiling issues.
- Fixed broken master build.

Removed
-------
- Removed neo_simulation2 package.
- Removed parameters in smacc.
- Removed test from main moveit CMake.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_4
=========

Added
-----
- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency
  - Added husky launch file in sm_dance_bot
  - Added dependencies for husky simulation
  - Fixed formatting
  - Updated dependencies for husky in rolling and galactic
  - Made progress on aws navigation and refactorings on navigation clients and behaviors
  - Added more on aws demo
  - Fixed broken build

- Feature/planner changes 16 12 (#191)
  - Made minor changes
  - Added more fixes

- Feature/replanning 16 dec (#193)
  - Made minor changes
  - Replanned for all examples
  - Made several fixes (#194)

Changed
-------
- SrConditional fixes and formatting (#168)
  - Fixed some formatting and templating on SrConditional
  - Moved trigger logic into headers
  - Linted

- Feature/undo motion 20 12 (#196)
  - Made minor changes
  - Replanned for all examples
  - Improved undo motion navigation in warehouse2

- Feature/sync 21 12 (#199)
  - Made minor changes
  - Replanned for all examples
  - Fixed format issues

Removed
-------
- Removed trailing spaces, corrected codespell, and Python linters warnings
- Added missing file from warehouse2 (#205)
- Merged code from backport foxy and updates about autoware (#208)
- Added galactic CI build because Navigation2 is broken in rolling
- Added partial changes for ament_cpplint
- Added tf2_ros as dependency to find include
- Disabled ament_cpplint
- Disabled some packages and updated workflows
- Bumped ccache version
- Ignored further packages
- Satisfied ament_lint_cmake
- Added missing licenses
- Disabled cpplint and cppcheck linters
- Corrected formatters
- Updated ci-build-source.yml
- Changed extension of imports
- Enabled cppcheck
- Corrected formatting of Python file
- Included necessary package and edited Threesome launch

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
``` 

*pabloinigoblasco*

```rst
Section 5
=========

Added
-----
- Opened new folder for additional tracing contents.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More on performance and other issues.
- Optimized dependencies in move_base_z_planners_common.
- Renaming of event generator library.
- Added galactic CI setup and renamed rolling files.
- Fixed source CI and corrected README overview.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More Readme Updates.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Feature/aws demo progress.
- Feature/wait nav2 nodes client behavior.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.

Changed
-------
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.

Fixed
-----
- Bug in smacc2 component.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Corrected trailing spaces.
- Minor formatting fixes.
- Several core improvements during navigation testing.
- Format improvements.
- Attempted precommit fixes.

Removed
-------
- Removed galactic builds from master and kept only rolling.
- Removed submodules and use .repos file.

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
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and confirm they are active. Optional node selection available.
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Navigation parameters fixes on sm_dance_bot.
- cb_pause_slam client behavior.
- sm_dance_bot_lite.
- sm_dance_bot visualizing turtlebot3.
- Feature/aws demo progress (#92).
- Feature/sm dance bot fixes (#93).
- Feature/sm aws warehouse (#94).
- Feature/sm dance bot fixes (#95).
- Remove some compile warnings (#96).
- Feature/cb pause slam (#98).
- Rename doxygen deployment workflow (#100).
- Feature/dance bot launch gz lidar choice (#102).
- Feature/sm dance bot lite gazebo fixes (#104).
- Feature/sm dance bot strikes back gazebo fixes (#105).
- Precommit cleanup run (#106).
- aws demo (#108).
- Brettpac branch (#110).
- Brettpac branch (#111).

Changed
-------
- Corrected all linters and formatters.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Formatting improvements.

Fixed
-----
- Minor format fixes.
- Fix format.

Removed
-------
- Minor hotfix.
```

## Section_7

### Added
- Added source build on PR for testing.
- Added progress in navigation, slam toggle client behaviors, and slam_toolbox components.
- Added smacc2::deep_history syntax.
- Added slam pausing/resuming functionality to sm_dance_bot.
- Added first working version of sm template and template generator.
- Added SM core test.
- Added local action messages.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.
- Added SM Atomic SM generator.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS config.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.

### Changed
- Changed method order to prevent recursion in sm_dance_bot.
- Changed "Finnaly" to "Finally" for correct spelling.
- Changed navigation 2 stack naming.
- Changed launch command in README.md for sm_dance_bot_strikes_back.
- Changed format to fix Python version in CI.
- Changed node creation to create only a logger.
- Changed Docker environment to be executed from any environment.

### Fixed
- Fixed minor format issues.
- Fixed launch command in README.md.
- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.

### Removed
- Removed neo_simulation2 package.
- Removed merge markers from a Python file.
- Removed parameters smacc.
- Removed sm_dance_bot_msgs.
- Removed test from main moveit CMake.

### Miscellaneous
- Minor tweaks and improvements.
- Minor format adjustments.
- Minor tuning to mitigate overshot issue cases.
- Minor navigation improvements.
- Minor linting and formatting.
- Minor build fixes.
- Minor configuration adjustments.
- Precommit cleanup.
- Pending references cleanup.
- Noticed and fixed typos.
- Noticed and fixed missing dependencies.
- Noticed and fixed missing dependency to ur5 client.
- Noticed and fixed some linting warnings.
- Noticed and fixed some build errors.
- Noticed and fixed some more linting warnings.
- Noticed and fixed some more build errors.
- Noticed and fixed some more formatting errors.
- Noticed and fixed some more readme updates.
- Noticed and fixed some more test errors.
- Noticed and fixed some more Dockerfile issues.
- Noticed and fixed some more compiling issues.
- Noticed and fixed some more pipeline errors.

```rst
Section_8
=========

Added
-----
- Progress on AWS navigation and some other refactorings on navigation clients and behaviors.
- More on AWS demo.
- Warehouse2 progress (#177).
- Waypoint Inputs (#178).
- Warehouse2 progress (#179).
- Format (#180).
- Sm_dance_bot_warehouse_3 (#181).
- Feature/sm warehouse 2 13 dec 2 (#182).
- Finetuning waypoints (#187).
- Feature/cb pure spinning (#188).
- Pure spinning behavior missing files.
- Feature/planner changes 16 12 (#191).
- Feature/replanning 16 dec (#193).
- Replanning for all our examples.
- Several fixes (#194).
- Feature/undo motion 20 12 (#196).
- Improving undo motion navigation warehouse2.
- Tuning warehouse3 (#197).
- Feature/undo motion 20 12 (#198).
- Undo tuning and errors.
- Feature/sync 21 12 (#199).
- Format issues.
- Feature/warehouse2 22 12 (#200).
- Finishing warehouse2.
- Feature/warehouse2 23 12 (#201).
- Tuning and fixes (#202).
- Feature/minor tune (#203).
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green.
- Added missing file from warehouse2 (#205).
- Add mergify rules file.
- Try fixing CI for rolling. (#209).
- Merging to get backport working.
- Remove example things from Foxy CI setup. (#214).
- Add Autoware Auto Msgs into not-released dependencies. (#220).
- Fix rolling builds (#222).
- Do not merge yet - Feature/odom tracker improvements and retry motion (#223).
- Odom tracker improvements.
- Adding forward behavior retry functionality.
- Removing warnings (#213).
- Backport to Foxy.
- Foxy backport (#206).
- Fix trailing spaces.
- Correct codespell.
- Correct Python linters warnings.
- Add Galactic CI build because Navigation2 is broken in rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licenses.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Branching example.
- Disable disabled packages.
- Update ci-build-source.yml.
- Change extension.
- Change extension of imports.
- Enable cppcheck.
- Correct formatting of Python file.
- Included necessary package and edited Threesome launch.

Changed
-------
- Fixing broken build.
- Minor changes (#175).
- Default values.
- Minor.
- More changes and headless.
- Merge.
- Headless and other fixes.
- Fix: some formatting and templating on SrConditional.
- Fix: move trigger logic into headers.
- Fix: lint.
- Minor changes.
- Minor formatting fixes.
- Minor linking errors Foxy.

Removed
-------
- Minor broken build.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```
Section_9
=========

Added
-----

- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Update description table
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
- Update README.md with new launch command
- Update doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- More on navigation
- Sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_c_1 (#88)
- Sm_multi_stage_1 (#90)
- Update README.md with updated launch command
- Wait topic message client behavior (#81)
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success

Changed
-------

- Correct GitHub branch reference
- Changed wording "smacc application" to "SMACC2 library"
- Reactivating smacc2 nav clients for rolling via submodules
- Updated smacc2_rta command across readmes
- Clean up of sm_atomic_24hr
- More sm_atomic_24hr cleanup
- Minor formatting changes

Fixed
-----

- Bug in smacc2 component
- Reverted markdowns to html
- Do not execute clang-format on smacc2_sm_reference_library package
- Sm_reference_library reformatting
- Correct trailing spaces
- Fixing precommit issues

Removed
-------

- Ignore all packages except smacc2 and smacc2_msgs
- Removed manual installation of ros-rolling-ros2trace
- Deleted tracing directory
- Removed tracing.md
- Removed tracing events after
- Deleted tracing.md
- Cleanup of redundant files
```

```rst
Section_10
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection.
- New feature: cb_pause_slam client behavior.
- New feature: sm_dance_bot_lite gazebo fixes, including showing the robot and the lidar.
- New feature: gazebo fixes for sm_dance_bot_strikes_back.

Changed
-------

- Corrected all linters and formatters.
- Minor format improvements.
- Navigation parameters fixes on sm_dance_bot.
- Cleaning and lidar show/hide option for sm_dance_bot visualizing turtlebot3.
- Updates to yaml files.
- Minor hotfixes.

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

```rst
Section_11
==========

Added
-----

- Added AWS demo.
- Added Brettpac branch (#110, #111).
- Added a3 (#113).
- Added diverse improvements in navigation and performance (#116).
- Added feature to toggle SLAM and SMACC deep history (#122).
- Added feature for refining SM dance bot (#131, #132).
- Added SM core test (#138).
- Added minor navigation improvements (#141).
- Added SM Atomic SM generator (#143).
- Added rolling Docker environment execution from any environment (#154).
- Added initial state machine transition timestamp (#165).
- Added QOS durability to SmaccPublisherClient (#163).
- Added feature for testing MoveIt behaviors (#167).
- Added SM pubsub 1 (#169) and part 2 (#170).
- Renamed sm_advanced_recovery_1 to sm_pubsub_1 (#171).

Changed
-------

- Updated package list (#142).
- Moved reference library SMs to smacc2_performance_tools (#166).

Fixed
-----

- Fixed formatting in "Remove neo_simulation2 package" (#112).
- Fixed method order to prevent recursion (#126).
- Fixed launch command in README.md for sm_dance_bot_strikes_back.
- Fixed CI format for Python version (#148).
- Fixed node creation to only create a logger (#149).
- Fixed minor issues in README updates.
- Fixed compiling issues.

Removed
-------

- Removed neo_simulation2 package.
- Removed parameters in SMACC.
- Removed test from main MoveIt CMake.

Authors
-------

- Pablo Iñigo Blasco (@pabloinigoblasco)
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_12
==========

Added
~~~~~
- Feature/aws navigation sm dance bot (#174)
  - Added multistage modes and sequences for sm_multi_stage_1 reworking.
  - Included repo dependency and husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Updated dependencies for husky in rolling and galactic.
  - Improved AWS navigation and refactored navigation clients and behaviors.

- Feature/wharehouse2 dec 14 (#185)
  - Added progress on warehouse2.
  
- Feature/sm warehouse 2 13 dec 2 (#186)
  - Added more changes and headless mode.
  - Merged changes and fixed default values.

Changed
~~~~~~~
- SrConditional fixes and formatting (#168)
  - Improved formatting and templating on SrConditional.
  - Moved trigger logic into headers.
  - Linted code.

- Feature/undo motion 20 12 (#196)
  - Enhanced undo motion navigation for warehouse2.
  
- Feature/sync 21 12 (#199)
  - Resolved format issues.

- Feature/warehouse2 22 12 (#200)
  - Fixed format issues.
  - Finished warehouse2.

- Feature/warehouse2 23 12 (#201)
  - Tuned and fixed issues.

- Feature/minor tune (#203)
  - Tuned and fixed minor issues.

Fixed
~~~~~
- Fix code generators (#221)
  - Resolved build issues.
  - Updated SM template and example code visibility.
  - Removed node usage in the sm performance template.
  - Updated template to use Blackboard storage.
  - Corrected global data resolution in the template.
  - Updated sm_name.hpp.

Removed
~~~~~~~
- Foxy backport (#206)
  - Removed trailing spaces.
  - Corrected codespell and Python linters warnings.
  - Added galactic CI build due to Navigation2 issues in rolling.
  - Partially updated ament_cpplint changes.
  - Added tf2_ros as a dependency.
  - Disabled ament_cpplint and some packages in workflows.
  - Updated ccache version.
  - Ignored further packages.

Collaborators
~~~~~~~~~~~~~
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
Section_13
==========

Added
-----
- Satisfy ament_lint_cmake.
- Add missing licenses.
- Enable cppcheck.
- Update ci-build-source.yml.
- Add workflow for checking doc build.
- Create doxygen-deploy.yml.
- Create workflow for testing prerelease builds.
- Use docs/ as source folder for documentation.
- Use docs/ as output directory.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Add smacc2_performance_tools.
- Performance tests improvements.
- Optimized deps in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files.
- Fix source CI and correct README overview.
- Update c_cpp_properties.json.
- Update doxygen links.
- More Readme Updates.
- Created new sm from sm_respira_1.
- Feature/core and navigation fixes.
- Feature/aws demo progress.
- sm_advanced_recovery_1 reworked.
- Brettpac branch.
- sm_atomic_performance_test_a_2.
- sm_atomic_performance_test_a_1.
- sm_atomic_performance_test_c_1.
- Modifying sm_atomic_performance_test_a_2.

Changed
-------
- Correct formatters.
- Change extension of imports.
- Change extension.
- Update name of package and package.xml to pass liter.
- Reset all versions to 0.0.0.
- Update description table.
- Update table.
- Update smacc2_rta command across readmes.
- Clean up of sm_atomic_24hr.
- Minor formatting.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on navigation rolling.
- More changes on performance tests.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Sm_reference_library reformatting.
- Correct trailing spaces.
- Reactivating smacc2 nav clients for rolling via submodules.
- Bug in smacc2 component.
- Reverted markdowns to html.
- More on performance and other issues.
- Format improvements.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- More on navigation.

Removed
-------
- Disable cpplint and cppcheck linters.
- Disable disabled packages.
- Ignore all packages except smacc2 and smacc2_msgs.
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Delete tracing directory.
- Moved tracing.md to tracing directory.
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh.

Co-Authored-By
--------------
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_14
==========

Added
~~~~~
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success (#81, #91, #92, #93, #94, #95, #98)
- New client behavior for nav2: wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive, with optional node selection (#82, #92, #93, #94, #95, #98)
- New client behavior: cb_pause_slam (#98)
- New client behavior: sm_dance_bot_lite (#99)

Changed
~~~~~~~
- Updated launch command in README.md
- Corrected all linters and formatters (#82)
- Minor format improvements in navigation progress and demo (#92, #93, #94, #95, #98)
- Navigation parameters fixes on sm_dance_bot (#95, #98)
- Cleaning and lidar show/hide option in dance bot launch gz lidar choice (#102)

Fixed
~~~~
- Fixed precommit issues (#90, #91, #99)
- Removed some compile warnings (#96)
- Minor hotfix in doxygen deployment workflow (#100)
- Fixed formatting in sm_dance_bot visualizing turtlebot3 (#101)
``` 

*pabloinigoblasco*

Section_15
===========

Added
-----
- Feature/sm dance bot lite gazebo fixes (#104)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- aws demo (#108)
- Brettpac branch (#110)
- a3 (#113)
- mm (#115)
- diverse improvements navigation and performance (#116)
- Feature/diverse improvemets navigation performance (#117)
- Remove neo_simulation2 package. (#112)
- Move method after the method it calls. Otherwise recursion could happen. (#126)
- Feature/dance bot s pattern (#128)
- First working version of sm template and template generator. (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- added SVGs to READMEs of atomic, dance_bot, and others (#140)
- added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Fix CI: format fix python version (#148)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
- initial migration to smacc2
- update readme (#164)
- initial state machine transition timestamp (#165)
- moved reference library SMs to smacc2_performance_tools (#166)

Changed
-------
- sm_dance_bot visualizing turtlebot3
- cleaning and lidar show/hide option
- cleaning files and making formatting work
- more fixes
- gazebo fixes, to show the robot and the lidar
- format fixes
- sm_multi_stage_1 doubling (#103)
- got sm_multi_stage_1 working (barely) (#109)
- got sm_multi_stage_1 working (barely)
- gaining traction sm_multi_stage_1
- more
- don't remember
- making progress
- More
- keep hammering
- two stages
- 3 part
- 4th stage
- 5th stage
- additional linting and formatting
- polishing sm_dance_bot and s-pattern
- noticed typo
- Finnaly > Finally
- more refinement in sm_dance_bot
- minor tweaks
- minor navigation improvements
- using local action msgs
- removing sm_dance_bot_msgs
- pending references
- navigation 2 stack renaming
- formatting
- workflows update
- workflow
- Noticed launch command was incorrect in README.md
- fixed launch command for sm_dance_bot_strikes_back and removed some comments I had made in the past.
- missing dependency
- fixing some more linting warnings
- removing test from main moveit cmake
- test ur5
- progressing in the moveit migration testing
- updating format
- adding .reps dependencies and also fixing some build errors
- repos dependency
- adding dependency to ur5 client
- docker refactoring
- progress on move_it PR
- minor dockerfile test workaround
- improving dockerfile for building local tests
- fixing compiling issues
- more readme updates

Fixed
-----
- Remove neo_simulation2 package.
- Correct formatting.
- Adjust build packages of source CI
- minor format issues
- minor tuning to mitigate overshot issue cases
- some more progress on markers cleanup

Removed
-------
- Remove neo_simulation2 package.
- removing parameters smacc

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

Section_16
===========

Added
-----

- Added QOS durability to SmaccPublisherClient (#163).
- Added reliability QOS configuration.
- Added feature for testing moveit behaviors (#167).
- Added multistage modes, sequences, steps, and sequences D and C to sm_multi_stage_1 (#172).
- Added mode 5 sequence B and mode 4 sequence B to sm_multi_stage_1.
- Added finishing touches 1 to sm_multi_stage_1.
- Added repo dependency and husky launch file to sm_dance_bot.
- Added dependencies for husky simulation and updated dependencies for husky in rolling and galactic.
- Added progress on AWS navigation, refactorings on navigation clients and behaviors, and more on AWS demo.
- Added warehouse2 progress, waypoint inputs, and progress on sm_dance_bot_warehouse_3.
- Added redoing waypoints and more waypoints to sm_dance_bot_warehouse_3.
- Added SrConditional fixes and formatting.
- Added warehouse2 and minor changes.
- Added finetuning waypoints.
- Added pure spinning behavior, missing files, and minor changes.
- Added planner changes and more fixes.
- Added replanning for all examples and several fixes.
- Added improving undo motion navigation in warehouse2.
- Added tuning warehouse3.
- Added format issues to sync 21 12 and warehouse2 22 12.
- Added finishing warehouse2.
- Added tuning and fixes to minor tune.
- Added fixing warehouse 3 problems and other core improvements.
- Added backport to foxy and minor format and linking errors for foxy.
- Added missing, updating subscriber publisher components, progress in autoware machine, refining cp subscriber cp publisher, improvements in smacc core, autoware demo, foxy CI, and fix.
- Added docker files for different revisions, warnings removal, and more testing on navigation.
- Added fixing docker for foxy and galactic, update file for fake hardware simulation, and add file for gazebo simulation.
- Added docker build files for all versions.
- Added retry behavior warehouse 1, missing file, and minor format fix.
- Added other minor changes.

Changed
-------

- Moved reference library SMs to smacc2_performance_tools.
- Pre-commit cleanup.
- Added a missing colon.
- Removed a line.
- Moved trigger logic into headers.
- Linted code.

Fixed
-----

- Fixed pipeline error and broken master build.
- Fixed broken build.
- Fixed some formatting and templating on SrConditional.
- Fixed warehouse 3 problems, deadlocks, and made continuous integration green.
- Fixed weird moveit not downloaded repo.

Removed
-------

- Removed some reordering fixes.

## Section_17

### Added
- Added ignition file and updated repos files.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as a dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-check-build.yml.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added Dockerfile with ROS distro as argument.
- Added setupTracing.sh for automated installation.
- Added alternative ManualTracing.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

### Changed
- Changed `ros2 launch sm_three_some sm_three_some` to `ros2 launch sm_three_some sm_three_some.launch`.
- Changed wording from "smacc application" to "SMACC2 library".
- Updated name of package and package.xml.
- Updated GitHub branch reference.
- Updated description table.
- Updated table.
- Updated name to smacc2 and smacc2_msgs.
- Updated ci-build-source.yml.
- Updated extension of imports.
- Updated formatting of python file.
- Updated tracing event names.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated smacc2_rta command across readmes.
- Updated c_cpp_properties.json.
- Updated launch command to `ros2 launch sm_respira_1 sm_respira_1.launch`.

### Fixed
- Fixed broken source build (#227).
- Fixed Focal-Rolling builds by correcting the version of rosdep yaml (#234).
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed formatting issues.
- Fixed source CI and corrected README overview (#62).
- Fixed doxygen links (#70).

### Removed
- Removed manual installation of ros-rolling-ros2trace.
- Removed tracing directory.
- Removed galactic builds from master and kept only rolling.
- Removed submodules and used .repos file.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace.

### Miscellaneous
- Reverted "Ignore all packages except smacc2 and smacc2_msgs".
- Reverted markdowns to html.
- Reactivated smacc2 nav clients for rolling via submodules.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_reference_library.
- Cleaned up sm_atomic_performance_trace_1.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_respira_1.
- Cleaned up sm_respira_test_2.
- Cleaned up sm_respira_test_2.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_atomic_24hr.
- Cleaned up sm_atomic_24hr.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```rst
Section_18
==========

Added
~~~~~
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior: add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Nodes to wait can be optionally selected.
- Progress in aws navigation demo.

Changed
~~~~~~~
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in aws navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Corrected all linters and formatters.

Fixed
~~~~
- Fix pre-commit issues.
- Remove some compile warnings.

Removed
~~~~~~~
- Minor format changes.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

*pabloinigoblasco*

```rst
Section_19
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior `add` for `nav2`: waits for `nav2` nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available
- Progress in AWS navigation demo
- Gazebo fixes to show the robot and the lidar
- First working version of `sm` template and template generator
- Added SVGs to READMEs of `atomic`, `dance_bot`, and others
- Rolling Docker environment to be executed from any environment

Changed
-------
- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Minor hotfixes
- Cleaning and lidar show/hide option
- More fixes in various components
- Adjusted build packages of source CI
- Additional linting and formatting
- Corrected formatting
- Fixed launch command for `sm_dance_bot_strikes_back`
- Removed node creation and created only a logger

Fixed
-----
- Minor tuning to mitigate overshot issue cases in waypoints navigator
- Resolved compile warnings
- Fixed CI: format fix python version

Removed
-------
- Removed `neo_simulation2` package
- Removed parameters from `smacc`

Contributors
------------
- Pablo Iñigo Blasco <pablo@ibrobotics.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- DecDury <declandury@gmail.com>
- Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_20
==========

Added
-----

- Added feature/migration moveit client (#151).
- Added initial migration to smacc2.
- Added missing dependency.
- Added repos dependency.
- Added dependency to ur5 client.
- Added docker refactoring.
- Added progress on move_it PR.
- Added pre-commit cleanup.
- Added QOS durability to SmaccPublisherClient (#163).
- Added feature/testing moveit behaviors (#167).
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added update dependencies for husky in rolling and galactic.
- Added warehouse2 progress (#179).
- Added waypoint inputs (#178).
- Added sm_dance_bot_warehouse_3 (#181).
- Added redoing sm_dance_bot_warehouse_3 waypoints.
- Added more waypoints.
- Added SrConditional fixes and formatting (#168).
- Added finetuning waypoints (#187).
- Added feature/cb pure spinning (#188).
- Added feature/planner changes 16 12 (#191).
- Added feature/replanning 16 dec (#193).
- Added several fixes (#194).
- Added feature/undo motion 20 12 (#196).
- Added tuning warehouse3 (#197).
- Added feature/sync 21 12 (#199).
- Added feature/warehouse2 22 12 (#200).
- Added feature/warehouse2 23 12 (#201).
- Added tuning and fixes (#202).
- Added feature/minor tune (#203).
- Added fixing warehouse 3 problems, and other core improvements (#204).
- Added backport to foxy.
- Added missing file from warehouse2 (#205).
- Added updating subscriber publisher components.
- Added progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine.
- Added refining cp subscriber cp publisher.

Changed
-------

- Changed some errors introduced on formatting.
- Changed some more linting warnings.
- Changed format.
- Changed progress on moveit migration testing.
- Changed format issues.

Fixed
-----

- Fixed compiling issues.
- Fixed broken master build.
- Fixed pipeline error.
- Fixed broken build.
- Fixed some formatting and templating on SrConditional.
- Fixed move trigger logic into headers.
- Fixed lint.
- Fixed formatting.
- Fixed default values.
- Fixed pure spinning behavior missing files.
- Fixed undo tuning and errors.
- Fixed format issues.
- Fixed minor linking errors foxy.

Removed
-------

- Removed test from main moveit cmake.

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

```rst
Section_21
==========

Added
-----
- Added more components to smacc core, mainly for autoware demo.
- Added docker files for different revisions, warnings removal, and more navigation testing.
- Added barrel search build fix and warehouse3 improvements.
- Added progress in barrel husky development.
- Added branching example.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml for manual deployment.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh for automated installation of tracing packages.
- Added alternative ManualTracing method.
- Added new SM markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added performance tests improvements.
- Added optimizations in move_base_z_planners_common.
- Added renaming of event generator library.

Changed
-------
- Changed launch command to ros2 launch sm_three_some sm_three_some.launch.
- Changed wording from "smacc application" to "SMACC2 library".
- Updated name of package and package.xml.
- Updated GitHub branch reference.
- Updated description table.
- Updated table.
- Updated changelogs.
- Updated ci-build-source.yml.
- Updated doxygen-check-build.yml.
- Updated smacc2_rta command across readmes.
- Updated formatting of python file.
- Updated tracing event names.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Updated README.md to reflect changes in tracing events.
- Updated doxygen links.
- Updated c_cpp_properties.json.
- Reverted markdowns to HTML format.
- Reverted "Ignore all packages except smacc2 and smacc2_msgs" commit.

Fixed
-----
- Fixed minor broken build issues.
- Fixed docker build for Foxy and Galactic.
- Fixed startup problems in warehouse3.
- Fixed format and minor issues.
- Fixed trailing spaces.
- Fixed codespell and python linters warnings.
- Fixed cppcheck formatting.
- Fixed linking errors for Foxy.
- Fixed build of missing rolling repositories.
- Fixed Navigation2 for semi-binary build.
- Fixed build setup for missing rolling repositories.
- Fixed source CI setup and corrected README overview.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
- Removed disabled packages and updated workflows.
- Removed manual installation of ros-rolling-ros2trace.
- Removed tracing directory.
- Removed submodules and use .repos file for galactic builds.
- Removed galactic builds from master and kept only rolling.
- Removed tracing.md file.

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
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_22
==========

Added
-----

- More Readme Updates (#72) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- More Readme (#74) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Feature/core and navigation fixes (#78)
- Feature/aws demo progress (#80)
- sm_advanced_recovery_1 reworked (#83) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- more sm_advanced_recovery_1 (#84) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- More sm_advanced_recovery_1 work (#85) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_advanced_recovery_1 round 4 (#86) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Brettpac branch (#87)
- sm_atomic_performance_test_a_2
- sm_atomic_performance_test_a_1 (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_atomic_performance_test_c_1 (#88) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- modifying sm_atomic_performance_test_a_2 (#89) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- sm_multi_stage_1 (#90) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- more sm_multi_stage_1 (#91) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>)
- Wait topic message client behavior (#81) (Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, Denis Štogl <destogl@users.noreply.github.com>)
- Feature/wait nav2 nodes client behavior (#82) (Co-authored-by: Denis Štogl <denis@stogl.de>, Denis Štogl <destogl@users.noreply.github.com>)
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)
```

Section 23
-----------

### Added
- Introduce `cb_wait_topic_message`: an asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for `nav2`, allowing waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive. Optional selection of nodes to wait.
- Initial work on `sm_aws_warehouse` navigation.
- Add `cb_pause_slam` feature (#98).
- Implement `sm_dance_bot_lite` (#99).
- Visualize TurtleBot3 in `sm_dance_bot` (#101).
- Add choice for launching Gazebo lidar in `sm_dance_bot` (#102).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#103).
- Implement gazebo fixes for `sm_dance_bot_strikes_back` (#105).
- Implement gazebo fixes for `sm_multi_stage_1` (#

```rst
Section_24
==========

Added
-----
- Added remaining SVGs to READMEs (#145).
- Added SM Atomic SM generator (#143).
- Added QOS durability to SmaccPublisherClient.
- Added dependencies for husky simulation in AWS navigation.
- Added warehouse2 progress (#179).
- Added Waypoint Inputs (#178).
- Added SrConditional fixes and formatting (#168).
- Added SM warehouse 2 13 dec 2 (#182).
- Added CB pure spinning (#188).
- Added planner changes 16 12 (#191).
- Added replanning 16 dec (#193).
- Added undo motion 20 12 (#196).
- Added sync 21 12 (#199).
- Added warehouse2 22 12 (#200).
- Added warehouse2 23 12 (#201).
- Added minor tune (#203).

Changed
-------
- Updated package list (#142).
- Fixed launch command for sm_dance_bot_strikes_back.
- Fixed CI: format fix python version (#148).
- Moved reference library SMs to smacc2_performance_tools (#166).
- Redoing sm_dance_bot_warehouse_3 waypoints.
- Finetuning waypoints (#187).
- Tuning warehouse3 (#197).
- Tuning and fixes (#202).
- Fixed some formatting and templating on SrConditional.
- Moved trigger logic into headers.

Removed
-------
- Removed parameters smacc (#147).
- Removed node creation and create only a logger (#149).
- Removed test from main moveit cmake.
- Removed some comments in the past.
- Removed some linting warnings.
- Removed test workaround in minor dockerfile.
- Removed compiling issues.

Fixed
-----
- Noticed launch command was incorrect in README.md.
- Fixed errors introduced on formatting.
- Fixed missing dependency.
- Fixed broken master build.
- Fixed broken build.
- Fixed formatting.
- Fixed some errors.
- Fixed warehouse 3 problems.

Workflows
---------
- Precommit cleanup.
- Workflow.

Collaborators
-------------
- Co-authored-by: DecDury <declandury@gmail.com>.
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: Denis Štogl <denis@stogl.de>.
```

```rst
Section_25
==========

Added
-----
- Added missing file from warehouse2 (#205).
- Added docker files for different revisions, warnings removal, and more testing on navigation.
- Added barrel demo.
- Added multiple controllable LEDs plugin.
- Added progress in Husky demo.
- Added improvements in navigation behaviors.
- Added more merge.
- Added feature/docker improvements march 2022 (#235).
- Added replanning for all our examples.
- Added Foxy backport (#206).
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added manual deployment for now.
- Added workflow for checking doc build.
- Added smacc2_performance_tools.

Changed
-------
- Changed ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch.
- Changed wording "smacc application" to "SMACC2 library".
- Changed extension of imports.
- Changed extension.
- Changed formatters.
- Changed formatting of python file.
- Changed GitHub branch reference.
- Changed description table.
- Changed name of package and package.xml to pass linter.
- Changed wording "smacc application" to "SMACC2 library".

Fixed
-----
- Fixed warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green.
- Fixed weird moveit not downloaded repo.
- Fixed minor broken build.
- Fixed trailing spaces.
- Fixed codespell.
- Fixed python linters warnings.
- Fixed docker for Foxy and Galactic.
- Fixed startup problems in warehouse 3.
- Fixed format and minor.
- Fixed bug in SMACC2 component.
- Fixed missing rolling repositories build.
- Fixed Navigation2 for semi-binary build.
- Fixed format and minor.
- Fixed some reordering fixes.
- Fixed barrel search build and warehouse3.
- Fixed progress in autoware machine.
- Fixed progress in barrel Husky.
- Fixed progress in Husky demo.
- Fixed progress in navigation rolling.
- Fixed renamed tracing events after.
- Fixed progress in autoware demo.
- Fixed progress in SMACC core adding more components mostly developed for autoware demo.
- Fixed progress in barrel search updates.
- Fixed progress in barrel demo.
- Fixed progress in Husky demo.
- Fixed progress in autoware demo.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.
- Removed tracing directory.
- Removed disabled packages and update workflows.
- Removed galactic builds from master and keep only rolling. Remove submodules and use .repos file.
- Removed manual installation of ros-rolling-ros2trace. This is now automated in setupTracing.sh.

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
```

```rst
Section_26
==========

Added
~~~~~
- Performance tests improvements.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Renaming of event generator library.
- Optimized dependencies in move_base_z_planners_common.
- Added galactic CI setup and renamed rolling files. (#58)
- Fix source CI and correct README overview. (#62)
- Update c_cpp_properties.json.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Adding new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.
- Navigation parameters fixes on sm_dance_bot.

Changed
~~~~~~~
- Updated smacc2_rta command across readmes.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated README.md with launch command.
- Corrected all linters and formatters.

Fixed
~~~~
- Correct trailing spaces.
- Several core improvements during navigation testing.
- Minor formatting fixes.
- Attempted pre-commit fixes.

Removed
~~~~~~~
- Removed redundant entries related to aws navigation progress and format improvements.

Authors
~~~~~~~
- Pablo Iñigo Blasco (@pabloinigoblasco)
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_27
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive; optional node selection
- New gazebo fixes for `sm_dance_bot_strikes_back`

Changed
~~~~~~~
- Progress in AWS navigation demo
- Minor format improvements
- Navigation parameters fixes on `sm_dance_bot`
- Cleaning and lidar show/hide option in `sm_dance_bot` visualizing TurtleBot3
- More refinement in `sm_dance_bot` S-pattern

Fixed
~~~~
- Remove some compile warnings
- Correct formatting in removing `neo_simulation2` package
- Adjust build packages of source CI
- Move method after the method it calls to prevent recursion
- Noticed typo correction: "Finnaly" to "Finally"

Removed
~~~~~~~
- Removed `neo_simulation2` package

Other
~~~~~
- Several core improvements during navigation testing
- Formatting improvements
- Merge and progress
- Base for the `sm_aws_warehouse` navigation
- Precommit cleanup run
- Updates YAML
- Enable source build on PR for testing
- Additional linting and formatting
- Remove merge markers from a Python file
- Progress in navigation, slam toggle client behaviors, and `slam_toolbox` components
- Introducing slam pausing/resuming functionality in testing `sm_dance_bot`
- First working version of `sm` template and template generator
- Minor tweaks
- Various improvements in navigation and performance
- Going forward in testing `sm_dance_bot` introducing slam pausing/resuming functionality
- Polishing `sm_dance_bot` and S-pattern
- More refinement in `sm_dance_bot`
- First working version of `sm` template and template generator
``` 

*pabloinigoblasco*

Section_28
===========

Added
-----

- Feature/sm dance bot refine (#131)
  - More changes in sm_dance_bot
  - Minor refinements

- Feature/sm dance bot refine 2 (#132)
  - More changes in sm_dance_bot
  - Minor refinements

- Waypoints navigator bug (#133)
  - Minor tuning to mitigate overshot issues
  - Progress in the sm_dance_bot tests (#135)
    - Some progress on markers cleanup
  - Minor format issues (#134)

- sm_dance_bot_lite (#136)
  - Resolve compile warnings (#137)
  - Add SM core test (#138)
  - Minor navigation improvements (#141)
  - Using local action messages (#139)
  - Removing sm_dance_bot_msgs
  - Pending references

- Feature/nav2z renaming (#144)
  - Using local action messages
  - Removing sm_dance_bot_msgs
  - Pending references
  - Navigation 2 stack renaming
  - Formatting

- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Precommit cleanup
- Update package list (#142)
- Removing parameters smacc (#147)
- Workflows update
- Noticed launch command was incorrect in README.md
  - Fixed launch command for sm_dance_bot_strikes_back and removed some comments

- Fix CI: format fix python version (#148)
- Add SM Atomic SM generator (#143)
- Remove node creation and create only a logger (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
  - Slight waypoint 4 and iterations changes so robot can complete course (#155)

- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixing some errors introduced on formatting
  - Missing dependency
  - Fixing some more linting warnings
  - Removing test from main moveit cmake
  - Test ur5
  - Progressing in the moveit migration testing
  - Updating format
  - Adding .reps dependencies and also fixing some build errors
  - Repos dependency
  - Adding dependency to ur5 client
  - Docker refactoring
  - Progress on move_it PR
  - Minor dockerfile test workaround
  - Improving dockerfile for building local tests
  - Fixing compiling issues

- Update readme (#164)
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Pre-commit cleanup
- Add QOS durability to SmaccPublisherClient (#163)
  - Add QOS durability to SmaccPublisherClient
  - Add reliability QOS config

- Feature/testing moveit behaviors (#167)
  - More testing on moveit
  - Progress on moveit
  - More testing on moveit behaviors
  - Minor configuration
  - Fixing pipeline error
  - Fixing broken master build

- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)
  - Multistage modes
  - sm_multi_stage sequences
  - sm_multi_state_1 steps
  - sm_multi_stage_1 sequence d
  - sm_multi_stage_1 c sequence
  - Mode_5_sequence_b
  - Mode_4_sequence_b
  - sm_multi_stage_1 most
  - Finishing touches 1
  - Readme

- Feature/aws navigation sm dance bot (#174)
  - Repo dependency
  - Husky launch file in sm_dance_bot
  - Add dependencies for husky simulation
  - Fix formatting
  - Progress on aws navigation and some other refactorings on navigation clients and behaviors
  - More on aws demo
  - Fixing broken build

- Minor changes (#175)
- Warehouse2 (#177)
- Waypoint Inputs (#178)
- Warehouse2 progress (#179)
- Format (#180)
- sm_dance_bot_warehouse_3 (#181)

- Feature/sm warehouse 2 13 dec 2 (#182)
  - Format
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
  - Brettpac branch (#184)
    - Redoing sm_dance_bot_warehouse_3 waypoints
    - More waypoints

- SrConditional fixes and formatting (#168)
  - Some formatting and templating on SrConditional
  - Move trigger logic into headers
  - Lint

- Feature/wharehouse2 dec 14 (#185)
  - Warehouse2
  - Minor

- Feature/sm warehouse 2 13 dec 2 (#186)
  - Format
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
  - Minor

- Finetuning waypoints (#187)

- Feature/cb pure spinning (#188)
  - Format
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values

- Feature/cb pure spinning (#189)
  - Format
  - More changes and headless
  - Merge
  - Headless and other fixes
  - Default values
  - Pure spinning behavior missing files

- Minor changes (#190)

- Feature/planner changes 16 12 (#191)
  - Minor changes
  - More fixes
  - Minor

- Feature/replanning 16 dec (#193)
  - Minor changes
  - Replanning for all our examples
  - Several fixes (#194)
  - Minor changes (#195)

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Author: Pablo Iñigo Blasco (pabloinigoblasco)

Section_29
===========

Added
-----

- Feature/undo motion 20 12 (#196)
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Use correct upstream .repos files for source builds (#243)
- Correct mergify branch names (#246)
- Update galactic source build job name (#250)
- Galactic source build: update .repos file, bump action version and use correct version of upstream packages (backport #241) (#248)
- restoring workflow files (#252)
- restoring files (#253)
- Fix checkout branches for scheduled builds (#254)
- Feature/fixing husky build rolling (#257)
- Feature/fixing husky build rolling (#258)
- Update README.md (#262)
- Feature/fixing ur demos (#261)
- Feature/fixing type string walker (#263)
- Update README.md (#266)
- Update README.md (#267)
- Update README.md (#268)
- Significant update in Getting Started Instructions (#269)
- Fix urls to index.ros.org (#284)
- Fix foxy source build config to use repos file from foxy branch. (#285)

Changed
-------

- Correct name of source-build job and bump version of action (#242) (#247)

Fixed
-----

- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green (#204)
- fixing docker for foxy and galactic
- fixing startup problems in warehouse 3
- fixing format and minor
- fixing rolling build (#239)
- trying to fix dependencies
- fixing to focal by the moment
- fixing building issue
- fixing broken build
- fix: initialise conditionFlag as false (#274)
- precommit fix (#280)
- Fix foxy source build config to use repos file from foxy branch. (#285)

Removed
-------

- Revert "Ignore packages which should not be released."
- Ignore packages which should not be released.

Other
-----

- minor changes
- replanning for all our examples
- improving undo motion navigation warehouse2
- tuning warehouse3 (#197)
- tuning and fixes (#202)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- weird moveit not downloaded repo
- added missing file from warehouse2 (#205)
- backport to foxy
- minor format
- minor linking errors foxy
- missing
- missing sm
- updating subscriber publisher components
- progress in autowarrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrre machine
- refining cp subscriber cp publisher
- improvements in smacc core adding more components mostly developed for autoware demo
- autoware demo
- foxy ci
- minor broken build
- some reordering fixes
- minor
- docker files for different revisions, warnings removval and more testing on navigation
- docker build files for all versions
- barrel demo
- barrel search build fix and warehouse3
- minor
- progress in barrel husky
- minor
- barrel demo
- minor
- barrel search updates
- making models local
- red picuup
- multiple controllable leds plugin
- progress in husky demo
- progressing in husky demo
- improving navigation behaviors
- more merge
- docker improvements
- cache matrix rolling and source build package
- minor
- minor
- missing repo
- missing deps
- typo
- restoring workflow files (#252)
- restoring files (#253)
- restoring files
- making husky project build on rolling
- husky progress
- more on backport
- more on backport
- disappeared ur_msgs denis repo
- fixing sm_dance_bot examples
- working on fix of image messages for husky_barrel demo

Co-authored-by
-------------

- Denis Štogl <denis@stogl.de>
- Denis Štogl <destogl@users.noreply.github.com>
- Declan Dury <44791484+DecDury@users.noreply.github.com>
- DecDury <declandury@gmail.com>
- reelrbtx <brett2@reelrobotics.com>
- brettpac <brett@robosoft.ai>
- David Revay <MrBlenny@users.noreply.github.com>
- pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
- Pablo Iñigo Blasco <pablo@ibrobotics.com>

```rst
Section_30
==========

0.3.0 (2022-04-04)
------------------
- Reverted "Ignore packages which should not be released" (commit dec14a936a877b2ef722a6a32f1bf3df09312542).
- Contributors: Denis Štogl, Pablo Iñigo Blasco

0.0.0 (2022-11-09)
------------------
Added
~~~~~
- Feature/galactic rolling merge (#288)
- Update description table
- Update table
- Copy initial docs
- Dockerfile w/ ROS distro as argument
- Opened new folder for additional tracing contents
- Added setupTracing.sh to install necessary packages and configure tracing group
- Created alternative ManualTracing
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling, removing submodules and using .repos file
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Some progress on navigation rolling
- Added smacc2_performance_tools
- Performance tests improvements
- More on performance and other issues
- Format cleanup for sm_respira_1
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library
- Added galactic CI setup and renamed rolling files (#58)
- Fixed source CI and corrected README overview (#62)
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
- Updated doxygen links (#70)
- More Readme Updates (#72)
- More Readme (#74)
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Base for the sm_aws_aarehouse navigation
- Progressing in AWS navigation
- Several core improvements during navigation testing
- Format improvements
- Progress in AWS navigation demo
- More on navigation
- Feature/aws demo progress (#80)
- Format improvements
- More on navigation
- Reworked sm_advanced_recovery_1 (#83)
- More sm_advanced_recovery_1 work (#85)
- Round 4 of sm_advanced_recovery_1 (#86)
- Brettpac branch (#87)
- Added sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Added sm_multi_stage_1 (#90)
- Fixing precommit for sm_multi_stage_1 (#91)
- Wait topic message client behavior (#81)
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- Attempting precommit fixes

Changed
~~~~~~~
- Changed wording "smacc application" to "SMACC2 library"
- Reactivated smacc2 nav clients for rolling via submodules
- Renamed tracing events
- Bug fixes in smacc2 component
- Corrected trailing spaces
- Cleaned up sm_reference_library
- Updated smacc2_rta command across readmes
- Cleaned up sm_atomic_24hr

Removed
~~~~~~~
- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh
- Tracing.md file
- Deleted tracing directory
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh
- Deleted tracing directory
- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh
- Deleted tracing directory
``` 

*pabloinigoblasco*

```rst
Section_31
==========

Added
-----

- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to become active. Optional node selection.
- New feature: cb pause slam client behavior.
- New feature: sm_dance_bot_lite gazebo fixes to show the robot and lidar.
- New feature: gazebo fixes for sm_dance_bot_strikes_back.

Changed
-------

- Corrected all linters and formatters.
- Navigation parameters fixes on sm_dance_bot.
- Minor formatting improvements.
- Cleaning and lidar show/hide option for sm_dance_bot visualizing turtlebot3.

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

```rst
Section_32
==========

Added
-----

- Implemented sm_multi_stage_1 functionality (#109, #110, #111, #114, #172)
  - Initial work on multi-stage process
  - Progress and traction gained
  - Development of multiple stages
  - Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Changed
-------

- Removed neo_simulation2 package (#112)
  - Corrected formatting and enabled source build testing
- Improved navigation and performance (#116)
  - Various enhancements made
  - Co-authored by pabloinigoblasco <pablo@ibrobotics.com>

Fixed
-----

- Fixed recursion issue in method call sequence (#126)
- Resolved waypoint navigation overshot problem (#133)
- Addressed minor format issues (#134)
- Fixed CI format for Python version (#148)

Removed
-------

- Removed parameters from smacc (#147)
  - Updated workflows and launch command corrections
- Removed node creation for logger only (#149)
- Removed unnecessary comments in README.md (#147)

Other
-----

- Added SM Atomic SM generator (#143)
- Updated Docker environment for cross-environment execution (#154)
- Added QOS durability to SmaccPublisherClient (#163)
  - Improved reliability and durability settings
- Moved reference library SMs to smacc2_performance_tools (#166)
- Added durability and reliability configuration to SmaccPublisherClient
- Renamed sm_advanced_recovery_1 to sm_pubsub_1 (#171)
- Co-authored by DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_33
==========

Added
-----
- Feature/aws navigation sm dance bot (#174)
- Added repo dependency and husky launch file in sm_dance_bot.
- Added dependencies for husky simulation and updated dependencies for husky in rolling and galactic.
- Added progress on aws navigation and refactorings on navigation clients and behaviors.
- Added more on aws demo.
- Added fixing broken build.

Changed
-------
- Updated progress on warehouse2.
- Updated Waypoint Inputs.
- Updated sm_dance_bot_warehouse_3 with more changes, headless mode, and other fixes.
- Updated Brettpac branch.
- Redone sm_dance_bot_warehouse_3 waypoints and added more waypoints.
- Finetuned waypoints.
- Improved undo motion navigation in warehouse2.
- Tuned warehouse3.
- Fixed warehouse 3 problems and made core improvements to remove deadlocks.
- Added missing file from warehouse2.
- Merged code from backport foxy and updated autoware.
- Updated cb_navigate_global_position.hpp.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency.
- Disabled ament_cpplint and some packages, updated workflows.
- Bumped ccache version.
- Ignored further packages.
- Satisfied ament_lint_cmake.
- Added missing licenses.
- Disabled cpplint and cppcheck linters.
- Corrected formatters.
- Updated ci-build-source.yml.
- Changed extension of imports.
- Enabled cppcheck.
- Corrected formatting of python file.
- Included necessary package and edited Threesome launch.

Removed
-------
- Removed trailing spaces.
- Removed some disabled packages.
- Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
``` 

*pabloinigoblasco*

```rst
Section_34
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
-----
- Bug in smacc2 component.
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Optimized dependencies in move_base_z_planners_common.
- Correct trailing spaces.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic_24hr.
- Clean up of sm_atomic

```rst
Section_35
==========

Added
-----
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive. Optionally select the nodes to wait.

Changed
-------
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.

Fixed
-----
- Remove some compile warnings.
- Minor hotfix.
- Format fixes for `sm_dance_bot` visualizing `turtlebot3`.
- Cleaning and lidar show/hide option.
- Gazebo fixes to show the robot and the lidar.
- Format fixes for `sm_multi_stage_1`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.
- Precommit cleanup run.

Removed
-------
- Removed `neo_simulation2` package.

Other
-----
- Progress in AWS navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Progress in AWS navigation demo.
- Merge and progress.
- Base for the `sm_aws_warehouse` navigation.
- Precommit.
- Updates YAML.
- Got `sm_multi_stage_1` working (barely).
- Gaining traction `sm_multi_stage_1`.
- More progress on stages.
- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
```

Section_36
==========

Version 0.1.0 (2022-01-01)
---------------------------

### Added
- Feature/diverse improvements navigation performance (#117)
- Additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components
- Introducing slam pausing/resuming functionality in sm_dance_bot tests (#135)
- First working version of sm template and template generator (#127)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Minor navigation improvements (#141)
- Waypoint 4 and iterations changes so the robot can complete the course (#155)

### Fixed
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing broken master build in aws navigation (#174)

### Removed
- Remove merge markers from a python file (#119)
- Removing parameters smacc (#147)
- Removing sm_dance_bot_msgs in favor of using local action messages

Version 0.2.0 (2022-02-01)
---------------------------

### Added
- Feature/migration moveit client (#151)
- Initial migration to smacc2
- Progressing in the moveit migration testing
- Added .reps dependencies and fixed some build errors
- Added dependency to ur5 client
- Improving dockerfile for building local tests

### Changed
- Moved reference library SMs to smacc2_performance_tools (#166)
- Renamed husky launch file in sm_dance_bot for AWS navigation (#174)

### Fixed
- Fixing some errors introduced on formatting in moveit migration (#151)
- Fixing some more linting warnings in moveit migration
- Mitigate overshot issue cases in waypoints navigator (#133)
- Fixing compiling issues in moveit migration

### Removed
- Removing test from main moveit cmake
- Removing some comments in the past from README.md

Version 0.3.0 (2022-03-01)
---------------------------

### Added
- Feature/testing moveit behaviors (#167)
- More testing on moveit behaviors
- Minor configuration improvements
- Progress on moveit testing
- More testing on moveit behaviors

### Changed
- Minor changes in warehouse2 (#177)
- Waypoint Inputs (#178)

### Removed
- Pending references in sm_dance_bot_msgs

Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>

```rst
Section_37
==========

Added
-----
- Feature/sm_dance_bot_warehouse_3 (#181)
- Feature/wharehouse2 dec 14 (#185)
- Feature/sm warehouse 2 13 dec 2 (#186)
- Feature/cb pure spinning (#188)
- Feature/undo motion 20 12 (#196)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Feature/replanning 16 dec (#193)
- Feature/odom tracker improvements and retry motion (#223)

Changed
-------
- SrConditional fixes and formatting (#168)
- finetuning waypoints (#187)
- tuning warehouse3 (#197)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green (#204)
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

Fixed
-----
- Remove example things from Foxy CI setup. (#214)
- Add Autoware Auto Msgs into not-released dependencies. (#220)
- Fix rolling builds (#222)
- do not merge yet - Feature/odom tracker improvements and retry motion (#223)
- removing warnings (#213)

Removed
-------
- Add mergify rules file.
- Try fixing CI for rolling. (#209)
- backport to foxy (#206)

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
Section_38
==========

Added
-----

- Initial docs copied.
- Dockerfile with ROS distro as argument.
- New folder for additional tracing contents opened.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Created alternative ManualTracing.
- Added new sm markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Added smacc2_performance_tools.
- Performance tests improvements.
- More changes on performance tests.
- Added README tutorial for Dockerfile.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success.

Changed
-------

- Updated description table.
- Updated table.
- Changed wording "smacc application" to "SMACC2 library".
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed event generator library.
- Cleaned up sm_reference_library.
- Corrected trailing spaces.
- Optimized dependencies in move_base_z_planners_common.
- Renamed tracing events.
- Updated smacc2_rta command across readmes.
- Cleaned up sm_atomic_24hr.
- Reformatted sm_reference_library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- Updated c_cpp_properties.json.
- Renamed rolling files for Galactic CI setup.
- Fixed source CI and corrected README overview.

Fixed
-----

- Bug in smacc2 component.

Removed
-------

- Manual installation of ros-rolling-ros2trace.

Authors
-------

- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_39
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: wait for `nav2` nodes subscribing to the `/bond` topic and wait for them to be alive. Optionally select the nodes to wait.
- Base for the `sm_aws_warehouse` navigation.

Changed
~~~~~~~
- Minor format improvements.
- Several core improvements during navigation testing.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.
- `sm_dance_bot_lite` visualizing `turtlebot3`.
- Cleaning and lidar show/hide option for `sm_dance_bot_lite`.
- Gazebo fixes to show the robot and the lidar for `sm_dance_bot_lite`.
- Gazebo fixes for `sm_dance_bot_strikes_back`.

Fixed
~~~~
- Corrected all linters and formatters.
- Removed some compile warnings.

Removed
~~~~~~~
- Redundant format improvements.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Commits
~~~~~~~
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)
- Remove some compile warnings. (#96)
- Feature/cb pause slam (#98)
- Rename doxygen deployment workflow (#100)
- sm_dance_bot_lite (#99)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- sm_multi_stage_1 doubling (#103)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- precommit cleanup run (#106)
- aws demo (#108)
- Brettpac branch (#110)
```

```rst
Section 40
==========

Added
-----
- Brettpac branch (#111)
- a3 (#113)
- Remove neo_simulation2 package. (#112)
- Feature/diverse improvements navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122)
- Feature/dance bot s pattern (#128)
- First working version of sm template and template generator. (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- waypoints navigator bug (#133)
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
- initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- sm_pubsub_1 (#169)
- sm_pubsub_1 part 2 (#170)
- sm_advanced_recovery_1 renaming (#171)
- sm_multi_stage_1 reworking (#172)

Changed
-------
- Correct formatting for Remove neo_simulation2 package. (#112)
- Adjust build packages of source CI for Remove neo_simulation2 package. (#112)
- progress in navigation, slam toggle client behaviors and slam_toolbox components. Also smacc2::deep_history syntax for Feature/slam toggle and smacc deep history (#122)
- Move method after the method it calls. Otherwise recursion could happen. (#126)
- Noticed typo to Finnaly > Finally
- fixing some errors introduced on formatting for Feature/migration moveit client (#151)
- fixing some more linting warnings for Feature/migration moveit client (#151)
- fixing compiling issues for Feature/migration moveit client (#151)
- update readme (#164)
- more readme updates for update readme (#164)
- feat: add qos durability to SmaccPublisherClient for Add QOS durability to SmaccPublisherClient (#163)
- fix: add a missing colon for Add QOS durability to SmaccPublisherClient (#163)
- refactor: remove line for Add QOS durability to SmaccPublisherClient (#163)
- feat: add reliability qos config for Add QOS durability to SmaccPublisherClient (#163)

Fixed
-----
- Enable source build on PR for testing for Remove neo_simulation2 package. (#112)
- minor format issues for sm_dance_bot_lite (#136)
- minor tuning to mitigate overshot issue cases for waypoints navigator bug (#133)
- some more progress on markers cleanup for progress in the sm_dance_bot tests (#135)
- fixing pipeline error for Feature/testing moveit behaviors (#167)
- fixing broken master build for Feature/testing moveit behaviors (#167)

Removed
-------
- Remove neo_simulation2 package.
- removing sm_dance_bot_msgs
- removing parameters smacc
- workflows update
- workflow
- removing test from main moveit cmake
- test ur5
- repos dependency
- adding dependency to ur5 client
- removing parameters smacc
- removing node creation and create only a logger.
- removing some comments in the past from launch command for sm_dance_bot_strikes_back
- removing some build errors for improving dockerfile for building local tests
- removing some comments I had made in the past for fixing launch command for sm_dance_bot_strikes_back

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_41
==========

Added
-----
- Feature/aws navigation sm dance bot (#174): Added repo dependency and husky launch file in sm_dance_bot. Dependencies for husky simulation were included. Updated dependencies for husky in rolling and galactic. Progress on aws navigation and refactorings on navigation clients and behaviors. More work on aws demo.

Changed
-------
- Minor changes (#175): Minor adjustments made.
- Warehouse2 (#177): Progress made on warehouse2.
- Waypoint Inputs (#178): Introduced waypoint inputs.
- Sm_dance_bot_warehouse_3 (#181): Redoing waypoints for sm_dance_bot_warehouse_3. More waypoints added.
- SrConditional fixes and formatting (#168): Fixed formatting and templating on SrConditional. Moved trigger logic into headers and performed linting.
- Finetuning waypoints (#187): Fine-tuned waypoints.

Fixed
-----
- Fixing broken build: Resolved broken build issues.
- Several fixes (#194): Various fixes implemented.
- Fixing warehouse 3 problems, and other core improvements (#204): Fixed warehouse 3 problems, removed deadlocks, and improved continuous integration. Added missing file from warehouse2.
- Fix code generators (#221): Fixed build issues, updated SM template, and made example code clearer. Removed use of node in the SM performance template. Updated template to use Blackboard storage. Resolved global data resolution in the template. Updated sm_name.hpp.

Removed
-------
- Pure spinning behavior missing files: Removed missing files related to pure spinning behavior.

Co-authored-by: Denis Štogl <denis@stogl.de>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: reelrbtx <brett2@reelrobotics.com>
Co-authored-by: brettpac <brett@robosoft.ai>
Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
Co-authored-by: pabloinigoblasco <pabloinigoblasco@ibrobotics.com>
```

```rst
Section_42
==========

Version 0.1.0 (Fecha por determinar)
------------------------------------

Added
~~~~~

- Ensure the necessary package is installed before running the command:
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Renamed header files and corrected format.
- Added workflow for checking doc build.
- Updated doxygen-check-build.yml.
- Created doxygen-deploy.yml.
- Created workflow for testing prerelease builds.
- Used 'docs/' as source folder and output directory.
- Renamed to 'smacc2' and 'smacc2_msgs'.
- Corrected GitHub branch reference.
- Updated package name and package.xml to pass liter.
- Executed master update.
- Reset all versions to 0.0.0.
- Ignored all packages except 'smacc2' and 'smacc2_msgs'.
- Updated changelogs.
- Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Updated description table.
- Updated table.
- Copied initial docs.
- Created Dockerfile with ROS distro as argument.
- Opened new folder for additional tracing contents.
- Deleted tracing directory.
- Moved tracing.md to tracing directory.
- Added setupTracing.sh to install necessary packages and configure tracing group.
- Removed manual installation of 'ros-rolling-ros2trace'; now automated in setupTracing.sh.
- Created alternative 'ManualTracing'.
- Added new 'sm' markdowns.
- Added a Dockerfile for Rolling and Galactic.
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh.
- Updated tracing/ManualTracing.md.
- Changed wording from "smacc application" to "SMACC2 library".
- Updated smacc_sm_reference_library/sm_atomic/README.md.
- Reactivated smacc2 nav clients for Rolling via submodules.
- Renamed tracing events.
- Fixed bug in smacc2 component.
- Reverted markdowns to HTML.
- Added README tutorial for Dockerfile.
- Cleaned up tracing.md to reflect new tracing event names.
- Enabled build of missing Rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Removed Galactic builds from master, keeping only Rolling; removed submodules and used .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Made progress on navigation Rolling.
- Renamed folders, deleted tracing.md, edited README.md.
- Added 'smacc2_performance_tools'.
- Made improvements to performance tests.
- Made more improvements on performance and other issues.
- Cleaned up 'sm_respira_1' format.
- Cleaned up 'sm_respira_1' format pre-commit.
- Added 'sm_respira_test_2'.
- Made more changes on performance tests.
- Did not execute clang-format on 'smacc2_sm_reference_library' package.
- Reformatted 'sm_reference_library'.
- Corrected trailing spaces.
- Added galactic CI setup and renamed Rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Changed launch command to 'ros2 launch sm_respira_1 sm_respira_1.launch' (#69).
- Updated doxygen links (#70).
- Made more Readme updates (#72).
- Made more Readme updates (#74).
- Created new 'sm' from 'sm_respira_1' (#76).
- Made core and navigation fixes (#78).
- Progressed in AWS navigation.
- Made several core improvements during navigation testing.
- Made formatting improvements.
- Made progress in AWS navigation demo.
- Made more format improvements.
- Reworked 'sm_advanced_recovery_1' (#83).
- Fixed pre-commit in 'sm_advanced_recovery_1'.
- Made more improvements on 'sm_advanced_recovery_1' (#85).
- Reworked 'sm_advanced_recovery_1' round 4 (#86).
- Created 'Brettpac' branch (#87).
- Added 'sm_atomic_performance_test_a_2' and 'sm_atomic_performance_test_a_1'.
- Added 'sm_atomic_performance_test_c_1' (#88).
- Modified 'sm_atomic_performance_test_a_2' (#89).
- Added 'sm_multi_stage_1'.
- Fixed precommit in 'sm_multi_stage_1'.
- Made more improvements on 'sm_multi_stage_1' (#91).
- Updated README.md with launch command.
- Waited for topic message client behavior (#81).

Changed
~~~~~~~

- Updated launch command from 'ros2 launch sm_three_some sm_three_some' to 'ros2 launch sm_three_some sm_three_some.launch'.
```

```rst
Section_43
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: wait for nav2 nodes to subscribe to the /bond topic and wait for them to be alive, with optional node selection
- New client behavior: cb pause slam

Changed
-------
- Minor format improvements
- Navigation parameters fixes on sm_dance_bot
- Gazebo fixes to show the robot and the lidar
- Cleaning and lidar show/hide option for sm_dance_bot visualizing turtlebot3

Fixed
-----
- Corrected all linters and formatters
- Removed some compile warnings

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
```

*pabloinigoblasco*

Section_44
===========

Added
-----
- Feature/sm dance bot strikes back gazebo fixes (#105)
  - Visualizing turtlebot3 for sm_dance_bot
  - Added cleaning and lidar show/hide option
  - Improved formatting and fixed gazebo issues
- Precommit cleanup run (#106)
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- AWS demo (#108)
- Got sm_multi_stage_1 working (#109)
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Brettpac branch (#110)
  - Improved sm_multi_stage_1 functionality
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai)
- A3 (#113)
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More sm_multi_stage_1 enhancements (#114)
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- MM (#115)
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Diverse improvements in navigation and performance (#116)
  - Minor enhancements
  - Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
- Feature/diverse improvements in navigation performance (#117)
  - Additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing slam pausing/resuming functionality for sm_dance_bot
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern
  - Fixed typo "Finnaly" to "Finally"
- First working version of sm template and template generator (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
  - Minor changes and build fixes
- Waypoints navigator bug (#133)
  - Tuning to mitigate overshot issue cases
- Progress in sm_dance_bot tests (#135)
  - Progress on markers cleanup
- Minor format issues (#134)
- Sm_dance_bot_lite (#136)
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Resolve compile warnings (#137)
- Add SM core test (#138)
- Minor navigation improvements (#141)
- Using local action messages (#139)
  - Removed sm_dance_bot_msgs
- Feature/nav2z renaming (#144)
  - Removed sm_dance_bot_msgs
  - Renamed navigation 2 stack
  - Added SVGs to READMEs of atomic, dance_bot, and others (#140)
  - Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Removing parameters smacc (#147)
  - Workflows update
  - Fixed launch command in README.md for sm_dance_bot_strikes_back
- Fix CI: format fix python version (#148)
- Add SM Atomic SM generator (#143)
- Remove node creation and create only a logger (#149)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
  - Co-authored-by: DecDury <declandury@gmail.com>, Denis Štogl <destogl@users.noreply.github.com>
- Slight waypoint 4 and iterations changes for robot course completion (#155)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Fixed formatting errors and missing dependencies
  - Progress in moveit migration testing
  - Updated format and dependencies
  - Docker refactoring and build improvements
  - Progress on move_it PR
- Initial state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
  - Pre-commit cleanup
- Add QOS durability to SmaccPublisherClient (#163)
  - Added QOS durability and reliability configurations

Changed
-------
- Move method after the method it calls to prevent recursion (#126)

Removed
-------
- Remove neo_simulation2 package (#112)
  - Corrected formatting and adjusted build packages for source CI

Fixed
-----
- Minor format issues (#124)
- Minor format issues (#134)
- Minor format issues (#141)
- Minor format issues (#148)
- Minor format issues (#163)
- Minor format issues (#164)
- Minor format issues (#166)
- Minor format issues (#167)

```rst
Section_45
==========

Added
~~~~~
- Added more testing on moveit behaviors.
- Added husky launch file in sm_dance_bot.
- Added dependencies for husky simulation.
- Added progress on aws navigation and some refactorings on navigation clients and behaviors.
- Added more on aws demo.
- Added finishing touches 1.
- Added warehouse2 progress.
- Added Waypoint Inputs.
- Added more changes and headless in sm_dance_bot_warehouse_3.
- Added redoing sm_dance_bot_warehouse_3 waypoints.
- Added more Waypoints.
- Added several fixes.
- Added replanning for all examples.
- Added improving undo motion navigation warehouse2.
- Added tuning warehouse3.
- Added fixing warehouse 3 problems and other core improvements.
- Added retry behavior warehouse 1.
- Added updating subscriber publisher components.
- Added progress in autoware machine.
- Added refining cp subscriber cp publisher.
- Added improvements in smacc core.
- Added more components mostly developed for autoware demo.
- Added docker files for different revisions.
- Added warnings removal and more testing on navigation.
- Added update file for fake hardware simulation and add file for gazebo simulation.
- Added fixing docker for foxy and galactic.
- Added docker build files for all versions.
- Added missing files.
- Added weird moveit not downloaded repo.
- Added backport to foxy.
- Added minor linking errors foxy.
- Added fixing broken source build.
- Added correcting Focal-Rolling builds by fixing the version of rosdep yaml.

Changed
~~~~~~~
- Changed minor configuration.
- Changed progress on aws navigation and some other refactorings on navigation clients and behaviors.
- Changed tuning and fixes.
- Changed fixing warehouse 3 problems and other core improvements to remove dead lock.
- Changed default values.
- Changed format issues.
- Changed tuning and fixes.
- Changed minor tune.

Fixed
~~~~
- Fixed pipeline error.
- Fixed broken master build.
- Fixed broken build.
- Fixed lint.
- Fixed some formatting and templating on SrConditional.
- Fixed move trigger logic into headers.
- Fixed missing sm.
- Fixed minor broken build.
- Fixed some reordering fixes.
- Fixed minor format fix.
- Fixed other minor changes.
``` 

*pabloinigoblasco*

```rst
Section_46
==========

Added
-----
- Added ignition file and updated repos files.
- Added galactic CI build due to Navigation2 issues in rolling.
- Added partial changes for ament_cpplint.
- Added tf2_ros as dependency to find include.
- Added workflow for checking doc build.
- Added doxygen-deploy.yml for manual deployment.
- Added workflow for testing prerelease builds.
- Added Dockerfile with ROS distro as argument.
- Added setupTracing.sh for automated installation.
- Added alternative ManualTracing.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added sm_atomic_performance_trace_1.
- Added galactic CI setup and renamed rolling files.
- Added base for the sm_aws_warehouse navigation.
- Added sm_advanced_recovery_1 reworked.

Changed
-------
- Updated file for fake hardware simulation and added file for gazebo simulation.
- Updated warehouse3 feature/improvements (#228).
- Backported to foxy.
- Renamed header files and corrected format.
- Renamed to smacc2 and smacc2_msgs.
- Changed extension of imports.
- Updated name of package and package.xml.
- Reset all versions to 0.0.0.
- Updated description table.
- Updated table.
- Updated smacc2_rta command across readmes.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Cleaned up sm_atomic_24hr.
- Corrected trailing spaces.
- Updated c_cpp_properties.json.
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Updated doxygen links.
- More readme updates.
- More on navigation.
- Fixed pre-commit issues.

Fixed
-----
- Fixed trailing spaces.
- Corrected codespell.
- Corrected python linters warnings.
- Corrected formatting of python file.
- Enabled cppcheck.
- Fixed source CI and corrected README overview.

Removed
-------
- Removed manual installation of ros-rolling-ros2trace.

Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Author: Pablo Iñigo Blasco (pabloinigoblasco)
```

```rst
Section_47
==========

Added
~~~~~
- Introduce more sm_advanced_recovery_1 work (#84)
- Implement sm_atomic_performance_test_a_2 and sm_atomic_performance_test_a_1
- Develop sm_atomic_performance_test_c_1 (#88)
- Enhance sm_multi_stage_1 functionality (#90, #91)
- Update README.md and launch command
- Add new feature cb_wait_topic_message for asynchronous client behavior
- Implement new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic
- Add navigation parameters fixes on sm_dance_bot
- Merge and progress in sm_aws_aarehouse navigation
- Introduce cb pause slam feature (#98)

Changed
~~~~~~~
- Correct all linters and formaters for Feature/wait nav2 nodes client behavior (#82)

Removed
~~~~~~~
- Remove some compile warnings (#96)

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

pabloinigoblasco
```

## Section_48

### Added
- New client behavior for nav2: Now waits for nav2 nodes to subscribe to the /bond topic and confirms they are active. Optional selection of nodes to wait for.
- Progress in AWS navigation demo.

### Changed
- Navigation parameters fixes on sm_dance_bot.
- CB pause slam client behavior.
- Updates yaml.
- Renamed doxygen deployment workflow.
- Sm_multi_stage_1 doubling.
- Gazebo fixes to show the robot and lidar.
- Sm_dance_bot visualizing turtlebot3.
- Cleaning and lidar show/hide option.
- Format fixes.
- Sm_dance_bot_strikes_back gazebo fixes.
- Precommit cleanup run.
- AWS demo format.
- Got sm_multi_stage_1 working (barely).
- Brettpac branch progress.
- Diverse improvements in navigation and performance.
- Feature/slam toggle and smacc deep history: Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Introducing slam pausing/resuming functionality for testing sm_dance_bot.

### Fixed
- Move method after the method it calls to prevent recursion.
- Minor tuning to mitigate overshot issue cases.
- Minor format issues.

### Removed
- Removed neo_simulation2 package.
- Removed parameters smacc.
- Removed node creation and created only a logger.

### Miscellaneous
- Various minor format improvements and linting.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Precommit cleanup.
- Update package list.
- Fixed launch command for sm_dance_bot_strikes_back in README.md.
- Rolling Docker environment to be executed from any environment.
- Missing dependency fixed in migration moveit client.
- Slight waypoint 4 and iterations changes so the robot can complete the course.

```rst
Section_49
==========

Added
~~~~~
- Added dependency to ur5 client.
- Added .reps dependencies and fixed build errors.
- Added QOS durability to SmaccPublisherClient.
- Added reliability QOS configuration.
- Added dependencies for husky simulation in sm_dance_bot.
- Added Waypoint Inputs.
- Added default values for various features.

Changed
~~~~~~~
- Updated format.
- Improved dockerfile for building local tests.
- Refactored docker.
- Progressed in move_it PR.
- Reworked sm_multi_stage_1.
- Finetuned waypoints.
- Tuned warehouse3.
- Tuned and fixed various components.
- Improved undo motion navigation in warehouse2.
- Made core improvements to remove deadlocks and ensure continuous integration is green.
- Backported to foxy.

Fixed
~~~~
- Fixed compiling issues.
- Fixed pipeline error.
- Fixed broken master build.
- Fixed formatting issues.
- Fixed broken build.
- Fixed linting.
- Fixed some formatting and templating on SrConditional.
- Fixed various errors and issues.

Removed
~~~~~~~
- Removed test from main moveit cmake.

Other
~~~~~
- Progressed in moveit migration testing.
- Pre-commit cleanup.
- Minor configuration changes.
- Minor changes and refactorings on navigation clients and behaviors.
- More testing on moveit behaviors.
- More on aws demo.
- More changes and headless.
- Redoing sm_dance_bot_warehouse_3 waypoints.
- More Waypoints.
- Several fixes.
- Replanning for all examples.
- Improving undo motion navigation in warehouse2.
- Finishing warehouse2.
- Tuning and fixes.
- Weird moveit not downloaded repo.
- Added missing file from warehouse2.
- Updated subscriber publisher components.
- Progress in autoware machine.
- Refining cp subscriber cp publisher.
- Improvements in smacc core adding more components mostly developed for autoware demo.
- Autoware demo.
- Foxy CI.
- Some reordering fixes.
``` 

*pabloinigoblasco*

```rst
Section_50
==========

Added
-----
- Docker build files for all versions.
- Barrel demo.
- Barrel search build fix and Warehouse3.
- Progress in Barrel Husky.
- Feature/barrel - do not merge yet (#233).
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
  Then run this command.
- Workflow for checking doc build.
- Workflow for testing prerelease builds.
- Use manual deployment for now.
- Created alternative ManualTracing.
- Added new SM markdowns.
- Added a Dockerfile for Rolling and Galactic.
- README tutorial for Dockerfile.
- SMACC2 performance tools.
- Performance tests improvements.
- More on performance and other issues.
- SM Respira 1 format cleanup.
- SM Respira test 2.
- SM Atomic 24hr.
- SM Atomic performance trace 1.
- Clean up of SM Atomic 24hr.
- Optimized dependencies in move_base_z_planners_common.
- Renaming of event generator library.
- Add galactic CI setup and rename rolling files (#58).
- Fix source CI and correct README overview (#62).
- Update doxygen links (#70).
- More Readme Updates (#72).
- More Readme (#74).

Changed
-------
- Fixing Docker for Foxy and Galactic.
- Fixing startup problems in Warehouse 3.
- Fixing broken build.
- Replanning for all our examples.
- Backport to Foxy.
- Correct codespell.
- Correct Python linters warnings.
- Add Galactic CI build because Navigation2 is broken in Rolling.
- Add partial changes for ament_cpplint.
- Add tf2_ros as dependency to find include.
- Disable ament_cpplint.
- Disable some packages and update workflows.
- Bump ccache version.
- Ignore further packages.
- Satisfy ament_lint_cmake.
- Add missing licenses.
- Disable cpplint and cppcheck linters.
- Correct formatters.
- Branching example.
- Disable disabled packages.
- Update ci-build-source.yml.
- Change extension of imports.
- Enable cppcheck.
- Correct formatting of Python file.
- Included necessary package and edited Threesome launch.
- Reset all versions to 0.0.0.
- Ignore all packages except SMACC2 and SMACC2_msgs.
- Update changelogs.
- Update description table.
- Update table.
- Copy initial docs.
- Enable build of missing Rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove Galactic builds from master and keep only Rolling. Remove submodules and use .repos file.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Some progress on Navigation Rolling.
- More changes on performance tests.
- Do not execute clang-format on SMACC2_SM_reference_library package.
- SM_reference_library reformatting.
- Correct trailing spaces.
- Update SMACC2_RTA command across readmes.
- Clean up of SM Atomic 24hr.
- More SM Atomic 24hr cleanup.
- Minor formatting.

Removed
-------
- Warnings removal and more testing on navigation.
- Minor linking errors Foxy.
- Disable disabled packages.
- Deleted tracing.md.
- Cleanup.
- Cleanup.
- Cleanup.
- Edited tracing.md to reflect new tracing event names.
- Do not execute clang-format on SMACC2_SM_reference_library package.
- Reverted markdowns to HTML.
- Bug in SMACC2 component.
- Renamed tracing events after.
- Minor format.
- Minor.
- Minor.
- Minor.

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
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_51
==========

Version 1.0.0 (2022-01-01)
---------------------------

Added
~~~~~

- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Base for the sm_aws_aarehouse navigation
- Progressing in AWS navigation
- Several core improvements during navigation testing
- Formatting improvements
- Progress in AWS navigation demo
- Feature/aws demo progress (#80)
- More on navigation
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait
- Navigation parameters fixes on sm_dance_bot
- Merge and progress

Changed
~~~~~~~

- Sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 (#84)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Sm_multi_stage_1
- More sm_multi_stage_1 (#91)
- Correct all linters and formatters

Fixed
~~~~~

- Fix pre-commit
- Trying to fix Pre-Commit
- Fixing precommit

Removed
~~~~~~~

- Update README.md updated launch command

Collaborators
~~~~~~~~~~~~~

- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_52
==========

Added
-----
- New client behavior for nav2: Wait for nav2 nodes to subscribe to the /bond topic and ensure they are alive. Optional node selection available.
- New feature: cb_wait_topic_message - Asynchronous client behavior that waits for a topic message and optionally checks its contents for success.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.
- Progress in AWS navigation.
- Several core improvements during navigation testing.
- Formatting improvements.
- Gazebo fixes to show the robot and the lidar.
- Progress in navigation, slam toggle client behaviors, and slam_toolbox components. Also smacc2::deep_history syntax.
- Progress in testing sm_dance_bot introducing slam pausing/resuming functionality.
- Polishing sm_dance_bot and s-pattern.
- More refinement in sm_dance_bot.
- First working version of sm template and template generator.
- Minor tuning to mitigate overshot issue cases.
- Minor navigation improvements.
- Using local action messages.
- Navigation 2 stack renaming.
- Added SVGs to READMEs of atomic, dance_bot, and others.
- Added remaining SVGs to READMEs.

Fixed
-----
- Removed some compile warnings.
- Remove neo_simulation2 package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.
- Noticed typo: "Finnaly" corrected to "Finally".
- Minor format issues.

Removed
-------
- Removed neo_simulation2 package.
- Removed parameters smacc.

Authors
-------
- Pablo Iñigo Blasco (pabloinigoblasco)
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

## Section_53

### Added
- Added SM Atomic SM generator. (#143)
- Added QOS durability to SmaccPublisherClient (#163)

### Changed
- Updated workflows
- Refactored feature "sm dance bot strikes back" (#152)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Reworked "sm_multi_stage_1" (#172)
- Progress on AWS navigation and refactorings on navigation clients and behaviors
- Finetuned waypoints (#187)
- Improved undo motion navigation in "warehouse2" (#198)
- Tuning and fixes (#202)
- Minor tune (#203)

### Fixed
- Fixed launch command in README.md
- Fixed CI: format fix python version (#148)
- Fixed node creation and created only a logger (#149)
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed broken build
- Fixed formatting
- Fixed some linting warnings
- Fixed some errors introduced on formatting
- Fixed missing dependency
- Fixed some build errors
- Fixed warehouse 3 problems and other core improvements to remove deadlock, also making continuous integration green

### Removed
- Removed parameters "smacc"
- Removed some comments in the past
- Removed test from main moveit cmake
- Removed test "ur5"
- Removed test workaround in minor dockerfile
- Removed test "warehouse2"
- Removed some redundancy in "sm_dance_bot_warehouse_3" waypoints
- Removed pure spinning behavior missing files
- Removed weird moveit not downloaded repo
- Removed missing file from "warehouse2"

### Miscellaneous
- Pre-commit cleanup
- Added missing colon
- Added reliability QOS config
- Added dependencies for husky simulation
- Updated dependencies for husky in rolling and galactic
- Backported to foxy

Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

```rst
Section_54
==========

Added
-----
- Feature/docker improvements march 2022 (#235)
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
- Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic
- Update tracing/ManualTracing.md
- changed wording "smacc application" to "SMACC2 library"
- Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax

Changed
-------
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- branching example
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- Reset all versions to 0.0.0
- Update changelogs
- Update description table.
- Update table
- Copy initial docs
- Enable build of missing rolling repositories.
- Enable Navigation2 for semi-binary build.
- Remove galactic builds from master and kepp only rolling. Remove submodules and use .repos file
- updated mentions of SMACC/ROS to SMACC2/ROS2
- some progress on navigation rolling
- renamed folders, deleted tracing.md, edited README.md
- performance tests improvements
- more on performance and other issues
- sm_respira_1 format cleanup
- sm_respira_test_2
- more changes on performance tests

Fixed
-----
- minor linking errors foxy
- minor broken build
- fixing docker for foxy and galactic
- barrel search build fix and warehouse3
- fixing startup problems in warehouse 3
- fixing format and minor
- bug in smacc2 component

Removed
-------
- minor
- some reordering fixes
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
-

```rst
Section_55
==========

Added
-----

- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait.

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

Other
-----

- Reformatting of sm_reference_library.
- Optimized dependencies in move_base_z_planners_common.
- Renamed event generator library.
- Cleaned up sm_atomic_24hr.
- Updated smacc2_rta command across readmes.
- Minor formatting improvements.
- Noticed and removed a note left during production.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- Navigation parameters fixes on sm_dance_bot.

Collaborators
-------------

- Co-authored by Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored by Denis Štogl <destogl@users.noreply.github.com>.
- Co-authored by Denis Štogl <denis@stogl.de>.
```

```rst
Section_56
==========

Added
-----
- Introduce new feature `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally verifies its contents for success.
- Implement new client behavior for nav2: `add` behavior waits for nav2 nodes subscribing to the `/bond` topic and ensures they are alive. Optional node selection available.
- Progress made in AWS navigation demo.

Changed
-------
- Minor formatting improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- `cb_pause_slam` client behavior added.

Fixed
-----
- Remove some compile warnings.
- Fix gazebo visualization for `sm_dance_bot_lite`.
- Fix gazebo visualization for `sm_dance_bot_strikes_back`.
- Fix formatting in various areas.
- Fix recursion possibility by moving method after the one it calls.

Removed
-------
- Remove `neo_simulation2` package.

Other
-----
- Various core improvements during navigation testing.
- Precommit cleanup run.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Additional linting and formatting.
- Remove merge markers from a Python file.
- Waypoints navigator bug fix.

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
```

```rst
Section_57
==========

Added
-----
- Progress in the sm_dance_bot tests (#135)
- Added SM core test (#138)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list (#142)
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Add QOS durability to SmaccPublisherClient (#163)
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
- Some progress on markers cleanup
- Minor format issues (#134)
- Minor navigation improvements (#141)
- Using local action msgs
- Feature/nav2z renaming (#144)
- Resolve compile warnings (#137)
- Fix CI: format fix python version (#148)
- Remove node creation and create only a logger (#149)
- Fixing some errors introduced on formatting
- Progress on move_it PR
- Progress on moveit migration testing
- Improving dockerfile for building local tests
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Minor changes (#175)
- Format (#180)
- Format (#182)
- Finetuning waypoints (#187)
- Several fixes (#194)
- Tuning warehouse3 (#197)

Fixed
-----
- Noticed launch command was incorrect in README.md
- Fixing launch command for sm_dance_bot_strikes_back and removing some comments
- Fixing compiling issues

Removed
-------
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc
- Removing test from main moveit cmake
- Removing parameters smacc

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_58
==========

Added
-----
- Feature/sync 21 12 (#199): Includes minor changes and replanning for all examples.
- Feature/warehouse2 22 12 (#200): Addresses format issues and replanning for all examples.
- Feature/warehouse2 23 12 (#201): Involves minor changes, replanning, tuning, and fixes.
- Feature/minor tune (#203): Introduces tuning, fixes, and minor adjustments.
- Added missing file from warehouse2 (#205): Backports to foxy, fixes format issues, and resolves minor linking errors.
- Use correct upstream .repos files for source builds (#243).
- Correct mergify branch names (#246).
- Update galactic source build job name (#250).
- Galactic source build: Updates .repos file, action version, and upstream packages (#248).
- Fixing rolling build (#239): Addresses dependencies, missing repos, and building issues.
- Restoring workflow files (#252) and (#253).
- Fix checkout branches for scheduled builds (#254).
- Feature/fixing husky build rolling (#257) and (#258): Restores files and enables husky project build on rolling.
- Feature/fixing ur demos (#261): Fixes issues related to UR demos.
- Feature/fixing type string walker (#263): Resolves issues with type string walker demo.
- Significant update in Getting Started Instructions (#269): Provides significant updates to the instructions.
- Fix urls to index.ros.org (#284): Corrects URLs for index.ros.org.
- Fix foxy source build config to use repos file from foxy branch (#285).
- Working on fix of image messages for husky_barrel demo: Collaborative effort to address image message issues.

Changed
-------
- Improvements in smacc core adding more components mostly developed for autoware demo.
- Progress in autoware machine navigation behaviors.
- Docker improvements for different revisions, warnings removal, and more testing on navigation.
- Fixes in warehouse 3 problems and other core improvements to remove deadlocks.
- Progress in barrel husky and barrel demo.
- Progressing in husky demo with multiple controllable LEDs plugin.
- More merge and docker improvements.

Removed
-------
- Revert "Ignore packages which should not be released."

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
- mergify[bot] <37929162+mergify[bot]@users.noreply.github.com>
- brettpac <brettpac@users.noreply.github.com>
```

```rst
Section_59
==========

Added
-----
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Added smacc2_performance_tools.
- Optimized dependencies in move_base_z_planners_common.
- Added galactic CI setup and renamed rolling files (#58).
- Fixed source CI and corrected README overview (#62).
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- More Readme Updates (#72, #74).
- Created new sm from sm_respira_1 (#76).
- Base for the sm_aws_aarehouse navigation.
- Several core improvements during navigation testing.
- Progress in AWS navigation demo.
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally select the nodes to wait.

Changed
-------
- Renamed folders, deleted tracing.md, edited README.md.
- Updated smacc2_rta command across readmes.
- Corrected trailing spaces.
- Renamed event generator library.
- Minor formatting improvements.
- Updated launch command in README.md.
- Corrected all linters and formatters.

Fixed
-----
- Do not execute clang-format on smacc2_sm_reference_library package.
- Fixed pre-commit issues.
- Attempted pre-commit fixes.

Removed
-------
- Ignored packages which should not be released.

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

Section_60
==========

Added
-----
- New client behavior `cb_wait_topic_message` for asynchronous waiting of a topic message with optional content verification (#94, #98, #100, #102, #104, #105)
- New client behavior for nav2: wait for nodes subscribing to the `/bond` topic to become active, with customizable node selection (#96, #99, #110, #111, #113)
- Base for the `sm_aws_warehouse` navigation
- Progress in AWS navigation demo
- Gazebo fixes for visualizing TurtleBot3 and lidar display
- Progress in navigation testing and core improvements
- `smacc2::deep_history` syntax integration for `slam_toolbox` components
- Slam toggle client behaviors for pausing/resuming functionality in `sm_dance_bot`

Changed
-------
- Minor formatting improvements in various commits

Fixed
-----
- Navigation parameters fixes on `sm_dance_bot`
- Compile warnings removed (#96)
- Recursion prevention by moving methods to avoid potential recursion (#126)
- Various minor fixes and improvements

Removed
-------
- `neo_simulation2` package removed (#112)

Collaborators
-------------
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- pabloinigoblasco <pablo@ibrobotics.com>

Section_61
-----------

### Added
- First working version of sm template and template generator. (#127) [@brett]
- Add SM Atomic SM generator. (#143) [@DecDury, @destogl]
- Add QOS durability to SmaccPublisherClient (#163)

### Changed
- Renamed Feature/dance bot s pattern to Feature/sm dance bot refine (#131)
- Renamed Feature/sm dance bot refine 2 to Feature/sm dance bot refine (#132)
- Renamed Feature/sm dance bot strikes back refactoring to Feature/sm dance bot strikes back refactoring (#152)
- Renamed Feature/migration moveit client to Feature/migration moveit client (#151)
- Moved reference library SMs to smacc2_performance_tools (#166)

### Fixed
- Fix CI: format fix python version (#148)
- Fixing pipeline error
- Fixing broken master build

### Removed
- Removing sm_dance_bot_msgs
- Removing parameters smacc
- Remove node creation and create only a logger. (#149)

### Miscellaneous
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update package list. (#142)
- Update readme (#164)
- Initial state machine transition timestamp (#165)
- Added dependency to ur5 client
- Rolling Docker environment to be executed from any environment (#154)
- Progress on move_it PR
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- Warehouse2 progress (#179)
- Waypoint Inputs (#178)
- Finetuning waypoints (#187)
- Redoing sm_dance_bot_warehouse_3 waypoints
- More Waypoints

### Contributors
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_62
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
- Foxy backport (#206)
- Add galactic CI build because Navigation2 is broken in rolling
- Add partial changes for ament_cpplint
- Add tf2_ros as dependency to find include
- Create workflow for testing prerelease builds
- Use manual deployment for now
- Rename to smacc2 and smacc2_msgs
- Update name of package and package.xml to pass liter
- Update ci-build-source.yml
- Update doxygen-check-build.yml
- Create doxygen-deploy.yml
- Update changelogs
- Enable cppcheck
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Remove galactic builds from master and keep only rolling. Remove submodules and use .repos file
- updated mentions of SMACC/ROS to SMACC2/ROS2
- Add smacc2_performance_tools
- Update smacc2_rta command across readmes
- Renaming of event generator library

Changed
-------
- ros2 launch sm_three_some sm_three_some to ros2 launch sm_three_some sm_three_some.launch
- First ensure you have the necessary package installed
- Rename header files and correct format
- Add workflow for checking doc build
- Use docs/ as source folder for documentation
- Use docs/ as output directory
- Correct GitHub branch reference
- Execute on master update
- Reset all versions to 0.0.0
- Update description table
- Update table
- Copy initial docs
- Change extension of imports
- Update smacc_sm_reference_library/sm_atomic/README.md edit from html to markdown syntax
- changed wording "smacc application" to "SMACC2 library"
- Optimized deps in move_base_z_planners_common
- minor formatting

Fixed
-----
- Several fixes (#194)
- fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Correct formatters
- Correct formatting of python file
- Fixing warehouse 3 problems, and other core improvements to remove dead lock, also making continuous integration green
- weird moveit not downloaded repo
- added missing file from warehouse2
- minor format
- minor linking errors foxy
- Correct trailing spaces
- Correct codespell
- Correct python linters warnings
- Bump ccache version
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licences
- Disable cpplint and cppcheck linters
- Disable disabled packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows
- Disable ament_cpplint
- Disable some packages and update workflows
- Disable cppcheck and cppcheck linters
- Disable some packages and update workflows

```rst
Section_63
==========

Added
-----
- New feature: cb_wait_topic_message, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success. It includes a new client behavior for nav2, waiting for nodes subscribing to the /bond topic and ensuring they are alive. You can optionally select the nodes to wait for. (#81, #82, #92, #93, #94)
- Created new state machine (sm) from sm_respira_1. (#76)
- Base for the sm_aws_aarehouse navigation. (#78, #80, #81, #82, #92, #93, #94)
- Feature: sm_atomic_performance_test_a_1, sm_atomic_performance_test_a_2, sm_atomic_performance_test_c_1. (#87, #88, #89)
- Feature: sm_multi_stage_1. (#90, #91)

Changed
-------
- Updated c_cpp_properties.json, changed launch command to 'ros2 launch sm_respira_1 sm_respira_1.launch'. (#69)
- More Readme updates. (#72, #74)
- Update README.md, updated launch command. (#81)
- Navigation parameters fixes on sm_dance_bot. (#93)

Fixed
-----
- Fixed source CI and corrected README overview. (#62)
- Update doxygen links. (#70)
- Several core improvements during navigation testing. (#78, #80, #81, #82, #92, #93, #94)
- Corrected all linters and formatters. (#82)

Removed
-------
- Removed note not removed during production. (#69)

Authors
-------
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
```

```rst
Section_64
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive.

Changed
~~~~~~~
- Navigation parameters fixes on `sm_dance_bot`.
- Progress in AWS navigation demo.
- Gazebo fixes for showing the robot and the lidar.
- Progress in navigation, `slam` toggle client behaviors, and `slam_toolbox` components.
- Introducing `smacc2::deep_history` syntax.
- Polishing `sm_dance_bot` and `s-pattern`.
- Minor tuning to mitigate overshot issue cases.
- Minor navigation improvements.
- Using local action messages.

Fixed
~~~~
- Remove some compile warnings. (#96)
- Remove `neo_simulation2` package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Move method after the method it calls to prevent recursion.
- Resolve compile warnings.

Removed
~~~~~~~
- Remove `neo_simulation2` package.
- Remove `sm_dance_bot_msgs`.

Other
~~~~~
- Precommit cleanup run.
- Updates `yaml`.
- Cleaning and lidar show/hide option.
- Cleaning files and making formatting work.
- More fixes.
- Pending references.
- First working version of `sm` template and template generator.
- Minor tweaks.
- Build fix.
- Waypoints navigator bug.
- Some more progress on markers cleanup.
- Minor format issues.
```

```rst
Section_65
==========

Added
-----
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Added SM Atomic SM generator. (#143)
- Add QOS durability to SmaccPublisherClient (#163)
- Added dependencies for husky simulation in AWS navigation (#174)
- Added warehouse2 progress (#179)
- Added Waypoint Inputs (#178)
- Added SrConditional fixes and formatting (#168)
- Added finetuning waypoints (#187)
- Added pure spinning behavior missing files (#189)
- Added planner changes 16 12 (#191)
- Added replanning for all examples (#193)
- Added undo motion improvements and navigation for warehouse2 (#196)
- Added tuning for warehouse3 (#197)
- Added sync 21 12 (#199)
- Added finishing touches to warehouse2 (#200)
- Added tuning and fixes (#202)
- Added minor tune (#203)

Changed
-------
- Renamed navigation 2 stack to smacc2_performance_tools (#166)
- Refactored SM dance bot strikes back (#152)
- Refactored moveit behaviors (#167)
- Refactored sm_pubsub_1 (#169, #170)
- Refactored sm_advanced_recovery_1 (#171)
- Reworked sm_multi_stage_1 (#172)

Fixed
-----
- Fixed launch command in README.md for sm_dance_bot_strikes_back (#148)
- Fixed CI format for Python version (#148)
- Fixed node creation and logger creation in SMacc (#149)
- Fixed compiling issues in Docker environment
- Fixed broken master build in AWS navigation (#174)
- Fixed formatting in warehouse2 (#177)
- Fixed broken build in AWS demo
- Fixed pipeline error in moveit testing
- Fixed linting issues in SrConditional (#168)
- Fixed format issues in sync 21 12 (#199) and warehouse2 22 12 (#200)

Removed
-------
- Removed sm_dance_bot_msgs
- Removed parameters in SMacc
- Removed test from main moveit CMake
- Removed some comments in launch command for sm_dance_bot_strikes_back

Other
-----
- Updated package list (#142)
- Updated format in several READMEs (#164)
- Updated Docker environment for local tests
- Updated dependencies for husky in rolling and galactic
- Updated format in several READMEs
- Updated READMEs with more information
- Updated format in warehouse3
- Updated default values in several features
- Updated warehouse2 with more changes and headless mode
- Updated warehouse2 with more changes and headless mode
- Updated warehouse2 with more changes and headless mode
- Updated default values in pure spinning behavior
```

```rst
Section_66
==========

Added
-----
- Added missing file from warehouse2 (#205)
- Added workflow for checking doc build
- Added galactic CI build because Navigation2 is broken in rolling
- Added tf2_ros as dependency to find include
- Added setupTracing.sh to automate installation of ros-rolling-ros2trace
- Added new sm markdowns
- Added a dockerfile for Rolling and Galactic

Changed
-------
- Tuning and fixes
- Minor tune
- Fixed warehouse 3 problems and other core improvements (#204)
- Fixed warehouse 3 problems and other core improvements to remove dead lock, also making continuous integration green
- Updated subscriber publisher components
- Progress in autoware machine
- Refining cp subscriber cp publisher
- Improvements in smacc core adding more components mostly developed for autoware demo
- Replanning for all examples
- Merging code from backport foxy and updates about autoware (#208)
- Renamed "sm_three_some" launch command
- Renamed header files and corrected format
- Renamed to smacc2 and smacc2_msgs
- Renamed tracing events
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Updated smacc2_rta command across readmes
- Cleaned up sm_atomic_24hr
- Optimized dependencies in move_base_z_planners_common
- Renamed event generator library

Fixed
-----
- Minor broken build
- Fix trailing spaces
- Correct codespell
- Correct python linters warnings
- Disable ament_cpplint
- Disable cpplint and cppcheck linters
- Correct formatters
- Enable cppcheck
- Correct formatting of python file
- Enable build of missing rolling repositories
- Enable Navigation2 for semi-binary build
- Removed galactic builds from master and kept only rolling
- Updated description table
- Updated table
- Copy initial docs
- Updated name of package and package.xml to pass liter
- Reset all versions to 0.0.0
- Updated changelogs
- Reverted "Ignore all packages except smacc2 and smacc2_msgs"
- Reverted markdowns to html
- Edited README tutorial for Dockerfile
- Edited tracing.md to reflect new tracing event names
- Updated mentions of SMACC/ROS to SMACC2/ROS2
- Some progress on navigation rolling
- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax
- Updated ci-build-source.yml
- Updated doxygen-check-build.yml
- Created doxygen-deploy.yml
- Created workflow for testing prerelease builds
- Updated ci-build-source.yml
- Updated tracing/ManualTracing.md
- Updated smacc_sm_reference_library/sm_atomic/README.md from html to markdown syntax
- Updated smacc2_ci/docker/ros_rollingAndGalactic_ubuntu_20.04/buildGalactic.sh
- Updated tracing/ManualTracing.md
- Updated c_cpp_properties.json
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)

Removed
-------
- Weird moveit not downloaded repo
- Missing
- Missing sm
- Disable some packages and update workflows
- Bump ccache version
- Ignore further packages
- Satisfy ament_lint_cmake
- Add missing licenses
- Disable disabled packages
- Change extension of imports
- Ignore all packages except smacc2 and smacc2_msgs
- Do not execute clang-format on smacc2_sm_reference_library package
```

```rst
Section_67
==========

Added
-----

- Update doxygen links (#70) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More Readme Updates (#72) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- More Readme (#74) by Pablo Iñigo Blasco and Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Created new sm from sm_respira_1 (#76)
- Feature/core and navigation fixes (#78)
- Base for the sm_aws_aarehouse navigation
- Progressing in AWS navigation
- Several core improvements during navigation testing
- Formatting improvements
- Progress in AWS navigation demo
- Feature/aws demo progress (#80)
- More on navigation
- Sm_advanced_recovery_1 reworked (#83)
- More sm_advanced_recovery_1 work (#85)
- Sm_advanced_recovery_1 round 4 (#86)
- Brettpac branch (#87)
- Sm_atomic_performance_test_a_2
- Sm_atomic_performance_test_a_1
- Sm_atomic_performance_test_c_1 (#88)
- Modifying sm_atomic_performance_test_a_2 (#89)
- Sm_multi_stage_1
- Fixing precommit
- More sm_multi_stage_1 (#91)
- Wait topic message client behavior (#81)
- New feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success
- Attempting precommit fixes
- Feature/wait nav2 nodes client behavior (#82)
- Adding new client behavior add for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive. You optionally can select the nodes to wait
- Correct all linters and formaters

Changed
-------

- Update README.md with updated launch command
- Navigation parameters fixes on sm_dance_bot

Removed
-------

- None
```

```rst
Section_68
==========

Added
-----
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- Gazebo fixes for showing the robot and the lidar.
- `sm_multi_stage_1` doubling.
- `sm_dance_bot_strikes_back` gazebo fixes.
- `sm_dance_bot_lite` improvements.
- `smacc2::deep_history` syntax for slam toggle client behaviors and `slam_toolbox` components.
- First working version of `sm` template and template generator.
- Minor tuning to mitigate overshot issue cases in the waypoints navigator.
- SM core test added.
- Navigation 2 stack renaming.

Changed
-------
- Progress in AWS navigation demo.
- Navigation parameters fixes on `sm_dance_bot`.
- Formatting improvements.
- Cleaning and lidar show/hide option in `sm_dance_bot`.
- Polishing `sm_dance_bot` and `s-pattern`.
- Minor format issues.

Fixed
-----
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting.
- Enable source build on PR for testing.
- Adjust build packages of source CI.
- Remove merge markers from a Python file.

Removed
-------
- Remove `sm_dance_bot_msgs`.

Contributors
------------
- Pablo Iñigo Blasco
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_69
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
- Fixed launch command for sm_dance_bot_strikes_back in README.md
- Fixed CI: format fix python version (#148)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Refactored SM dance bot strikes back (#152)
- Reworked sm_multi_stage_1 (#172)
- Redoing sm_dance_bot_warehouse_3 waypoints (#184)
- Finetuned waypoints (#187)
- Tuned warehouse3 (#197)
- Tuned and fixed warehouse3 problems (#204)

Fixed
-----

- Noticed launch command was incorrect in README.md
- Fixed compiling issues
- Fixed broken master build
- Fixed pipeline error
- Fixed formatting
- Fixed broken build

Removed
-------

- Removed parameters smacc (#147)
- Removed node creation and create only a logger (#149)
- Removed test from main moveit cmake
- Removed test ur5
- Removed some comments in the past

Other
-----

- Precommit cleanup
- Workflows update
- Docker refactoring
- Progress on moveit migration testing
- Progress on moveit PR
- Progress on aws navigation and some other refactorings on navigation clients and behaviors
- More on aws demo
- More testing on moveit
- More testing on moveit behaviors
- More readme updates
- More changes and headless
- More on moveit
- More on warehouse2
- Several fixes
- Improving undo motion navigation warehouse2
- Finishing warehouse2
- Minor configuration
- Minor changes
- Minor tune
- Minor format issues
``` 

*pabloinigoblasco*

```rst
Section_70
==========

Added
-----
- Added missing file from warehouse2 (#205).
- Added mergify rules file.
- Added Autoware Auto Msgs into not-released dependencies. (#220).
- Added workflow for checking doc build.
- Added doxygen-deploy.yml.
- Added workflow for testing prerelease builds.
- Added setupTracing.sh to automate installation of ros-rolling-ros2trace.
- Added new sm markdowns.
- Added a dockerfile for Rolling and Galactic.

Changed
-------
- Renamed "smacc application" to "SMACC2 library" for clarity.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Progress on navigation rolling.
- Renamed folders, deleted tracing.md, and edited README.md.
- Performance tests improvements.
- More on performance and other issues.

Fixed
-----
- Fixed warehouse 3 problems and core improvements to remove deadlocks, also making continuous integration green.
- Fixed weird moveit not downloaded repo.
- Fixed minor format issues.
- Fixed minor linking errors in Foxy.
- Fixed trailing spaces.
- Corrected codespell.
- Corrected python linters warnings.
- Fixed rolling builds (#222).
- Fixed minor broken build.
- Fixed docker for Foxy and Galactic.
- Removed warnings (#213).
- Fixed formatting of python file.

Removed
-------
- Removed example things from Foxy CI setup. (#214).
- Removed manual installation of ros-rolling-ros2trace.

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
Section_71
==========

Added
~~~~~
- Added galactic CI setup and renamed rolling files. (#58)
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. You can optionally select the nodes to wait for.
- Added navigation parameters fixes on sm_dance_bot.

Changed
~~~~~~~
- Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69).
- Updated doxygen links (#70).
- Updated README.md with launch command.
- Corrected all linters and formatters.

Fixed
~~~~
- Fixed source CI and corrected README overview. (#62).
- Fixed trailing spaces.
- Fixed pre-commit issues.

Removed
~~~~~~~
- Removed execution of clang-format on smacc2_sm_reference_library package.

Authors
~~~~~~~
- Pablo Iñigo Blasco
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>
```

```rst
Section_72
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add` for waiting for `nav2` nodes subscribing to the `/bond` topic and ensuring they are alive. Optional node selection available.
- Base for the `sm_aws_warehouse` navigation.
- `sm_dance_bot_lite` gazebo fixes.
- `sm_dance_bot_strikes_back` gazebo fixes.
- AWS demo progress.
- `sm_multi_stage_1` doubling.
- `sm_multi_stage_1` gazebo fixes.
- `sm_multi_stage_1` working progress.
- `smacc2::deep_history` syntax integration.
- `sm_dance_bot` S-pattern polishing.
- First working version of `sm` template and template generator.

Changed
~~~~~~~
- Minor format improvements.
- Navigation parameters fixes on `sm_dance_bot`.
- `sm_dance_bot` visualizing `turtlebot3` improvements.
- `sm_dance_bot` cleaning and lidar show/hide option enhancements.
- Gazebo fixes for showing the robot and the lidar.

Fixed
~~~~
- Remove some compile warnings.
- Remove `neo_simulation2` package.
- Correct formatting issues.
- Adjust build packages of source CI.
- Remove merge markers from a Python file.
- Move method after the method it calls to prevent recursion.

Removed
~~~~~~~
- `neo_simulation2` package removal.

Other
~~~~~
- Precommit cleanup run.
- Additional linting and formatting.
- Noticed typo correction.
- `Finnaly` corrected to `Finally`.
- Minor tweaks.

Commits
~~~~~~~
- Feature/sm dance bot fixes (#95).
- Feature/cb pause slam (#98).
- Feature/dance bot launch gz lidar choice (#102).
- Feature/sm dance bot lite gazebo fixes (#104).
- Feature/sm dance bot strikes back gazebo fixes (#105).
- Rename doxygen deployment workflow (#100).
- Brettpac branch (#110).
- Feature/diverse improvements navigation performance (#117).
- Remove merge markers from a Python file. (#119).
- Feature/slam toggle and smacc deep history (#122).
- Feature/dance bot s pattern (#128).
- First working version of sm template and template generator. (#127).
- Feature/sm dance bot refine (#131).
- Move method after the method it calls. Otherwise recursion could happen. (#126).
- Feature/dance bot s pattern (#129).
- Minor (#124).
- Minor (#130).
- A3 (#113).
- MM (#115).
- Diverse improvements navigation and performance (#116).
- More changes in sm_dance_bot (#125).
- More changes in sm_dance_bot (#125).
- More changes in sm_dance_bot (#125).
- More refinement in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.
- More changes in sm_dance_bot.

Collaborators
~~~~~~~~~~~~~
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>.
- Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>.
```

Section_73
==========

Added
-----
- Feature/sm dance bot refine 2 (#132)
- Waypoints navigator bug (#133)
- Progress in the sm_dance_bot tests (#135)
- Feature/nav2z renaming (#144)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
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
- Remove node creation and create only a logger. (#149)
- Initial migration to smacc2
- Fixing some errors introduced on formatting
- Fixing some more linting warnings
- Progress on moveit migration testing
- Improving dockerfile for building local tests
- Update readme (#164)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Finetuning waypoints (#187)

Fixed
-----
- Noticed launch command was incorrect in README.md
- Fixing compiling issues
- Fixing broken master build
- Fixing broken build
- Several fixes (#194)

Removed
-------
- Removing sm_dance_bot_msgs
- Removing parameters smacc

Collaborators
-------------
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: DecDury <declandury@gmail.com>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Denis Štogl <denis@stogl.de>

```rst
Section_74
==========

Added
-----
- Feature/undo motion 20 12 (#198)
- Feature/sync 21 12 (#199)
- Feature/warehouse2 22 12 (#200)
- Feature/warehouse2 23 12 (#201)
- Feature/minor tune (#203)
- Added missing file from warehouse2 (#205)
- dockerfiles (#225)
- Feature/retry behavior warehouse 1 (#226)
- First ensure you have the necessary package installed.
  ```
  sudo apt-get install ros-rolling-ros2trace
  ```
- Added workflow for checking doc build.
- Create doxygen-deploy.yml
- Create workflow for testing prerelease builds
- Dockerfile w/ ROS distro as argument
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
- Opened new folder for additional tracing contents
- added setupTracing.sh
  Installs necessary packages and configures tracing group.
- Created alternative ManualTracing
- added new sm markdowns
- added a dockerfile for Rolling and Galactic

Changed
-------
- Progress in autoware machine
- Improvements in smacc core adding more components mostly developed for autoware demo
- Changed wording "smacc application" to "SMACC2 library"
- ros2 launch sm_three_some sm_three_some
  to
  ros2 launch sm_three_some sm_three_some.launch

Fixed
-----
- Fix code generators (#221)
- Fix other build issues.
- Update SM template and make example code clearly visible.
- Remove use of node in the sm performance template.
- Updated templated to use Blackboard storage.
- Update template to resolve the global data correctly.
- Update sm_name.hpp
- minor broken build
- some reordering fixes
- minor formatting fixes
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
- Disable disabled packages
- Update ci-build-source.yml
- Change extension
- Change extension of imports.
- Enable cppcheck
- Correct formatting of python file.
- Included necessary package and edited Threesome launch
- Update name of package and package.xml to pass liter.
- Execute on master update
- Reset all versions to 0.0.0
- Ignore all packages except smacc2 and smacc2_msgs
- Update changelogs
- Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
- Update description table.
- Update table
- Copy initial docs
- Update smacc_sm_reference_library/sm_atomic/README.md
  edit from html to markdown syntax

Removed
-------
- weird moveit not downloaded repo
- missing
- missing sm
- foxy ci
- minor format
- minor linking errors foxy
- Delete tracing directory
- Moved tracing.md to tracing directory
- Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
``` 

*pabloinigoblasco*

```rst
Section_75
==========

Added
-----
- Reactivated smacc2 nav clients for rolling via submodules.
- Added README tutorial for Dockerfile.
- Added smacc2_performance_tools.
- Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents for success.
- Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive. Optionally, you can select the nodes to wait.

Changed
-------
- Renamed tracing events.
- Updated mentions of SMACC/ROS to SMACC2/ROS2.
- Renamed folders, deleted tracing.md, and edited README.md.
- Renamed sm_reference_library event generator library.
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch.
- Corrected all linters and formatters.

Fixed
-----
- Bug in smacc2 component.
- Reverted markdowns to html.
- Fixed source CI and corrected README overview.
- Fixed trailing spaces.
- Do not execute clang-format on smacc2_sm_reference_library package.
- Optimized dependencies in move_base_z_planners_common.

Removed
-------
- Removed galactic builds from master and kept only rolling. Removed submodules and used .repos file.

Other Changes
-------------
- Enabled build of missing rolling repositories.
- Enabled Navigation2 for semi-binary build.
- Edited tracing.md to reflect new tracing event names.
- Some progress on navigation rolling.
- More changes on performance tests.
- Minor formatting improvements.
- Several core improvements during navigation testing.
- Progress in aws navigation demo.
- Progress in aws navigation.
- Progressing in aws navigation.
- More on performance and other issues.
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
- Progress in aws navigation demo

```rst
Section_76
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
- New client behavior for nav2: `add` for waiting nav2 nodes subscribing to the `/bond` topic and ensuring they are alive, with optional node selection
- New client behavior: `cb_pause_slam` for pausing SLAM
- New client behavior: `sm_dance_bot_lite` for visualizing TurtleBot3 in Gazebo
- New client behavior: `sm_dance_bot_strikes_back` for visualizing TurtleBot3 in Gazebo
- New client behavior: `sm_multi_stage_1` for multi-stage functionality
- New client behavior: `smacc2::deep_history` syntax for deep history management

Changed
~~~~~~~
- Formatting improvements throughout the changelog
- Navigation parameters fixes on `sm_dance_bot`
- Minor format adjustments
- Gazebo fixes for robot and lidar visualization
- Progress in AWS navigation demo
- Progress in navigation testing
- Progress in AWS navigation development
- Progress in `sm_aws_warehouse` navigation
- Progress in `sm_multi_stage_1` development
- Diverse improvements in navigation and performance

Fixed
~~~~
- Removed some compile warnings
- Removed `neo_simulation2` package
- Corrected formatting issues
- Enabled source build on PR for testing
- Adjusted build packages for source CI
- Removed merge markers from a Python file
```


*pabloinigoblasco*

```rst
Section_77
==========

Added
-----

- Introducing slam pausing/resuming functionality to sm_dance_bot (#124, #125, #126)
- First working version of sm template and template generator (#127)
- Added SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Add SM core test (#138)
- Add QOS durability to SmaccPublisherClient (#163)
- Waypoint Inputs (#178)

Changed
-------

- Move method after the method it calls to prevent recursion (#126)
- Renamed state machine transition timestamp (#165)
- Moved reference library SMs to smacc2_performance_tools (#166)
- Minor navigation improvements (#141)
- Progress on aws navigation and some other refactorings on navigation clients and behaviors (#174)
- More refinement in sm_dance_bot (#129, #131, #132, #136)
- Progress in the sm_dance_bot tests (#135)
- Progress on moveit migration testing (#151, #167)
- Progress on moveit behaviors testing (#167)
- Progress on aws demo (#174)
- Progress on warehouse2 (#179, #180)
- Redoing sm_dance_bot_warehouse_3 waypoints (#184)
- More Waypoints in sm_dance_bot_warehouse_3 (#184)
- Minor tuning to mitigate overshot issue cases (#134)
- Minor format issues (#134)
- Minor tweaks (#130)
- Minor changes (#175)
- Minor format changes (#180)
- Minor configuration changes (#167)
- Minor changes in sm_dance_bot (#125, #129, #131, #132, #136)
- Minor changes in sm_dance_bot_lite (#136)
- Minor changes in sm_dance_bot_warehouse_3 (#181)
- Minor changes in sm_pubsub_1 (#169, #170)
- Minor changes in sm_advanced_recovery_1 (#171)
- Minor changes in sm_multi_stage_1 (#172)
- Minor changes in warehouse2 (#177)
- Fix CI: format fix python version (#148)
- Fixing pipeline error (#167)
- Fixing broken master build (#167)
- Fixing compiling issues (#167)
- Fixing some errors introduced on formatting (#151)
- Fixing some more linting warnings (#151)
- Fixing launch command in README.md (#142)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on SrConditional (#168)
- Fixing some formatting and templating on Sr

```rst
Section_78
==========

Added
-----
- Finetuned waypoints (#187)
- Added feature "cb pure spinning" (#188)
- Added feature "cb pure spinning" (#189)
- Added feature "planner changes 16 12" (#191)
- Added feature "replanning 16 dec" (#193)
- Added feature "undo motion 20 12" (#196)
- Added feature "sync 21 12" (#199)
- Added feature "warehouse2 22 12" (#200)
- Added feature "warehouse2 23 12" (#201)
- Added feature "minor tune" (#203)
- Added feature "improvements warehouse3" (#228)
- Added Foxy backport (#206)
- Added multiple controllable LEDs plugin
- Added ignition file and updated repos files
- Added necessary package and edited Threesome launch

Changed
-------
- Updated ros2 launch sm_three_some to ros2 launch sm_three_some.launch
- Renamed header files and corrected format

Fixed
-----
- Fixed broken source build (#227)
- Fixed format and minor issues
- Fixed trailing spaces
- Corrected codespell
- Corrected Python linters warnings
- Corrected Focal-Rolling builds by fixing the version of rosdep yaml
- Fixed startup problems in warehouse 3
- Fixed broken build
- Fixed format issues
- Fixed warehouse3 search build
- Fixed warehouse3 startup problems

Removed
-------
- Removed some reordering fixes
- Removed some disabled packages

Other Changes
-------------
- Improved undo motion navigation in warehouse2
- Tuned warehouse3
- Made continuous integration green
- Updated subscriber publisher components
- Progressed in autoware machine
- Refined cp subscriber cp publisher
- Added more components to smacc core
- Progressed in autoware demo
- Progressed in barrel husky
- Progressed in barrel demo
- Progressed in husky demo
- Improved navigation behaviors
- Updated fake hardware simulation files
- Added gazebo simulation files
- Added docker build files for all versions
- Added retry behavior for warehouse 1
- Added missing files
- Added missing licenses
- Added galactic CI build
- Added partial changes for ament_cpplint
- Added tf2_ros as dependency
- Disabled ament_cpplint, cpplint, and cppcheck linters
- Ignored further packages
- Satisfied ament_lint_cmake
- Updated workflows
- Bumped ccache version
- Changed extension of imports
- Enabled cppcheck
- Corrected formatting of Python files
- Included necessary package for Threesome launch
- Created workflows for checking doc build, doxygen deployment, testing prerelease builds
- Used manual deployment for now
- Used docs/ as source folder and output directory for documentation
- Renamed packages to smacc2 and smacc2_msgs
- Corrected GitHub branch reference
- Updated package name and package.xml
- Executed update on master branch

Collaborators
-------------
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
Section_79
==========

Added
-----

- Initial setup:
  - Reset all versions to 0.0.0
  - Ignore all packages except smacc2 and smacc2_msgs
  - Update changelogs
  - Copy initial docs
  - Dockerfile w/ ROS distro as argument
    - Use command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  - Opened new folder for additional tracing contents
  - Added setupTracing.sh:
    - Installs necessary packages and configures tracing group
  - Created alternative ManualTracing
  - Added new sm markdowns
  - Added a dockerfile for Rolling and Galactic
  - Enable build of missing rolling repositories
  - Enable Navigation2 for semi-binary build
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Added smacc2_performance_tools
  - Performance tests improvements
  - More on performance and other issues
  - Added README tutorial for Dockerfile
  - Added new feature, cb_wait_topic_message: asynchronous client behavior that waits a topic message and optionally checks its contents to success

Changed
-------

- Updated description table
- Updated table
- Changed wording "smacc application" to "SMACC2 library"
- Reactivated smacc2 nav clients for rolling via submodules
- Renamed tracing events after
- Bug fix in smacc2 component
- Optimized deps in move_base_z_planners_common
- Renaming of event generator library
- Updated smacc2_rta command across readmes
- Cleaned up of sm_atomic_24hr
- Minor formatting improvements
- Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
- Updated doxygen links
- Updated c_cpp_properties.json
- Updated README.md

Fixed
-----

- Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh
- Do not execute clang-format on smacc2_sm_reference_library package
- Corrected trailing spaces

Removed
-------

- Deleted tracing directory
- Removed galactic builds from master and kept only rolling, removed submodules and use .repos file
- Deleted tracing.md

Contributors
------------

- Pablo Iñigo Blasco
- Denis Štogl <destogl@users.noreply.github.com>
- Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
```

```rst
Section_80
==========

Added
~~~~~
- New feature: `cb_wait_topic_message`, an asynchronous client behavior that waits for a topic message and optionally checks its contents for success.
- New client behavior for `nav2`: `add`, which waits for `nav2` nodes to subscribe to the `/bond` topic and ensures they are alive. Optional node selection available.
- `cb_pause_slam` client behavior.

Changed
~~~~~~~
- Corrected all linters and formatters.
- Minor format fixes.
- Navigation parameters fixes on `sm_dance_bot`.
- Hotfix for `sm_dance_bot` visualizing `turtlebot3`.
- Cleaning and lidar show/hide option for `sm_dance_bot`.
- Gazebo fixes to show the robot and lidar for various dance bot versions.

Fixed
~~~~
- Removed some compile warnings.

Removed
~~~~~~~
- Unused code related to `sm_multi_stage_1`.

Contributors
~~~~~~~~~~~~
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>

Commits
~~~~~~~
- Feature/aws demo progress (#92)
- Feature/sm dance bot fixes (#93)
- Feature/sm aws warehouse (#94)
- Feature/sm dance bot fixes (#95)
- Remove some compile warnings. (#96)
- Feature/cb pause slam (#98)
- Rename doxygen deployment workflow (#100)
- sm_dance_bot_lite (#99)
- Feature/dance bot launch gz lidar choice (#102)
- Feature/sm dance bot lite gazebo fixes (#104)
- sm_multi_stage_1 doubling (#103)
- Feature/sm dance bot strikes back gazebo fixes (#105)
- precommit cleanup run (#106)
- aws demo (#108)
- Brettpac branch (#110)
```

# Section_81

## Added
- Brettpac branch (#111)
- a3 (#113)
- Remove neo_simulation2 package. (#112)
- Feature/diverse improvements navigation performance (#117)
- Feature/slam toggle and smacc deep history (#122)
- Feature/dance bot s pattern (#128)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
- Add SM core test (#138)
- Feature/nav2z renaming (#144)
- Add SM Atomic SM generator. (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- slight waypoint 4 and iterations changes so robot can complete course (#155)
- Feature/migration moveit client (#151)
- initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)

## Changed
- Move method after the method it calls. Otherwise recursion could happen. (#126)

## Fixed
- Resolve compile warnings (#137)
- Fix CI: format fix python version (#148)
- Update package list. (#142)
- Update readme (#164)

## Removed
- Remove neo_simulation2 package.
- Remove merge markers from a python file. (#119)
- Remove node creation and create only a logger. (#149)
- Remove parameters smacc (#147)
- Remove sm_dance_bot_msgs

## Miscellaneous
- Various improvements in navigation and performance
- Minor format, linting, and build fixes
- Progress in testing and refining various components
- Docker refactoring and improvements
- Cleanup of pre-commit and pipeline issues
- Update READMEs with SVGs
- Add missing dependencies and fix build errors
- Update formats and configurations
- Fix compiling issues
- Fix broken master build
- Update launch command in README.md
- Fix waypoint and iteration changes for course completion
- Update moveit migration and testing progress
- Add reliability and durability configurations
- Move reference library SMs to smacc2_performance_tools
- Add QOS durability to SmaccPublisherClient
- Progress in various testing scenarios
- Rename and rework state machines for clarity and functionality

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Pablo Iñigo Blasco

Section_82
===========

Added
-----

- Feature/aws navigation sm dance bot (#174)
  - Added repo dependency for husky launch file in sm_dance_bot.
  - Added dependencies for husky simulation.
  - Added progress on aws navigation and refactorings on navigation clients and behaviors.
  - Added more on aws demo.

- Added warehouse2 (#177).
- Added Waypoint Inputs (#178).
- Added wharehouse2 progress (#179).
- Added sm_dance_bot_warehouse_3 (#181).
- Added finetuning waypoints (#187).
- Added pure spinning behavior missing files.
- Added undo motion navigation warehouse2 (#196).
- Added tuning warehouse3 (#197).
- Added tuning and fixes (#202).
- Added fixing warehouse 3 problems and other core improvements to remove dead lock, also making continuous integration green (#204).
- Added backport to foxy and minor format and linking errors foxy (#205).
- Added updating subscriber publisher components.
- Added progress in autoware machine.
- Added refining cp subscriber cp publisher.
- Added improvements in smacc core adding more components mostly developed for autoware demo.
- Added autoware demo.
- Added foxy ci.
- Added docker files for different revisions, warnings removal, and more testing on navigation.
- Added fixing docker for foxy and galactic.
- Added docker build files for all versions.
- Added barrel demo.
- Added barrel search build fix and warehouse3.
- Added fixing startup problems in warehouse 3.
- Added fixing format and minor.
- Added progress in barrel husky.
- Added progress in barrel demo.
- Added fixing broken build.
- Added more merge.
- Added docker improvements.
- Added master rolling to galactic backport.
- Added testing dance bot demos.
- Added updating galactic repos.
- Added runtime dependency.
- Added restoring ur dependency.

Changed
-------

- Updated dependencies for husky in rolling and galactic.

Fixed
-----

- Fixed formatting.
- Fixed broken build.
- Fixed some formatting and templating on SrConditional.
- Fixed moving trigger logic into headers.
- Fixed lint.
- Fixed weird moveit not downloaded repo.

Removed
-------

- Removed default values.

Contributors
------------

- Denis Štogl
- Pablo Iñigo Blasco
