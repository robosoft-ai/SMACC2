Changelog for package sm_multi_stage_1
=======================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2`
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further work on buildfarm issue
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
- Feature/galactic rolling merge (#288)
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs"
  - Updated description table
  - Updated table
  - Copied initial docs
  - Dockerfile with ROS distro as argument
  - Opened new folder for additional tracing contents
  - Deleted tracing directory
  - Moved tracing.md to tracing directory
  - Added setupTracing.sh for necessary package installation and tracing group configuration
  - Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh
  - Created alternative ManualTracing
  - Added new sm markdowns
  - Added Dockerfile for Rolling and Galactic
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
  - Renamed tracing events
  - Bug fixes in smacc2 component
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup for sm_respira_1
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Minor formatting changes
  - Added galactic CI setup and renamed rolling files (#58)
  - Fixed source CI and corrected README overview (#62)
  - Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  - Updated doxygen links (#70)
  - More Readme Updates (#72)
  - More Readme (#74)
  - Created new sm from sm_respira_1 (#76)
  - Feature/core and navigation fixes (#78)
  - Several core improvements during navigation testing
  - Progress in aws navigation demo
  - Feature/aws demo progress (#80)
  - Format improvements
  - Feature/aws demo progress (#80)
  - sm_advanced_recovery_1 reworked (#83)
  - More sm_advanced_recovery_1 work (#85)
  - sm_advanced_recovery_1 round 4 (#86)
  - sm_atomic_performance_test_a_2
  - sm_atomic_performance_test_a_1
  - sm_atomic_performance_test_c_1 (#88)
  - Modifying sm_atomic_performance_test_a_2 (#89)
  - sm_multi_stage_1
  - Update README.md
  - Wait topic message client behavior (#81)

0.1.0 (2022-01-01)
---------------------------

### Added
- Feature/diverse improvements navigation performance (#117)
  - Additional linting and formatting
- Feature/slam toggle and smacc deep history (#122)
  - Progress in navigation, slam toggle client behaviors, and slam_toolbox components
  - Introducing slam pausing/resuming functionality for testing sm_dance_bot
- Feature/dance bot s pattern (#128)
  - Polishing sm_dance_bot and s-pattern
- First working version of sm template and template generator (#127)
- Feature/sm dance bot refine (#131)
- Feature/sm dance bot refine 2 (#132)
  - Build fix
- Feature/nav2z renaming (#144)
  - Navigation 2 stack renaming
- Add SM Atomic SM generator (#143)
- Rolling Docker environment to be executed from any environment (#154)
- Feature/sm dance bot strikes back refactoring (#152)
- Feature/migration moveit client (#151)
  - Initial migration to smacc2
  - Progressing in the moveit migration testing
- Initial state machine transition timestamp (#165)
- Add QOS durability to SmaccPublisherClient (#163)
- Feature/testing moveit behaviors (#167)
- Sm_pubsub_1 (#169)
- Sm_pubsub_1 part 2 (#170)
- Sm_advanced_recovery_1 renaming (#171)
- Sm_multi_stage_1 reworking (#172)
  - Multistage modes, sequences, steps, and finishing touches
- Feature/aws navigation sm dance bot (#174)
  - Progress on aws navigation and refactorings on navigation clients and behaviors

### Changed
- Move method after the method it calls to prevent recursion (#126)
- Resolve compile warnings (#137)
- Minor navigation improvements (#141)
- Using local action messages (#139)
- Removing parameters smacc (#147)
- Update package list (#142)
- Remove node creation and create only a logger (#149)
- Warehouse2 progress (#179)

### Fixed
- Minor tuning to mitigate overshot issue cases (#133)
- Fix CI: format fix python version (#148)
- Fixing broken master build (#167)
- Fixing broken build (#174)

### Removed
- Remove merge markers from a python file (#119)
- Removing sm_dance_bot_msgs (#144)
- Removing test from main moveit cmake (#151)
- Removing parameters smacc (#147)

### Miscellaneous
- Added SVGs to READMEs of atomic, dance_bot, and others (#140)
- Added remaining SVGs to READMEs (#145)
- Update README (#164)
- More readme updates (#164)
- Noticed launch command was incorrect in README.md, fixed launch command for sm_dance_bot_strikes_back, and removed some comments (#147)
- Precommit cleanup
- Workflows update
- Pending references
- Repos dependency
- Docker refactoring
- Minor format issues (#134)
- Minor tweaks (#130)
- Minor (#124)
- Minor
- Noticed typo
- Finnaly > Finally

Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
Co-authored-by: DecDury <declandury@gmail.com>
Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
Co-authored-by: Denis Štogl <denis@stogl.de>
