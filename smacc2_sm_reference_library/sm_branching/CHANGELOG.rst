Changelog for package sm_branching
======================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` into humble

### Changed
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted to fix weird issue with ros buildfarm
  - Made more adjustments related to the buildfarm issue

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
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542
- Ignored packages which should not be released
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Made progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Improved performance tests
  - Cleaned up sm_respira_1 format
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Added galactic CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Updated doxygen links
  - Created new sm from sm_respira_1
  - Made core and navigation fixes
  - Reworked sm_advanced_recovery_1
  - Worked on sm_atomic_performance_test_a_2, sm_atomic_performance_test_a_1, sm_atomic_performance_test_c_1
  - Modified sm_atomic_performance_test_a_2
  - Worked on sm_multi_stage_1
  - Updated README.md
  - Implemented new feature, cb_wait_topic_message
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting they are alive
  - Corrected all linters and formatters

### Contributors
- brettpac, Ubuntu 20-04-02-amd64, Denis Štogl
