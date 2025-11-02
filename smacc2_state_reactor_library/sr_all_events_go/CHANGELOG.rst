Changelog for package sr_all_events_go
=========================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2 <https://github.com/robosoft-ai/SMACC2>`_
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue

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
- Ignored packages not to be released
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - Format cleanup for sm_respira_1
  - Updated smacc2_rta command across readmes
  - Cleaned up sm_atomic_24hr
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Added galactic CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Updated c_cpp_properties.json
  - Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Updated doxygen links
  - More Readme Updates
  - Created new sm from sm_respira_1
  - Feature/core and navigation fixes
  - Several core improvements during navigation testing
  - Progress in aws navigation demo
  - New feature, cb_wait_topic_message: asynchronous client behavior
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic

### Contributors
- brettpac, Denis Štogl, Ubuntu 20-04-02-amd64
