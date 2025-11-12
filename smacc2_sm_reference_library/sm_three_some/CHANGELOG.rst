Changelog for package sm_three_some
=======================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted to fix an issue with ros buildfarm
  - Addressed buildfarm issue
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
- Reverted "Ignore packages which should not be released."
- Contributors: Denis Štogl

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
### Added
- Reverted "Ignore packages which should not be released."
- Ignored packages not meant for release
- Galactic type walker (#264)
- Feature/master rolling to galactic backport (#236)
  - Updated references from SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Improved performance tests
  - Cleaned up sm_respira_1 format
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Updated CI setup and renamed rolling files
  - Fixed source CI and corrected README overview
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Updated doxygen links
  - Added new feature, cb_wait_topic_message
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic
  - Corrected all linters and formatters
- Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>
- Co-authored-by: Denis Štogl <denis@stogl.de>
- Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>
- Contributors: brettpac
