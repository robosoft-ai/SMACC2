Changelog for package multirole_sensor_client
=============================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further work on buildfarm problem
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

2.3.6 (2023-03-12)
------------------

1.22.1 (2022-11-09)
-------------------
### Added
- Pre-release
- Contributors: pabloinigoblasco

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
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
  - Format cleanup for sm_respira_1
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - CI setup and file renaming
  - Updated launch command to ros2 launch sm_respira_1 sm_respira_1.launch
  - Added new feature, cb_wait_topic_message
  - Corrected all linters and formatters
  - Co-authored-by: Ubuntu 20-04-02-amd64 <brett@robosoft.ai>, Denis Štogl <denis@stogl.de>
- Contributors: Denis Štogl, pabloinigoblasco

0.1.0 (Date: TBD)

- Build-status table
- Detailed install instructions (adjusted from [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme))

- Default build type set to `Release` for faster performance and smaller executables
- Updated examples section

- Resolved missing dependency in smacc_msgs and reorganized for better overview
- Fixed build issues
- Fixed bug in smacc2 component
- Cleaned up formatting in various packages
- Optimized dependencies in move_base_z_planners_common
- Corrected build-overview table
- Updated and unified CI configurations

- Manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh

- Reverted changes in commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61
- Restored all versions to 0.0.0
- Ignored all packages except smacc2 and smacc2_msgs
- Updated changelogs
- Reformatting the whole project
- Enabled all packages to compile
- Added getLogger functionality and refactoring
- Reactivated smacc2 nav clients for rolling via submodules
- Edited tracing.md to reflect new tracing event names
- Performance tests improvements
- Various performance and bug fixes
- Renamed event generator library
- Used tf_geometry_msgs.h in galactic
- Used galactic branches in .repos-file
- Do not execute clang-format on smacc2_sm_reference_library package
- Cleaned up trailing spaces
- Added README tutorial for Dockerfile
- Updated README.md
- Updated tracing/ManualTracing.md
- Added setupTracing.sh for automated installation of ros-rolling-ros2trace
- Location of sh file assumed if user follows README.md under "Getting started"

- Denis Štogl
- Pablo Iñigo Blasco
- pabloinigoblasco
- reelrbtx
- Declan Dury
- brettpac
- David Revay
