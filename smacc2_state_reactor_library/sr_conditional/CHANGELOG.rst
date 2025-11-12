Changelog for package sr_conditional
=====================================

2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from https://github.com/robosoft-ai/SMACC2 into humble
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted to fix weird issue with ros buildfarm
  - More details on the buildfarm issue
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

2.3.6 (2023-03-12)
--------------------------
No changes listed.

1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release
- Contributors: pabloinigoblasco

### Changed
- Progress made in humble SMACC2 deb generation
- Reverted "Ignore packages which should not be released."
- Contributors: Denis Štogl, pabloinigoblasco

0.3.0 (2022-04-04)
--------------------------
No changes listed.

0.0.0 (2022-11-09)
---------------------------
### Added
- More progress in humble SMACC2 deb generation
- Reverted "Ignore packages which should not be released."
- Ignored packages that should not be released
- Fixed: Initialized conditionFlag as false (#274) (#278)
  - Fixed: Initialized conditionFlag as false (#274)
  - Fixed precommit
  - Co-authored-by: David Revay <MrBlenny@users.noreply.github.com>
- Feature/master rolling to galactic backport (#236)
  - Updated mentions of SMACC/ROS to SMACC2/ROS2
  - Progress on navigation rolling
  - Renamed folders, deleted tracing.md, edited README.md
  - Added smacc2_performance_tools
  - Performance tests improvements
  - More on performance and other issues
  - Format cleanup for sm_respira_1
  - Format cleanup for sm_respira_1 pre-commit
  - Added sm_respira_test_2
  - More changes on performance tests
  - Do not execute clang-format on smacc2_sm_reference_library package
  - Reformatted sm_reference_library
  - Corrected trailing spaces
  - Optimized dependencies in move_base_z_planners_common
  - Renamed event generator library
  - Minor formatting
  - Added galactic CI setup and renamed rolling files (#58)
  - Fixed source CI and corrected README overview (#62)
  - Updated c_cpp_properties.json
  - Changed launch command to ros2 launch sm_respira_1 sm_respira_1.launch (#69)
  - Updated doxygen links (#70)
  - More Readme Updates (#72)
  - More Readme (#74)
  - Created new sm from sm_respira_1 (#76)
  - Feature/core and navigation fixes (#78)
  - Base for the sm_aws_aarehouse navigation
  - Progressing in AWS navigation
  - Several core improvements during navigation testing
  - Formatting improvements
  - Progress in AWS navigation demo
  - Feature/aws demo progress (#80)
  - More on navigation
  - Reworked sm_advanced_recovery_1 (#83)
  - More sm_advanced_recovery_1 work (#85)
  - Sm_advanced_recovery_1 round 4 (#86)
  - Brettpac branch (#87)
  - Added sm_atomic_performance_test_a_2
  - Added sm_atomic_performance_test_a_1
  - Added sm_atomic_performance_test_c_1 (#88)
  - Modified sm_atomic_performance_test_a_2 (#89)
  - Added sm_multi_stage_1
  - More sm_multi_stage_1 (#91)
  - Updated README.md
  - Wait topic message client behavior (#81)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Attempted precommit fixes
  - Feature/wait nav2 nodes client behavior (#82)
  - New feature, cb_wait_topic_message: asynchronous client behavior that waits for a topic message and optionally checks its contents for success
  - Added new client behavior for nav2, wait nav2 nodes subscribing to the /bond topic and waiting for them to be alive; optionally, you can select the nodes to wait for
  - Corrected all linters and formatters
  - Co-authored-by: Denis Štogl <denis@stogl.de>
  - Co-authored-by: Denis Štogl <destogl@users.noreply.github.com>

0.1.0 (Date: TBD)
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
- Cleaned up formatting in `sm_respira_1` and `sm_atomic_24hr`
- Optimized dependencies in `move_base_z_planners_common`
- Renamed event generator library
- Corrected build-overview table
- Updated and unified CI configurations
- Used `tf_geometry_msgs.h` in Galactic
- Switched to Galactic branches in `.repos-file`

### Removed
- Manual installation of `ros-rolling-ros2trace`, now automated in `setupTracing.sh`

### Miscellaneous
- More merge
- Docker improvements
- Master rolling to Galactic backport
- Fixing build
- Testing dance bot demos
- Updating Galactic repos
- Runtime dependency
- Restoring UR dependency
- Reverted markdowns to HTML
- Added README tutorial for Dockerfile
- Edited `tracing.md` to reflect new tracing event names
- Performance tests improvements
- More on performance and other issues
- Do not execute `clang-format` on `smacc2_sm_reference_library`
- Cleaned up `sm_atomic_24hr`
- More cleanup on `sm_atomic_24hr`
- Use of `galactic` branches in `.repos-file`

Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco
