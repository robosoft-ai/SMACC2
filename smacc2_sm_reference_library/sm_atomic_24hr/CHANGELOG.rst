Changelog for package sm_atomic_24hr
=========================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2`
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue
  - Further details on buildfarm problem
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
---------------------------
### Added
- Pre-release
- Contributors: pabloinigoblasco

Version 1.22.1 (2022-11-09)
---------------------------
### Changed
- Reverted "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Contributors: Denis Štogl

Version 0.3.0 (2022-04-04)
---------------------------
No changes recorded.

Version 0.0.0 (2022-11-09)
---------------------------
### Added
- Reverted "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Ignored packages not meant for release
- Backport/initial to galactic (#61)
  - Reformatted entire project
  - Removed test phase from CMake and package.xml dependencies
  - Compiled with navigation and slam_toolbox
  - Enabled compilation of all packages
  - Resolved missing dependency in smacc_msgs and reorganized for clarity
  - Refactored getLogger functionality
  - Updated README.md
  - Added build-status table and detailed install instructions
  - Default build type set to `Release` for faster performance
  - Updated examples section
  - Reset all versions to 0.0.0
  - Ignored all packages except smacc2 and smacc2_msgs
  - Updated changelogs
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs"
    This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  - Added setupTracing.sh for necessary package installation and configuration
  - Automated ros-rolling-ros2trace installation in setupTracing.sh
  - Updated tracing/ManualTracing.md
  - Reactivated smacc2 nav clients for rolling via submodules
  - Resolved bug in smacc2 component
  - Updated README tutorial for Dockerfile
  - Edited tracing.md for new tracing event names
  - Improved performance tests
  - Cleaned up sm_respira_1 format
  - Optimized deps in move_base_z_planners_common
  - Renamed event generator library
  - Corrected build-overview table
  - Updated and unified CI configurations
  - Used tf_geometry_msgs.h in galactic
  - Used galactic branches in .repos-file
  - Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  - Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  - Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  - Co-authored-by: DecDury <declandury@gmail.com>
  - Co-authored-by: brettpac <brett@robosoft.ai>
- Contributors: Denis Štogl
