Changelog for package sr_event_countdown
=========================================

2.3.16 (2023-07-16)
-------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue.
  - Further updates on buildfarm problem.
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

2.3.6 (2023-03-12)
------------------

1.22.1 (2022-11-09)
-------------------
### Added
- Pre-release.
- Contributors: pabloinigoblasco

### Changed
- Reverted "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Contributors: Denis Štogl

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
### Added
- Reverted "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Ignored packages that should not be released.
- Backport/initial to galactic (#61)
  - Reformatted the entire project.
  - Removed test phase from CMake and dependencies from package.xml.
  - Compiled with navigation and slam_toolbox.
  - Enabled compilation of all packages.
  - Resolved missing dependency in smacc_msgs and reorganized for better overview.
  - Refactored getLogger functionality.
  - Updated README.md.
  - Added build-status table.
  - Provided detailed install instructions.
  - Reset all versions to 0.0.0.
  - Ignored all packages except smacc2 and smacc2_msgs.
  - Updated changelogs.
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs."
  - Added setupTracing.sh for necessary package installation and tracing group configuration.
  - Automated ros-rolling-ros2trace installation in setupTracing.sh.
  - Updated tracing/ManualTracing.md.
  - Reactivated smacc2 nav clients for rolling via submodules.
  - Fixed bugs in smacc2 component.
  - Added README tutorial for Dockerfile.
  - Edited tracing.md for new tracing event names.
  - Improved performance tests.
  - Cleaned up formats in sm_respira_1.
  - Optimized dependencies in move_base_z_planners_common.
  - Renamed event generator library.
  - Updated and unified CI configurations.
  - Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>, reelrbtx <brett2@reelrobotics.com>, Declan Dury <44791484+DecDury@users.noreply.github.com>, DecDury <declandury@gmail.com>, brettpac <brett@robosoft.ai>
- Contributors: Denis Štogl
