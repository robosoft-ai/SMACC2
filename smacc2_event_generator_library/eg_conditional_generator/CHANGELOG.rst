Changelog for package eg_conditional_generator
=================================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from `robosoft-ai/SMACC2` repository.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for a strange issue with ros buildfarm.
  - Further details on the buildfarm issue.
  - Co-authored-by: brettpac <brettpac@pop-os.localdomain>
- Contributors: brettpac, pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
No changes listed.

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release.
- Contributors: pabloinigoblasco

### Changed
- Reverted "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Contributors: Denis Štogl

Version 0.3.0 (2022-04-04)
--------------------------
No changes listed.

Version 0.0.0 (2022-11-09)
---------------------------
### Added
- Reverted "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Ignored packages that should not be released.
- Backport/initial to galactic (#61)
  - Reformatted the entire project.
  - Removed test phase from CMake and dependencies from package.xml.
  - Compiled with navigation and slam_toolbox.
  - Enabled all packages to compile.
  - Resolved missing dependency in smacc_msgs and reorganized for better overview.
  - Added getLogger functionality and refactored code.
  - Updated README.md.
  - Added build-status table.
  - Provided detailed install instructions.
  - Reset all versions to 0.0.0.
  - Ignored all packages except smacc2 and smacc2_msgs.
  - Updated changelogs.
  - Reverted "Ignore all packages except smacc2 and smacc2_msgs."
  - Added setupTracing.sh for necessary package installation and tracing group configuration.
  - Removed manual installation of ros-rolling-ros2trace, now automated in setupTracing.sh.
  - Updated tracing/ManualTracing.md.
  - Reactivated smacc2 nav clients for rolling via submodules.
  - Performance tests improvements.
  - Cleaned up formats.
  - Optimized dependencies.
  - Renamed event generator library.
  - Updated and unified CI configurations.
  - Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>, reelrbtx <brett2@reelrobotics.com>, Declan Dury <44791484+DecDury@users.noreply.github.com>, DecDury <declandury@gmail.com>, brettpac <brett@robosoft.ai>
- Contributors: Denis Štogl
