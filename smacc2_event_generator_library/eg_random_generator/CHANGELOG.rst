```
Changelog for package eg_random_generator
=========================================

Version 2.3.16 (2023-07-16)
---------------------------
### Added
- Merged branch 'humble' from robosoft-ai/SMACC2 repository.
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempted fix for ros buildfarm issue.
  - Further work on buildfarm problem.
  Co-authored-by: brettpac <brettpac@pop-os.localdomain>
### Contributors
- brettpac
- pabloinigoblasco

Version 2.3.6 (2023-03-12)
--------------------------
No significant changes.

Version 1.22.1 (2022-11-09)
---------------------------
### Added
- Pre-release version.
### Contributors
- pabloinigoblasco

Version 0.3.0 (2022-04-04)
--------------------------
No significant changes.

Version 0.0.0 (2022-11-09)
---------------------------
### Added
- Reverted commit dec14a936a877b2ef722a6a32f1bf3df09312542.
- Ignored packages not to be released.
- Backported/initial to galactic (#61)
  - Reformatted the project.
  - Removed test phase from CMake and dependencies from package.xml.
  - Compiled with navigation and slam_toolbox.
  - Enabled all packages to compile.
  - Resolved missing dependency in smacc_msgs and reorganized for clarity.
  - Added getLogger functionality and refactored code.
  - Updated README.md.
  ## Additions
  - Build-status table.
  - Detailed install instructions.
  ## Changes
  - Default build type set to `Release`.
  - Updated examples section.
  - Reset all versions to 0.0.0.
  - Ignored all packages except smacc2 and smacc2_msgs.
  - Updated changelogs.
  - Reverted commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  - Added setupTracing.sh for automated installation.
  - Removed manual installation of ros-rolling-ros2trace.
  - Updated tracing/ManualTracing.md.
  - Reactivated smacc2 nav clients for rolling via submodules.
  - Bug fixes and performance improvements.
  - Cleaned up code in various components.
  - Optimized dependencies in move_base_z_planners_common.
  - Renamed event generator library.
  - Unified CI configurations.
  - Updated to use galactic branches.
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
### Contributors
- Denis Štogl
```
