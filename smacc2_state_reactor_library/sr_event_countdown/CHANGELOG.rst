^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sr_event_countdown
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.16 (2023-07-16)
-------------------
* Merge branch 'humble' of https://github.com/robosoft-ai/SMACC2 into humble
* Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  * Attempt to fix weird issue with ros buildfarm
  * More on this buildfarm issue
  ---------
  Co-authored-by: brettpac <brettpac@pop-os.localdomain>
* Contributors: brettpac, pabloinigoblasco

2.3.6 (2023-03-12)
------------------

1.22.1 (2022-11-09)
-------------------
* pre-release
* Contributors: pabloinigoblasco

* Revert "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
* Contributors: Denis Štogl

0.3.0 (2022-04-04)
------------------

0.0.0 (2022-11-09)
------------------
* Revert "Ignore packages which should not be released."
  This reverts commit dec14a936a877b2ef722a6a32f1bf3df09312542.
* Ignore packages which should not be released.
* Backport/initial to galactic (#61)
  * reformatting the whole project
  * Remove test phase from CMake and remove dependencies from package.xml.
  * Compile with navigation and slam_toolbox.
  * Enable all packages to compile.
  * Resolve missing dependency in smacc_msgs and reorganize them for better overview.
  * getLogger functionality and refactoring
  * broken sm_respira
  * sm_respira code
  * Update README.md
  ## Additions
  - build-status table
  - detailed install instructions (adjusted from [here](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver#readme))
  ## Changes
  - default build type as `Release` (it is faster than `Debug` and executables are smaller)
  - updated examples section
  * Reset all versions to 0.0.0
  * Ignore all packages except smacc2 and smacc2_msgs
  * Update changelogs
  * 0.1.0
  * Revert "Ignore all packages except smacc2 and smacc2_msgs"
  This reverts commit f603166a4b3ccdfe96c64d9f9fb9d8b49fbf0e61.
  use this command "sudo docker build --build-arg ROS2_DISTRO=(desiredRosTag) (directoryHoldingDockerfile)/"
  * added setupTracing.sh
  Installs necessary packages and configures tracing group.
  * Removed manual installation of ros-rolling-ros2trace
  This is now automated in setupTracing.sh
  location of sh file assumed if user follows README.md under "Getting started"
  * Update tracing/ManualTracing.md
  * reactivating smacc2 nav clients for rolling via submodules
  * bug in smacc2 component
  * reverted markdowns to html
  * added README tutorial for Dockerfile
  * edited tracing.md to reflect new tracing event names
  * performance tests improvements
  * more on performance and other issues
  * sm_respira_1 format cleanup
  * sm_respira_1 format cleanup pre-commit
  * sm_respira_test_2
  * sm_respira_test_2
  * more changes on performance tests
  * Do not execute clang-format on smacc2_sm_reference_library package.
  * sm_reference_library reformatting
  * Correct trailing spaces.
  * sm_atomic_24hr
  * sm_atomic_performance_trace_1
  * Clean up of sm_atomic_24hr
  * more sm_atomic_24hr cleanup
  * Optimized deps in move_base_z_planners_common.
  * Renaming of event generator library
  * Correct build-overview table.
  * Update and unify CI configurations.
  * Use tf_geometry_msgs.h in galactic.
  * Use galactic branches in .repos-file.
  Co-authored-by: pabloinigoblasco <pablo@ibrobotics.com>
  Co-authored-by: reelrbtx <brett2@reelrobotics.com>
  Co-authored-by: Declan Dury <44791484+DecDury@users.noreply.github.com>
  Co-authored-by: DecDury <declandury@gmail.com>
  Co-authored-by: brettpac <brett@robosoft.ai>
* Contributors: Denis Štogl
