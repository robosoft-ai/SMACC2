Changelog for package smacc2
================================

3.0.1 (2025-01-16)
-------------------
### Added
- **ROS2 Jazzy Support**: Official support for ROS2 Jazzy Jalisco

  SMACC2 now defaults to ROS2 Jazzy (Ubuntu 24.04 Noble) as the primary development branch.
  This marks the transition to the latest LTS ROS2 distribution with improved performance
  and new features.

### Fixed
- **CI Format Pipeline**: Fixed PEP 668 externally-managed-environment errors in GitHub Actions

  Updated CI workflow and documentation to use apt-installed `pre-commit` and `clang-format`
  instead of pip, ensuring compatibility with Ubuntu 24.04's PEP 668 Python packaging
  restrictions. This provides a more robust and future-proof CI/CD pipeline.

  - Updated `.github/workflows/ci-format.yml` to install pre-commit via apt
  - Updated `CONTRIBUTING.md` with correct installation instructions
  - Updated `README.md` to include development tools in setup instructions

### Documentation
- Enhanced developer documentation for Ubuntu 24.04+ environments
- Added comprehensive PEP 668 compliance documentation

### Contributors
- Brett Aldrich (@brettpac)

2.3.20 (2025-11-01)
-------------------
### Fixed
- **CRITICAL**: Fix double onExit() calls in client behaviors (`#556 <https://github.com/robosoft-ai/SMACC2/issues/556>`_, `#558 <https://github.com/robosoft-ai/SMACC2/issues/558>`_)

  Client behavior's onExit method was being invoked twice during state transitions due to
  duplicate notifyOnStateExitting() call. This caused issues including deadlocks in behaviors
  with thread joins in onExit. Fixed by removing redundant call outside mutex lock.

  - Root cause identified by @yassiezar
  - Issue reported in apt packages by @Crowdedlight

- Fix toggle functionality (`#587 <https://github.com/robosoft-ai/SMACC2/issues/587>`_)

### Added
- New cl_moveit2z client library (`#638 <https://github.com/robosoft-ai/SMACC2/issues/638>`_)
- New cl_keyboard client and removal of sm_pubsub_1 (`#621 <https://github.com/robosoft-ai/SMACC2/issues/621>`_)
- New cl_ros2_timer unit test (`#616 <https://github.com/robosoft-ai/SMACC2/issues/616>`_)
- Nova Carter navigation behaviors (`#608 <https://github.com/robosoft-ai/SMACC2/issues/608>`_)
- Progress on requiresComponent (`#628 <https://github.com/robosoft-ai/SMACC2/issues/628>`_)

### Changed
- Refactoring of cl_moveit2z to component-based architecture & header-only implementation (`#639 <https://github.com/robosoft-ai/SMACC2/issues/639>`_)
- Refactoring cl_nav2z to remove legacy API support and update client behaviors (`#625 <https://github.com/robosoft-ai/SMACC2/issues/625>`_)
- Refactor of cl_nav2z to component-based architecture (`#624 <https://github.com/robosoft-ai/SMACC2/issues/624>`_)
- Refactor of cl_nav2z, moved cp_nav2_action_interface.hpp into folder (`#626 <https://github.com/robosoft-ai/SMACC2/issues/626>`_)
- Refactor of cl_ros2_timer namespace structure (include paths) (`#623 <https://github.com/robosoft-ai/SMACC2/issues/623>`_)
- Refactored cl_ros2_timer components to header-only (`#619 <https://github.com/robosoft-ai/SMACC2/issues/619>`_)
- Refactoring cl_ros2_timer to component-based architecture (`#618 <https://github.com/robosoft-ai/SMACC2/issues/618>`_)
- Refactoring cl_ros2_timer to header-lite (`#617 <https://github.com/robosoft-ai/SMACC2/issues/617>`_)
- Final keyboard client refactor changes with formatting (`#599 <https://github.com/robosoft-ai/SMACC2/issues/599>`_)
- Refactor keyboard client to remove cb.cpp file (`#609 <https://github.com/robosoft-ai/SMACC2/issues/609>`_)
- Refactoring: renaming onOrthogonalAllocation (`#600 <https://github.com/robosoft-ai/SMACC2/issues/600>`_)
- Refactor base components (`#606 <https://github.com/robosoft-ai/SMACC2/issues/606>`_)
- sm_panda_moveit2z_cb_inventory refactor (`#633 <https://github.com/robosoft-ai/SMACC2/issues/633>`_)
- Moving reference library from ros_timer_client and keyboard_client to cl_ros2_timer and cl_keyboard (`#645 <https://github.com/robosoft-ai/SMACC2/issues/645>`_)
- Trimming sm_atomic_services and sm_atomic_24hr from sm_reference_library (`#644 <https://github.com/robosoft-ai/SMACC2/issues/644>`_)
- Update ROS distribution from Galactic to Humble (`#631 <https://github.com/robosoft-ai/SMACC2/issues/631>`_)
- Update include path for cl_ros2_timer (`#629 <https://github.com/robosoft-ai/SMACC2/issues/629>`_)

### Documentation
- Updating CLAUDE.md files (`#643 <https://github.com/robosoft-ai/SMACC2/issues/643>`_)
- CLAUDE.MD file for client behavior libraries (`#586 <https://github.com/robosoft-ai/SMACC2/issues/586>`_)
- Fixing sm readmes (`#632 <https://github.com/robosoft-ai/SMACC2/issues/632>`_)
- Updating sm_simple_action_client launch file (`#642 <https://github.com/robosoft-ai/SMACC2/issues/642>`_)
- Update README.md (`#576 <https://github.com/robosoft-ai/SMACC2/issues/576>`_)

### Contributors
- Pablo Iñigo Blasco (@pabloinigoblasco)
- Brett Aldrich (@brettpac)
- Jaycee Lock (@yassiezar)
- Crowdedlight (@Crowdedlight)

0.4.0 (2022-04-04)
------------------
### Added
- Feature/fixing type string walker (`#263 <https://github.com/StoglRobotics-forks/SMACC2/issues/263>`_)
- Feature/fixing husky build rolling (`#258 <https://github.com/StoglRobotics-forks/SMACC2/issues/258>`_)
- Merging code from backport foxy and updates about autoware (`#208 <https://github.com/StoglRobotics-forks/SMACC2/issues/208>`_)
- wharehouse2 progress (`#179 <https://github.com/StoglRobotics-forks/SMACC2/issues/179>`_)
- Feature/aws navigation sm dance bot (`#174 <https://github.com/StoglRobotics-forks/SMACC2/issues/174>`_)
- Feature/sm dance bot strikes back refactoring (`#152 <https://github.com/StoglRobotics-forks/SMACC2/issues/152>`_)
- Feature/cb pause slam (`#98 <https://github.com/StoglRobotics-forks/SMACC2/issues/98>`_)
- Merge branch 'renameTracingEvents' of https://github.com/DecDury/SMACC2 into DecDury-renameTracingEvents

### Contributors
- David Revay, DecDury, Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco, reelrbtx

0.1.0 (2021-08-31)
------------------
### Added
- Initial release of SMACC2 core

### Contributors
- Brett Aldrich, Pablo Inigo Blasco, Denis Štogl

2.3.16 (2023-07-16)
-------------------
### Added
- Merge branch 'humble' of https://github.com/robosoft-ai/SMACC2 into humble

### Fixed
- Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
  - Attempt to fix weird issue with ros buildfarm
  - More on this buildfarm issue

### Contributors
- brettpac, pabloinigoblasco

2.3.6 (2023-03-12)
------------------

1.22.1 (2022-11-09)
-------------------
### Added
- pre-release

### Contributors
- pabloinigoblasco

0.0.0 (2021-08-30)
------------------
### Added
- Initial release of SMACC2 core

### Contributors
- Pablo Inigo Blasco
