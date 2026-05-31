Changelog for package cl_ros2_timer
====================================

.. note::
   This package was refactored in September 2025 from ros_timer_client.
   For historical changes prior to 2.3.20, see ros_timer_client/CHANGELOG.rst.
   The ros_timer_client package is now deprecated in favor of cl_ros2_timer.

3.1.0 (2026-05-31)
-------------------
### Changed (Breaking)
- **cl_ros2_timer duration-based API** (`#61 <https://github.com/robosoft-ai/SMACC2/issues/61>`_)

  ``CbTimerCountdownOnce`` and ``CbTimerCountdownLoop`` now accept ``rclcpp::Duration``
  via C++ chrono literals (``5s``, ``500ms``, ``4min``) instead of raw tick counts.
  Each behavior creates its own dedicated ``rclcpp::WallTimer``, giving exact,
  period-independent timing with ~50ms residual from the SMACC2 SignalDetector
  20 Hz polling loop.

  Migration::

    // Before (tick count, period-dependent)
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);  // 10 ticks × timer period

    // After (duration, period-independent)
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10s); // exactly 10 seconds

- **ClRos2Timer** is now a pure marker client — the duration constructor argument has been
  removed. ``or_timer.hpp`` files simply call ``createClient<ClRos2Timer>()`` with no args.

### Removed
- ``CbRos2Timer`` — was unused in all reference state machines; removed.
- ``CpTimerListener1`` — now orphaned after the tick-stack removal; removed.

### Contributors
- Brett Aldrich

2.3.20 (2025-11-01)
-------------------
* New cl_ros2_timer unit test (`#616 <https://github.com/robosoft-ai/SMACC2/issues/616>`_)
* Refactoring cl_ros2_timer to header-lite (`#617 <https://github.com/robosoft-ai/SMACC2/issues/617>`_)
* Refactoring cl_ros2_timer to component-based architecture (`#618 <https://github.com/robosoft-ai/SMACC2/issues/618>`_)
* Refactored cl_ros2_timer components to header-only (`#619 <https://github.com/robosoft-ai/SMACC2/issues/619>`_)
* Refactor of cl_ros2_timer namespace structure (include paths) (`#623 <https://github.com/robosoft-ai/SMACC2/issues/623>`_)
* Update include path for cl_ros2_timer (`#629 <https://github.com/robosoft-ai/SMACC2/issues/629>`_)
* Moving reference library from ros_timer_client and keyboard_client to cl_ros2_timer and cl_keyboard (`#645 <https://github.com/robosoft-ai/SMACC2/issues/645>`_)
* Contributors: Brett Aldrich, Pablo Iñigo Blasco
