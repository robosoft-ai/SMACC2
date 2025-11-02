Changelog for package cl_keyboard
==================================

.. note::
   This package was created in September 2025 as a refactored version of keyboard_client.
   For historical changes prior to 2.3.20, see keyboard_client/CHANGELOG.rst.
   The keyboard_client package is now deprecated in favor of cl_keyboard.

2.3.20 (2025-11-01)
-------------------
* New cl_keyboard client library created as replacement for keyboard_client (`#621 <https://github.com/robosoft-ai/SMACC2/issues/621>`_)

  * Refactored keyboard client to remove cb.cpp file (`#609 <https://github.com/robosoft-ai/SMACC2/issues/609>`_)
  * Base cp_keyboard implementation (`#605 <https://github.com/robosoft-ai/SMACC2/issues/605>`_)
  * Final keyboard client refactor changes with formatting (`#599 <https://github.com/robosoft-ai/SMACC2/issues/599>`_)

* Moving reference library from ros_timer_client and keyboard_client to cl_ros2_timer and cl_keyboard (`#645 <https://github.com/robosoft-ai/SMACC2/issues/645>`_)
* Contributors: Brett Aldrich, Pablo Iñigo Blasco
