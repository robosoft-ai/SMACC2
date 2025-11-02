Changelog for package nav2z_client
===================================

.. warning::
   **DEPRECATED**: This package is deprecated in favor of cl_nav2z.
   Please use cl_nav2z for all new development.
   This package is maintained for backwards compatibility only.

2.3.16 (2023-07-16)
-------------------
* Brettpac branch (`#518 <https://github.com/robosoft-ai/SMACC2/issues/518>`_)
* Contributors: brettpac, pabloinigoblasco

2.3.6 (2023-03-12)
------------------
* No changes

1.22.1 (2022-11-09)
-------------------
* Progress in humble SMACC2 deb generation
* Feature/fix mutex galactic (`#319 <https://github.com/robosoft-ai/SMACC2/issues/319>`_)

  * Bug fix for galactic mutex
  * Testing undo motion and improving action client
  * Refactored smacc action client
  * Added smaccServiceServer client to galactic
  * Updates and testing for husky robot

* Undo motion in stEvasion after detecting enemy (`#315 <https://github.com/robosoft-ai/SMACC2/issues/315>`_)
* Feature/husky barrel improvements (`#314 <https://github.com/robosoft-ai/SMACC2/issues/314>`_)

  * Improvements in navigation client behaviors and husky barrel demo
  * More navigation behaviors on husky barrel search demo

* Husky_improvements (`#299 <https://github.com/robosoft-ai/SMACC2/issues/299>`_)

  * Different planners profiles for navigation
  * Planner switcher and fixes

* Feature/barrel husky improvements (`#293 <https://github.com/robosoft-ai/SMACC2/issues/293>`_)

  * Renamed to smacc2 and smacc2_msgs
  * Added Dockerfile for Rolling and Galactic
  * Updated mentions of SMACC/ROS to SMACC2/ROS2

* Contributors: Denis Štogl, Pablo Iñigo Blasco, pabloinigoblasco

0.3.0 (2022-04-04)
------------------
* Added galactic CI setup
* Performance tests improvements
* Contributors: Ubuntu 20-04-02-amd64, Denis Štogl

0.1.0 (2021-08-31)
------------------
* Initial release of Nav2 client
* Contributors: Brett Aldrich, Pablo Iñigo Blasco, Denis Štogl
