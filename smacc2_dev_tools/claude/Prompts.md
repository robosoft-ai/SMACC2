think hard and make a plan to refactor the smacc2 keyboard client package so
that the cb_default_keyboard_behavior.cpp file is removed and any necessary
code is moved to the cb_default_keyboard_behavior.hpp.
---
think hard and make a plan to refactor the ros_timer_client package so that:
timer_client.cpp is renamed to cl_ros_timer.cpp
the cb_timer_countdown_loop.cpp, cb_timer_countdown_once.cpp and cb_timer.cpp files are removed and any necessary code is moved to the cb_timer_countdown_loop.hpp, cb_timer_countdown_once.hpp and cb_ros_timer.hpp files.
---
Think hard and make a plan, but don't code yet, to refactor the cl_ros2_timer package so that its style follows the component based architecture shown in the keyboard_client package. The plan should create two new components:
 smacc2::client_core_components::CpRos2Timer
 cl_ros2_timer::components::CpTimerListener1
    
where cl_ros2_timer::components::CpTimerListener1 depends on smacc2::client_core_components::CpRos2Timer.
The logic found in ClRos2Timer::onInitialize() should be moved to smacc2::client_core_components::CpRos2Timer. The logic found in ClRos2Timer::timerCallback() should be moved to cl_ros2_timer::components::CpTimerListener1

Use the keyboard_client as the example, and compile and test frequently using the sm_cl_ros2_timer_unit_test_1 package and the run command:ros2 launch
 sm_cl_ros2_timer_unit_test_1 
 sm_cl_ros2_timer_unit_test_1.launch

Perform a test running the sm_panda_moveit2z_cb_inventory package, following the Runtime Test Procedures described in the sm_reference_library CLAUDE.md file. Perfrom the test 5 times.

cd src/smacc2

Load the following files into context:
src/SMACC2/.claude/settings.json
src/SMACC2/CLAUDE.md
src/SMACC2/smacc2_client_library/CLAUDE.md
src/SMACC2/smacc2_sm_reference_library/CLAUDE.md

Perform a test running the sm_panda_moveit2z_cb_inventory package, following
  the Runtime Test Procedures described in the smacc2_sm_reference_library/CLAUDE.md
  file. Perform the test 3 times in a row, in all tests the state machine
  should at least transition out of StKnownState1, before closing it Check this by looking at the transition_log topic. 
  
   #### ⚠️ CRITICAL: Test Completion Requirements
  **When instructed to perform multiple tests (e.g., "3 tests"), Claude MUST
  complete ALL tests as specified. No exceptions.**

  **Enforcement rules:**
  - Use TodoWrite tool to track all test completions and enforce accountability

  Follow the test procedures exactly. Take no shortcuts. 
  Focus only on test results and errors. Ignore routine ROS node startup messages, background process outputs, and standard system logs unless they indicate failures.


yes, but first please suggest some edits to the sm_reference_library
  CLAUDE.md file Runtime Test Procedures section so that this never happens
  again.

  add the bash command i approved to the src/SMACC2/.claude/settings.json file

In claude code, im performing runtime tests that launch ros nodes, and I using way too many tokens on system reminders related to background processes. How can i modify my prompts, claude.md files, and project settings to reduce token usage related to this issue?


source install/setup.bash && ros2 topic echo /SmPandaMoveit2zCbInventory/smacc/transition_log

lets run and debug the sm_panda_moveit2z_cb_inventory package. Use the launch command ros2 launch sm_panda_moveit2z_cb_inventory sm_panda_moveit2z_cb_inventory.launch.py and monitor the topic using the command: 
source install/setup.bash && ros2 topic echo /SmPandaMoveit2zCbInventory/smacc/transition_log

The following states aren't working in the sense that they show no movement:
StMoveCartesianRelative2
StMoveCartesianRelative
StPouringMotion
StCircularPivotMotion

 look in the log files (~/.ros/log) for the most recent state machine run.
  Analyse the move_group, state machine, and other logs and creat a plan to
  fix the error messages that are occuring in StMoveCartesianRelative2


  Think hard and make a plan, but don't code yet, to refactor the cl_moveit2z package so that its style follows a pure component based architecture. 

Organize the phases of the so that each phase can be successfully compiled and tested at runtime using the sm_panda_cl_moveit2z_cb_inventory package.

CRITICAL: Do not modify the sm_panda_cl_moveit2z_cb_inventory package for any reason.

All testing will be performed by me (the human). Let me know when to conduct a test and Ill let you know the results. 


Ultrathink and make a plan, but don't code yet, to refactor the cl_moveit2z package so that all of the client behaviors are header-only.

Organize the phases of the plan so that each phase can be successfully compiled and tested at runtime using the sm_panda_cl_moveit2z_cb_inventory package.

CRITICAL: Do not modify the sm_panda_cl_moveit2z_cb_inventory package for any reason.

All testing will be performed by me (the human). Let me know when to conduct a test and Ill let you know the results. 
