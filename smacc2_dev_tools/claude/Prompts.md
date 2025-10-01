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
  file. Perform the test 5 times in a row, in all tests the state machine
  should at least transition out of StKnownState1, before closing it. Do this exactly, do not deviate from the plan. 

yes, but first please suggest some edits to the sm_reference_library
  CLAUDE.md file Runtime Test Procedures section so that this never happens
  again.

  add the bash command i approved to the src/SMACC2/.claude/settings.json file
