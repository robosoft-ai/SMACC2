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
