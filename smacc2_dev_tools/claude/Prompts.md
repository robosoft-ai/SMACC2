> think hard and make a plan to refactor the smacc2 keyboard client package so
  that the cb_default_keyboard_behavior.cpp file is removed and any necessary
  code is moved to the cb_default_keyboard_behavior.hpp.

think hard and make a plan to refactor the ros_timer_client package so that:
timer_client.cpp is renamed to cl_ros_timer.cpp
the cb_timer_countdown_loop.cpp, cb_timer_countdown_once.cpp and cb_timer.cpp files are removed and any necessary code is moved to the cb_timer_countdown_loop.hpp, cb_timer_countdown_once.hpp and cb_ros_timer.hpp files.

create a new component. See
a new file cp_ros2_timer.hpp is created in the smacc2/include/client_core_components folder 
