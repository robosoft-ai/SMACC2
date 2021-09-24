cmake_minimum_required(VERSION 3.5)
project(sm_dance_bot_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "servers/led_action_server/action/LEDControl.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
