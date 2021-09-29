cmake_minimum_required(VERSION 3.8)
project(sm_respira_1)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(MAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(smacc2)
find_package(std_msgs)
find_package(multirole_sensor_client)
find_package(sr_all_events_go)
find_package(ros_timer_client)
find_package(ros_publisher_client)
find_package(keyboard_client)

set(dependencies
  rclcpp
  smacc2
  std_msgs
  multirole_sensor_client
  sr_all_events_go
  ros_timer_client
  keyboard_client
  ros_publisher_client
)

include_directories(include
  ${rclcpp_INCLUDE_DIRS}
  ${smacc2_INCLUDE_DIRS}
  ${ros_timer_client_INCLUDE_DIRS}
  ${keyboard_client_INCLUDE_DIRS}
  ${ros_publisher_client_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sr_all_events_go_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/sm_respira_1_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
ament_export_include_directories(include)

target_link_libraries(${PROJECT_NAME}_node
  ${rclcpp_LIBRARIES}
  ${smacc2_LIBRARIES}
  ${ros_timer_client_LIBRARIES}
  ${keyboard_client_LIBRARIES}
  ${ros_publisher_client_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sr_all_events_go_LIBRARIES}
)

ament_package()

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib                 # static libraries
  LIBRARY DESTINATION lib                 # dynamic libraries
  RUNTIME DESTINATION lib/${PROJECT_NAME} # executables
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
