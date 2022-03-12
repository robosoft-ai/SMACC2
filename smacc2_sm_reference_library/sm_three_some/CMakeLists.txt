cmake_minimum_required(VERSION 3.5)
project(sm_three_some)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smacc2_msgs REQUIRED)
find_package(multirole_sensor_client REQUIRED)
find_package(sr_all_events_go REQUIRED)
find_package(ros_timer_client REQUIRED)
find_package(keyboard_client REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_publisher_client REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

set(dependencies
  smacc2
  std_msgs
  smacc2_msgs
  multirole_sensor_client
  sr_all_events_go
  ros_timer_client
  keyboard_client
  ros_publisher_client
  rclcpp
)

add_executable(${PROJECT_NAME}_node src/sm_three_some_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_node
  ${Boost_LIBRARIES}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
