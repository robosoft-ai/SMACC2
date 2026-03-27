cmake_minimum_required(VERSION 3.5)
project(cl_px4_mr)

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
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
)

set(SOURCES
  src/cl_px4_mr/cl_px4_mr.cpp
  src/cl_px4_mr/components/cp_vehicle_command.cpp
  src/cl_px4_mr/components/cp_offboard_keep_alive.cpp
  src/cl_px4_mr/components/cp_trajectory_setpoint.cpp
  src/cl_px4_mr/components/cp_vehicle_status.cpp
  src/cl_px4_mr/components/cp_vehicle_local_position.cpp
  src/cl_px4_mr/components/cp_vehicle_command_ack.cpp
  src/cl_px4_mr/components/cp_goal_checker.cpp
  src/cl_px4_mr/components/cp_micro_ros_agent.cpp
  src/cl_px4_mr/client_behaviors/cb_arm_px4.cpp
  src/cl_px4_mr/client_behaviors/cb_disarm_px4.cpp
  src/cl_px4_mr/client_behaviors/cb_takeoff.cpp
  src/cl_px4_mr/client_behaviors/cb_land.cpp
  src/cl_px4_mr/client_behaviors/cb_go_to_location.cpp
  src/cl_px4_mr/client_behaviors/cb_orbit_location.cpp
  src/cl_px4_mr/client_behaviors/cb_hold_position.cpp
  src/cl_px4_mr/client_behaviors/cb_yaw_rotate.cpp
  src/cl_px4_mr/client_behaviors/cb_change_altitude.cpp
  src/cl_px4_mr/client_behaviors/cb_follow_waypoints.cpp
  src/cl_px4_mr/client_behaviors/cb_figure_eight.cpp
  src/cl_px4_mr/client_behaviors/cb_return_to_home.cpp
  src/cl_px4_mr/client_behaviors/cb_spiral_pattern.cpp
  src/cl_px4_mr/client_behaviors/cb_connect_micro_ros_agent.cpp
)

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} smacc2 px4_msgs rclcpp)

ament_export_include_directories(include)
ament_export_dependencies(smacc2 px4_msgs rclcpp Boost)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/)

ament_package()
