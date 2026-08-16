cmake_minimum_required(VERSION 3.5)
project(sm_nav2_gazebo_test_3)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(cl_ros2_timer REQUIRED)
find_package(cl_keyboard REQUIRED)
find_package(cl_nav2z REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

# Include directories
include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
  ${cl_ros2_timer_INCLUDE_DIRS}
  ${cl_keyboard_INCLUDE_DIRS}
  ${cl_nav2z_INCLUDE_DIRS}
)

# Executable
add_executable(${PROJECT_NAME}_node
  src/sm_nav2_gazebo_test_3/sm_nav2_gazebo_test_3_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${cl_ros2_timer_LIBRARIES}
  ${cl_keyboard_LIBRARIES}
  ${cl_nav2z_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  smacc2
  cl_ros2_timer
  cl_keyboard
  cl_nav2z
  geometry_msgs
)

# Install
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config worlds maps models DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

ament_package()
