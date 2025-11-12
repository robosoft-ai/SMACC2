cmake_minimum_required(VERSION 3.5)
project(sm_panda_cl_moveit2z_cb_inventory)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cl_ros2_timer REQUIRED)
find_package(smacc2 REQUIRED)
find_package(smacc2_msgs REQUIRED)
find_package(std_msgs)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(cl_moveit2z REQUIRED)
find_package(cl_keyboard REQUIRED)
find_package(rclcpp REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  cl_ros2_timer
  smacc2
  smacc2_msgs
  std_msgs
  cl_moveit2z
  cl_keyboard
  rclcpp
)

include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
  ${moveit2z_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}/${PROJECT_NAME}_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${cl_ros2_timer_LIBRARIES}
  ${moveit2z_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
