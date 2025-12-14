cmake_minimum_required(VERSION 3.5)
project(cl_isaac_apriltag)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

ament_export_include_directories(include)
ament_export_dependencies(
  smacc2
  isaac_ros_apriltag_interfaces
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
