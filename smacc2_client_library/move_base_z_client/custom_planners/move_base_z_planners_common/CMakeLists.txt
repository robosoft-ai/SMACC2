cmake_minimum_required(VERSION 3.5)
project(move_base_z_planners_common)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(angles)
find_package(geometry_msgs)
find_package(nav_msgs)
find_package(rclcpp)
find_package(tf2)
find_package(tf2_ros)
find_package(tf2_geometry_msgs)

set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  angles
  tf2
  tf2_geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
            src/${PROJECT_NAME}/common.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_include_directories(${PROJECT_NAME} PRIVATE include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
