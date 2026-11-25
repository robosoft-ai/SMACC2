cmake_minimum_required(VERSION 3.10)
project(cl_rgb)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(SOURCES
  src/cl_rgb/cl_rgb.cpp
  src/cl_rgb/geometry.cpp
  src/cl_rgb/colour.cpp
  src/cl_rgb/box_fit.cpp
  src/cl_rgb/box_edge_search.cpp
  src/cl_rgb/mesh_silhouette.cpp
  src/cl_rgb/rim_edge_search.cpp
  src/cl_rgb/table_survey.cpp
  src/cl_rgb/components/cp_transforms.cpp
  src/cl_rgb/components/cp_camera.cpp
  src/cl_rgb/components/cp_table_plane.cpp
  src/cl_rgb/components/cp_table_segmenter.cpp
  src/cl_rgb/components/cp_scene_survey.cpp
  src/cl_rgb/client_behaviors/cb_wait_for_frame.cpp
  src/cl_rgb/client_behaviors/cb_survey_table.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}
  smacc2 rclcpp sensor_msgs geometry_msgs cv_bridge tf2 tf2_ros tf2_geometry_msgs)

ament_export_include_directories(include)
ament_export_dependencies(
  smacc2 rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV tf2 tf2_ros tf2_geometry_msgs)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry ${PROJECT_NAME})
  ament_add_gtest(test_box_fit test/test_box_fit.cpp)
  target_link_libraries(test_box_fit ${PROJECT_NAME})
  ament_add_gtest(test_colour test/test_colour.cpp)
  target_link_libraries(test_colour ${PROJECT_NAME})
  ament_add_gtest(test_table_survey test/test_table_survey.cpp)
  target_link_libraries(test_table_survey ${PROJECT_NAME})
endif()

ament_package()
