cmake_minimum_required(VERSION 3.5)
project(template_client)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(smacc2 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${yaml-cpp_INCLUDE_DIRS}
)

# Set dependencies
set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  smacc2
  yaml-cpp
)

# ==============================================================================
# Build library
# ==============================================================================
add_library(${PROJECT_NAME} SHARED
  # Client source files
  src/${PROJECT_NAME}/cl_template.cpp
  
  # Client behavior source files
  src/${PROJECT_NAME}/client_behaviors/cb_template_behavior.cpp
  
  # Component source files
  src/${PROJECT_NAME}/components/cp_template_component.cpp
)

# Set target properties
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}
  ${yaml-cpp_LIBRARIES}
)

# Export compile commands for IDE integration
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ==============================================================================
# Installation
# ==============================================================================

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Export library
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

# ==============================================================================
# Testing
# ==============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Add unit tests if they exist
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(${PROJECT_NAME}_test
  #   test/test_template_client.cpp
  # )
  # target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

# ==============================================================================
# Documentation
# ==============================================================================
# Optionally build documentation
option(BUILD_DOC "Build documentation" OFF)
if(BUILD_DOC)
  find_package(Doxygen REQUIRED)
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
  
  add_custom_target(doc_${PROJECT_NAME} ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
endif()

ament_package()