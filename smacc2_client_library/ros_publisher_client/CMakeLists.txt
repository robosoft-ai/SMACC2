cmake_minimum_required(VERSION 3.5)
project(ros_publisher_client)

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

include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC_FILES "src/*.cpp")

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
)

target_link_libraries(${PROJECT_NAME} ${smacc2_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} smacc2)

ament_export_dependencies(smacc2)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/)

ament_package()
