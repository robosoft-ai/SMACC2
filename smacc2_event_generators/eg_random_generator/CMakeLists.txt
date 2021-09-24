cmake_minimum_required(VERSION 3.5)
project(eg_random_generator)

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

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include)

ament_package()
