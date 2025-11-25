cmake_minimum_required(VERSION 3.5)
project(cl_http)

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

file(GLOB_RECURSE SRC_FILES src *.cpp)

add_library(http_session
  src/cl_http/ssl_http_session.cpp
  src/cl_http/http_session.cpp
)

add_library(${PROJECT_NAME}
  src/cl_http/cl_http.cpp
  # Component sources
  src/cl_http/components/cp_http_connection_manager.cpp
  src/cl_http/components/cp_http_session_manager.cpp
  src/cl_http/components/cp_http_request_executor.cpp
)

target_link_libraries(${PROJECT_NAME}
  http_session
  ${smacc2_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME} smacc2)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} http_session)

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS
  ${PROJECT_NAME}
  http_session
  DESTINATION lib/)

ament_package()
