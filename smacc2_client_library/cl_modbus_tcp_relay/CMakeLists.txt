cmake_minimum_required(VERSION 3.5)
project(cl_modbus_tcp_relay)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smacc2 REQUIRED)
find_package(PkgConfig REQUIRED)

# Find libmodbus - prefer system-installed, fallback to workspace source
pkg_check_modules(MODBUS libmodbus)

if(NOT MODBUS_FOUND)
  # Check if libmodbus source exists in workspace
  set(LIBMODBUS_WORKSPACE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../libmodbus/src")
  if(EXISTS "${LIBMODBUS_WORKSPACE_PATH}/modbus.h")
    message(STATUS "Using workspace libmodbus source at: ${LIBMODBUS_WORKSPACE_PATH}")
    set(MODBUS_INCLUDE_DIRS ${LIBMODBUS_WORKSPACE_PATH})
    # Note: When using workspace source, libmodbus must be built separately
    # For full functionality, install libmodbus-dev: sudo apt install libmodbus-dev
    set(MODBUS_LIBRARIES "modbus")
    set(MODBUS_FOUND TRUE)
  else()
    message(FATAL_ERROR "libmodbus not found. Install with: sudo apt install libmodbus-dev")
  endif()
endif()

set(dependencies
  rclcpp
  smacc2
)

include_directories(
  include
  ${MODBUS_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/cl_modbus_tcp_relay.cpp
  src/${PROJECT_NAME}/components/cp_modbus_connection.cpp
  src/${PROJECT_NAME}/components/cp_modbus_relay.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${MODBUS_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

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
