cmake_minimum_required(VERSION 3.5)
project(sm_modbus_tcp_relay_test_1)

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
find_package(cl_modbus_tcp_relay REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(include
                    ${smacc2_INCLUDE_DIRS}
                    ${cl_modbus_tcp_relay_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
                src/sm_modbus_tcp_relay_test_1/sm_modbus_tcp_relay_test_1_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${cl_modbus_tcp_relay_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node smacc2 cl_modbus_tcp_relay)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
