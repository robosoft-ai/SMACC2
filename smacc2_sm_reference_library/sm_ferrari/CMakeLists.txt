cmake_minimum_required(VERSION 3.5)
project(sm_ferrari)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(MAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(keyboard_client REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eg_conditional_generator REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(
  include
  ${smacc2_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${smacc_msgs_INCLUDE_DIRS}
  ${eg_conditional_generator_INCLUDE_DIRS}
  ${keyboard_client_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(${PROJECT_NAME}_node ${SRC_FILES})

target_link_libraries(
  ${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${Boost_LIBRARIES}
  ${keyboard_client_LIBRARIES}
  ${eg_conditional_generator_LIBRARIES}
)

file(GLOB_RECURSE SRC_FILES2 servers/*.cpp)

add_executable(temperature_sensor_node ${SRC_FILES2})

target_link_libraries(
  temperature_sensor_node
  ${smacc2_LIBRARIES}
  ${Boost_LIBRARIES}
  ${keyboard_client_LIBRARIES}
  ${eg_conditional_generator_LIBRARIES})

ament_target_dependencies(temperature_sensor_node smacc2)

ament_target_dependencies(${PROJECT_NAME}_node smacc2)

ament_package()

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
        temperature_sensor_node
        DESTINATION lib/${PROJECT_NAME})
