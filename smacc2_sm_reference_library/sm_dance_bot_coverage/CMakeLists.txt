cmake_minimum_required(VERSION 3.5)
project(sm_dance_bot_coverage)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

find_package(smacc2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2z_client REQUIRED)

find_package(ros_publisher_client REQUIRED)
find_package(multirole_sensor_client REQUIRED)

find_package(sr_all_events_go REQUIRED)
find_package(sr_event_countdown REQUIRED)
find_package(sr_conditional REQUIRED)
find_package(ros_timer_client REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(opennav_coverage_msgs REQUIRED)

set(dependencies
  smacc2
  std_msgs
  sensor_msgs
  nav2z_client
  ros_publisher_client
  multirole_sensor_client
  sr_all_events_go
  sr_event_countdown
  sr_conditional
  ros_timer_client
  visualization_msgs
  opennav_coverage_msgs
)

include_directories(include
                    ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}/sm_dance_bot_coverage.cpp
  src/${PROJECT_NAME}/clients/cl_navigation_coverage/cl_navigate_coverage.cpp
  src/${PROJECT_NAME}/clients/cl_navigation_coverage/client_behaviors/cb_navigate_coverage.cpp

  )

ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node ${Boost_LIBS})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/sm_dance_bot_coverage.dsv.in")

if(BUILD_TESTING)
endif()

install(TARGETS
  ${PROJECT_NAME}_node

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
