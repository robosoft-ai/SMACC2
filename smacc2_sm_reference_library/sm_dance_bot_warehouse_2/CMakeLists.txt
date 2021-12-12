cmake_minimum_required(VERSION 3.5)
project(sm_dance_bot_warehouse_2)

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
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "servers/led_action_server/action/LEDControl.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs sensor_msgs
)

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${CMAKE_BINARY_DIR}/rosidl_generator_cpp)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}/sm_dance_bot_warehouse_2.cpp
  src/${PROJECT_NAME}/clients/cl_led/cl_led.cpp
)

add_executable(temperature_sensor_node_${PROJECT_NAME} servers/temperature_sensor_node/src/temperature_sensor_node.cpp)
add_executable(led_action_server_node_${PROJECT_NAME} servers/led_action_server/src/led_action_server_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node ${Boost_LIBS} ${PROJECT_NAME}__rosidl_typesupport_cpp)

ament_target_dependencies(led_action_server_node_${PROJECT_NAME} ${dependencies})
set_target_properties(led_action_server_node_${PROJECT_NAME} PROPERTIES OUTPUT_NAME "led_action_server_node")
target_link_libraries(led_action_server_node_${PROJECT_NAME} ${Boost_LIBS} ${PROJECT_NAME}__rosidl_typesupport_cpp)

ament_target_dependencies(temperature_sensor_node_${PROJECT_NAME} ${dependencies})
set_target_properties(temperature_sensor_node_${PROJECT_NAME} PROPERTIES OUTPUT_NAME "temperature_sensor_node")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/sm_dance_bot_warehouse_2.dsv.in")

if(BUILD_TESTING)
endif()

install(TARGETS
  ${PROJECT_NAME}_node
  temperature_sensor_node_${PROJECT_NAME}
  led_action_server_node_${PROJECT_NAME}

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#ament_python_install_package(${PROJECT_NAME}
#  PACKAGE_DIR servers/service_node_3
#  )


install(FILES
        servers/service_node_3/service_node_3.py
        DESTINATION
        lib/${PROJECT_NAME}
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
                    GROUP_EXECUTE GROUP_READ)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

ament_package()
