cmake_minimum_required(VERSION 3.5)
project(sm_husky_barrel_search_1)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(smacc2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2z_client REQUIRED)
find_package(Boost COMPONENTS thread system REQUIRED)
find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_tools REQUIRED)
find_package(rclcpp REQUIRED)

find_package(assimp REQUIRED)

find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(OGRE REQUIRED)

set(dependencies
  smacc2
  std_msgs
  sensor_msgs
  nav2z_client
  visualization_msgs
  image_tools
  rclcpp
  #gazebo_dev
  #gazebo_ros
)

link_directories(${gazebo_dev_LIBRARY_DIRS} /usr/lib/x86_64-linux-gnu/OGRE-1.9.0/)


include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${gazebo_dev_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${CMAKE_BINARY_DIR}/rosidl_generator_cpp)

#---------------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}/sm_husky_barrel_search_1_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node
  boost_thread
  boost_system
  ${Boost_LIBS}
  ${std_msgs_LIBRARIES}
  #${OGRE_LIBRARIES}
  #${gazebo_LIBRARIES}
  #${gazebo_ros_LIBRARIES}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/sm_husky_barrel_search_1.dsv.in")

#---------------------------------------------------------

add_library(controllable_led_plugin SHARED
   src/sm_husky_barrel_search_1/gz_plugins/controllable_led_plugin.cpp
   src/sm_husky_barrel_search_1/gz_plugins/FlashLightPlugin.cc
   src/sm_husky_barrel_search_1/gz_plugins/LedPlugin.cc)

 target_include_directories(controllable_led_plugin PUBLIC include ${gazebo_ros_INCLUDE_DIRS})

 ament_target_dependencies(controllable_led_plugin
   "std_msgs" ${dependencies})

 target_link_libraries(controllable_led_plugin
 #/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libFlashLightPlugin.so
 #/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/libLedPlugin.so
 #FlashLightPlugin
 #LedPlugin
 ${std_msgs_LIBRARIES}
 #${OGRE_LIBRARIES}
 #${gazebo_LIBRARIES}
 #${gazebo_ros_LIBRARIES}

 )

 ament_export_libraries(controllable_led_plugin)
#---------------------------------------------------------
add_executable(opencv_perception_node
  servers/opencv_perception_node/opencv_perception_node.cpp
)
ament_target_dependencies(opencv_perception_node ${dependencies})

target_link_libraries(opencv_perception_node
                        boost_thread
                        boost_system
                        ${Boost_LIBS} ${OpenCV_LIBS})

#---------------------------------------------------------

if(BUILD_TESTING)
endif()

 install(TARGETS
   ${PROJECT_NAME}_node
   opencv_perception_node
   controllable_led_plugin
#   # temperature_sensor_node_${PROJECT_NAME}
#   # led_action_server_node_${PROJECT_NAME}

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
