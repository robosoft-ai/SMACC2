cmake_minimum_required(VERSION 3.8)
project(move_group_interface_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(smacc2 REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

#set(YAML_CPP_LIBRARIES yaml-cpp)

set(dependencies
moveit_ros_planning_interface
tf2
tf2_ros
smacc2
tf2_geometry_msgs
yaml_cpp_vendor
)

include_directories(include)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/move_group_interface_client/cl_movegroup.cpp
  src/move_group_interface_client/common.cpp
  src/move_group_interface_client/components/cp_trajectory_history.cpp
  src/move_group_interface_client/components/cp_grasping_objects.cpp

  src/move_group_interface_client/client_behaviors/cb_execute_last_trajectory.cpp
  src/move_group_interface_client/client_behaviors/cb_move_end_effector_relative.cpp
  src/move_group_interface_client/client_behaviors/cb_move_end_effector.cpp
  src/move_group_interface_client/client_behaviors/cb_circular_pivot_motion.cpp
  src/move_group_interface_client/client_behaviors/cb_move_known_state.cpp
  src/move_group_interface_client/client_behaviors/cb_move_joints.cpp
  src/move_group_interface_client/client_behaviors/cb_pouring_motion.cpp
  src/move_group_interface_client/client_behaviors/cb_detach_object.cpp
  src/move_group_interface_client/client_behaviors/cb_undo_last_trajectory.cpp
  src/move_group_interface_client/client_behaviors/cb_move_end_effector_trajectory.cpp
  src/move_group_interface_client/client_behaviors/cb_end_effector_rotate.cpp
  src/move_group_interface_client/client_behaviors/cb_move_cartesian_relative.cpp
  src/move_group_interface_client/client_behaviors/cb_move_cartesian_relative2.cpp
  src/move_group_interface_client/client_behaviors/cb_move_named_target.cpp
  src/move_group_interface_client/client_behaviors/cb_attach_object.cpp
  src/move_group_interface_client/client_behaviors/cb_move_last_trajectory_initial_state.cpp
)

target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}
                          ${dependencies})

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
