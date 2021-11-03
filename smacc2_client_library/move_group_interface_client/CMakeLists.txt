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

set(dependencies
moveit_ros_planning_interface
tf2
tf2_ros
smacc2
)

include_directories(include)

## Declare a C++ library
add_library(${PROJECT_NAME}
            src/move_group_interface_client/cl_movegroup.cpp
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
            src/move_group_interface_client/client_behaviors/cb_move_named_target.cpp
            src/move_group_interface_client/client_behaviors/cb_attach_object.cpp
            src/move_group_interface_client/client_behaviors/cb_move_last_trajectory_initial_state.cpp


ament_target_dependencies(${PROJECT_NAME}
                          ${dependencies})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
