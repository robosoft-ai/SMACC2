cmake_minimum_required(VERSION 3.8)
project(cl_moveit2z)

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

set(dependencies
moveit_ros_planning_interface
tf2
tf2_ros
smacc2
tf2_geometry_msgs
yaml_cpp_vendor
)

include_directories(include)

## Declare a C++ library (header-only implementations with compilation anchor)
add_library(${PROJECT_NAME}
  src/cl_moveit2z/cl_moveit2z.cpp  # Compilation anchor stub
)

# All implementations are header-only (inline):
# src/cl_moveit2z/common.cpp  # PHASE 1: NOW HEADER-ONLY (inline stream operators)
# src/cl_moveit2z/components/cp_trajectory_history.cpp  # PHASE 2: NOW HEADER-ONLY (inline methods)
# src/cl_moveit2z/components/cp_grasping_objects.cpp  # PHASE 3: NOW HEADER-ONLY (inline methods)

# ALL CLIENT BEHAVIORS ARE NOW HEADER-ONLY
# src/cl_moveit2z/client_behaviors/cb_move_end_effector.cpp  # HEADER-ONLY
# src/cl_moveit2z/client_behaviors/cb_move_known_state.cpp  # HEADER-ONLY
# src/cl_moveit2z/client_behaviors/cb_move_joints.cpp  # HEADER-ONLY
# src/cl_moveit2z/client_behaviors/cb_undo_last_trajectory.cpp  # HEADER-ONLY
# src/cl_moveit2z/client_behaviors/cb_move_end_effector_trajectory.cpp  # HEADER-ONLY
# src/cl_moveit2z/client_behaviors/cb_end_effector_rotate.cpp  # HEADER-ONLY
# src/cl_moveit2z/client_behaviors/cb_move_cartesian_relative2.cpp  # HEADER-ONLY
# src/cl_moveit2z/client_behaviors/cb_circular_pivot_motion.cpp  # HEADER-ONLY

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
