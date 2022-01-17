cmake_minimum_required(VERSION 3.5)
project(smacc2_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

set(msg_files
  msg/SmaccSMCommand.msg
  msg/SmaccStatus.msg
  msg/SmaccContainerInitialStatusCmd.msg
  msg/SmaccContainerStructure.msg
  msg/SmaccContainerStatus.msg
  msg/SmaccState.msg
  msg/SmaccTransition.msg
  msg/SmaccEvent.msg
  msg/SmaccOrthogonal.msg
  msg/SmaccStateReactor.msg
  msg/SmaccEventGenerator.msg
  msg/SmaccStateMachine.msg
  msg/SmaccTransitionLogEntry.msg
)

set(srv_files
  srv/SmaccGetTransitionHistory.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces action_msgs std_msgs ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
