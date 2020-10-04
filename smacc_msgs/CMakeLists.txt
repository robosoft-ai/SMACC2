cmake_minimum_required(VERSION 3.5)
project(smacc_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/SmaccSMCommand.msg"
    "msg/SmaccStatus.msg"
    "msg/SmaccContainerInitialStatusCmd.msg"
    "msg/SmaccContainerStructure.msg"
    "msg/SmaccContainerStatus.msg"
    "msg/SmaccState.msg"
    "msg/SmaccTransition.msg"
    "msg/SmaccEvent.msg"
    "msg/SmaccOrthogonal.msg"
    "msg/SmaccStateReactor.msg"
    "msg/SmaccEventGenerator.msg"
    "msg/SmaccStateMachine.msg"
    "msg/SmaccTransitionLogEntry.msg"
    "srv/SmaccGetTransitionHistory.srv"
    DEPENDENCIES builtin_interfaces std_msgs
 )

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

ament_package()
