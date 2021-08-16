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
find_package(action_msgs REQUIRED)
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
    DEPENDENCIES builtin_interfaces std_msgs action_msgs
 )

if(BUILD_TESTING)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
