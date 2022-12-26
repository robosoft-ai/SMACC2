cmake_minimum_required(VERSION 3.22)
project(panda_arm_2_moveit_config)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
