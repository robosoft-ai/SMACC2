cmake_minimum_required(VERSION 3.10)
project(sm_cl_px4_mr_test_4)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(cl_px4_mr REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(include
  ${CMAKE_CURRENT_SOURCE_DIR}  # config/*.hpp mission headers
  ${smacc2_INCLUDE_DIRS}
  ${cl_px4_mr_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/sm_cl_px4_mr_test_4/sm_cl_px4_mr_test_4_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${cl_px4_mr_LIBRARIES}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  smacc2
  cl_px4_mr
  px4_msgs
  rclcpp
)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
