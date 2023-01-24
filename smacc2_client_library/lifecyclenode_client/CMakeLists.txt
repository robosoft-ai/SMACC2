cmake_minimum_required(VERSION 3.5)
project(lifecyclenode_client)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(smacc2 REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  smacc2
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/lifecyclenode_client.cpp
)

ament_target_dependencies(${PROJECT_NAME}
                          ${dependencies})

target_link_libraries(${PROJECT_NAME})

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
