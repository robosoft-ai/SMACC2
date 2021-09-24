cmake_minimum_required(VERSION 3.5)
project(ros_timer_client)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(
  ${PROJECT_NAME}
  include
  ${smacc2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/timer_client.cpp
  src/${PROJECT_NAME}/cb_timer.cpp
  src/${PROJECT_NAME}/cb_timer_countdown_loop.cpp
  src/${PROJECT_NAME}/cb_timer_countdown_once.cpp
)
target_link_libraries(${PROJECT_NAME} SHARED ${Boost_LIBRARIES} smacc2)
ament_target_dependencies(${PROJECT_NAME} smacc2)

ament_export_include_directories(include)
ament_export_dependencies(smacc2 Boost)
ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/)

ament_package()
