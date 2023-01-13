cmake_minimum_required(VERSION 3.5)
project(sm_atomic_lifecycle)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost COMPONENTS thread REQUIRED)

include_directories(include)

ament_auto_add_executable(${PROJECT_NAME}_node
                          src/sm_atomic_lifecycle/sm_atomic_lifecycle_node.cpp)

ament_auto_add_executable(lifecycle_example_node
                      src/lifecycle_examplenode/lifecycle_example_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${Boost_LIBRARIES}
)

#ament_target_dependencies(${PROJECT_NAME}_node smacc2)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
        ${PROJECT_NAME}_node
        lifecycle_example_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
