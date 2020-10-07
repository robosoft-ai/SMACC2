cmake_minimum_required(VERSION 3.5)
project(sm_ferrari)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(MAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(keyboard_client REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eg_conditional_generator REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

include_directories(include
                    ${smacc2_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS}
                    ${smacc_msgs_INCLUDE_DIRS}
                    ${eg_conditional_generator_INCLUDE_DIRS}
                    ${keyboard_client_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE SRC_FILES src *.cpp) 
add_executable(${PROJECT_NAME}_node ${SRC_FILES})
target_link_libraries(
  ${PROJECT_NAME}_node 
  ${smacc2_LIBRARIES} 
  ${Boost_LIBRARIES}
  ${keyboard_client_LIBRARIES}
  ${eg_conditional_generator_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node smacc2)

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
