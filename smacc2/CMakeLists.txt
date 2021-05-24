cmake_minimum_required(VERSION 3.5)
project(smacc2)

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
find_package(rclcpp_action REQUIRED)
find_package(smacc_msgs REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(tracetools)
find_package(LTTngUST REQUIRED)

set(dependencies rclcpp 
                 smacc_msgs
                 rclcpp_action
                 tracetools)

include_directories(include ${LTTNGUST_INCLUDE_DIRS})

file(GLOB_RECURSE SRC_FILES src *.cpp) 
add_library(smacc2 SHARED ${SRC_FILES})

ament_target_dependencies(smacc2 ${dependencies})
target_link_libraries(smacc2 
      ${Boost_LIBRARIES} 
      ${LTTNGUST_LIBRARIES}
      )

ament_export_include_directories(include)
ament_export_libraries(smacc2)
ament_export_dependencies(${dependencies})

install(
  DIRECTORY include/
  DESTINATION include
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

install(TARGETS
  smacc2
  DESTINATION lib/)

install(FILES
  scripts/trace.sh
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
