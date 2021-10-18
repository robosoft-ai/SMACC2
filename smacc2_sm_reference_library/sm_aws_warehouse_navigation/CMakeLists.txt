cmake_minimum_required(VERSION 3.8)
project(sm_aws_warehouse_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(smacc2 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(nav2z_client REQUIRED)
find_package(bond REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(include
                    ${smacc2_INCLUDE_DIRS}
                    ${nav2z_client_INCLUDE_DIRS}
                    ${bond_INCLUDE_DIRS}
                    ${ament_cmake_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/sm_aws_warehouse_navigation/sm_aws_warehouse_navigation.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${smacc2_LIBRARIES}
  ${Boost_LIBRARIES}
  ${nav2z_client_LIBRARIES}
  ${bond_LIBRARIES}
  ${ament_cmake_LIBRARIES}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/sm_aws_warehouse_navigation.dsv.in")
ament_target_dependencies(${PROJECT_NAME}_node smacc2)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
#install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib                 # static libraries
  LIBRARY DESTINATION lib                 # dynamic libraries
  RUNTIME DESTINATION lib/${PROJECT_NAME} # executables
)

ament_package()
