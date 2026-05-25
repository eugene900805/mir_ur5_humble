cmake_minimum_required(VERSION 3.8)
project(gazebo_grasp_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_version_helpers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo REQUIRED)

add_subdirectory(msgs)

add_library(gazebo_grasp_fix SHARED
  src/GazeboGraspFix.cpp
  src/GazeboGraspGripper.cpp
)

target_include_directories(gazebo_grasp_fix
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs
)

ament_target_dependencies(gazebo_grasp_fix
  rclcpp
  gazebo_ros
  geometry_msgs
  std_msgs
  gazebo_version_helpers
)

target_link_libraries(gazebo_grasp_fix
  gazebo_grasp_msgs
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS gazebo_grasp_fix gazebo_grasp_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(gazebo_grasp_fix gazebo_grasp_msgs)
ament_export_dependencies(rclcpp gazebo_ros geometry_msgs std_msgs gazebo_version_helpers)

ament_package()
