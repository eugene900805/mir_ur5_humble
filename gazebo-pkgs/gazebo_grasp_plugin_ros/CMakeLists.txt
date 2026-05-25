cmake_minimum_required(VERSION 3.8)
project(gazebo_grasp_plugin_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_grasp_plugin REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo REQUIRED)
find_package(std_msgs REQUIRED)

################################################
## Messages
################################################

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GazeboGraspEvent.msg"
  DEPENDENCIES std_msgs
)

###########
## Build ##
###########

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_executable(${PROJECT_NAME}_republisher_node src/grasp_event_republisher.cpp)
set_target_properties(${PROJECT_NAME}_republisher_node PROPERTIES OUTPUT_NAME grasp_event_republisher PREFIX "")

ament_target_dependencies(${PROJECT_NAME}_republisher_node
  rclcpp
  gazebo_grasp_plugin
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_republisher_node
  "${cpp_typesupport_target}"
  ${GAZEBO_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
