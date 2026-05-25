find_package(Protobuf REQUIRED)

set(msgs
  grasp_event.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(gazebo_grasp_msgs SHARED ${PROTO_SRCS})
target_link_libraries(gazebo_grasp_msgs ${PROTOBUF_LIBRARY})

install(TARGETS gazebo_grasp_msgs
  EXPORT gazebo_grasp_msgsTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ${PROTO_HDRS}
  DESTINATION include/gazebo_grasp_plugin/msgs
)
