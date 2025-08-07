add_subdirectory(lib)

add_executable(minimal_publisher_node minimimal_publisher_node.cpp)
ament_target_dependencies(minimal_publisher_node rclcpp geometry_msgs)

add_executable(model_loader_node model_loader_node.cpp)
target_link_libraries(model_loader_node model_loader_lib)

ament_target_dependencies(model_loader_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(model_loader_node
  ${rclcpp_LIBRARIES}
  ${rosidl_default_runtime_LIBRARIES}
  ${TORCH_LIBRARIES}
  model_loader_lib
)

install(TARGETS minimal_publisher_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS model_loader_node DESTINATION lib/${PROJECT_NAME})