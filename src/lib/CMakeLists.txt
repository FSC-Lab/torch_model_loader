set(MODEL_LIB_NAME model_loader_lib)

add_library(${MODEL_LIB_NAME} STATIC torch_model_loader.cpp)

target_include_directories(${MODEL_LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${MODEL_LIB_NAME}
  ${TORCH_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(${MODEL_LIB_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# Add dependency on the generated message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${MODEL_LIB_NAME} "${cpp_typesupport_target}")