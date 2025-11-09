cmake_minimum_required(VERSION 3.8)
project(network_loader)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)


# # Force ROS to use system Python
# set(Python3_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH "System Python for ROS 2")
# find_package(Python3 COMPONENTS Interpreter REQUIRED)

# # Point to the Python that has torch (your Conda env)
# set(TORCH_PY "$ENV{HOME}/miniconda3/envs/CCM_hpinn/lib/python3.13" CACHE FILEPATH "Python with torch")

# # Ask that Python where Torch's CMake files live, then append to search path
# execute_process(
#   COMMAND ${TORCH_PY} -c "import torch, sys; sys.stdout.write(torch.utils.cmake_prefix_path)"
#   OUTPUT_VARIABLE TORCH_CMAKE_PREFIX
#   RESULT_VARIABLE _torch_ret
#   ERROR_QUIET
# )
# if(_torch_ret EQUAL 0 AND EXISTS "${TORCH_CMAKE_PREFIX}")
#   list(APPEND CMAKE_PREFIX_PATH "${TORCH_CMAKE_PREFIX}")
# endif()

list(APPEND CMAKE_PREFIX_PATH "/home/fsc-jupiter/miniconda3/envs/CCM_hpinn/lib/python3.13/site-packages/torch/share/cmake")
find_package(Torch REQUIRED)

if (Torch_FOUND)
    message(WARNING "Torch found")
endif()

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
set(_ob_deps
    ament_cmake
    rclcpp
    std_msgs
    geometry_msgs
    tf2_ros
    tf2_eigen
    nav_msgs
    rosidl_default_generators)

# export
ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime)

# export
ament_export_dependencies(${_ob_deps})


# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModelInput.msg"
  "msg/ModelOutput.msg"
  DEPENDENCIES 
  geometry_msgs 
  nav_msgs 
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              rosidl_typesupport_cpp)


# include directory
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# add source files
add_subdirectory(src)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# install launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# install parameter file
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
