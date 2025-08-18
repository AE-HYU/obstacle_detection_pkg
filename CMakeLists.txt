cmake_minimum_required(VERSION 3.8)
project(obstacle_detection_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search segmentation)
find_package(nav2_map_server REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleArray.msg"
  "msg/Obstacle.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(obstacle_detector_node src/obstacle_detector_node.cpp src/obstacle_detector.cpp)

# Dependencies for the executable
ament_target_dependencies(obstacle_detector_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  Eigen3
  nav2_map_server
)

# Link PCL libraries
target_link_libraries(obstacle_detector_node ${PCL_LIBRARIES})

# Link directories for PCL
link_directories(${PCL_LIBRARY_DIRS})

# Link custom messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(obstacle_detector_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()