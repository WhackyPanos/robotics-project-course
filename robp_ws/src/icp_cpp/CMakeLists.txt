cmake_minimum_required(VERSION 3.28)

project(icp_cpp)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# Include the 'include' directory so the compiler can find header files
include_directories(
  include
)

# Create an executable
add_executable(icp_node src/icp_node.cpp)

# Link dependencies to the executable
ament_target_dependencies(icp_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
)

# Install the executable
install(TARGETS icp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
