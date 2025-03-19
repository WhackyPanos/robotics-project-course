cmake_minimum_required(VERSION 3.28)

project(icp_cpp)

# ROS 2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_eigen REQUIRED)


# PCL package
find_package(PCL REQUIRED)

# Include the 'include' directory so the compiler can find header files
include_directories(
  include
  ${PCL_INCLUDE_DIRS}  # Include PCL headers
)
include_directories(/opt/ros/jazzy/include/tf2_geometry_msgs)
include_directories(/opt/ros/jazzy/include/tf2)
include_directories(/opt/ros/jazzy/include/tf2_ros)
include_directories(/opt/ros/jazzy/include/tf2_eigen)
include_directories(${tf2_eigen_INCLUDE_DIRS})

# Create an executable
add_executable(icp_node src/icp_node.cpp)

# Link dependencies to the executable
ament_target_dependencies(icp_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  tf2_eigen
)

# Link PCL libraries
target_link_libraries(icp_node
  ${PCL_LIBRARIES}  # Link PCL libraries
)

# Install the executable
install(TARGETS icp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()