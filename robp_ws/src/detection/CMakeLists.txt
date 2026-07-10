cmake_minimum_required(VERSION 3.5)
project(detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE Release)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)

# Create the executables
include_directories(include)
add_executable(clustering_node src/clustering_node.cpp src/clustering.cpp)
add_executable(classifier_node src/classifier_node.cpp src/classifier.cpp)
add_executable(clustering_collection_node src/clustering_collection_node.cpp src/clustering_collection.cpp)

# Link dependencies
ament_target_dependencies(clustering_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  nav_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(classifier_node 
  rclcpp
  std_msgs
  sensor_msgs 
  geometry_msgs
  visualization_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  pcl_conversions
  pcl_ros
)

ament_target_dependencies(clustering_collection_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  nav_msgs
  tf2 
  tf2_ros 
  tf2_geometry_msgs 
  pcl_conversions
  pcl_ros
)

# Install the node
install(TARGETS
  clustering_node
  classifier_node
  clustering_collection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package configuration
ament_package()
