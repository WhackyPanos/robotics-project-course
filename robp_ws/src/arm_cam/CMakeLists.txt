cmake_minimum_required(VERSION 3.8)
project(arm_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE Release)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})
add_executable(object_segmentation_node src/object_segmentation_node.cpp src/object_segmentation.cpp)

ament_target_dependencies(object_segmentation_node 
                        rclcpp
                        tf2_ros
                        tf2_geometry_msgs
                        std_msgs
                        geometry_msgs
                        sensor_msgs 
                        cv_bridge 
                        OpenCV)

# Install the node
install(TARGETS
  object_segmentation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
