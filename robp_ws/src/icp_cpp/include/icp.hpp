#ifndef ICP_HPP
#define ICP_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // LaserScan message
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"  // TransformStamped message
#include "sensor_msgs/msg/point_cloud2.hpp"         // PointCloud2 message
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // For PCL compatibility
#include <Eigen/Dense>
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>
#include <builtin_interfaces/msg/time.hpp>



class ICP : public rclcpp::Node
{
public:
    ICP();
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);  // Callback for LaserScan
    void perform_icp(const std_msgs::msg::String::SharedPtr msg);          // ICP algorithm triggered by topic message

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;  // Subscribe to LaserScan
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr icp_type_subscription_; // Subscribe to ICP type messages

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;      // Publisher for transformed point cloud
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_publisher_;  // Publisher for transformation
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_point_cloud_publisher_;  // Publisher for global map
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_point_cloud_publisher_odom_;  // Publisher for global map

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_map;  // Global map in map (updated during ICP)
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_odom;  // Global map in odom (updated during ICP)
    pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud_;  // Incoming scan point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud_;     // First point cloud for loop closure
    rclcpp::Time last_stamp_;  // Store the timestamp of the latest point cloud
    

    // tf2 Buffer and Listener for transforming point clouds
    tf2_ros::Buffer tf_buffer_{this->get_clock()};
    tf2_ros::TransformListener tf_listener_;

    // Store the previous ICP transform as the initial guess for the next iteration.
    Eigen::Matrix4f previous_icp_transform_;

        // Parameters

    int KNN_N_neighbours_;
    double std_dev_mul_thresh_;
    double max_correspondence_distance_;
    int maximum_iterations_;
    double transformation_epsilon_;
    double euclidean_fitness_epsilon_;
    double icp_fitness_threshold_;
};

#endif  // ICP_HPP