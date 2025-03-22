#ifndef CLUSTERING_NODE_H
#define CLUSTERING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class Clustering : public rclcpp::Node {
public:
    Clustering();
    bool perform_clustering(bool new_req = true);
    
private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void new_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg);

    pcl::PointXYZ computeOBBPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    bool is_occupied(float x, float y);

    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr new_trigger_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double angular_z_;
    nav_msgs::msg::OccupancyGrid latest_map_;
    sensor_msgs::msg::PointCloud2 latest_cloud_;
    bool new_request;

    // Parameters
    std::string cloud_topic_;
    std::string cluster_topic_;
    std::string twist_topic_;
    std::string map_topic_;
    std::string trigger_topic_;
    std::string result_topic_;
    double z_filter_min_;
    double z_filter_max_;
    double y_filter_min_;
    double y_filter_max_;
    double cluster_tolerance_;
    int cluster_min_size_;
    int occupancy_margin_;
    int occupancy_value_;
    double ang_vel_threshold_;
};

#endif // CLUSTERING_NODE_H
