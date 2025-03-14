#ifndef CLUSTERING_NODE_H
#define CLUSTERING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>

class Clustering : public rclcpp::Node {
public:
    Clustering();
    
private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void tf_callback(
        const tf2_ros::TransformStampedFuture &tf_future,
        const geometry_msgs::msg::PointStamped &point,
        const rclcpp::Time &stamp,
        const std::string &label);
    pcl::PointXYZ computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double angular_z_ = 0.0;

    // Parameters
    std::string cloud_topic_;
    std::string cluster_topic_;
    std::string twist_topic_;
    std::string detection_topic_;
    double z_filter_min_;
    double z_filter_max_;
    double y_filter_min_;
    double y_filter_max_;
    double cluster_tolerance_;
    int cluster_min_size_;
    double ang_vel_threshold_;
};

#endif // CLUSTERING_NODE_H
