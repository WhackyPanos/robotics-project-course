#ifndef ICP_HPP
#define ICP_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"  // Correct include for LaserScan
#include "geometry_msgs/msg/transform_stamped.hpp"  // Include for TransformStamped
#include "sensor_msgs/msg/point_cloud2.hpp"  // Include for PointCloud2
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/timer.hpp"  // For creating timers

class ICP : public rclcpp::Node
{
public:
    ICP();
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);  // Callback for LaserScan
    void perform_icp();  // ICP algorithm to be called by the timer

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;  // Subscribe to LaserScan
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;  // Publisher for transformed point cloud
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_publisher_;  // Publisher for transformation
    rclcpp::TimerBase::SharedPtr timer_;  // Timer for periodic ICP execution
    pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud_;  // For storing the previous point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud_;  // For storing the incoming point cloud
};

#endif  // ICP_HPP
