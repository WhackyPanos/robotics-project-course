#ifndef DETECTION_NODE_HPP
#define DETECTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>

struct OBBData {
    float width, height, depth;
    pcl::PointXYZRGB position;
    Eigen::Matrix3f rotation;
};

class Classifier : public rclcpp::Node {
public:
    Classifier();
    
private:
    // Callbacks
    void cluster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void tf_callback(
        const tf2_ros::TransformStampedFuture &tf_future,
        const geometry_msgs::msg::PoseStamped &pose,
        const rclcpp::Time &stamp,
        const std::string &label);
    
    // Computes oriented bounding box
    OBBData computeOBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    // Computes average curvature of the point cloud
    double computeCurvature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

#endif // DETECTION_NODE_HPP
