#ifndef CLASSIFIER_NODE_H
#define CLASSIFIER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
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
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/keypoints/harris_3d.h> 

struct OBBData {
    float width, height, depth;
    pcl::PointXYZ position;
    Eigen::Matrix3f rotation;
};

class Classifier : public rclcpp::Node {
public:
    Classifier();
    bool perform_classification();
    
private:
    // Callbacks
    void cluster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void tf_callback(
        const tf2_ros::TransformStampedFuture &tf_future,
        const geometry_msgs::msg::PoseStamped &pose,
        const rclcpp::Time &stamp,
        const std::string &label,
        const OBBData &obb);
    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg);

    bool tf(
        const geometry_msgs::msg::PoseStamped &pose,
        const rclcpp::Time &stamp,
        const std::string &label,
        const OBBData &obb);
    // Computes oriented bounding box
    OBBData computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    // Function to interpolate filter limits based on distance
    double interpolate(double min_val, double max_val, double dist);
    // Computes average curvature of the point cloud
    double computeCurvature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    size_t sphereSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    size_t planeSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    size_t harrisKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    std::vector<float> computeSliceDiameters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int num_slices);

    // ROS 2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr class_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    double angular_z_;
    double linear_x_;
    double linear_y_;
    // std::shared_ptr<sensor_msgs::msg::PointCloud2> latest_cluster_;
    std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> cluster_queue_;

    // Parameters
    std::string classification;
    std::string cloud_topic_;
    std::string twist_topic_;
    std::string classification_topic_;
    std::string trigger_topic_;
    std::string result_topic_;
    double z_filter_min_;
    double z_filter_max_; 
    double box_filter_min_;
    double box_filter_max_;
    int box_filter_threshold_;
    double animal_filter_min_;
    double animal_filter_max_;
    double sphere_filter_min_;
    double sphere_filter_max_;
    bool visualize_OBB_;
    double ang_vel_threshold_;
    double lin_vel_threshold_;
};

#endif // CLASSIFIER_NODE_H


