#include "icp.hpp"

ICP::ICP() : Node("icp_node"), previous_cloud_(new pcl::PointCloud<pcl::PointXYZ>), incoming_cloud_(new pcl::PointCloud<pcl::PointXYZ>) {
    // Initialize the subscriber to /scan topic (LaserScan)
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ICP::scan_callback, this, std::placeholders::_1));

    // Publisher for transformed point cloud
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_point_cloud", 10);

    // Publisher for transformation (to be used by a Python node)
    transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("icp_transform", 10);

    // Create a timer that triggers every 2 seconds to run ICP
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2), std::bind(&ICP::perform_icp, this));
}

void ICP::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Convert LaserScan to pcl::PointCloud<pcl::PointXYZ> (assuming z=0)
    pcl::PointCloud<pcl::PointXYZ>::Ptr incoming_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert LaserScan message to PointCloud
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float angle = msg->angle_min + i * msg->angle_increment;
        float range = msg->ranges[i];

        // If range is valid (not NaN, inf, or below threshold)
        if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max) {
            pcl::PointXYZ point;
            point.x = range * cos(angle);  // Convert polar (range, angle) to Cartesian (x)
            point.y = range * sin(angle);  // Convert polar (range, angle) to Cartesian (y)
            point.z = 0.0;                 // Z-coordinate is 0 for 2D LaserScan
            incoming_cloud->push_back(point);
        }
    }

    // Store the incoming cloud to process later in ICP
    *incoming_cloud_ = *incoming_cloud;
}

void ICP::perform_icp() {
    // If no cloud has been received yet, just return
    if (incoming_cloud_->empty()) {
        return;
    }

    // If previous cloud is not set yet, set it as the reference
    if (previous_cloud_->empty()) {
        *previous_cloud_ = *incoming_cloud_;
        return;
    }

    // Perform ICP alignment: Use the previous cloud as the reference, and incoming cloud as the target
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    icp.setInputSource(incoming_cloud_);  // Set incoming cloud as source (moving cloud)
    icp.setInputTarget(previous_cloud_);  // Set previous cloud as target (fixed cloud)

    icp.align(*aligned_cloud);  // Perform ICP alignment

    if (icp.hasConverged()) {
        // Get the resulting transformation (rotation and translation)
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        RCLCPP_INFO(this->get_logger(), "ICP Converged with fitness score: %.4f", icp.getFitnessScore());

        // Create TransformStamped message to publish the transformation
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = this->get_clock()->now();
        transform_msg.header.frame_id = "map";  // Choose your frame_id here
        transform_msg.child_frame_id = "odom"; // Choose your child_frame_id here

        // Fill in the transformation matrix
        transform_msg.transform.translation.x = transformation(0, 3);
        transform_msg.transform.translation.y = transformation(1, 3);
        transform_msg.transform.translation.z = transformation(2, 3);

        // Convert rotation matrix to quaternion
        Eigen::Quaternionf quat(transformation.block<3, 3>(0, 0));
        transform_msg.transform.rotation.x = quat.x();
        transform_msg.transform.rotation.y = quat.y();
        transform_msg.transform.rotation.z = quat.z();
        transform_msg.transform.rotation.w = quat.w();

        // Publish the transformation
        transform_publisher_->publish(transform_msg);

        // Update the previous cloud to the current aligned cloud for next iteration
        *previous_cloud_ = *aligned_cloud;
    } else {
        RCLCPP_WARN(this->get_logger(), "ICP did not converge");
    }

    // Convert aligned cloud to PointCloud2 message and publish
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*aligned_cloud, output_msg);
    publisher_->publish(output_msg);
}

// Main function to initialize and run the ROS 2 node
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // Initialize the ROS 2 system
    rclcpp::spin(std::make_shared<ICP>());  // Spin the ICP node
    rclcpp::shutdown();  // Shutdown ROS 2 system
    return 0;  // Return success
}
