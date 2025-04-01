#include "icp.hpp"
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
// Voxel filter implementation using PCL's VoxelGrid filter.
void voxel_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);  // Adjust leaf size to suit your application
    vg.filter(*output);
}
void printNumberOfPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::cout << "PointCloud has " << cloud->points.size() << " points." << std::endl;
}

ICP::ICP() : Node("icp_node", rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
                .automatically_declare_parameters_from_overrides(true)),
                global_cloud_map(new pcl::PointCloud<pcl::PointXYZ>),
                global_cloud_odom(new pcl::PointCloud<pcl::PointXYZ>),
                incoming_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
                first_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
                tf_listener_(tf_buffer_),
                previous_icp_transform_(Eigen::Matrix4f::Identity()),
                last_stamp_(rclcpp::Time(0)) // Initialize with time 0
                
{
    // Retrieve parameters with get_parameter_or()
    this->get_parameter_or("KNN_N_neighbours", KNN_N_neighbours_, 10);
    this->get_parameter_or("std_dev_mul_thresh", std_dev_mul_thresh_, 1.0);
    this->get_parameter_or("max_correspondence_distance", max_correspondence_distance_, 0.1);
    this->get_parameter_or("maximum_iterations", maximum_iterations_, 2000);
    this->get_parameter_or("transformation_epsilon", transformation_epsilon_, 1e-6);
    this->get_parameter_or("euclidean_fitness_epsilon", euclidean_fitness_epsilon_, 0.01);
    this->get_parameter_or("icp_fitness_threshold", icp_fitness_threshold_, 1.0);


    // Subscribe to /scan topic (LaserScan)
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ICP::scan_callback, this, std::placeholders::_1));

    // Publisher for transformed point cloud
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/icp/transformed_point_cloud", 10);

    // Publisher for transformation (used by other nodes)
    transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
        "/icp/transform", 10);

    // Publisher for the global point cloud (for visualization in RViz2)
    global_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/icp/global_point_cloud", 10);
    global_point_cloud_publisher_odom_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/icp/global_point_cloud_odom", 10);

    // Subscription to decide ICP type (normal vs. loop closure)
    icp_type_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/icp/type", 10, std::bind(&ICP::perform_icp, this, std::placeholders::_1));
}

void ICP::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Store timestamp from incoming LaserScan message
    last_stamp_ = msg->header.stamp;
    // RCLCPP_INFO(this->get_logger(), "Received new scan");
    // Convert LaserScan to pcl::PointCloud<pcl::PointXYZ> (assuming z=0)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        float angle = msg->angle_min + i * msg->angle_increment;
        float range = msg->ranges[i];
        if (std::isfinite(range) && range >= msg->range_min && range <= msg->range_max)
        {
            pcl::PointXYZ point;
            point.x = range * cos(angle);
            point.y = range * sin(angle);
            point.z = 0.0;
            cloud->push_back(point);
        }
    }

    //std::cout << "Let us transform the point cloud";

    // Obtain the transform from the lidar frame to the target frame (e.g., 'odom')
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg = tf_buffer_.lookupTransform("odom", "lidar_link", rclcpp::Time(0), rclcpp::Duration(1, 0));
    // Convert the transform to a PCL transformation matrix.
    Eigen::Isometry3d transform = tf2::transformToEigen(transform_msg.transform);
    // Transform the incoming cloud to the target frame (odom).
    pcl::transformPointCloud(*cloud, *cloud, transform.matrix());
    // Store the incoming (transformed) cloud for ICP processing.
    *incoming_cloud_ += *cloud;
    //printNumberOfPoints(cloud);

}


void ICP::perform_icp(const std_msgs::msg::String::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Trying to perform ICP");

    if (incoming_cloud_->empty())
    {
        RCLCPP_WARN(this->get_logger(), "ICP skipped: No incoming cloud.");
        return;
    }

    // For the first frame, initialize the global map and store the first scan for loop closure.
    if (global_cloud_odom->empty())
    {
        *first_cloud_ = *incoming_cloud_;
        *global_cloud_odom = *incoming_cloud_; // first cloud is in odom but odom is equal to map at the beginning
        *global_cloud_map = *incoming_cloud_;
        return;
    }

    // Create msg to be published so we get the correct timestamp
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = last_stamp_;  // Use the timestamp from the latest point cloud
    transform_msg.header.stamp = this->get_clock()->now();

    // Apply Statistical Outlier Removal to incoming and global clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_incoming_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_global_cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK(KNN_N_neighbours_);  // Consider 50 nearest neighbors
    sor.setStddevMulThresh(std_dev_mul_thresh_);  // Remove points that deviate more than 1 std dev
    // Filter incoming cloud, which is in odom frame
    sor.setInputCloud(incoming_cloud_);
    sor.filter(*filtered_incoming_cloud);
    // Filter global cloud -> not sure if it is that necessary
    sor.setInputCloud(global_cloud_odom);
    sor.filter(*filtered_global_cloud_odom);

 
    // Downsample both the incoming scan and the global map to improve ICP performance.
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_incoming(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_global_cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter(filtered_incoming_cloud, downsampled_incoming);
    voxel_filter(filtered_global_cloud_odom, downsampled_global_cloud_odom);

    // Setup ICP using the downsampled incoming scan as source and the global map in odom frame as target.
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
    icp.setInputSource(downsampled_incoming);
    icp.setInputTarget(downsampled_global_cloud_odom);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);  // Adjust for your environment.
    icp.setMaximumIterations(maximum_iterations_);           // Faster convergence.
    icp.setTransformationEpsilon(transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
    
    // Use the previous ICP transform as the initial guess.
    icp.align(*aligned_cloud_odom, previous_icp_transform_);
    double fitness = icp.getFitnessScore();

    if (icp.hasConverged() && fitness < icp_fitness_threshold_)
    {
        RCLCPP_INFO(this->get_logger(), "ICP converged with fitness score: %.4f", fitness);

        // Update the previous ICP transform for the next iteration. Send transform to localization node
        previous_icp_transform_ = icp.getFinalTransformation();
        Eigen::Matrix4f transformation =  previous_icp_transform_;
        transform_msg.transform.translation.x = transformation(0, 3);
        transform_msg.transform.translation.y = transformation(1, 3);
        transform_msg.transform.translation.z = transformation(2, 3);
        Eigen::Quaternionf quat(transformation.block<3,3>(0,0));
        transform_msg.transform.rotation.x = quat.x();
        transform_msg.transform.rotation.y = quat.y();
        transform_msg.transform.rotation.z = quat.z();
        transform_msg.transform.rotation.w = quat.w();
        transform_publisher_->publish(transform_msg);

        // Get transform between map and odom
        geometry_msgs::msg::TransformStamped map_transform_msg;
        try
        {
            map_transform_msg = tf_buffer_.lookupTransform("map", "odom", transform_msg.header.stamp, rclcpp::Duration(1, 0)); //target frame, parent frame
        }
        catch(...)
        {
            map_transform_msg = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero); //target frame, parent frame
        }       
        

        // Update global point cloud in odom frame
        *global_cloud_odom += *aligned_cloud_odom;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_global_cloud_odom(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter(global_cloud_odom, filtered_global_cloud_odom);
        *global_cloud_odom = *filtered_global_cloud_odom;

        // Transform the incoming point cloud to the map frame and add to global point cloud
        Eigen::Isometry3d map_transform = tf2::transformToEigen(map_transform_msg.transform);
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*aligned_cloud_odom, *aligned_cloud_map, map_transform.matrix());
        *global_cloud_map += *aligned_cloud_map;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_global_cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_filter(global_cloud_map, filtered_global_cloud_map);
        *global_cloud_map = *filtered_global_cloud_map;

        // Publish the updated global point cloud (in map frame) for visualization.
        sensor_msgs::msg::PointCloud2 global_msg;
        pcl::toROSMsg(*global_cloud_map, global_msg);
        global_msg.header.frame_id = "map";
        global_msg.header.stamp = transform_msg.header.stamp; //this->get_clock()->now();
        global_point_cloud_publisher_->publish(global_msg);

        // Publish the updated global point cloud (in odom frame) for visualization.
        sensor_msgs::msg::PointCloud2 global_msg_odom;
        pcl::toROSMsg(*global_cloud_odom, global_msg_odom);
        global_msg_odom.header.frame_id = "odom";
        global_msg_odom.header.stamp = transform_msg.header.stamp; //this->get_clock()->now();
        global_point_cloud_publisher_->publish(global_msg_odom);
        printNumberOfPoints(global_cloud_odom);

         incoming_cloud_->clear();

    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "ICP did not converge or fitness score too high Fitness score: %.4f", fitness);
    }

    // Publish the aligned cloud for visualization.
    // sensor_msgs::msg::PointCloud2 output_msg;
    // pcl::toROSMsg(*aligned_cloud, output_msg);
    // output_msg.header.frame_id = "map";
    // output_msg.header.stamp = this->get_clock()->now();
    // publisher_->publish(output_msg);

    // Publish the transformation computed by ICP.
    // transform_msg.header.frame_id = "odom";
    // transform_msg.child_frame_id = "odom";


}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP>());
    rclcpp::shutdown();
    return 0;
}
