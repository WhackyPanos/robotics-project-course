#include "clustering.h"

using std::placeholders::_1;

Clustering::Clustering() : Node("clustering", rclcpp::NodeOptions()
                            .allow_undeclared_parameters(true)
                            .automatically_declare_parameters_from_overrides(true)) 
{
    // Retrieve parameters with get_parameter_or()
    this->get_parameter_or("cloud_topic", cloud_topic_, std::string("/camera/camera/depth/color/points"));
    this->get_parameter_or("cluster_topic", cluster_topic_, std::string("/camera/camera/depth/color/cluster_points"));
    this->get_parameter_or("dist_filter_min", z_filter_min_, 0.0);
    this->get_parameter_or("dist_filter_max", z_filter_max_, 1.0);
    this->get_parameter_or("height_filter_min", y_filter_min_, 0.0);
    this->get_parameter_or("height_filter_max", y_filter_max_, 0.075);
    this->get_parameter_or("cluster_tolerance", cluster_tolerance_, 0.05);
    this->get_parameter_or("cluster_min_size", cluster_min_size_, 100);

    // QoS for keeping only the latest message
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

    // Subscribe to point cloud topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos_profile,
        std::bind(&Clustering::cloud_callback, this, _1));

    // Publisher for filtered point cloud
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cluster_topic_, 10);
}

void Clustering::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    // Apply passthrough filtering
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_filter_min_, z_filter_max_);
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_filter_min_, y_filter_max_);
    pass.filter(*cloud);
    // RCLCPP_INFO(this->get_logger(), "After filtering: %zu points", cloud->size());

    // Perform clustering using Euclidean clustering
    if (cloud->size()>0)
    {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(cluster_min_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        RCLCPP_INFO(this->get_logger(), "Clusters found: %zu", cluster_indices.size());

        for (const auto& cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            for (const auto& idx : cluster.indices) {
                cloud_cluster->push_back((*cloud)[idx]);
            }
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            
            sensor_msgs::msg::PointCloud2 output;
            pcl::toROSMsg(*cloud_cluster, output);
            output.header.stamp = msg->header.stamp;
            output.header.frame_id = msg->header.frame_id;
            publisher_->publish(output);
        }
    }
}
