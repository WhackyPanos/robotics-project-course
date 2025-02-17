#include "clustering.hpp"

using std::placeholders::_1;

Clustering::Clustering() : Node("clustering") {
    // QoS for keeping only the latest message
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

    // Subscribe to point cloud topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/points", qos_profile,
        std::bind(&Clustering::cloud_callback, this, _1));

    // Publisher for filtered point cloud
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/cluster_points", 10);
}

void Clustering::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    // Apply passthrough filtering
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 0.08);
    pass.filter(*cloud);

    // RCLCPP_INFO(this->get_logger(), "After filtering: %zu points", cloud->size());

    // // Downsampling using voxel grid
    // pcl::VoxelGrid<pcl::PointXYZ> voxel;
    // voxel.setInputCloud(cloud);
    // voxel.setLeafSize(0.02f, 0.02f, 0.02f);
    // voxel.filter(*cloud);

    // Perform clustering using Euclidean clustering
    if (cloud->size()>0)
    {
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(0.02);
        ec.setMinClusterSize(100);
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
