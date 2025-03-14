#include "clustering.h"
#include <tf2_ros/create_timer_ros.h>
#include <sstream>

using std::placeholders::_1;

Clustering::Clustering() : Node("clustering", rclcpp::NodeOptions()
                            .allow_undeclared_parameters(true)
                            .automatically_declare_parameters_from_overrides(true)) 
{
    // Retrieve parameters with get_parameter_or()
    this->get_parameter_or("cloud_topic", cloud_topic_, std::string("/camera/camera/depth/color/points"));
    this->get_parameter_or("cluster_topic", cluster_topic_, std::string("/detection/cluster_points"));
    this->get_parameter_or("twist_topic", twist_topic_, std::string("/cmd_vel"));
    this->get_parameter_or("detection_topic", detection_topic_, std::string("/detection/object"));
    this->get_parameter_or("dist_filter_min", z_filter_min_, 0.0);
    this->get_parameter_or("dist_filter_max", z_filter_max_, 1.0);
    this->get_parameter_or("height_filter_min", y_filter_min_, -0.025);
    this->get_parameter_or("height_filter_max", y_filter_max_, 0.075);
    this->get_parameter_or("cluster_tolerance", cluster_tolerance_, 0.05);
    this->get_parameter_or("cluster_min_size", cluster_min_size_, 100);
    this->get_parameter_or("ang_vel_threshold", ang_vel_threshold_, 0.3);

    // QoS for keeping only the latest message
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

    // Subscribe to point cloud topic
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos_profile,
        std::bind(&Clustering::cloud_callback, this, _1));

    // Subscriber for /cmd_vel to get the twist message
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_topic_, 10, std::bind(&Clustering::twist_callback, this, _1));

    // Publisher for filtered point cloud
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cluster_topic_, 10);

    // Publisher for detection position
    detect_pub_ = this->create_publisher<std_msgs::msg::String>(detection_topic_, 10);

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void Clustering::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    angular_z_ = msg->angular.z;
}

void Clustering::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    // Only cluster if robot is not rotating 
    if (std::abs(angular_z_) < ang_vel_threshold_)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Apply passthrough filtering
        pcl::PassThrough<pcl::PointXYZ> pass;
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
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(cluster_tolerance_);
            ec.setMinClusterSize(cluster_min_size_);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);
            RCLCPP_INFO(this->get_logger(), "Clusters found: %zu", cluster_indices.size());

            for (const auto& cluster : cluster_indices)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& idx : cluster.indices) {
                    cloud_cluster->push_back((*cloud)[idx]);
                }

                // Only publish cluster if no obstacle
                pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle (new pcl::PointCloud<pcl::PointXYZ>);
                pass.setInputCloud(cloud_cluster);
                pass.setFilterFieldName("y");
                pass.setFilterLimits(y_filter_min_, -0.02);
                pass.filter(*obstacle);
                
                if (obstacle->empty())
                {
                    pcl::PointXYZ centre;
                    centre = computeOBB(cloud_cluster);

                    geometry_msgs::msg::PointStamped point;
                    point.header.stamp = msg->header.stamp;
                    point.header.frame_id = msg->header.frame_id;

                    // Set the position of the OBB (center)
                    point.point.x = centre.x;
                    point.point.y = centre.y;
                    point.point.z = 0.0;

                    // Broadcast TF
                    std::string label = "object";
                    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);
                    tf_buffer_->waitForTransform("map", msg->header.frame_id, msg->header.stamp, timeout, 
                                                std::bind(&Clustering::tf_callback, this, std::placeholders::_1, point, msg->header.stamp, label));

                    
                    cloud_cluster->width = cloud_cluster->size();
                    cloud_cluster->height = 1;
                    cloud_cluster->is_dense = true;

                    sensor_msgs::msg::PointCloud2 output;
                    pcl::toROSMsg(*cloud_cluster, output);
                    output.header.stamp = msg->header.stamp;
                    output.header.frame_id = msg->header.frame_id;
                    cluster_pub_->publish(output);

                }
            }
        }
    }
}

void Clustering::tf_callback(const tf2_ros::TransformStampedFuture &tf_future, 
                            const geometry_msgs::msg::PointStamped &point, 
                            const rclcpp::Time &stamp, 
                            const std::string &label)
{
    try {
        // Extract the transform
        auto tf_base_map = tf_future.get();

        // Transform the pose
        geometry_msgs::msg::PointStamped transformed_point;
        tf2::doTransform(point, transformed_point, tf_base_map);

        std_msgs::msg::String detection_msg;
        std::ostringstream ss;

        // Format position to two decimal places
        ss << label << " " 
           << std::fixed << std::setprecision(2)
           << transformed_point.point.x << " " 
           << transformed_point.point.y;
        
        // Publish the message
        detection_msg.data = ss.str();
        detect_pub_->publish(detection_msg);

    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

pcl::PointXYZ Clustering::computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    return position_OBB;
}