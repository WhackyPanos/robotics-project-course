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
    this->get_parameter_or("map_topic", map_topic_, std::string("/occupancy_grid"));
    this->get_parameter_or("trigger_topic", trigger_topic_, std::string("/detection/request"));
    this->get_parameter_or("result_topic", result_topic_, std::string("/detection/result"));    
    this->get_parameter_or("dist_filter_min", z_filter_min_, 0.0);
    this->get_parameter_or("dist_filter_max", z_filter_max_, 1.0);
    this->get_parameter_or("height_filter_min", y_filter_min_, -0.025);
    this->get_parameter_or("height_filter_max", y_filter_max_, 0.075);
    this->get_parameter_or("cluster_tolerance", cluster_tolerance_, 0.05);
    this->get_parameter_or("cluster_min_size", cluster_min_size_, 100);
    this->get_parameter_or("occupancy_margin", occupancy_margin_, 0);
    this->get_parameter_or("occupancy_value", occupancy_value_, 0);
    this->get_parameter_or("ang_vel_threshold", ang_vel_threshold_, 0.2);

    latest_cloud_ = sensor_msgs::msg::PointCloud2();
    latest_cloud_.width = 0;
    latest_cloud_.height = 0;
    latest_cloud_.row_step = 0;
    latest_cloud_.data.clear();  // Ensures data is empty

    latest_map_ = nav_msgs::msg::OccupancyGrid();
    latest_map_.info.width = 0;
    latest_map_.info.height = 0;
    latest_map_.data.clear();  // Ensures data is empty

    new_request = true;
    angular_z_ = 0.0;


    // QoS for keeping only the latest message
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

    // Subscribe to point cloud topic
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos_profile,
        std::bind(&Clustering::cloud_callback, this, _1));

    // Subscriber for /cmd_vel to get the twist message
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_topic_, qos_profile, std::bind(&Clustering::twist_callback, this, _1));
    
    // Subscriber to occupancy grid
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, qos_profile, std::bind(&Clustering::map_callback, this, _1));

    // Publisher for filtered point cloud
    cluster_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        cluster_topic_, 10);

    // Publisher for stopping robot
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        twist_topic_, 10);

    // Subscribe to trigger topic
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        trigger_topic_, qos_profile, std::bind(&Clustering::trigger_callback, this, _1));

    // Subscribe to trigger topic
    new_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/detection/new_request", qos_profile, std::bind(&Clustering::new_trigger_callback, this, _1));

    // Publisher for clustering result
    result_pub_ = this->create_publisher<std_msgs::msg::Bool>(result_topic_, 10);

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool Clustering::perform_clustering(bool new_req)
{
    // RCLCPP_INFO(this->get_logger(), "Enter clustering");
    // if (std::abs(angular_z_) >= ang_vel_threshold_) {
    //     return false;
    // }

    RCLCPP_INFO(this->get_logger(), "Pass Filter");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(latest_cloud_, *cloud);


    // Apply passthrough filtering
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_filter_min_, z_filter_max_);
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_filter_min_, y_filter_max_);
    pass.filter(*cloud);

    if (cloud->empty()) return false;
    

    RCLCPP_INFO(this->get_logger(), "Clustering");


    // Clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(cluster_min_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // If no cluster found
    if (cluster_indices.empty()) return false;

    // For each cluster 
    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(cloud_cluster);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_filter_min_, -0.02);
        pass.filter(*obstacle);
        if (!obstacle->empty()) continue;

        RCLCPP_INFO(this->get_logger(), "Cluster found");
    // If new, check also for obstacle or occupation in grid
        if(new_req)
        {
            RCLCPP_INFO(this->get_logger(), "Enter new req");

            pcl::PointXYZ centre = computeOBBPosition(cloud_cluster);

            geometry_msgs::msg::PointStamped centre_base, centre_map;
            centre_base.header.frame_id = latest_cloud_.header.frame_id;
            centre_base.header.stamp = latest_cloud_.header.stamp;
            centre_base.point.x = centre.x;
            centre_base.point.y = centre.y;
            centre_base.point.z = centre.z;

            try {
                tf_buffer_->transform(centre_base, centre_map, "map", tf2::durationFromSec(1.0));
            } catch (tf2::TransformException &ex) {
                RCLCPP_INFO(this->get_logger(), "Transform failed: %s", ex.what());
                continue;
            }
            RCLCPP_INFO(this->get_logger(), "Check occupation");
            if (is_occupied(centre_map.point.x, centre_map.point.y)) continue;
        }
    
    //Publish found cluster and stop the robot
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_cluster, output);
        output.header.stamp = latest_cloud_.header.stamp;
        output.header.frame_id = latest_cloud_.header.frame_id;
        cluster_pub_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Publish cluster");

        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = stop_msg.linear.y = stop_msg.linear.z = 0.0;
        stop_msg.angular.x = stop_msg.angular.y = stop_msg.angular.z = 0.0;
        twist_pub_->publish(stop_msg);

        return true;  // At least one cluster was found and published
    }

    return false;
}

void Clustering::trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    std_msgs::msg::Bool result_msg;
    result_msg.data = false;

    if (msg->data){
            result_msg.data = perform_clustering(new_request);
    }

    result_pub_->publish(result_msg);
}

void Clustering::new_trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    new_request = msg->data;
}

void Clustering::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    angular_z_ = msg->angular.z;
}

void Clustering::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_map_ = *msg;
}

void Clustering::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    latest_cloud_ = *msg;
}

pcl::PointXYZ Clustering::computeOBBPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    return position_OBB;
}

bool Clustering::is_occupied(float x, float y)
{
    // if(latest_cloud_.data.empty()) return false;

    int mx = static_cast<int>((x - latest_map_.info.origin.position.x) / latest_map_.info.resolution);
    int my = static_cast<int>((y - latest_map_.info.origin.position.y) / latest_map_.info.resolution);
    int width = latest_map_.info.width;
    RCLCPP_INFO(this->get_logger(), "Check Occupation at (%d, %d).", mx,my);
    
    for (int dx = -occupancy_margin_; dx <= occupancy_margin_; ++dx) {
        for (int dy = -occupancy_margin_; dy <= occupancy_margin_; ++dy) {
            int index = (my + dy) * width + (mx + dx);
            if (latest_map_.data[index] >= occupancy_value_ ) {
                return true;
            }
        }
    }
    return false;
}
