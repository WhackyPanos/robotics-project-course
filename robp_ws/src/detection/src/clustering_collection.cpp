#include "clustering_collection.h"
#include <tf2_ros/create_timer_ros.h>
#include <sstream>

using std::placeholders::_1;

ClusteringCollection::ClusteringCollection() : Node("clustering_collection", rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) 
{
    // Retrieve parameters with get_parameter_or()
    this->get_parameter_or("cloud_topic", cloud_topic_, std::string("/camera/camera/depth/color/points"));
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
    this->get_parameter_or("clustering_runs", required_clustering_runs, 10);

    new_request_ = true;
    goal_type = "Object";


    // QoS for keeping only the latest message
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

    // Subscribe to point cloud topic
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos_profile,
        std::bind(&ClusteringCollection::cloud_callback, this, _1));
    
    // Subscriber to occupancy grid
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, qos_profile , std::bind(&ClusteringCollection::map_callback, this, _1));

    // Publisher for updated goal point
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/goal_point", rclcpp::QoS(1).reliable());

    // Subscribe to goal point and type
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", rclcpp::QoS(1).reliable(), std::bind(&ClusteringCollection::goal_point_callback, this, _1));

    goal_type_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/goal_type", rclcpp::QoS(1).reliable(), std::bind(&ClusteringCollection::goal_type_callback, this, _1));

    // Publisher for stopping robot
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        twist_topic_, 10);

    // Subscribe to trigger topic
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        trigger_topic_, rclcpp::QoS(1).reliable(), std::bind(&ClusteringCollection::trigger_callback, this, _1));

    // Publisher for clustering result
    result_pub_ = this->create_publisher<std_msgs::msg::Bool>(result_topic_, rclcpp::QoS(1).reliable());

    // Publisher for retry
    retry_pub_ = this->create_publisher<std_msgs::msg::Bool>("/detection/retry", rclcpp::QoS(1).reliable());

    // Initialize TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool ClusteringCollection::perform_clustering(bool new_req)
{
    bool cluster_found = false;
    std::vector<geometry_msgs::msg::PointStamped> centers;

    if (!latest_cloud_ || !goal_point) {
        RCLCPP_WARN(this->get_logger(), "No cloud or goal point received yet.");
        return false;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*latest_cloud_, *cloud);

    // Apply passthrough filtering
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_filter_min_, z_filter_max_);
    pass.filter(*cloud);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_filter_min_, y_filter_max_);
    pass.filter(*cloud);

    if (cloud->empty()) 
    {
        RCLCPP_INFO(this->get_logger(), "Cloud empty after pass filter");
        return false;
    }
    
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
    if (cluster_indices.empty()) 
    {
        RCLCPP_INFO(this->get_logger(), "No cluster found");
        return false;
    }

    // For each cluster 
    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : cluster.indices) {
            cloud_cluster->push_back((*cloud)[idx]);
        }

        // Check for obstacle or occupation in grid
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(cloud_cluster);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_filter_min_, -0.02);
        pass.filter(*obstacle);
        if (!obstacle->empty()) 
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle clustered");
            continue;
        }   

        pcl::PointCloud<pcl::PointXYZ>::Ptr box(new pcl::PointCloud<pcl::PointXYZ>);
        pass.setInputCloud(cloud_cluster);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_filter_min_, 0.0);
        pass.filter(*box);
        
        if ((goal_type == "Object" && !box->empty()) || (goal_type == "Box" && box->empty())) {
            RCLCPP_INFO(this->get_logger(), "Type does not match");
            continue;
        }

        pcl::PointXYZ centre = computeOBBPosition(cloud_cluster);

        geometry_msgs::msg::PointStamped centre_base, centre_map;
        centre_base.header.frame_id = latest_cloud_->header.frame_id;
        centre_base.header.stamp = latest_cloud_->header.stamp;
        centre_base.point.x = centre.x;
        centre_base.point.y = centre.y;
        centre_base.point.z = centre.z;

        try {
            tf_buffer_->transform(centre_base, centre_map, "map", tf2::durationFromSec(1.0));
        } catch (tf2::TransformException &ex) {
            RCLCPP_INFO(this->get_logger(), "Transform failed: %s", ex.what());
            continue;
        }

        if (is_occupied(centre_map.point.x, centre_map.point.y)) continue;

        if(!new_req)
        {
            centers.push_back(centre_map);
        }

        cluster_found = true;  // At least one cluster was found
    }

    if(cluster_found) {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = stop_msg.linear.y = stop_msg.linear.z = 0.0;
        stop_msg.angular.x = stop_msg.angular.y = stop_msg.angular.z = 0.0;
        twist_pub_->publish(stop_msg);

        if (!centers.empty()){
            double min_dist = std::numeric_limits<double>::max();
            geometry_msgs::msg::PointStamped closest_point;

            for (auto& p : centers) {
                double dx = p.point.x - goal_point->point.x;
                double dy = p.point.y - goal_point->point.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist < min_dist) {
                    min_dist = dist;
                    closest_point = p;
                }
            }

            all_centers.push_back(closest_point);

            // goal_pub_->publish(closest_point);
            // RCLCPP_INFO(this->get_logger(), "Set new goal (%f, %f) -> (%f, %f)", goal_point->point.x, goal_point->point.y, closest_point.point.x, closest_point.point.y);
        }
    }

    return cluster_found;
}

void ClusteringCollection::trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    RCLCPP_INFO(this->get_logger(), "Received detection request");
    std_msgs::msg::Bool result_msg;
    
    if (!msg->data) clustering_runs++;
    result_msg.data = perform_clustering(msg->data);
    
    if (clustering_runs >= required_clustering_runs) {
        if (!all_centers.empty()) {
            double sum_x = 0.0, sum_y = 0.0;

            for (const auto& p : all_centers) {
                sum_x += p.point.x;
                sum_y += p.point.y;
            }

            geometry_msgs::msg::PointStamped averaged_point;
            averaged_point.header = all_centers.front().header;
            averaged_point.point.x = sum_x / all_centers.size();
            averaged_point.point.y = sum_y / all_centers.size();
            averaged_point.point.z = 0.0;

            goal_pub_->publish(averaged_point);
            RCLCPP_INFO(this->get_logger(), "Set new goal (%f, %f) -> (%f, %f)", goal_point->point.x, goal_point->point.y, averaged_point.point.x, averaged_point.point.y);
        }
        else
        {
            std_msgs::msg::Bool retry_msg;
            retry_msg.data = true;
            retry_pub_->publish(retry_msg);
        }
        // Reset accumulation
        all_centers.clear();
        clustering_runs = 0;
    }

    result_pub_->publish(result_msg);
}

void ClusteringCollection::goal_type_callback(const std_msgs::msg::String::SharedPtr msg)
{
    goal_type = msg->data;
}

void ClusteringCollection::goal_point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    goal_point = msg;
}

void ClusteringCollection::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) 
{
    latest_map_ = msg;
}

void ClusteringCollection::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    latest_cloud_ = msg;
}

pcl::PointXYZ ClusteringCollection::computeOBBPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    return position_OBB;
}

bool ClusteringCollection::is_occupied(float x, float y)
{
    if (!latest_map_) {
        RCLCPP_WARN(this->get_logger(), "No map received yet.");
        return false;
    }

    int width = latest_map_->info.width;
    int height = latest_map_->info.height;
    int mx = static_cast<int>((x - latest_map_->info.origin.position.x) / latest_map_->info.resolution);
    int my = static_cast<int>((y - latest_map_->info.origin.position.y) / latest_map_->info.resolution);

    for (int dx = -occupancy_margin_; dx <= occupancy_margin_; ++dx) {
        for (int dy = -occupancy_margin_; dy <= occupancy_margin_; ++dy) {
            int nx = mx + dx;
            int ny = my + dy;

            // Prevent out-of-bounds access
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                return true;
            }

            int index = ny * width + nx;
            RCLCPP_INFO(this->get_logger(), "Occupation at (%d, %d) = %d.", nx, ny, latest_map_->data[index]);

            // No obstacle
            if (latest_map_->data[index] < 100 && latest_map_->data[index] > 50) { 
                return true;
            }
        }
    }
    return false;
}
