#include "classifier.h"
#include <tf2_ros/create_timer_ros.h>
#include <sstream>

using std::placeholders::_1;

Classifier::Classifier() : Node("clustering", rclcpp::NodeOptions()
                            .allow_undeclared_parameters(true)
                            .automatically_declare_parameters_from_overrides(true)) 
    {
    // Retrieve parameters with get_parameter_or()
    this->get_parameter_or("cloud_topic", cloud_topic_, std::string("/detection/cluster_points"));
    this->get_parameter_or("twist_topic", twist_topic_, std::string("/cmd_vel"));
    this->get_parameter_or("classification_topic", classification_topic_, std::string("/classification/class"));
    this->get_parameter_or("trigger_topic", trigger_topic_, std::string("/classification/request"));
    this->get_parameter_or("result_topic", result_topic_, std::string("/classification/result"));   
    this->get_parameter_or("box_filter_min", box_filter_min_, 0.0);
    this->get_parameter_or("box_filter_max", box_filter_max_, 0.008);
    this->get_parameter_or("box_filter_threshold", box_filter_threshold_, 50);
    this->get_parameter_or("animal_filter_min", animal_filter_min_, 0.045);
    this->get_parameter_or("animal_filter_max", animal_filter_max_, 0.05);
    this->get_parameter_or("sphere_filter_min", sphere_filter_min_, 0.056);
    this->get_parameter_or("sphere_filter_max", sphere_filter_max_, 0.059);
    this->get_parameter_or("visualize_OBB", visualize_OBB_, false);
    this->get_parameter_or("ang_vel_threshold", ang_vel_threshold_, 0.0);
    this->get_parameter_or("lin_vel_threshold", lin_vel_threshold_, 0.0);
    
    latest_cluster_ = sensor_msgs::msg::PointCloud2();
    latest_cluster_.width = 0;
    latest_cluster_.height = 0;
    latest_cluster_.row_step = 0;
    latest_cluster_.data.clear();  // Ensures data is empty

    angular_z_ = linear_x_ = linear_y_ = 0.0;


    // QoS reliable profile
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Subscribe to cluster point cloud topic
    cluster_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, qos_profile,
        std::bind(&Classifier::cluster_callback, this, _1));

    // Subscriber for /cmd_vel to get the twist message
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_topic_, 10, std::bind(&Classifier::twist_callback, this, _1));

    // Publisher for classification result
    class_pub_ = this->create_publisher<std_msgs::msg::String>(classification_topic_, 10);

    if (visualize_OBB_)
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("obb_marker", 10);
    }

    // Subscribe to trigger topic
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        trigger_topic_, qos_profile, std::bind(&Classifier::trigger_callback, this, _1));

    // Publisher for clustering result
    result_pub_ = this->create_publisher<std_msgs::msg::Bool>(result_topic_, 10);

    // Initialize TF2 buffer, listener, and broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

bool Classifier::perform_classification()
{
    // if (std::abs(angular_z_) >= ang_vel_threshold_ || std::abs(linear_x_) >= lin_vel_threshold_ || std::abs(linear_y_) >= lin_vel_threshold_) {
    //     return false;
    // }

    // RCLCPP_INFO(this->get_logger(), "enter classification");    

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(latest_cluster_, *cloud);

    // Initialize variables for classification
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr animal_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sphere_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    OBBData obb;
    obb = computeOBB(cloud);

    // Filter based on height values
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(box_filter_min_, box_filter_max_);
    pass.filter(*box_filtered);
    // RCLCPP_INFO(this->get_logger(), "Box filtering: %zu", box_filtered->size());

    RCLCPP_INFO(this->get_logger(), "start classification");
    if (box_filtered->size() > box_filter_threshold_) 
    {
        classification = "Box";
    }
    else 
    {
        pass.setFilterLimits(animal_filter_min_, animal_filter_max_);
        pass.filter(*animal_filtered);
        // RCLCPP_INFO(this->get_logger(), "Animal filtering: %zu", animal_filtered->size());
        if(animal_filtered->size()>0)
        {
            classification = "Animal";
        }
        else
        {   
            // std::vector<float> diameters = computeSliceDiameters(cloud, 10);
            // std::cout << "Diameters: ";
            // for (float d : diameters) {
            //     std::cout << d << " ";
            // }
            // std::cout << std::endl;

            // bool sphere = true;
            // for (size_t i = 1; i < diameters.size(); i++) {
            //     if (diameters[i] - diameters[i-1] < 0 && i < diameters.size() / 2) {
            //         sphere = false;
            //         break; 
            //     }
            //     // if (diameters[i] > diameters[i - 1] && i > diameters.size() / 2) {
            //     //     sphere = false;  // Should decrease in second half
            //     // }
            // }

            // Eigen::Vector4f min_pt, max_pt;
            // pcl::getMinMax3D(*cloud, min_pt, max_pt);
            
            // float max_y = min_pt[1]; // Topmost y (highest point)
            // float min_y = max_pt[1]; // Bottommost y (lowest point)
            // std::cout << "max_y: " << max_y << ", min_y: " << min_y << std::endl;

            pass.setFilterLimits(sphere_filter_min_, sphere_filter_max_);
            pass.filter(*sphere_filtered);
            // RCLCPP_INFO(this->get_logger(), "Sphere filtering: %zu", sphere_filtered->size());

            if(sphere_filtered->size()>0)
            {
                classification = "Sphere";

            } 
            else
            {
                classification = "Cube";
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Classified as: %s", classification.c_str());
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = latest_cluster_.header.stamp;
    pose.header.frame_id = latest_cluster_.header.frame_id;

    // Set the position of the OBB (center)
    pose.pose.position.x = obb.position.x;
    pose.pose.position.y = obb.position.y;
    pose.pose.position.z = obb.position.z;

    // Convert the Eigen rotation matrix (3x3) to a quaternion
    Eigen::Quaternionf quat(obb.rotation);  // Eigen provides the conversion directly

    // Set the orientation using the quaternion
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    // Broadcast TF and return result
    std::string label = classification;
    return tf(pose, latest_cluster_.header.stamp, label, obb);

    // tf_buffer_->waitForTransform("map", latest_cluster_.header.frame_id, latest_cluster_.header.stamp, timeout, 
                                // std::bind(&Classifier::tf_callback, this, std::placeholders::_1, pose, latest_cluster_.header.stamp, label, obb));
}

void Classifier::trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    std_msgs::msg::Bool result_msg;
    result_msg.data = false;

    if (msg->data){
        result_msg.data = perform_classification();
    }

    result_pub_->publish(result_msg);
}

void Classifier::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    linear_x_ = msg->linear.x;
    linear_y_ = msg->linear.y;
    angular_z_ = msg->angular.z;
}

void Classifier::cluster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    latest_cluster_ = *msg;
}

bool Classifier::tf(const geometry_msgs::msg::PoseStamped &pose, const rclcpp::Time &stamp, const std::string &label, const OBBData &obb)
{
    // Transform the pose
    geometry_msgs::msg::PoseStamped transformed_pose;

    try {
        tf_buffer_->transform(pose, transformed_pose, "map", tf2::durationFromSec(1.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Transform failed: %s", ex.what());
    }

    // Create a transform message
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = "map";
    transform_msg.child_frame_id = label;

    // Populate transform message with the transformed pose
    transform_msg.transform.translation.x = transformed_pose.pose.position.x;
    transform_msg.transform.translation.y = transformed_pose.pose.position.y;
    transform_msg.transform.translation.z = transformed_pose.pose.position.z;

    transform_msg.transform.rotation.x = transformed_pose.pose.orientation.x;
    transform_msg.transform.rotation.y = transformed_pose.pose.orientation.y;
    transform_msg.transform.rotation.z = transformed_pose.pose.orientation.z;
    transform_msg.transform.rotation.w = transformed_pose.pose.orientation.w;

    std_msgs::msg::String detection_msg;
    std::ostringstream ss;

    // Format position to two decimal places
    ss << label << " " 
       << std::fixed << std::setprecision(2)
       << transformed_pose.pose.position.x << " " 
       << transformed_pose.pose.position.y;

    double roll, pitch, yaw;
    if(label != "Box") 
    {
        yaw = 0.0;
    } 
    else
    {
        // Extract yaw angle from quaternion
        tf2::Quaternion q(
            transformed_pose.pose.orientation.x,
            transformed_pose.pose.orientation.y,
            transformed_pose.pose.orientation.z,
            transformed_pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);  // Extract yaw
        // Convert radians to degrees
        yaw = yaw * (180.0 / M_PI);

        if (visualize_OBB_)
        {
            // Create and publish the marker
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = stamp;
            marker.ns = "obb";
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;        
            marker.pose.position.x = transformed_pose.pose.position.x;
            marker.pose.position.y = transformed_pose.pose.position.y;
            marker.pose.position.z = transformed_pose.pose.position.z;        
            marker.pose.orientation = transformed_pose.pose.orientation;        
            marker.scale.x = obb.width;
            marker.scale.y = obb.height;
            marker.scale.z = obb.depth;        
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;        
            marker.lifetime = rclcpp::Duration::from_seconds(10);        
            marker_pub_->publish(marker);
        }
    }

    // Append yaw value to message
    ss << " " << std::fixed << std::setprecision(0) << yaw;
    
    // Publish the message
    detection_msg.data = ss.str();
    class_pub_->publish(detection_msg);

    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform_msg);

    RCLCPP_INFO(this->get_logger(), "Classification successful");
    
    return true;
}

void Classifier::tf_callback(const tf2_ros::TransformStampedFuture &tf_future, 
                             const geometry_msgs::msg::PoseStamped &pose,
                             const rclcpp::Time &stamp,
                             const std::string &label,
                             const OBBData &obb)
{
    try {
        // Extract the transform
        auto tf_base_map = tf_future.get();

        // Transform the pose
        geometry_msgs::msg::PoseStamped transformed_pose;
        tf2::doTransform(pose, transformed_pose, tf_base_map);

        // Create a transform message
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = label;

        // Populate transform message with the transformed pose
        transform_msg.transform.translation.x = transformed_pose.pose.position.x;
        transform_msg.transform.translation.y = transformed_pose.pose.position.y;
        transform_msg.transform.translation.z = transformed_pose.pose.position.z;

        transform_msg.transform.rotation.x = transformed_pose.pose.orientation.x;
        transform_msg.transform.rotation.y = transformed_pose.pose.orientation.y;
        transform_msg.transform.rotation.z = transformed_pose.pose.orientation.z;
        transform_msg.transform.rotation.w = transformed_pose.pose.orientation.w;

        std_msgs::msg::String detection_msg;
        std::ostringstream ss;

        // Format position to two decimal places
        ss << label << " " 
           << std::fixed << std::setprecision(2)
           << transformed_pose.pose.position.x << " " 
           << transformed_pose.pose.position.y;

        double roll, pitch, yaw;
        if(label != "Box") 
        {
            yaw = 0.0;
        } 
        else
        {
            // Extract yaw angle from quaternion
            tf2::Quaternion q(
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z,
                transformed_pose.pose.orientation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);  // Extract yaw
            // Convert radians to degrees
            yaw = yaw * (180.0 / M_PI);

            if (visualize_OBB_)
            {
                // Create and publish the marker
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = stamp;
                marker.ns = "obb";
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;        
                marker.pose.position.x = transformed_pose.pose.position.x;
                marker.pose.position.y = transformed_pose.pose.position.y;
                marker.pose.position.z = transformed_pose.pose.position.z;        
                marker.pose.orientation = transformed_pose.pose.orientation;        
                marker.scale.x = obb.width;
                marker.scale.y = obb.height;
                marker.scale.z = obb.depth;        
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;        
                marker.lifetime = rclcpp::Duration::from_seconds(10);        
                marker_pub_->publish(marker);
            }
        }

        // Append yaw value to message
        ss << " " << std::fixed << std::setprecision(0) << yaw;
        
        // Publish the message
        detection_msg.data = ss.str();
        class_pub_->publish(detection_msg);

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_msg);
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

OBBData Classifier::computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{   
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZ min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    OBBData obb;
    obb.width = max_point_OBB.x - min_point_OBB.x;
    obb.height = max_point_OBB.y - min_point_OBB.y;
    obb.depth = max_point_OBB.z - min_point_OBB.z;
    obb.position = position_OBB;
    obb.rotation = rotational_matrix_OBB;

    return obb;
}

double Classifier::computeCurvature(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(10);
    normal_estimator.compute(*normals);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> curvature_estimator;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);

    curvature_estimator.setInputCloud(cloud);
    curvature_estimator.setInputNormals(normals);
    curvature_estimator.setSearchMethod(tree);
    curvature_estimator.setKSearch(10);
    curvature_estimator.compute(*curvatures);

    double avg_curvature = 0.0;
    for (const auto& pc : curvatures->points) {
        avg_curvature += (pc.principal_curvature_x + pc.principal_curvature_y + pc.principal_curvature_z) / 3.0;
    }
    return avg_curvature / curvatures->size();
}

size_t Classifier::sphereSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Sphere segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.001); // Tolerance for sphere fitting
    seg.setRadiusLimits(0.019, 0.021); // Radius of 1.5 to 2.5 cm
    seg.setInputCloud(cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.segment(*inliers, *coefficients);

    // If no inliers found, return 0
    if (inliers->indices.empty()) {
        return 0;
    }

    
    RCLCPP_INFO(this->get_logger(), "Circle radius: %f", coefficients->values[2]);

    // Return the number of inliers
    return inliers->indices.size();
}

size_t Classifier::planeSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // Plane segmentation
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // Tolerance for plane fitting
    seg.setInputCloud(cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    seg.segment(*inliers, *coefficients);

    // If no inliers found, return 0
    if (inliers->indices.empty()) {
        return 0;
    }

    // RCLCPP_INFO(this->get_logger(), "Plane inliers: %zu", inliers->indices.size());

    // Return the number of inliers
    return inliers->indices.size();
}

size_t Classifier::harrisKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr upper_half (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 0.07); 
    pass.filter(*upper_half);
    RCLCPP_INFO(this->get_logger(), "Harris filtering: %zu", upper_half->size());

    pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> harris;
    harris.setInputCloud(cloud);
    harris.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS);
    // harris.setThreshold(0.0);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
    harris.compute(*keypoints);
    
    return keypoints->size();
}

std::vector<float> Classifier::computeSliceDiameters(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int num_slices) {
    std::vector<float> diameters(num_slices, 0);
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    float max_y = min_pt[1]; // Topmost y (highest point)
    float min_y = max_pt[1]; // Bottommost y (lowest point)
    std::cout << "max_y: " << max_y << ", min_y: " << min_y << std::endl;
    float slice_height = abs(max_y - min_y) / num_slices;

    for (int i = 0; i < num_slices; i++) {
        float y_max = max_y + i * slice_height; // Start from top
        float y_min = y_max + slice_height;     // Move downward
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud->points) {
            if (point.y < y_min && point.y >= y_max) {
                slice->push_back(point);
            }
        }

        if (!slice->empty()) {
            Eigen::Vector4f slice_min, slice_max;
            pcl::getMinMax3D(*slice, slice_min, slice_max);
            diameters[i] = slice_max[0] - slice_min[0];  // Compute width along x-axis
        }
    }
    return diameters;
}
