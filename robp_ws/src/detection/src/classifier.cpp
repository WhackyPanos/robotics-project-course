#include "classifier.hpp"
#include <tf2_ros/create_timer_ros.h>

using std::placeholders::_1;

Classifier::Classifier() : Node("classifier") {
    // QoS reliable profile
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Subscribe to cluster point cloud topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/camera/depth/color/cluster_points", qos_profile,
        std::bind(&Classifier::cluster_callback, this, _1));

    // Publisher for classification result
    publisher_ = this->create_publisher<std_msgs::msg::String>("/detection/class", 10);

    // Initialize TF2 buffer, listener, and broadcaster
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void Classifier::cluster_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    // Initialize variables for classification
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr animal_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cube_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    std::string classification = "Unknown"; // Default classification

    OBBData obb;
    obb = computeOBB(cloud);
    std::cout << "OBB Dimensions: " << obb.width << " x " << obb.height << " x " << obb.depth << std::endl;
    std::cout << "OBB Position: (" << obb.position.x << ", " << obb.position.y << ", " << obb.position.z << ")\n";
    std::cout << "OBB Rotation Matrix:\n" << obb.rotation << std::endl;

    // Filter based on height values
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 0.01);  //9-10cm
    pass.filter(*box_filtered);
    RCLCPP_INFO(this->get_logger(), "Box filtering: %zu", box_filtered->size());

    if (box_filtered->size() > 200) 
    {
        classification = "Box";
    }
    else 
    {
        pass.setFilterLimits(0.045, 0.055);  //4.5-5.5cm
        pass.filter(*animal_filtered);
        RCLCPP_INFO(this->get_logger(), "Animal filtering: %zu", animal_filtered->size());
        if(animal_filtered->size()>0)
        {
            classification = "Animal";
        }
        else
        {
            // pass.setFilterLimits(0.06, 0.08);  //2-4cm
            // pass.filter(*sphere_filtered);
            // RCLCPP_INFO(this->get_logger(), "Sphere filtering: %zu", sphere_filtered->size());
            float size_diff = std::max({obb.width, obb.height, obb.depth}) - std::min({obb.width, obb.height, obb.depth});
            bool is_near_cube = (size_diff < 0.05 * std::max({obb.width, obb.height, obb.depth}));

            double avg_curvature = computeCurvature(cloud);
            std::cout << "Average Curvature: " << avg_curvature << std::endl;

            // Distinguish between cube and sphere
            if (is_near_cube && avg_curvature < 0.1) {
                classification = "Cube";
            } else if (!is_near_cube || avg_curvature > 0.1) {
                classification = "Sphere";
            }
        }
    }

    // Log and publish the classification result
    std_msgs::msg::String classification_msg;
    classification_msg.data = classification;

    // Publish the classification result
    publisher_->publish(classification_msg);

    RCLCPP_INFO(this->get_logger(), "Classified as: %s", classification.c_str());

    
    if(classification != "Unknown")
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = msg->header.stamp;
        pose.header.frame_id = msg->header.frame_id;

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

        // Broadcast TF
        std::string label = classification;
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);
        tf_buffer_->waitForTransform("map", msg->header.frame_id, msg->header.stamp, timeout, 
                                    std::bind(&Classifier::tf_callback, this, std::placeholders::_1, pose, msg->header.stamp, label));
    }
    
}

void Classifier::tf_callback(const tf2_ros::TransformStampedFuture &tf_future, 
                             const geometry_msgs::msg::PoseStamped &pose,
                             const rclcpp::Time &stamp,
                             const std::string &label)
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

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_msg);
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}

OBBData Classifier::computeOBB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{   
    pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZRGB min_point_OBB, max_point_OBB, position_OBB;
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

double Classifier::computeCurvature(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.setKSearch(10);
    normal_estimator.compute(*normals);

    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> curvature_estimator;
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
