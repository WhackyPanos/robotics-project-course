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

    // Filter based on height values to distinguish box vs objects
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.0, 0.01);  //9-10cm
    pass.filter(*box_filtered);
    RCLCPP_INFO(this->get_logger(), "Box filtering: %zu", box_filtered->size());

    if (box_filtered->size() > 50) 
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
            pass.setFilterLimits(0.06, 0.075);  //3.5-4cm
            pass.filter(*sphere_filtered);
            RCLCPP_INFO(this->get_logger(), "Sphere filtering: %zu", sphere_filtered->size());
            if(sphere_filtered->size()>0)
            {
                classification = "Sphere";
            }
            else
            {   
                pass.setFilterLimits(0.07, 0.08);  //2-3cm
                pass.filter(*cube_filtered);
                RCLCPP_INFO(this->get_logger(), "Cube filtering: %zu", cube_filtered->size());
                if(cube_filtered->size()>0)
                {
                    classification = "Cube";
                }
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
        // Compute the centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        geometry_msgs::msg::PointStamped point;
        point.header.stamp = msg->header.stamp;
        point.header.frame_id = msg->header.frame_id;
        point.point.x = centroid[0];
        point.point.y = centroid[1];
        point.point.z = centroid[2];

        // Broadcast TF
        std::string label = classification;
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);
        tf_buffer_->waitForTransform("map", msg->header.frame_id, msg->header.stamp, timeout, 
                                    std::bind(&Classifier::tf_callback, this, std::placeholders::_1, point, msg->header.stamp, label));
    }
    
}

void Classifier::tf_callback(const tf2_ros::TransformStampedFuture &tf_future, 
                             const geometry_msgs::msg::PointStamped &point,
                             const rclcpp::Time &stamp,
                             const std::string &label)
{
    try {
        // Extract the transform
        auto tf_base_map = tf_future.get();

        // Transform the point
        geometry_msgs::msg::PointStamped transformed_point;
        tf2::doTransform(point, transformed_point, tf_base_map);

        // Create a transform message
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = stamp;
        transform_msg.header.frame_id = "map";
        transform_msg.child_frame_id = label;

        // Populate transform message with the transformed point
        transform_msg.transform.translation.x = transformed_point.point.x;
        transform_msg.transform.translation.y = transformed_point.point.y;
        transform_msg.transform.translation.z = transformed_point.point.z;

        transform_msg.transform.rotation.x = 0.0;
        transform_msg.transform.rotation.y = 0.0;
        transform_msg.transform.rotation.z = 0.0;
        transform_msg.transform.rotation.w = 1.0;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_msg);
    }
    catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform failed: %s", ex.what());
    }
}
