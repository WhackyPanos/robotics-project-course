#ifndef OBJECT_SEGMENTATION_H
#define OBJECT_SEGMENTATION_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <std_msgs/msg/bool.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ObjectSegmentation : public rclcpp::Node
{
public:
    ObjectSegmentation();
    bool perform_segmentation();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void image_to_camera(std::vector<std::pair<cv::Point, float>>& objects_2d);
    int find_k(cv::Mat& ab_channel);

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_lab_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr world_points_pub_;

    //Parameters
    cv::Mat camera_matrix_;
    sensor_msgs::msg::Image::SharedPtr latest_img_;

    std::string image_topic_, point_obj_topic_, trigger_topic_, result_topic_;
    double Zc_, min_obj_distance_;
    int max_obj_size_, max_k_;
    bool visualization_;
};

#endif // IMAGE_SEGMENTATION_H
