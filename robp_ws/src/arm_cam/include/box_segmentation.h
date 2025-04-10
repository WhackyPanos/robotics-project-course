#ifndef BOX_SEGMENTATION_H
#define BOX_SEGMENTATION_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class BoxSegmentation : public rclcpp::Node
{
public:
    BoxSegmentation();
    bool perform_segmentation();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void image_to_camera(const cv::Point& box_pos);

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_lab_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr world_points_pub_;

    //Parameters
    cv::Mat camera_matrix_;
    sensor_msgs::msg::Image::SharedPtr latest_img_;

    std::string image_topic_, point_box_topic_, trigger_topic_, result_topic_;
    double Zc_;
    int min_box_size_;
    bool visualization_;
};

#endif // BOX_SEGMENTATION_H
