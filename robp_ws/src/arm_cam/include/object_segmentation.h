#ifndef OBJECT_SEGMENTATION_H
#define OBJECT_SEGMENTATION_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ObjectSegmentation : public rclcpp::Node
{
public:
    ObjectSegmentation();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    cv::Mat perform_segmentation(const cv::Mat &image);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

#endif // IMAGE_SEGMENTATION_H
