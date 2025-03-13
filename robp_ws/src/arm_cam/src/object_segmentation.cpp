#include "object_segmentation.h"


ObjectSegmentation::ObjectSegmentation() : Node("object_segmentation")
{
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/arm_camera/filtered/image_raw", 10,
        std::bind(&ObjectSegmentation::image_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/segmented_image", 10);
}

void ObjectSegmentation::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        // Convert ROS Image to OpenCV format
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // Convert to LAB color space
        cv::Mat lab;
        cv::cvtColor(image, lab, cv::COLOR_BGR2Lab);

        // Split LAB channels
        std::vector<cv::Mat> lab_channels;
        cv::split(lab, lab_channels);
        cv::Mat l = lab_channels[0], a = lab_channels[1], b = lab_channels[2];

        // Apply CLAHE to normalize brightness
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        clahe->apply(l, l);

        // Flatten A and B channels for K-Means
        cv::Mat ab_channels, labels, centers;
        cv::merge(std::vector<cv::Mat>{a, b}, ab_channels);
        ab_channels = ab_channels.reshape(1, image.rows * image.cols);
        ab_channels.convertTo(ab_channels, CV_32F);

        // K-Means Clustering
        int k = 2;
        cv::kmeans(ab_channels, k, labels, 
                   cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.2), 
                   10, cv::KMEANS_RANDOM_CENTERS, centers);

        // Assign each pixel to its cluster's color
        labels = labels.reshape(1, image.rows);
        centers.convertTo(centers, CV_8U);
        cv::Mat segmented_ab(image.rows, image.cols, CV_8UC2);
        for (int i = 0; i < image.rows; i++) {
            for (int j = 0; j < image.cols; j++) {
                int cluster_idx = labels.at<int>(i, j);
                segmented_ab.at<cv::Vec2b>(i, j) = cv::Vec2b(centers.at<uchar>(cluster_idx, 0),
                                                             centers.at<uchar>(cluster_idx, 1));
            }
        }

        // Compute variance for A and B channels
        cv::Scalar mean_a, stddev_a, mean_b, stddev_b;
        cv::meanStdDev(segmented_ab.col(0), mean_a, stddev_a);
        cv::meanStdDev(segmented_ab.col(1), mean_b, stddev_b);
        double total_variance = stddev_a[0] * stddev_a[0] + stddev_b[0] * stddev_b[0];

        RCLCPP_INFO(this->get_logger(), "Total Variance: %f", total_variance);

        // Create uniform L channel
        cv::Mat l_uniform = cv::Mat::ones(l.size(), CV_8U) * 128;

        // Merge back LAB and convert to BGR
        cv::Mat segmented_lab, segmented_image;
        cv::merge(std::vector<cv::Mat>{l_uniform, segmented_ab}, segmented_lab);
        cv::cvtColor(segmented_lab, segmented_image, cv::COLOR_Lab2BGR);

        // Publish the segmented image
        auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", segmented_image).toImageMsg();
        image_pub_->publish(*output_msg);
    }
    catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}