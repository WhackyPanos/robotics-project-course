#include "box_segmentation.h"

using std::placeholders::_1;

BoxSegmentation::BoxSegmentation() : Node("box_segmentation", rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) 
{

    // Retrieve parameters with get_parameter_or()
    this->get_parameter_or("image_topic", image_topic_, std::string("/arm_camera/filtered/image_raw"));
    this->get_parameter_or("point_box_topic", point_box_topic_, std::string("/arm_camera/box_pose"));
    this->get_parameter_or("trigger_topic", trigger_topic_, std::string("/arm_camera/box_request"));
    this->get_parameter_or("result_topic", result_topic_, std::string("/arm_camera/box_result"));   
    this->get_parameter_or("Z_camera", Zc_, 0.235);
    this->get_parameter_or("min_box_size", min_box_size_, 200);
    this->get_parameter_or("visualization", visualization_, false);

    // Define intrinsic matrix K
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
    438.783367, 0.0, 305.593336,
    0.0, 437.302876, 243.738352,
    0.0, 0.0, 1.0);


    // QoS for keeping only the latest message
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos_profile, std::bind(&BoxSegmentation::image_callback, this, std::placeholders::_1));

    if (visualization_) {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/segmented_image", 10);
        image_lab_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lab_image", 10);
    }

    // Subscribe to trigger topic
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        trigger_topic_, qos_profile, std::bind(&BoxSegmentation::trigger_callback, this, _1));

    // Publisher for result
    result_pub_ = this->create_publisher<std_msgs::msg::Bool>(result_topic_, 10);

    // Publisher for pose array
    world_points_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(point_box_topic_, 10);
}

void BoxSegmentation::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    latest_img_ = msg;
}

void BoxSegmentation::trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    std_msgs::msg::Bool result_msg;
    result_msg.data = false;

    if (msg->data){
            result_msg.data = perform_segmentation();
    }

    result_pub_->publish(result_msg);
}


bool BoxSegmentation::perform_segmentation(){

    if (!latest_img_) {
        RCLCPP_WARN(this->get_logger(), "No image received yet!");
        return false;
    }

    // Convert ROS Image to OpenCV format
    cv::Mat image = cv_bridge::toCvCopy(latest_img_, "bgr8")->image;

    // Get image dimensions
    int y_max = image.rows;
    int cutoff = static_cast<int>(0.65 * y_max);

    // Set pixels above the cutoff to gray
    for (int y = cutoff; y < y_max; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
        }
    }

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 16);

    // Detect edges with Canny
    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 250, 3);

    // Collect all edge points
    std::vector<cv::Point> edge_points;
    for (int y = 0; y < edges.rows; ++y) {
        for (int x = 0; x < edges.cols; ++x) {
            if (edges.at<uchar>(y, x) == 255) {
                edge_points.emplace_back(x, y);
            }
        }
    }

    if (int(edge_points.size()) < min_box_size_) {
        RCLCPP_WARN(this->get_logger(), "Not enough edges found (%lu).", edge_points.size());
        return false;
    }
    else
    {   
        RCLCPP_INFO(this->get_logger(), "Detected %lu edge points", edge_points.size());
        
        cv::Point sum(0, 0);
        for (const auto& p : edge_points) {
            sum += p;
        }
        cv::Point mean(sum.x / edge_points.size(), sum.y / edge_points.size());
        RCLCPP_INFO(this->get_logger(), "Mean edge point at (%d, %d)", mean.x, mean.y);
        cv::drawMarker(image, mean, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);

        image_to_camera(mean);
    }

    // Publish the result
    if (visualization_) {
        auto output_msg = cv_bridge::CvImage(latest_img_->header, "bgr8", image).toImageMsg();
        image_pub_->publish(*output_msg);
        
        cv::Mat edges_bgr;
        cv::cvtColor(edges, edges_bgr, cv::COLOR_GRAY2BGR);
        auto edge_msg = cv_bridge::CvImage(latest_img_->header, "bgr8", edges_bgr).toImageMsg();
        image_lab_pub_->publish(*edge_msg);
    }

    return true;
}

void BoxSegmentation::image_to_camera(const cv::Point& box_pos)
{  
    // Publish point stamped message
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header.stamp = latest_img_->header.stamp;
    point_msg.header.frame_id = latest_img_->header.frame_id;

    // Convert image coordinates (u, v) to camera coordinates (X_c, Y_c, Z_c)
    cv::Mat pixel_homogeneous = (cv::Mat_<double>(3, 1) << box_pos.x, box_pos.y, 1.0);
    cv::Mat camera_coords = camera_matrix_.inv() * pixel_homogeneous * Zc_;

    // Set the position of the point
    point_msg.point.x = camera_coords.at<double>(0, 0);
    point_msg.point.y = camera_coords.at<double>(1, 0);
    point_msg.point.z = Zc_;  // Assuming a constant Z value

    // Publish the point message
    world_points_pub_->publish(point_msg);
}

