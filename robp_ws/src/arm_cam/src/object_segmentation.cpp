#include "object_segmentation.h"

using std::placeholders::_1;

ObjectSegmentation::ObjectSegmentation() : Node("object_segmentation", rclcpp::NodeOptions()
                                        .allow_undeclared_parameters(true)
                                        .automatically_declare_parameters_from_overrides(true)) 
{

    // Retrieve parameters with get_parameter_or()
    this->get_parameter_or("image_topic", image_topic_, std::string("/arm_camera/filtered/image_raw"));
    this->get_parameter_or("point_obj_topic", point_obj_topic_, std::string("/arm_camera/points"));
    this->get_parameter_or("trigger_topic", trigger_topic_, std::string("/arm_camera/request"));
    this->get_parameter_or("result_topic", result_topic_, std::string("/arm_camera/result"));   
    this->get_parameter_or("Z_camera", Zc_, 0.235);
    this->get_parameter_or("max_obj_size", max_obj_size_, 10000);
    this->get_parameter_or("min_obj_distance", min_obj_distance_, 45.0);
    this->get_parameter_or("max_k", max_k_, 6);
    this->get_parameter_or("visualization", visualization_, false);

    // Define intrinsic matrix K
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 
    438.783367, 0.0, 305.593336,
    0.0, 437.302876, 243.738352,
    0.0, 0.0, 1.0);


    // QoS for keeping only the latest message
    auto qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos_profile, std::bind(&ObjectSegmentation::image_callback, this, std::placeholders::_1));

    if (visualization_) {
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/segmented_image", 10);
        image_lab_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lab_image", 10);
    }

    // Subscribe to trigger topic
    trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        trigger_topic_, qos_profile, std::bind(&ObjectSegmentation::trigger_callback, this, _1));

    // Publisher for result
    result_pub_ = this->create_publisher<std_msgs::msg::Bool>(result_topic_, 10);

    // Publisher for pose array
    world_points_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(point_obj_topic_, 10);
}

void ObjectSegmentation::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    latest_img_ = msg;
}

void ObjectSegmentation::trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
{   
    std_msgs::msg::Bool result_msg;
    result_msg.data = false;

    if (msg->data){
            result_msg.data = perform_segmentation();
    }

    result_pub_->publish(result_msg);
}


bool ObjectSegmentation::perform_segmentation(){

    if (!latest_img_) {
        RCLCPP_WARN(this->get_logger(), "No image received yet!");
        return false;
    }

    // Convert ROS Image to OpenCV format
    cv::Mat image = cv_bridge::toCvCopy(latest_img_, "bgr8")->image;

    // Get image dimensions
    int y_max = image.rows;  
    int cutoff = static_cast<int>(0.8 * y_max);

    // Set pixels above the cutoff to gray
    for (int y = cutoff; y < y_max; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(128, 128, 128);
        }
    }

    // Apply Gaussian Blur to reduce noise
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(5, 5), 8.0);
    // Convert to LAB color space
    cv::Mat lab;
    cv::cvtColor(blurred, lab, cv::COLOR_BGR2Lab);

    // Split LAB channels
    std::vector<cv::Mat> lab_channels;
    cv::split(lab, lab_channels);
    cv::Mat l = lab_channels[0], a = lab_channels[1], b = lab_channels[2];
    // Flatten A and B channels for K-Means
    cv::Mat ab_channels, labels, centers;
    cv::merge(std::vector<cv::Mat>{a, b}, ab_channels);
    ab_channels = ab_channels.reshape(1, image.rows * image.cols);
    ab_channels.convertTo(ab_channels, CV_32F);

    // Apply CLAHE to normalize brightness
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    clahe->apply(l, l);


    int best_k = find_k(ab_channels);
    // RCLCPP_INFO(this->get_logger(), "Best K: %d", best_k);


    // Apply K-Means with the chosen K
    cv::kmeans(ab_channels, best_k, labels, 
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

    // Compute cluster positions and sizes
    std::vector<std::vector<cv::Point>> cluster_pixels(best_k);
    std::vector<cv::Point> sum_cluster_positions(best_k, cv::Point(0, 0));
    std::vector<int> cluster_sizes(best_k, 0);

    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            int cluster_idx = labels.at<int>(y, x);
            cluster_pixels[cluster_idx].push_back(cv::Point(x, y));
            sum_cluster_positions[cluster_idx] += cv::Point(x, y);
            cluster_sizes[cluster_idx]++;
        }
    }

    // Filter out clusters larger than max_obj_size_ (background pixels)
    std::vector<std::vector<cv::Point>> obj_pixels;
    std::vector<cv::Point> sum_obj_positions, obj_positions, merged_obj_positions;
    std::vector<int> obj_sizes;

    for (int i = 0; i < best_k; i++) {
        RCLCPP_INFO(this->get_logger(), "Cluster size: %d", cluster_sizes[i]);
        if (cluster_sizes[i] <= max_obj_size_) {
            sum_obj_positions.push_back(sum_cluster_positions[i]);
            obj_sizes.push_back(cluster_sizes[i]);
            obj_pixels.push_back(cluster_pixels[i]);
        }
    }

    // Store the mean positions of object centers
    for (size_t i = 0; i < sum_obj_positions.size(); i++) {
        if (obj_sizes[i] > 0) {
            // Final center is the average of all points in the cluster
            sum_obj_positions[i].x /= obj_sizes[i];
            sum_obj_positions[i].y /= obj_sizes[i];
            
            obj_positions.push_back(sum_obj_positions[i]);
        }
    }

    // Merge object clusters that are closer than min_obj_distance
    std::vector<std::vector<cv::Point>> merged_obj_pixels;
    std::vector<bool> merged(obj_positions.size(), false); // Track if a center is merged

    for (size_t i = 0; i < obj_positions.size(); i++) {
        if (merged[i]) continue; // Skip already merged clusters

        cv::Point merge = obj_positions[i];
        std::vector<cv::Point> merged_pixels = obj_pixels[i];
        int count = 1;

        // Merge close centers
        for (size_t j = i + 1; j < obj_positions.size(); j++) {
            if (merged[j]) continue;

            double dist = cv::norm(obj_positions[i] - obj_positions[j]);
            RCLCPP_INFO(this->get_logger(), "Dist: %f", dist);
            if (dist < min_obj_distance_) {
                merge += obj_positions[j];
                merged_pixels.insert(merged_pixels.end(), obj_pixels[j].begin(), obj_pixels[j].end());
                count++;
                merged[j] = true; // Mark as merged
            }
        }

        // Compute average
        merge.x /= count;
        merge.y /= count;

        merged_obj_positions.push_back(merge);
        merged_obj_pixels.push_back(merged_pixels);
    }

    if(merged_obj_pixels.size()==0) return false;
    // RCLCPP_INFO(this->get_logger(), "Number of cluster: %ld", merged_obj_positions.size());

    std::vector<std::pair<cv::Point, float>> merged_obj(merged_obj_positions.size());
    for (size_t i = 0; i < merged_obj_positions.size(); i++) {

        cv::RotatedRect box = cv::minAreaRect(merged_obj_pixels[i]);
        cv::Point2f vertices[4];
        box.points(vertices);

        merged_obj.push_back(std::pair(merged_obj_positions[i], box.angle));

        if(visualization_){
            for (int i = 0; i < 4; ++i){
                cv::line(image, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
            }
            RCLCPP_INFO(this->get_logger(), "Object at (%d, %d) angle: %0.2f",
                    merged_obj_positions[i].x, merged_obj_positions[i].y, box.angle);
        }
    }

    if (visualization_) {
        for (const auto &pos : merged_obj_positions) {
            // Draw the markers
            cv::drawMarker(image, pos, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);
        }

        // Create uniform L channel
        cv::Mat l_uniform = cv::Mat::ones(l.size(), CV_8U) * 128;

        // Merge back LAB and convert to BGR
        cv::Mat segmented_lab, segmented_image;
        cv::merge(std::vector<cv::Mat>{l_uniform, segmented_ab}, segmented_lab);
        cv::cvtColor(segmented_lab, segmented_image, cv::COLOR_Lab2BGR);

        // Publish the segmented image
        auto output_msg = cv_bridge::CvImage(latest_img_->header, "bgr8", image).toImageMsg();
        image_pub_->publish(*output_msg);
        auto lab_msg = cv_bridge::CvImage(latest_img_->header, "bgr8", segmented_image).toImageMsg();
        image_lab_pub_->publish(*lab_msg);
    }

    image_to_camera(merged_obj);
    
    return true;
}


void ObjectSegmentation::image_to_camera(std::vector<std::pair<cv::Point, float>>& objects_2d)
{  
    // Publish pose array
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.stamp = latest_img_->header.stamp;
    pose_array_msg.header.frame_id = latest_img_->header.frame_id;

    for (const auto& [center, angle] : objects_2d) {

        // Convert image coordinates (u, v) to camera coordinates (X_c, Y_c, Z_c)
        cv::Mat pixel_homogeneous = (cv::Mat_<double>(3, 1) << center.x, center.y, 1.0);
        cv::Mat camera_coords = camera_matrix_.inv() * pixel_homogeneous * Zc_;

        geometry_msgs::msg::Pose pose;
        pose.position.x = camera_coords.at<double>(0, 0);
        pose.position.y = camera_coords.at<double>(1, 0);
        pose.position.z = Zc_;

        // Orientation (around Z axis for planar rotation)
        tf2::Quaternion q;
        q.setRPY(0, 0, angle * CV_PI / 180.0);  // degrees to radians
        pose.orientation = tf2::toMsg(q);

        pose_array_msg.poses.push_back(pose);
    }

    world_points_pub_->publish(pose_array_msg);
}

int ObjectSegmentation::find_k(cv::Mat& ab_channel)
{
    // Choose the k that yields the best trade-off between Within-Cluster Sum of Squares (WCCS) and total clusters.
    // Line Intersection Method is used to find the elbow point where WCSS stops decreasing significantly.

    std::vector<double> wcss;

    for (int k = 2; k <= max_k_; k++) {
        cv::Mat temp_labels, temp_centers;
        double wcss_k = cv::kmeans(ab_channel, k, temp_labels, 
                                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.2), 
                                    10, cv::KMEANS_RANDOM_CENTERS, temp_centers);
    
        wcss.push_back(wcss_k);
    }

    double best_distance = 0.0;
    int best_k = 2;

    double x1 = 2, y1 = wcss[0];
    double x2 = max_k_, y2 = wcss.back();

    for (size_t i = 0; i < wcss.size(); i++) {
        double x0 = i + 2, y0 = wcss[i];

        double num = std::abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1);
        double denom = std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));

        double distance = num / denom;

        if (distance > best_distance) {
            best_distance = distance;
            best_k = i + 2;
        }
    }
    return best_k;
}