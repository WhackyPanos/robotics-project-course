#include "box_segmentation.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BoxSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}