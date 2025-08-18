#include "obstacle_detection_pkg/obstacle_detector.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ObstacleDetector>();
    
    RCLCPP_INFO(node->get_logger(), "Starting obstacle detector node...");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}