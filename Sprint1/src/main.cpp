#include "rclcpp/rclcpp.hpp"
#include "laser_scan_filter_node.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Check if n value is provided
    if (argc != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: ros2 run <package_name> <executable_name> <n_value>");
        return 1;
    }

    int n = std::atoi(argv[1]);

    // Validate the input
    if (n <= 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "The value of n must be a positive integer.");
        return 1;
    }

    rclcpp::spin(std::make_shared<LaserScanFilterNode>(n));
    rclcpp::shutdown();
    return 0;
}
