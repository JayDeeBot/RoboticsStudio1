#ifndef LASER_SCAN_FILTER_NODE_HPP
#define LASER_SCAN_FILTER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanFilterNode : public rclcpp::Node {
public:
    LaserScanFilterNode(int n);

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
    int n_; // nth point variable
};

#endif // LASER_SCAN_FILTER_NODE_HPP
