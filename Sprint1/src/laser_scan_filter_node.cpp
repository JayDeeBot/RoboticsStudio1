#include "laser_scan_filter_node.h"

LaserScanFilterNode::LaserScanFilterNode(int n)
: Node("laser_scan_filter_node"), n_(n) {
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&LaserScanFilterNode::scan_callback, this, std::placeholders::_1));

    scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);
}

void LaserScanFilterNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    filtered_scan->ranges.clear();

    for (size_t i = 0; i < msg->ranges.size(); i += n_) {
        filtered_scan->ranges.push_back(msg->ranges.at(i));

        float angle_rad = msg->angle_min + i * msg->angle_increment;
        float angle_deg = angle_rad * (180.0 / M_PI); // Convert to degrees
        float range = msg->ranges.at(i);

        // Log the angle in degrees and the associated range
        RCLCPP_INFO(this->get_logger(), "Angle: %.2f degrees, Range: %.2f meters", angle_deg, range);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing filtered LaserScan message consisting of every %dth point of the scanning data array in a different topic.", n_);

    // Adjust the other relevant fields if necessary, or just republish the filtered scan
    scan_publisher_->publish(*filtered_scan);
}
