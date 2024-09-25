#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

class LaserScanVisualizerNode : public rclcpp::Node {
public:
    LaserScanVisualizerNode()
    : Node("laser_scan_visualizer_node") {
        // Subscriber for original scan
        scan_subscriber_1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanVisualizerNode::scan_callback_1, this, std::placeholders::_1));

        // Subscriber for filtered scan
        scan_subscriber_2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_filtered", 10, std::bind(&LaserScanVisualizerNode::scan_callback_2, this, std::placeholders::_1));

        // Publisher for visualizing original scan
        marker_publisher_1_ = this->create_publisher<visualization_msgs::msg::Marker>("/scan_visualization_1", 10);

        // Publisher for visualizing filtered scan
        marker_publisher_2_ = this->create_publisher<visualization_msgs::msg::Marker>("/scan_visualization_2", 10);

        // Timer to control the visualization rate
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3), 
            std::bind(&LaserScanVisualizerNode::timer_callback, this));
    }

private:
    void scan_callback_1(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_1_ = msg;  // Store the latest message
    }

    void scan_callback_2(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_2_ = msg;  // Store the latest message
    }

    void timer_callback() {
        if (last_scan_1_) {
            publish_visualization(last_scan_1_, marker_publisher_1_, 1);
        }
        if (last_scan_2_) {
            publish_visualization(last_scan_2_, marker_publisher_2_, 2);
        }
    }

    void publish_visualization(const sensor_msgs::msg::LaserScan::SharedPtr msg, 
                               rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
                               int id) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = msg->header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "laser_scan";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.02;  // Line width
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float angle = msg->angle_min + i * msg->angle_increment;
            float x = msg->ranges[i] * cos(angle);
            float y = msg->ranges[i] * sin(angle);

            geometry_msgs::msg::Point p_start;
            p_start.x = 0.0;
            p_start.y = 0.0;
            p_start.z = 0.0;

            geometry_msgs::msg::Point p_end;
            p_end.x = x * 10.0;  // Extend the line indefinitely
            p_end.y = y * 10.0;
            p_end.z = 0.0;

            marker.points.push_back(p_start);
            marker.points.push_back(p_end);
        }

        publisher->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_2_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_1_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_2_;
    rclcpp::TimerBase::SharedPtr timer_;  // Timer for controlled publishing
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_1_;  // Last received scan
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_2_;  // Last received filtered scan
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
