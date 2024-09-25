#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <vector>

class LaserScanProcessor : public rclcpp::Node
{
public:
    LaserScanProcessor() : Node("laser_scan_processor")
    {
        // Subscriber to laser scan topic
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanProcessor::scan_callback, this, std::placeholders::_1));

        // Publisher for selected range subset
        subset_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_subset", 10);

        // Publisher for a specific angle range reading
        range_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/range_at_angle", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Display range reading at a given angle (e.g., 90 degrees)
        int index = static_cast<int>((90.0 - msg->angle_min) / msg->angle_increment);

        // Ensure the index is within bounds
        if (index >= 0 && index < static_cast<int>(msg->ranges.size())) {
            std_msgs::msg::Float32 range_msg;
            range_msg.data = msg->ranges[index];
            range_publisher_->publish(range_msg);
            RCLCPP_INFO(this->get_logger(), "Range at 90 degrees: %f", msg->ranges[index]);
        } else {
            RCLCPP_WARN(this->get_logger(), "Index out of bounds: %d", index);
            return;
        }

        // Select a subset of range values (e.g., 0 to 30 degrees)
        int start_index = static_cast<int>((0.0 - msg->angle_min) / msg->angle_increment);
        int end_index = static_cast<int>((30.0 - msg->angle_min) / msg->angle_increment);

        if (start_index >= 0 && end_index < static_cast<int>(msg->ranges.size()) && start_index < end_index) {
            sensor_msgs::msg::LaserScan subset_msg = *msg;
            subset_msg.ranges.assign(msg->ranges.begin() + start_index, msg->ranges.begin() + end_index);
            subset_publisher_->publish(subset_msg);
            RCLCPP_INFO(this->get_logger(), "Published subset of ranges from %d to %d", start_index, end_index);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid range subset indices: start_index = %d, end_index = %d", start_index, end_index);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr subset_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr range_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanProcessor>());
    rclcpp::shutdown();
    return 0;
}
