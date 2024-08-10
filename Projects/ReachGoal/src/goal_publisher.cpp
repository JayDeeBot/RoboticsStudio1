#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GoalPublisher : public rclcpp::Node {
public:
    GoalPublisher() : Node("goal_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goals", 10);
        load_goals();
        publish_goals();
    }

private:
    void load_goals() {
        std::ifstream infile(ament_index_cpp::get_package_share_directory("turtlebot3_goal_sender") + "/src/GOALS.txt");
        std::string line;

        while (std::getline(infile, line)) {
            std::istringstream iss(line);
            double x, y, yaw;
            if (!(iss >> x >> y >> yaw)) {
                break;
            }
            goals_.emplace_back(x, y, yaw);
        }
    }

    void publish_goals() {
        for (const auto& goal : goals_) {
            auto message = geometry_msgs::msg::PoseStamped();
            message.pose.position.x = goal.x;
            message.pose.position.y = goal.y;

            tf2::Quaternion q;
            q.setRPY(0, 0, goal.yaw);
            message.pose.orientation = tf2::toMsg(q);

            publisher_->publish(message);
        }
    }

    struct Goal {
        double x, y, yaw;
        Goal(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
    };

    std::vector<Goal> goals_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
