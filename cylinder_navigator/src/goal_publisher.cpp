#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include <thread>
#include <chrono>

class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher()
    : Node("goal_publisher")
    {
        // Create a publisher for the goal_pose topic
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        // Create a subscriber for the /original_goal topic
        original_goal_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/original_goal", 10,
            std::bind(&GoalPublisher::trigger_callback, this, std::placeholders::_1)
        );
        std::this_thread::sleep_for(std::chrono::seconds(15));

        // Publish the initial goal and save it as the original goal
        publish_initial_goal();
    }

private:
    // Publishes the initial goal and saves it as the original goal
    void publish_initial_goal()
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = this->get_clock()->now();

        // Define goal position and orientation
        goal.pose.position.x = -0.7895557742337819;
        goal.pose.position.y = 11.605891467846984;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;

        // Save the goal as the original goal
        original_goal_ = goal;

        // Publish the goal
        goal_publisher_->publish(goal);
        RCLCPP_INFO(this->get_logger(), "Published initial goal to goal_pose");
    }

    // Callback function for the /original_goal topic
    void trigger_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)  // Only publish if the message is true
        {
            RCLCPP_INFO(this->get_logger(), "Received trigger, publishing original goal to goal_pose");
            goal_publisher_->publish(original_goal_);
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr original_goal_subscriber_;
    geometry_msgs::msg::PoseStamped original_goal_;  // Member variable to store the original goal
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}
