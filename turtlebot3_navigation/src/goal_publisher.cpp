#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

class GoalPublisher : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoalPublisher()
    : Node("goal_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Update the path in the load_goals function
        load_goals("/home/student/ros2_ws/src/turtlebot3_navigation/config/GOALS.txt");

        // Wait for the action server to become available
        this->wait_for_server();

        // Start publishing goals
        this->send_next_goal();
    }

private:
    void load_goals(const std::string &filename)
    {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream ss(line);
            geometry_msgs::msg::PoseStamped goal;
            ss >> goal.pose.position.x >> goal.pose.position.y >> goal.pose.orientation.z;
            goals_.push_back(goal);
        }
        current_goal_ = goals_.begin();
    }

    void wait_for_server()
    {
        while (!this->action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }
        RCLCPP_INFO(this->get_logger(), "Action server available!");
    }

    void send_next_goal()
    {
        if (current_goal_ != goals_.end())
        {
            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose = *current_goal_;
            RCLCPP_INFO(this->get_logger(), "Sending goal: x=%.2f, y=%.2f", goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);

            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
            send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted!");
                        return;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was canceled!");
                        return;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        return;
                }
                ++current_goal_;
                this->send_next_goal();  // Send the next goal after the current one is reached
            };

            action_client_->async_send_goal(goal_msg, send_goal_options);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "All goals have been reached. Shutting down.");
            rclcpp::shutdown();
        }
    }

    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    std::vector<geometry_msgs::msg::PoseStamped>::iterator current_goal_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}
