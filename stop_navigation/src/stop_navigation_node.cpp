#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

class StopNavigationNode : public rclcpp::Node
{
public:
    StopNavigationNode() : Node("stop_navigation_node")
    {
        client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/nav2_lifecycle_manager/change_state");

        // Wait until the service is available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Navigation2 lifecycle service...");
        }

        turnOffNavigation();
    }

    void turnOffNavigation()
    {
        std::vector<std::string> nodes = {
            "nav2_lifecycle_manager",
            "map_server",
            "planner_server",
            "controller_server",
            "recoveries_server",
            "bt_navigator",
            "waypoint_follower"
        };

        for (const auto &node_name : nodes) {
            auto client = this->create_client<lifecycle_msgs::srv::ChangeState>("/" + node_name + "/change_state");

            if (!client->wait_for_service(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Service not available for node %s", node_name.c_str());
                continue;
            }

            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

            auto result = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
                RCLCPP_INFO(this->get_logger(), "Node %s deactivated successfully.", node_name.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to deactivate node %s", node_name.c_str());
            }
        }
    }

private:
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StopNavigationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
