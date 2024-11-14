#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include <vector>
#include <cmath>
#include <mutex>
#include <chrono>

class CircumNavigation : public rclcpp::Node
{
public:
    CircumNavigation()
    : Node("circum_navigation")
    {
        // Subscriber for cylinder location
        cylinder_location_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/cylinder_location", 10, std::bind(&CircumNavigation::cylinderLocationCallback, this, std::placeholders::_1));

        // Publishers back to the original goal sender
        original_goal_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/original_goal", 10);

        // Action client for navigating to poses
        navigate_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");

        // Initialize state
        state_ = State::WAITING_FOR_CYLINDER;
    }

private:
    enum class State {
        WAITING_FOR_CYLINDER,
        CREATE_WAYPOINTS,
        SEND_WAYPOINTS,
        MONITOR_WAYPOINTS,
        RETURN_TO_ORIGINAL_GOAL,
    };

    State state_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cylinder_location_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr original_goal_publisher_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_to_pose_client_;
    std::mutex mtx_;

    geometry_msgs::msg::PoseStamped approach_point_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    geometry_msgs::msg::PoseStamped original_goal_;
    bool waypoints_finished_ = false;
    int waypoint_index_ = 0;
    double distance_from_cylinder_ = 0.5;
    int num_waypoints_ = 8;
    double threshold_ = 0.3;

    void cylinderLocationCallback(const geometry_msgs::msg::Point::SharedPtr cylinder_location)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (state_ == State::WAITING_FOR_CYLINDER) {
            approach_point_ = computeApproachPoint(*cylinder_location, distance_from_cylinder_);
            original_goal_ = approach_point_; // Save the original approach point as the final goal
            state_ = State::CREATE_WAYPOINTS;
            RCLCPP_INFO(this->get_logger(), "Cylinder detected; creating waypoints around it.");
            createWaypointsAroundCylinder(*cylinder_location);
            state_ = State::SEND_WAYPOINTS;
            sendWaypoint();
        }
    }

    void createWaypointsAroundCylinder(const geometry_msgs::msg::Point& cylinder_location)
    {
        waypoints_.clear();
        double radius = distance_from_cylinder_;

        for (int i = 0; i < num_waypoints_; ++i) {
            double angle = (2 * M_PI / num_waypoints_) * i;
            geometry_msgs::msg::PoseStamped waypoint;
            waypoint.header.frame_id = "map";
            waypoint.pose.position.x = cylinder_location.x + radius * cos(angle);
            waypoint.pose.position.y = cylinder_location.y + radius * sin(angle);
            waypoint.pose.orientation = createOrientationForWaypoint(angle + M_PI / 2); // Face direction of movement
            waypoints_.push_back(waypoint);
        }
        RCLCPP_INFO(this->get_logger(), "Waypoints around cylinder created.");
    }

    void sendWaypoint()
    {
        if (waypoint_index_ < waypoints_.size() && state_ == State::SEND_WAYPOINTS) {
            auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
            goal_msg.pose = waypoints_[waypoint_index_];

            auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&CircumNavigation::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&CircumNavigation::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback = std::bind(&CircumNavigation::result_callback, this, std::placeholders::_1);

            navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
            RCLCPP_INFO(this->get_logger(), "Sending waypoint %d to /navigate_to_pose", waypoint_index_ + 1);
        }
    }

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                           const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Distance remaining: %f", feedback->distance_remaining);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %d", waypoint_index_ + 1);
            waypoint_index_++;
            if (waypoint_index_ < waypoints_.size()) {
                sendWaypoint();  // Send the next waypoint
            } else {
                RCLCPP_INFO(this->get_logger(), "Completed all waypoints. Returning to original goal.");
                state_ = State::RETURN_TO_ORIGINAL_GOAL;
                sendFinalGoal();
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to reach waypoint");
        }
    }

    void sendFinalGoal()
    {
        // auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        // goal_msg.pose = original_goal_;

        // auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        // send_goal_options.goal_response_callback = std::bind(&CircumNavigation::goal_response_callback, this, std::placeholders::_1);
        // send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
        //     if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        //         RCLCPP_INFO(rclcpp::get_logger("CircumNavigation"), "Returned to original goal.");
        //     } else {
        //         RCLCPP_ERROR(rclcpp::get_logger("CircumNavigation"), "Failed to return to original goal.");
        //     }
        // };

        // navigate_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
        std_msgs::msg::Bool msg;
        msg.set__data(true);
        original_goal_publisher_->publish(msg);
    }

    geometry_msgs::msg::PoseStamped computeApproachPoint(const geometry_msgs::msg::Point& cylinder_location, double radius)
    {
        geometry_msgs::msg::PoseStamped approach_point;
        approach_point.header.frame_id = "map";
        approach_point.pose.position.x = cylinder_location.x - radius;
        approach_point.pose.position.y = cylinder_location.y;
        approach_point.pose.orientation.w = 1.0;
        return approach_point;
    }

    geometry_msgs::msg::Quaternion createOrientationForWaypoint(double angle)
    {
        geometry_msgs::msg::Quaternion q;
        q.z = sin(angle / 2.0);
        q.w = cos(angle / 2.0);
        return q;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircumNavigation>());
    rclcpp::shutdown();
    return 0;
}
