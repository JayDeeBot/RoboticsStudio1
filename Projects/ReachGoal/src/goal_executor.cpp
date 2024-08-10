#include <iostream>
#include <queue>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>  // Correct header for tf2::getYaw
#include <chrono>
using namespace std::chrono_literals;

class GoalExecutor : public rclcpp::Node {
public:
    GoalExecutor() : Node("goal_executor"), state_(State::IDLE) {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goals", 10, std::bind(&GoalExecutor::goal_callback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&GoalExecutor::odom_callback, this, std::placeholders::_1));

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_idle", std::bind(&GoalExecutor::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(
            100ms, std::bind(&GoalExecutor::execute, this));
    }

private:
    enum class State { IDLE, ORIENTATION, TRANSLATION };

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goals_.push(*msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_odom_ = *msg;
    }

    void handle_service(const std_srvs::srv::SetBool::Request::SharedPtr request,
                        std_srvs::srv::SetBool::Response::SharedPtr response) {
        if (request->data) {
            state_ = State::TRANSLATION;
            RCLCPP_INFO(this->get_logger(), "Switched to RUNNING state.");
        } else {
            state_ = State::IDLE;
            RCLCPP_INFO(this->get_logger(), "Switched to IDLE state.");
            stop_robot();
        }
        log_current_odom();
        response->success = true;
    }

    void execute() {
        switch (state_) {
            case State::IDLE:
                if (!goals_.empty()) {
                    state_ = State::ORIENTATION;
                } else {
                    RCLCPP_INFO(this->get_logger(), "No goals, staying IDLE.");
                }
                break;
            case State::ORIENTATION:
                if (align_with_goal()) {
                    state_ = State::TRANSLATION;
                }
                break;
            case State::TRANSLATION:
                if (move_towards_goal()) {
                    RCLCPP_INFO(this->get_logger(), "Goal reached at (%f, %f, %f)", 
                                goals_.front().pose.position.x, 
                                goals_.front().pose.position.y, 
                                tf2::getYaw(goals_.front().pose.orientation));
                    goals_.pop();
                    state_ = goals_.empty() ? State::IDLE : State::ORIENTATION;
                    stop_robot();
                }
                break;
        }
    }

    bool align_with_goal() {
        double goal_yaw = tf2::getYaw(goals_.front().pose.orientation);
        double current_yaw = tf2::getYaw(current_odom_.pose.pose.orientation);

        double angle_error = goal_yaw - current_yaw;
        angle_error = atan2(sin(angle_error), cos(angle_error));  // Normalize the angle error

        if (std::abs(angle_error) > 0.01) {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.angular.z = angle_error > 0 ? std::min(0.5, angle_error) : std::max(-0.5, angle_error);
            cmd_vel_publisher_->publish(twist_msg);
            return false;
        } else {
            stop_robot();
            return true;
        }
    }

    bool move_towards_goal() {
        double goal_x = goals_.front().pose.position.x;
        double goal_y = goals_.front().pose.position.y;
        double current_x = current_odom_.pose.pose.position.x;
        double current_y = current_odom_.pose.pose.position.y;

        double position_error = std::sqrt(std::pow(goal_x - current_x, 2) + std::pow(goal_y - current_y, 2));

        if (position_error > 0.1) {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = std::min(0.5, position_error);
            cmd_vel_publisher_->publish(twist_msg);
            return false;
        } else {
            stop_robot();
            return true;
        }
    }

    void stop_robot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        cmd_vel_publisher_->publish(twist_msg);
    }

    void log_current_odom() {
        RCLCPP_INFO(this->get_logger(), "Current position: (%f, %f, %f)", 
                    current_odom_.pose.pose.position.x, 
                    current_odom_.pose.pose.position.y, 
                    tf2::getYaw(current_odom_.pose.pose.orientation));
    }

    State state_;
    std::queue<geometry_msgs::msg::PoseStamped> goals_;
    nav_msgs::msg::Odometry current_odom_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
