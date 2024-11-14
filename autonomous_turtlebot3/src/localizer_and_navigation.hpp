#ifndef LOCALIZER_AND_NAVIGATION_HPP
#define LOCALIZER_AND_NAVIGATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <mutex>
#include <vector>

class LocalizerAndNavigation : public rclcpp::Node
{
public:
    LocalizerAndNavigation();

    // Method to run the state machine, now public
    void runStateMachine();

private:
    void publishInitialPose();
    void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    bool isGoalReached(const geometry_msgs::msg::Pose &pose);
    void publishNextGoal();

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    std::vector<geometry_msgs::msg::Pose> goals_;
    int current_goal_index_;
    bool started_;

    enum class State
    {
        IDLE,
        RUNNING,
        TASKED
    };
    State current_state_;

    geometry_msgs::msg::Pose current_pose_;
    std::mutex mtx_;
};

#endif // LOCALIZER_AND_NAVIGATION_HPP
