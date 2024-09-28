#include "localizer_and_navigation.hpp"

LocalizerAndNavigation::LocalizerAndNavigation()
    : Node("localizer_and_navigation"), current_goal_index_(0), started_(true), current_state_(State::IDLE)
{
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/scam_goals", 10,
                                                                         std::bind(&LocalizerAndNavigation::goalCallback, this, std::placeholders::_1));
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
                                                                                        std::bind(&LocalizerAndNavigation::amclCallback, this, std::placeholders::_1));
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    publishInitialPose();
}

void LocalizerAndNavigation::publishInitialPose()
{
    auto initial_pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    initial_pose_msg.header.stamp = this->now();
    initial_pose_msg.header.frame_id = "map";
    initial_pose_msg.pose.pose.position.x = 0.0;
    initial_pose_msg.pose.pose.position.y = 0.0;
    initial_pose_msg.pose.pose.orientation.w = 1.0;

    initial_pose_pub_->publish(initial_pose_msg);
}

void LocalizerAndNavigation::goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    goals_ = msg->poses;
    current_goal_index_ = 0;
    current_state_ = State::RUNNING;
}

void LocalizerAndNavigation::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    current_pose_ = msg->pose.pose;
}

bool LocalizerAndNavigation::isGoalReached(const geometry_msgs::msg::Pose &pose)
{
    double tolerance = 0.5;
    const auto &goal = goals_[current_goal_index_];

    double distance = std::hypot(pose.position.x - goal.position.x, pose.position.y - goal.position.y);
    return (distance < tolerance);
}

void LocalizerAndNavigation::publishNextGoal()
{
    if (current_goal_index_ < goals_.size())
    {
        auto goal_msg = geometry_msgs::msg::PoseStamped();
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose = goals_[current_goal_index_];

        goal_pub_->publish(goal_msg);
    }
}

void LocalizerAndNavigation::runStateMachine()
{
    switch (current_state_)
    {
    case State::IDLE:
        RCLCPP_INFO(this->get_logger(), "State: IDLE - Waiting for goals");
        break;
    case State::RUNNING:
        if (isGoalReached(current_pose_))
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            current_goal_index_++;
            current_state_ = State::TASKED;
        }
        else
        {
            publishNextGoal();
        }
        break;
    case State::TASKED:
        RCLCPP_INFO(this->get_logger(), "State: TASKED - Task complete");
        current_state_ = State::IDLE;
        break;
    default:
        break;
    }
}
