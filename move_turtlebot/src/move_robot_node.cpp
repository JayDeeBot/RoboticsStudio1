#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <random>

class MoveRobotNode : public rclcpp::Node
{
public:
    MoveRobotNode() : Node("move_robot_node"), initialized_(false)
    {
        // Initialize publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_noisy_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_noisy", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MoveRobotNode::odomCallback, this, std::placeholders::_1));
        
        // Initialize parameters
        this->declare_parameter<double>("speed", 0.1);  // Single speed input for both linear and angular speed

        // Initialize dead reckoning values
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;

        // Timer to control the motion
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MoveRobotNode::moveRobot, this));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!initialized_)
        {
            // Initialize starting position and orientation
            initial_x_ = msg->pose.pose.position.x;
            initial_y_ = msg->pose.pose.position.y;
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, initial_theta_);

            last_x_ = initial_x_;
            last_y_ = initial_y_;
            theta_ = initial_theta_;

            initialized_ = true;

            RCLCPP_INFO(this->get_logger(), "Initial position set. x: %.3f, y: %.3f, theta: %.3f",
                        initial_x_, initial_y_, initial_theta_);
        }
    }

    void moveRobot()
    {
        if (!initialized_)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for initial odometry data...");
            return;
        }

        // Retrieve the speed parameter (used for both linear and angular velocity)
        double speed = this->get_parameter("speed").as_double();

        // Define the radius for the circle
        double radius = 1.0; // 1 meter

        // Calculate the linear and angular velocities
        double linear_speed = speed;
        double angular_speed = linear_speed / radius;

        // Update dead reckoning position with noise
        std::normal_distribution<double> dist(0.0, 0.01);
        double delta_x = linear_speed * cos(theta_);
        double delta_y = linear_speed * sin(theta_);
        x_ += delta_x + dist(generator_);
        y_ += delta_y + dist(generator_);
        theta_ += angular_speed;

        // Create a noisy odometry message and publish it
        auto noisy_odom_msg = nav_msgs::msg::Odometry();
        noisy_odom_msg.header.stamp = this->now();
        noisy_odom_msg.header.frame_id = "odom";
        noisy_odom_msg.child_frame_id = "base_link";
        noisy_odom_msg.pose.pose.position.x = x_;
        noisy_odom_msg.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        noisy_odom_msg.pose.pose.orientation = tf2::toMsg(q);

        odom_noisy_pub_->publish(noisy_odom_msg);

        // Create a Twist message for robot movement
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = linear_speed;
        cmd_msg.angular.z = -angular_speed;

        // Publish the command
        cmd_vel_pub_->publish(cmd_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_noisy_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Dead reckoning variables
    double x_, y_, theta_;
    double initial_x_, initial_y_, initial_theta_;
    double last_x_, last_y_;
    bool initialized_;

    // Random number generator for Gaussian noise
    std::default_random_engine generator_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveRobotNode>());
    rclcpp::shutdown();
    return 0;
}
