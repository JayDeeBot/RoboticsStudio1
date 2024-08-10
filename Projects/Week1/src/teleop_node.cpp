#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

class TeleopTurtlebot3 : public rclcpp::Node
{
public:
    TeleopTurtlebot3()
        : Node("teleop_turtlebot3")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Initialize terminal settings for non-blocking keyboard input
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio_ = old_tio_;
        new_tio_.c_lflag &= (~ICANON & ~ECHO);
        new_tio_.c_cc[VMIN] = 0;
        new_tio_.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
    }

    ~TeleopTurtlebot3()
    {
        // Restore old terminal settings
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

    void run()
    {
        char key;
        geometry_msgs::msg::Twist twist;

        while (rclcpp::ok())
        {
            key = getKey();
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;

            switch (key)
            {
            case 'w':
                twist.linear.x = 0.5;
                break;
            case 's':
                twist.linear.x = -0.5;
                break;
            case 'a':
                twist.angular.z = 0.5;
                break;
            case 'd':
                twist.angular.z = -0.5;
                break;
            }

            publisher_->publish(twist);
            rclcpp::spin_some(this->shared_from_this());
        }
    }

private:
    char getKey()
    {
        char buf = 0;
        ssize_t n = read(STDIN_FILENO, &buf, 1);
        if (n < 0)
        {
            std::cerr << "Error reading from stdin" << std::endl;
        }
        return buf;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    struct termios old_tio_, new_tio_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTurtlebot3>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
