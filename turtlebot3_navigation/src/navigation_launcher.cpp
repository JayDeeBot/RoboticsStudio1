#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class NavigationLauncher : public rclcpp::Node
{
public:
    NavigationLauncher()
    : Node("navigation_launcher")
    {
        // Launching turtlebot3_bringup
        std::system("ros2 launch turtlebot3_bringup robot.launch.py &");

        // Launching map server
        std::system("ros2 launch nav2_bringup map_server.launch.py map:=/home/student/ros2_ws/src/turtlebot3_navigation/config/map.yaml &");

        // Launching AMCL
        std::system("ros2 launch nav2_bringup localization_launch.py use_sim_time:=true &");

        // Launching Navigation2
        std::system("ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true &");

        // Launching RViz
        std::system("rviz2 &");

        RCLCPP_INFO(this->get_logger(), "All necessary nodes for navigation have been launched.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationLauncher>());
    rclcpp::shutdown();
    return 0;
}
