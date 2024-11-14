#include "autonomous_mapping.hpp"
#include <chrono>
#include <thread>

AutonomousMapping::AutonomousMapping() : Node("autonomous_mapping")
{
    // Create services for restarting exploration and saving map
    restart_explore_service_ = this->create_service<std_srvs::srv::Trigger>(
        "restart_explore_lite", std::bind(&AutonomousMapping::restartExploreLiteService, this, std::placeholders::_1, std::placeholders::_2));

    save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
        "save_map", std::bind(&AutonomousMapping::saveMapService, this, std::placeholders::_1, std::placeholders::_2));

    startEnvironment();   // Start the environment, then SLAM and Nav2
    runExploreLite();     // Start exploration
}

void AutonomousMapping::restartExploreLiteService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    killExploreLite();
    runExploreLite();
    response->success = true;
    response->message = "Explore lite restarted.";
}

void AutonomousMapping::saveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::string map_name = "~/map";
    RCLCPP_INFO(this->get_logger(), "Saving map as: %s", map_name.c_str());
    std::string save_cmd = "ros2 run nav2_map_server map_saver_cli -f " + map_name;
    int save_status = system(save_cmd.c_str());
    if (save_status == 0) {
        response->success = true;
        response->message = "Map saved successfully.";
    } else {
        response->success = false;
        response->message = "Failed to save the map.";
    }
}

void AutonomousMapping::startEnvironment()
{
    RCLCPP_INFO(this->get_logger(), "Starting TurtleBot3 simulation environment...");

    // Launch TurtleBot3 world (run in background)
    std::string env_cmd = "export TURTLEBOT3_MODEL=waffle_pi && ros2 launch turtlebot3_gazebo turtlebot3_GroupWarehouse.launch.py use_sim_time:=true &";
    int env_status = system(env_cmd.c_str());
    if (env_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start simulation environment.");
        return;
    }

    // Wait for the world to fully load before starting Nav2 and SLAM
    std::this_thread::sleep_for(std::chrono::seconds(5));  // Add a delay to allow the world to load

    // Launch Nav2 (run in background)
    RCLCPP_INFO(this->get_logger(), "Starting Nav2...");
    std::string nav2_cmd = "ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true &";
    int nav2_status = system(nav2_cmd.c_str());
    if (nav2_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Nav2.");
        return;
    }

    // Wait for Nav2 to start properly
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Launch SLAM toolbox (run in background)
    RCLCPP_INFO(this->get_logger(), "Starting SLAM Toolbox...");
    std::string slam_cmd = "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &";
    int slam_status = system(slam_cmd.c_str());
    if (slam_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start SLAM toolbox.");
        return;
    }
}

void AutonomousMapping::killExploreLite()
{
    RCLCPP_INFO(this->get_logger(), "Killing explore lite...");
    system("pkill -f explore_lite");
}

void AutonomousMapping::runExploreLite()
{
    RCLCPP_INFO(this->get_logger(), "Running explore lite...");
    std::string explore_cmd = "ros2 launch explore_lite explore.launch.py use_sim_time:=true &";
    int explore_status = system(explore_cmd.c_str());
    if (explore_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start explore lite.");
    }
}
