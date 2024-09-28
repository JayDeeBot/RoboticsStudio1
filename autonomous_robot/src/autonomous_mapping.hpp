#ifndef AUTONOMOUS_MAPPING_HPP
#define AUTONOMOUS_MAPPING_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"  // Add this include for std_srvs
#include <string>

class AutonomousMapping : public rclcpp::Node
{
public:
    AutonomousMapping();

    // Service to restart explore lite
    void restartExploreLiteService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Service to save the map
    void saveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
    void startEnvironment();
    void killExploreLite();
    void runExploreLite();

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_explore_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
};

#endif // AUTONOMOUS_MAPPING_HPP
