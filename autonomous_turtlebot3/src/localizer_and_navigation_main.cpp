#include "rclcpp/rclcpp.hpp"
#include "localizer_and_navigation.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizerAndNavigation>();

    rclcpp::Rate loop_rate(10); // 10 Hz loop rate
    while (rclcpp::ok())
    {
        node->runStateMachine();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
