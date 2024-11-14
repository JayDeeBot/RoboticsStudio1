#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class TestCircleNode : public rclcpp::Node
{
public:
    TestCircleNode() : Node("test_circle_node")
    {
        // Path to the PGM map (you can change this path to your actual PGM file)
        std::string pgm_file_path = "/home/student/ros2_ws/src/tejasWarehouse/updatedWarehouse_map2.pgm";

        // Load the PGM file
        cv::Mat map_image = cv::imread(pgm_file_path, cv::IMREAD_GRAYSCALE);

        if (map_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PGM file: %s", pgm_file_path.c_str());
            return;
        }

        // Map resolution in meters per pixel (from the YAML file associated with the PGM)
        double map_resolution = 0.05;  // For example, 5 cm per pixel

        // Calculate the radius of the circle in pixels for a 30 cm diameter
        int circle_radius_pixels = static_cast<int>((1.0 / 2) / map_resolution);  // 30 cm diameter -> 15 cm radius

        // Draw a circle at pixel location (0, 0)
        cv::circle(map_image, cv::Point(0, 0), circle_radius_pixels, cv::Scalar(0), -1);  // Black circle

        // Display the image with the circle
        cv::imshow("PGM with Circle", map_image);
        cv::waitKey(0);  // Wait for a key press to close the window
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestCircleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
