// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>

// class ImageSubscriber : public rclcpp::Node
// {
// public:
//     ImageSubscriber()
//         : Node("image_subscriber")
//     {
//         subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera/image_raw", 10, std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1));

//         publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_modified", 10);
//     }

// private:
//     void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
//     {
//         try {
//             // Convert ROS image message to OpenCV image
//             cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

//             // Draw a circle at the center of the image
//             cv::Point center(image.cols / 2, image.rows / 2);
//             int radius = 30;
//             cv::Scalar color(0, 255, 0); // Green color
//             int thickness = 2;
//             cv::circle(image, center, radius, color, thickness);

//             // Convert OpenCV image back to ROS image message
//             auto modified_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();

//             // Publish the modified image
//             publisher_->publish(*modified_msg);
//         }
//         catch (cv_bridge::Exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//         }
//     }

//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
//     rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ImageSubscriber>());
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
        : Node("image_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ImageSubscriber::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_modified", 10);
    }

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        try {
            // Convert ROS image message to OpenCV image
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Convert the image to HSV color space
            cv::Mat hsv_image;
            cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

            // Define the range for green color in HSV
            cv::Scalar lower_green(35, 100, 100);
            cv::Scalar upper_green(85, 255, 255);

            // Threshold the HSV image to get only green colors
            cv::Mat mask;
            cv::inRange(hsv_image, lower_green, upper_green, mask);

            // Find contours in the mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Draw red borders around green objects
            for (const auto& contour : contours) {
                cv::Rect bounding_box = cv::boundingRect(contour);
                cv::rectangle(image, bounding_box, cv::Scalar(0, 0, 255), 2); // Red color
            }

            // Convert OpenCV image back to ROS image message
            auto modified_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();

            // Publish the modified image
            publisher_->publish(*modified_msg);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
