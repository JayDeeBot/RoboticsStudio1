#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>

class LaserScanToImageNode : public rclcpp::Node
{
public:
    LaserScanToImageNode() : Node("scan_matching_localizer")
    {
        // Subscribe to the laser scan topic
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanToImageNode::scanCallback, this, std::placeholders::_1));
        
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Convert the laser scan to an image
        cv::Mat img = laserScanToMat(scan_msg);

        if (!first_image_captured_) {
            first_image_ = img.clone();
            first_image_captured_ = true;
            // Display the first image
            cv::imshow("First Image", first_image_);
            cv::waitKey(1);
            // Rotate the robot to capture the second image
            rotateRobot();
        } else if (!second_image_captured_) {
            second_image_ = img.clone();
            second_image_captured_ = true;
            // Display the second image
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1);
            // Calculate the yaw change between the two images
            calculateYawChange();
        }
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Parameters
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat img = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            if (std::isfinite(range)) {
                float angle = scan->angle_min + i * scan->angle_increment;
                int x = static_cast<int>((range * cos(angle) / max_range) * img_size / 2 + img_size / 2);
                int y = static_cast<int>((range * sin(angle) / max_range) * img_size / 2 + img_size / 2);
                img.at<uchar>(y, x) = 255;
            }
        }
        return img;
    }

    void rotateRobot()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 0.1;  // Rotate with some angular velocity
        cmd_publisher_->publish(twist_msg);

        // Sleep for a while to allow the robot to rotate
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Stop rotation
        twist_msg.angular.z = 0.0;
        cmd_publisher_->publish(twist_msg);
    }

    void calculateYawChange()
    {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Affine transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0)) * 180.0 / CV_PI;
                relative_orientation_ += angle_difference_;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        } catch (const cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
    }

    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 15%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    cv::Mat first_image_, second_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;
    double angle_difference_ = 0.0;
    double relative_orientation_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}
