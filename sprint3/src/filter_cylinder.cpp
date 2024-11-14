#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <thread>         
#include <chrono>

class FilterCylinderNode : public rclcpp::Node
{
public:
    FilterCylinderNode() : Node("filter_cylinder"), cylinder_detection_count_(0)
    {
        this->declare_parameter<bool>("place_markers", false);
        this->get_parameter("place_markers", place_markers_);

        // Subscribers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FilterCylinderNode::laserScanCallback, this, std::placeholders::_1));
        amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&FilterCylinderNode::amclPoseCallback, this, std::placeholders::_1));

        // Publishers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        cylinder_location_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/cylinder_location", 10);

        // Parameters for detection and clustering
        clustering_threshold_ = 0.1;
        cylinder_diameter_ = 0.30;

        // Load map image
        map_image_ = cv::imread("/home/student/ros2_ws/src/autonomous_robot/map/real_warehouse.pgm", cv::IMREAD_GRAYSCALE);
        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load the map image.");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Map image loaded successfully: %dx%d", map_image_.cols, map_image_.rows);
        }

        map_resolution_ = 0.05;
        map_origin_x_ = 9.5;
        map_origin_y_ = 14.5;
    }

private:
    struct Point2D
    {
        double x;
        double y;
        int index;
    };

    struct Pose2D
    {
        double x;
        double y;
        double theta;
    } robot_pose_;

    // ROS2 communication objects
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cylinder_location_publisher_;

    // Parameters
    double clustering_threshold_;
    double cylinder_diameter_;
    bool place_markers_;
    int cylinder_detection_count_;
    std::vector<Point2D> detected_cylinder_positions_; // Correctly defined as std::vector<Point2D>

    // Map properties
    cv::Mat map_image_;
    cv::Mat edited_map_image_;
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;

    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Extract the robot’s pose from the AMCL message
        robot_pose_.x = msg->pose.pose.position.x;
        robot_pose_.y = msg->pose.pose.position.y;
        
        // Convert quaternion orientation to yaw
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        
        double roll, pitch;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, robot_pose_.theta);
    }

    Point2D rangeToPoint(double range, double angle, int index)
    {
        return {range * std::cos(angle), range * std::sin(angle), index};
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        if (!scan_msg || scan_msg->ranges.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid or empty laser scan message.");
            return;
        }

        std::vector<Point2D> points;
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            double range = scan_msg->ranges[i];
            if (range < scan_msg->range_min || range > scan_msg->range_max) continue;

            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            points.push_back(rangeToPoint(range, angle, i));
        }

        auto clusters = clusterPoints(points);
        for (const auto &cluster : clusters)
        {
            if (cluster.size() >= 2 && std::abs(calculateClusterWidth(cluster) - cylinder_diameter_) < 0.05)
            {
                Point2D cylinder_center = calculateClusterCenter(cluster);
                detected_cylinder_positions_.push_back(cylinder_center);
                cylinder_detection_count_++;

                if (place_markers_)
                {
                    publishMarker(cylinder_center);
                }
                drawCylinderOnMap(cylinder_center);

                if (cylinder_detection_count_ >= 100)
                {
                    auto averaged_location = averageCylinderPosition();
                    publishCylinderLocation(averaged_location);
                    resetDetectionData();
                }
            }
        }
    }

    Point2D calculateClusterCenter(const std::vector<Point2D> &cluster)
    {
        Point2D center{0.0, 0.0, 0};
        for (const auto &point : cluster)
        {
            // Convert from robot's local frame to the global frame using AMCL pose
            double global_x = robot_pose_.x + std::cos(robot_pose_.theta) * point.x - std::sin(robot_pose_.theta) * point.y;
            double global_y = robot_pose_.y + std::sin(robot_pose_.theta) * point.x + std::cos(robot_pose_.theta) * point.y;
            center.x += global_x;
            center.y += global_y;
        }
        center.x /= cluster.size();
        center.y /= cluster.size();
        return center;
    }

    void drawCylinderOnMap(const Point2D &cylinder_center)
    {
        edited_map_image_ = map_image_.clone();
        int map_x = static_cast<int>((map_origin_x_ + cylinder_center.x) / map_resolution_);
        int map_y = static_cast<int>((map_origin_y_ + cylinder_center.y) / map_resolution_);
        if (map_x >= 0 && map_x < map_image_.cols && map_y >= 0 && map_y < map_image_.rows)
        {
            int radius = static_cast<int>((cylinder_diameter_ / 2) / map_resolution_);
            cv::circle(edited_map_image_, cv::Point(map_x, map_y), radius, cv::Scalar(0), -1);
            cv::imshow("Map with Cylinder", edited_map_image_);
            cv::waitKey(1);
        }
    }

    std::vector<std::vector<Point2D>> clusterPoints(const std::vector<Point2D> &points)
    {
        std::vector<std::vector<Point2D>> clusters;
        std::vector<Point2D> current_cluster;

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (current_cluster.empty())
            {
                current_cluster.push_back(points[i]);
            }
            else
            {
                double distance = calculateDistance(current_cluster.back(), points[i]);
                if (distance < clustering_threshold_)
                {
                    current_cluster.push_back(points[i]);
                }
                else
                {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                    current_cluster.push_back(points[i]);
                }
            }
        }
        if (!current_cluster.empty()) clusters.push_back(current_cluster);
        return clusters;
    }

    double calculateDistance(const Point2D &p1, const Point2D &p2)
    {
        return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    }

    double calculateClusterWidth(const std::vector<Point2D> &cluster)
    {
        if (cluster.size() < 2) return 0.0;
        return calculateDistance(cluster.front(), cluster.back());
    }

    Point2D averageCylinderPosition()
    {
        double sum_x = 0.0, sum_y = 0.0;
        for (const auto &position : detected_cylinder_positions_)
        {
            sum_x += position.x;
            sum_y += position.y;
        }
        return {sum_x / detected_cylinder_positions_.size(), sum_y / detected_cylinder_positions_.size(), 0};
    }

    void publishCylinderLocation(const Point2D &averaged_location)
    {
        geometry_msgs::msg::Point cylinder_msg;
        cylinder_msg.x = averaged_location.x;
        cylinder_msg.y = averaged_location.y;
        cylinder_msg.z = 0.0;

        cylinder_location_publisher_->publish(cylinder_msg);
        RCLCPP_INFO(this->get_logger(), "Published cylinder location to /cylinder_location: (%.2f, %.2f)", cylinder_msg.x, cylinder_msg.y);
    }

    void resetDetectionData()
    {
        cylinder_detection_count_ = 0;
        detected_cylinder_positions_.clear();
    }

    void publishMarker(const Point2D &point)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder_marker";
        marker.id = point.index;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        double global_x = robot_pose_.x + std::cos(robot_pose_.theta) * point.x - std::sin(robot_pose_.theta) * point.y;
        double global_y = robot_pose_.y + std::sin(robot_pose_.theta) * point.x + std::cos(robot_pose_.theta) * point.y;

        marker.pose.position.x = global_x;
        marker.pose.position.y = global_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::this_thread::sleep_for(std::chrono::seconds(15));
    auto node = std::make_shared<FilterCylinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
