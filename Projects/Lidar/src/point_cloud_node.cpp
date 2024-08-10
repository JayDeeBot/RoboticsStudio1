#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudNode : public rclcpp::Node {
public:
    PointCloudNode() : Node("point_cloud_node") {
        // Create subscribers for LiDAR and odometry data
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PointCloudNode::lidarCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PointCloudNode::odomCallback, this, std::placeholders::_1));
        
        // Create publisher for point cloud
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        if (!odom_msg_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for odometry message...");
            return;
        }

        // Convert LiDAR data to point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud = lidarToPointCloud(scan_msg, odom_msg_);

        // Convert PCL point cloud to ROS PointCloud2 message
        sensor_msgs::msg::PointCloud2 point_cloud_msg;
        pcl::toROSMsg(cloud, point_cloud_msg);
        point_cloud_msg.header.frame_id = "map";
        point_cloud_msg.header.stamp = this->now();

        // Publish the point cloud
        point_cloud_pub_->publish(point_cloud_msg);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg_ = msg;
    }

    pcl::PointCloud<pcl::PointXYZ> lidarToPointCloud(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan,
        const nav_msgs::msg::Odometry::SharedPtr& odom) {
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // Extract pose from odometry
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;
        double z = odom->pose.pose.position.z;

        // Extract orientation (quaternion) from odometry
        tf2::Quaternion quat(
            odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y,
            odom->pose.pose.orientation.z,
            odom->pose.pose.orientation.w);
        
        // Convert quaternion to RPY (roll, pitch, yaw)
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // Convert laser scan to point cloud
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (std::isfinite(scan->ranges[i])) {
                double angle = scan->angle_min + i * scan->angle_increment;
                double range = scan->ranges[i];

                // Calculate the point in the local frame
                double local_x = range * std::cos(angle);
                double local_y = range * std::sin(angle);
                double local_z = 0.0; // 2D LiDAR

                // Transform point to global frame using odometry data
                double global_x = local_x * std::cos(yaw) - local_y * std::sin(yaw) + x;
                double global_y = local_x * std::sin(yaw) + local_y * std::cos(yaw) + y;
                double global_z = local_z + z;

                cloud.points.emplace_back(global_x, global_y, global_z);
            }
        }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;

        return cloud;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudNode>());
    rclcpp::shutdown();
    return 0;
}
