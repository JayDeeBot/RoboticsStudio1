#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class ThreeDModelNode : public rclcpp::Node {
public:
    ThreeDModelNode() : Node("three_d_model_node") {
        // Create subscribers for LiDAR and odometry data
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ThreeDModelNode::lidarCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ThreeDModelNode::odomCallback, this, std::placeholders::_1));

        // Create publisher for 3D model
        model_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/three_d_model", 10);
    }

private:
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        if (!odom_msg_) {
            RCLCPP_WARN(this->get_logger(), "Waiting for odometry message...");
            return;
        }

        // Create the 3D model from the LiDAR scan and odometry
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = create3DModel(*scan_msg, *odom_msg_);

        // Publish the 3D model
        visualization_msgs::msg::Marker model_msg;
        model_msg.header.frame_id = "map";
        model_msg.header.stamp = this->now();
        model_msg.ns = "lidar_model";
        model_msg.id = 0;
        model_msg.type = visualization_msgs::msg::Marker::POINTS;
        model_msg.action = visualization_msgs::msg::Marker::ADD;
        model_msg.pose.orientation.w = 1.0;
        model_msg.scale.x = 0.05;
        model_msg.scale.y = 0.05;
        model_msg.color.a = 1.0;
        model_msg.color.r = 1.0;

        for (const auto& point : cloud->points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            model_msg.points.push_back(p);
        }

        model_pub_->publish(model_msg);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg_ = msg;
    }

    // Function to convert a LiDAR scan to a 3D point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertScanToPointCloud(const sensor_msgs::msg::LaserScan& scan, const geometry_msgs::msg::TransformStamped& transformStamped)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (size_t i = 0; i < scan.ranges.size(); ++i)
        {
            if (std::isfinite(scan.ranges[i]))
            {
                float angle = scan.angle_min + i * scan.angle_increment;
                float x = scan.ranges[i] * cos(angle);
                float y = scan.ranges[i] * sin(angle);

                geometry_msgs::msg::Point point;
                point.x = x;
                point.y = y;
                point.z = 0.0;

                tf2::doTransform(point, point, transformStamped);

                pcl::PointXYZ pcl_point;
                pcl_point.x = point.x;
                pcl_point.y = point.y;
                pcl_point.z = point.z;

                cloud->points.push_back(pcl_point);
            }
        }

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;

        return cloud;
    }

    // Function to get the transformation from the odometry
    geometry_msgs::msg::TransformStamped getTransformStampedFromOdometry(const nav_msgs::msg::Odometry& odom)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header = odom.header;
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = odom.pose.pose.position.x;
        transformStamped.transform.translation.y = odom.pose.pose.position.y;
        transformStamped.transform.translation.z = odom.pose.pose.position.z;
        transformStamped.transform.rotation = odom.pose.pose.orientation;
        return transformStamped;
    }

    // Function to create a 3D model from LiDAR scan and odometry
    pcl::PointCloud<pcl::PointXYZ>::Ptr create3DModel(const sensor_msgs::msg::LaserScan& scan, const nav_msgs::msg::Odometry& odom)
    {
        // Get the transform from the odometry
        geometry_msgs::msg::TransformStamped transformStamped = getTransformStampedFromOdometry(odom);

        // Convert the LiDAR scan to a 3D point cloud in the global frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = convertScanToPointCloud(scan, transformStamped);

        return cloud;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr model_pub_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThreeDModelNode>());
    rclcpp::shutdown();
    return 0;
}
