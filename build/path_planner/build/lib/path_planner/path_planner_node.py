import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathThroughPoses, FollowPath
from rclpy.action import ActionClient
import math

class CircumNavigation(Node):
    def __init__(self):
        super().__init__('circum_navigation')

        # Subscribers
        self.cylinder_location_sub = self.create_subscription(
            Point, '/cylinder_location', self.cylinder_location_callback, 10)

        # Action clients for path computation and following
        self.compute_path_client = ActionClient(self, ComputePathThroughPoses, '/compute_path_through_poses')
        self.follow_path_client = ActionClient(self, FollowPath, '/follow_path')

    def cylinder_location_callback(self, msg):
        self.get_logger().info("Cylinder location received")
        
        approach_point = self.compute_approach_point(msg, 1.25 / 2.0)
        circle_waypoints = self.compute_waypoints_around_cylinder(msg, 1.25)

        # Create a path with approach point, circle waypoints, and back to the approach point
        goals = [approach_point] + circle_waypoints + [approach_point]

        self.get_logger().info(f"Total waypoints including approach and circle: {len(goals)}")

        # Compute path through poses
        if not self.compute_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("ComputePathThroughPoses action server not available")
            return

        compute_goal = ComputePathThroughPoses.Goal()
        compute_goal.goals = goals
        compute_goal.planner_id = "GridBased"

        self.compute_path_client.send_goal_async(compute_goal).add_done_callback(
            lambda future: self.handle_compute_path_result(future, goals))

    def handle_compute_path_result(self, future, goals):
        result = future.result()
        if result.status == 3:  # SUCCEEDED
            self.get_logger().info(f"Path computed successfully with {len(result.result.path.poses)} waypoints")
            self.follow_computed_path(result.result.path)
        else:
            self.get_logger().error("Failed to compute path through poses")

    def follow_computed_path(self, path):
        if not self.follow_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("FollowPath action server not available")
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path

        # Log waypoints
        self.get_logger().info(f"Sending follow path with {len(path.poses)} poses:")
        for i, pose in enumerate(path.poses):
            self.get_logger().info(f"Pose {i}: x={pose.pose.position.x}, y={pose.pose.position.y}")

        self.follow_path_client.send_goal_async(follow_goal).add_done_callback(
            lambda future: self.handle_follow_path_result(future))

    def handle_follow_path_result(self, future):
        result = future.result()
        if result.status == 3:  # SUCCEEDED
            self.get_logger().info("Path followed successfully")
        else:
            self.get_logger().error("Failed to follow path")

    def compute_approach_point(self, cylinder_location, radius):
        approach_point = PoseStamped()
        approach_point.header.frame_id = "map"
        approach_point.pose.position.x = cylinder_location.x - radius
        approach_point.pose.position.y = cylinder_location.y
        approach_point.pose.orientation.w = 1.0
        self.get_logger().info(f"Approach point: x={approach_point.pose.position.x}, y={approach_point.pose.position.y}")
        return approach_point

    def compute_waypoints_around_cylinder(self, cylinder_location, diameter):
        waypoints = []
        num_waypoints = 16
        radius = diameter / 2.0

        for i in range(num_waypoints):
            angle = (2 * math.pi / num_waypoints) * i
            waypoint = PoseStamped()
            waypoint.header.frame_id = "map"
            waypoint.pose.position.x = cylinder_location.x + radius * math.cos(angle)
            waypoint.pose.position.y = cylinder_location.y + radius * math.sin(angle)
            waypoint.pose.orientation = self.create_orientation_for_waypoint(angle + math.pi / 2)
            waypoints.append(waypoint)
            self.get_logger().info(f"Circle waypoint {i}: x={waypoint.pose.position.x}, y={waypoint.pose.position.y}")
        return waypoints

    def create_orientation_for_waypoint(self, angle):
        q = Quaternion()
        q.z = math.sin(angle / 2.0)
        q.w = math.cos(angle / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = CircumNavigation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
