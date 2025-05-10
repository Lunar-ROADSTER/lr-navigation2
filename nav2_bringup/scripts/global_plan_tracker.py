import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import math

class MultiGoalPathCollector(Node):
    def __init__(self):
        super().__init__('multi_goal_path_collector')

        self.current_goal = None
        self.waypoints = []
        self.full_path = Path()
        self.full_path.header.frame_id = 'map'

        self.plan_received = False

        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Path, '/plan', self.plan_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/global_tracked_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/waypoints_marker', 10)

        self.get_logger().info("📍 Click waypoints using '2D Nav Goal' tool in RViz one by one.")

    def goal_callback(self, msg):
        self.current_goal = msg.pose.position
        self.waypoints.append(self.current_goal)
        self.plan_received = False
        self.get_logger().info(f"🎯 Received Goal {len(self.waypoints)} at ({self.current_goal.x:.2f}, {self.current_goal.y:.2f})")
        self.publish_waypoint_marker()

    def plan_callback(self, msg):
        if not msg.poses:
            return
        if self.plan_received:
            return  # Avoid duplicate appending

        self.full_path.poses.extend(msg.poses)
        self.full_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.full_path)
        self.get_logger().info(f"✅ Added plan segment to full path. Total poses: {len(self.full_path.poses)}")
        self.plan_received = True

    def publish_waypoint_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = self.waypoints
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalPathCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
