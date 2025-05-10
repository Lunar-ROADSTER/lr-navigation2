import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import time

class CombinedPathTracker(Node):
    def __init__(self):
        super().__init__('combined_path_tracker')

        # Subscribers
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/total_station_prism', self.pose_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)

        # Publishers
        self.robot_path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.global_tracked_path_pub = self.create_publisher(Path, '/global_tracked_path', 10)

        # Paths
        self.robot_path = Path()
        self.robot_path.header.frame_id = "map"

        self.global_tracked_path = Path()
        self.global_tracked_path.header.frame_id = "map"

        self.global_path_distances = []

        # Index markers
        self.goal_start_index = 0

        # State tracking
        self.last_pose = None
        self.last_time = self.get_clock().now()
        self.goal_count = 0
        self.last_goal_time = time.time()
        self.goal_timeout = 10  # seconds to wait before concluding mission
        self.goal_reached = False
        self.current_goal = None
        self.saved_this_goal = False

        # Thresholds
        self.distance_threshold = 0.05  # meters
        self.time_threshold = 0.5       # seconds
        self.goal_tolerance = 0.3       # meters

    def plan_callback(self, msg):
        if not msg.poses:
            return

        new_goal = msg.poses[-1].pose.position

        if self.current_goal is None or self.reached_new_goal(new_goal):
            self.current_goal = new_goal
            self.saved_this_goal = False
            self.goal_reached = False
            self.get_logger().info(f"\U0001F3AF New goal detected. Waiting to store global plan...")

        if not self.saved_this_goal:
            global_distance = self.calculate_path_distance(msg.poses)
            self.global_path_distances.append(global_distance)
            self.global_tracked_path.poses.extend(msg.poses)
            self.global_tracked_path.header.stamp = self.get_clock().now().to_msg()
            self.global_tracked_path_pub.publish(self.global_tracked_path)
            self.saved_this_goal = True
            self.get_logger().info(f"Stored global path for Goal {self.goal_count + 1}. Distance: {global_distance:.2f} m")

    def pose_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9

        if elapsed < self.time_threshold:
            return

        current_pose = msg.pose.pose

        if self.last_pose:
            dx = current_pose.position.x - self.last_pose.position.x
            dy = current_pose.position.y - self.last_pose.position.y
            dist = math.hypot(dx, dy)
            if dist < self.distance_threshold:
                return

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = now.to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = current_pose

        self.robot_path.poses.append(pose_stamped)
        self.robot_path.header.stamp = now.to_msg()
        self.robot_path_pub.publish(self.robot_path)
        self.last_pose = current_pose
        self.last_time = now

        # Check goal reached
        if self.current_goal and not self.goal_reached:
            gx, gy = self.current_goal.x, self.current_goal.y
            rx, ry = current_pose.position.x, current_pose.position.y
            distance_to_goal = math.hypot(rx - gx, ry - gy)
            if distance_to_goal < self.goal_tolerance:
                self.goal_reached = True
                self.goal_count += 1
                self.last_goal_time = time.time()
                self.get_logger().info(f"\U0001F3AF Goal {self.goal_count} reached.")
                self.evaluate_current_goal_metrics()

    def evaluate_current_goal_metrics(self):
        robot_segment = self.robot_path.poses[self.goal_start_index:]
        actual_distance = self.calculate_path_distance(robot_segment)
        global_distance = self.global_path_distances[-1] if self.global_path_distances else 0.0
        margin = 0.1 * global_distance
        status = "✅ WITHIN" if actual_distance <= global_distance + margin else "❌ EXCEEDED"

        self.get_logger().info(f"Goal {self.goal_count} - Robot Path Distance: {actual_distance:.2f} m")
        self.get_logger().info(f"Goal {self.goal_count} - Deviation: {status} 10% margin ({margin:.2f} m)")

        self.goal_start_index = len(self.robot_path.poses)

    def calculate_path_distance(self, poses):
        total = 0.0
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i - 1].pose.position.x
            dy = poses[i].pose.position.y - poses[i - 1].pose.position.y
            total += math.hypot(dx, dy)
        return total

    def reached_new_goal(self, new_goal, tol=0.2):
        if self.current_goal is None:
            return True
        dx = new_goal.x - self.current_goal.x
        dy = new_goal.y - self.current_goal.y
        return (dx * dx + dy * dy) > (tol * tol)

    def check_mission_complete(self):
        now = time.time()
        if self.goal_reached and now - self.last_goal_time > self.goal_timeout:
            self.get_logger().info("✅ Mission complete. Calculating full path metrics...")

            total_robot_distance = self.calculate_path_distance(self.robot_path.poses)
            total_global_distance = sum(self.global_path_distances)
            margin = 0.1 * total_global_distance
            status = "✅ WITHIN" if total_robot_distance <= total_global_distance + margin else "❌ EXCEEDED"

            self.get_logger().info(f"Total Global Path Distance: {total_global_distance:.2f} m")
            self.get_logger().info(f"Total Robot Path Distance: {total_robot_distance:.2f} m")
            self.get_logger().info(f"Final Result: {status} 10% margin ({margin:.2f} m)")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CombinedPathTracker()
    timer_period = 100.0
    node.create_timer(timer_period, node.check_mission_complete)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
