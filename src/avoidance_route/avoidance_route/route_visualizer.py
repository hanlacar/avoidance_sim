"""Publish a recorded CSV as a transient-local nav_msgs/Path."""

from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from .route_io import load_path_points


class RouteVisualizer(Node):
    def __init__(self):
        super().__init__('route_visualizer')
        self.declare_parameter('csv_path', '')
        self.declare_parameter('frame_id', 'odom')
        self.csv_path = Path(str(self.get_parameter('csv_path').value)).expanduser()
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            PathMsg, '/avoidance/route/csv_path', qos)
        self.mtime = None
        self.create_timer(1.0, self._reload)
        self._reload()

    def _reload(self):
        try:
            mtime = self.csv_path.stat().st_mtime
            if mtime == self.mtime:
                return
            points = load_path_points(self.csv_path)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as exc:
            self.get_logger().warning(f'Cannot visualize CSV: {exc}', throttle_duration_sec=3.0)
            return
        path = PathMsg()
        path.header.frame_id = str(self.get_parameter('frame_id').value)
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.publisher.publish(path)
        self.mtime = mtime
        self.get_logger().info(f'Published CSV path: {len(points)} points from {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = RouteVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
