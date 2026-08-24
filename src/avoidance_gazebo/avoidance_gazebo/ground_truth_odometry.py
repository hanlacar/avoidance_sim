"""Publish spawn-relative ROS odometry from Gazebo's physical model pose."""

import math
import signal

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster


def relative_pose(x, y, z, yaw, origin_x, origin_y, origin_z, origin_yaw):
    """Convert a world pose into the fixed spawn-relative odom frame."""
    dx, dy = x-origin_x, y-origin_y
    cosine, sine = math.cos(origin_yaw), math.sin(origin_yaw)
    local_x = cosine*dx+sine*dy
    local_y = -sine*dx+cosine*dy
    local_yaw = math.atan2(
        math.sin(yaw-origin_yaw), math.cos(yaw-origin_yaw))
    return local_x, local_y, z-origin_z, local_yaw


class GroundTruthOdometry(Node):
    def __init__(self):
        super().__init__('ground_truth_odometry')
        defaults = {
            'input_topic': '/ground_truth/odom_world',
            'output_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'origin_x': 3.25,
            'origin_y': 0.0,
            # The 2D Gazebo OdometryPublisher intentionally reports z=0 even
            # though pose/info places the model frame on the road at -0.025.
            'origin_z': 0.0,
            'origin_yaw': 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self.publisher = self.create_publisher(
            Odometry, str(self.p['output_topic']), 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry, str(self.p['input_topic']), self._receive, 20)
        self.ready_logged = False

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0*(quaternion.w*quaternion.z+
                 quaternion.x*quaternion.y),
            1.0-2.0*(quaternion.y*quaternion.y+
                     quaternion.z*quaternion.z))

    def _receive(self, source):
        pose = source.pose.pose
        yaw = self._yaw(pose.orientation)
        x, y, z, local_yaw = relative_pose(
            pose.position.x, pose.position.y, pose.position.z, yaw,
            float(self.p['origin_x']), float(self.p['origin_y']),
            float(self.p['origin_z']), float(self.p['origin_yaw']))
        if not all(math.isfinite(value) for value in (x, y, z, local_yaw)):
            self.get_logger().error('GROUND_TRUTH_ODOM_REJECTED non-finite pose')
            return

        output = Odometry()
        output.header.stamp = source.header.stamp
        output.header.frame_id = str(self.p['odom_frame'])
        output.child_frame_id = str(self.p['base_frame'])
        output.pose.pose.position.x = x
        output.pose.pose.position.y = y
        output.pose.pose.position.z = z
        output.pose.pose.orientation.z = math.sin(local_yaw/2.0)
        output.pose.pose.orientation.w = math.cos(local_yaw/2.0)
        output.pose.covariance = source.pose.covariance
        # Gazebo OdometryPublisher reports the model's measured physical
        # linear/angular velocity. Preserve it for the stopped confirmation.
        output.twist = source.twist
        self.publisher.publish(output)

        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = output.child_frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        if not self.ready_logged:
            self.ready_logged = True
            self.get_logger().info(
                'GROUND_TRUTH_ODOM_READY source=Gazebo_OdometryPublisher '
                f'world_pose=({pose.position.x:.6f},{pose.position.y:.6f},'
                f'{pose.position.z:.6f}) local_pose=({x:.6f},{y:.6f},'
                f'{z:.6f})')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = None
    try:
        node = GroundTruthOdometry()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
