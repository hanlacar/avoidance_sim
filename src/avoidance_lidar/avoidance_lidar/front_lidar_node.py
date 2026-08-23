"""Visualize a configurable front ROI and detected scan clusters."""

import math
import signal

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray

from .processing import cluster_points, valid_roi_points


class FrontLidarNode(Node):
    def __init__(self):
        super().__init__('front_lidar_detector')
        defaults = {
            'scan_topic': '/scan_front', 'min_range_m': 0.12,
            'max_range_m': 12.0, 'roi_x_min_m': 0.12,
            'roi_x_max_m': 12.0, 'roi_half_width_m': 0.975,
            'self_x_min_m': -1.0, 'self_x_max_m': 0.10,
            'self_y_half_width_m': 0.45,
            'cluster_distance_m': 0.18, 'min_cluster_points': 3,
            'scan_timeout_s': 0.5, 'marker_lifetime_s': 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.values = {name: self.get_parameter(name).value for name in defaults}
        if self.values['roi_x_min_m'] >= self.values['roi_x_max_m']:
            raise ValueError('roi_x_min_m must be smaller than roi_x_max_m')
        if self.values['roi_half_width_m'] <= 0.0:
            raise ValueError('roi_half_width_m must be positive')

        self.roi_points_pub = self.create_publisher(
            PointCloud2, '/avoidance/lidar/roi_points', 10)
        self.roi_marker_pub = self.create_publisher(
            Marker, '/avoidance/lidar/roi_marker', 10)
        self.obstacles_pub = self.create_publisher(
            MarkerArray, '/avoidance/lidar/obstacles', 10)
        self.nearest_marker_pub = self.create_publisher(
            Marker, '/avoidance/lidar/nearest_marker', 10)
        self.nearest_point_pub = self.create_publisher(
            PointStamped, '/avoidance/lidar/nearest_point', 10)
        self.nearest_distance_pub = self.create_publisher(
            Float32, '/avoidance/lidar/nearest_distance', 10)
        self.detected_pub = self.create_publisher(
            Bool, '/avoidance/lidar/obstacle_detected', 10)
        self.valid_pub = self.create_publisher(
            Bool, '/avoidance/lidar/valid', 10)
        self.last_scan_time = None
        self.last_frame = ''
        self.create_subscription(
            LaserScan, str(self.values['scan_topic']), self._scan,
            qos_profile_sensor_data)
        self.create_timer(0.1, self._watchdog)
        self.get_logger().info(
            f"Front LiDAR diagnostics: {self.values['scan_topic']}, "
            f"ROI x=[{self.values['roi_x_min_m']:.2f},"
            f"{self.values['roi_x_max_m']:.2f}], "
            f"y=+/-{self.values['roi_half_width_m']:.3f} m")

    def _duration(self):
        value = max(0.0, float(self.values['marker_lifetime_s']))
        from builtin_interfaces.msg import Duration
        return Duration(sec=int(value), nanosec=int((value % 1.0) * 1e9))

    def _base_marker(self, frame, stamp, namespace, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = self._duration()
        return marker

    def _scan(self, scan):
        now = self.get_clock().now()
        self.last_scan_time = now
        self.last_frame = scan.header.frame_id
        if not self.last_frame:
            self.get_logger().warning('LaserScan has an empty frame_id', throttle_duration_sec=2.0)
            return
        points = valid_roi_points(
            scan.ranges, scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
            self.values['min_range_m'], self.values['max_range_m'],
            self.values['roi_x_min_m'], self.values['roi_x_max_m'],
            self.values['roi_half_width_m'], self.values['self_x_min_m'],
            self.values['self_x_max_m'], self.values['self_y_half_width_m'])
        clusters = cluster_points(
            points, float(self.values['cluster_distance_m']),
            int(self.values['min_cluster_points']))

        header = scan.header
        cloud = point_cloud2.create_cloud_xyz32(
            header, [(point.x, point.y, 0.0) for point in points])
        self.roi_points_pub.publish(cloud)
        self._publish_roi(header.frame_id, header.stamp)
        self._publish_clusters(header.frame_id, header.stamp, clusters)
        self.valid_pub.publish(Bool(data=True))
        self.detected_pub.publish(Bool(data=bool(clusters)))

        nearest = min(clusters, key=lambda item: item.nearest_distance) if clusters else None
        self.nearest_distance_pub.publish(Float32(
            data=float(nearest.nearest_distance) if nearest else math.inf))
        if nearest:
            point = PointStamped()
            point.header = header
            point.point.x, point.point.y = nearest.x, nearest.y
            self.nearest_point_pub.publish(point)
            marker = self._base_marker(
                header.frame_id, header.stamp, 'nearest_obstacle', 0,
                Marker.SPHERE)
            marker.pose.position.x, marker.pose.position.y = nearest.x, nearest.y
            marker.scale.x = marker.scale.y = marker.scale.z = 0.16
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 1.0, 1.0
            self.nearest_marker_pub.publish(marker)
        else:
            self._delete_nearest(header.frame_id, header.stamp)

    def _publish_roi(self, frame, stamp):
        marker = self._base_marker(frame, stamp, 'front_roi', 0, Marker.LINE_STRIP)
        marker.scale.x = 0.035
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.85, 0.0, 1.0
        x0, x1 = self.values['roi_x_min_m'], self.values['roi_x_max_m']
        half = self.values['roi_half_width_m']
        marker.points = [Point(x=x0, y=-half), Point(x=x1, y=-half),
                         Point(x=x1, y=half), Point(x=x0, y=half),
                         Point(x=x0, y=-half)]
        self.roi_marker_pub.publish(marker)

    def _publish_clusters(self, frame, stamp, clusters):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for marker_id, cluster in enumerate(clusters):
            marker = self._base_marker(frame, stamp, 'front_obstacles', marker_id, Marker.CUBE)
            marker.pose.position.x, marker.pose.position.y = cluster.x, cluster.y
            marker.scale.x = 0.18
            marker.scale.y = max(0.08, cluster.width)
            marker.scale.z = 0.20
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.08, 0.03, 0.85
            markers.markers.append(marker)
        self.obstacles_pub.publish(markers)

    def _delete_nearest(self, frame, stamp):
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = frame, stamp
        marker.ns, marker.id, marker.action = 'nearest_obstacle', 0, Marker.DELETE
        self.nearest_marker_pub.publish(marker)

    def _watchdog(self):
        if self.last_scan_time is None:
            self.valid_pub.publish(Bool(data=False))
            return
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9
        if age <= float(self.values['scan_timeout_s']):
            return
        self.valid_pub.publish(Bool(data=False))
        self.detected_pub.publish(Bool(data=False))
        self.nearest_distance_pub.publish(Float32(data=math.inf))
        if self.last_frame:
            stamp = self.get_clock().now().to_msg()
            self._publish_clusters(self.last_frame, stamp, [])
            self._delete_nearest(self.last_frame, stamp)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = FrontLidarNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
