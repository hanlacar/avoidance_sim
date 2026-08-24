"""Convert the single ROS Twist authority into explicit Gazebo joint commands."""

from dataclasses import dataclass
import math
import signal

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64


@dataclass(frozen=True)
class JointTargets:
    front_left_steering: float
    front_right_steering: float
    front_left_wheel: float
    front_right_wheel: float
    rear_left_wheel: float
    rear_right_wheel: float
    valid: bool = True


def ackermann_targets(linear, yaw_rate, wheelbase=0.77, wheel_track=0.70,
                      kingpin_width=0.32, wheel_radius=0.18,
                      max_center_steering=math.radians(25.0)):
    """Return true Ackermann steering positions and wheel angular speeds."""
    values = (linear, yaw_rate, wheelbase, wheel_track, kingpin_width,
              wheel_radius, max_center_steering)
    if (not all(math.isfinite(value) for value in values) or
            wheelbase <= 0.0 or wheel_track <= 0.0 or
            kingpin_width <= 0.0 or wheel_radius <= 0.0):
        return JointTargets(*(0.0,)*6, valid=False)
    if abs(linear) <= 1.0e-6:
        return JointTargets(*(0.0,)*6)

    center = math.atan(wheelbase*yaw_rate/linear)
    if abs(center) > max_center_steering+1.0e-9:
        return JointTargets(*(0.0,)*6, valid=False)
    base_speed = linear/wheel_radius
    if abs(yaw_rate) <= 1.0e-6 or abs(center) <= 1.0e-6:
        return JointTargets(0.0, 0.0, *(base_speed,)*4)

    radius = abs(wheelbase/math.tan(center))
    half_kingpin = kingpin_width/2.0
    if radius <= half_kingpin:
        return JointTargets(*(0.0,)*6, valid=False)
    inner = math.atan(wheelbase/(radius-half_kingpin))
    outer = math.atan(wheelbase/(radius+half_kingpin))
    if center > 0.0:
        left_steering, right_steering = inner, outer
    else:
        left_steering, right_steering = -outer, -inner

    signed_radius = linear/yaw_rate
    half_track = wheel_track/2.0
    rear_left_linear = linear*(signed_radius-half_track)/signed_radius
    rear_right_linear = linear*(signed_radius+half_track)/signed_radius
    front_left_linear = math.copysign(
        abs(linear)*math.hypot(wheelbase, signed_radius-half_track) /
        abs(signed_radius), linear)
    front_right_linear = math.copysign(
        abs(linear)*math.hypot(wheelbase, signed_radius+half_track) /
        abs(signed_radius), linear)
    return JointTargets(
        left_steering, right_steering,
        front_left_linear/wheel_radius,
        front_right_linear/wheel_radius,
        rear_left_linear/wheel_radius,
        rear_right_linear/wheel_radius)


class AckermannJointAdapter(Node):
    TOPICS = (
        '/turtle_car/front_left_steering_cmd',
        '/turtle_car/front_right_steering_cmd',
        '/turtle_car/front_left_wheel_cmd',
        '/turtle_car/front_right_wheel_cmd',
        '/turtle_car/rear_left_wheel_cmd',
        '/turtle_car/rear_right_wheel_cmd',
    )

    def __init__(self):
        super().__init__('ackermann_joint_adapter')
        defaults = {
            'cmd_topic': '/cmd_vel', 'wheelbase_m': 0.77,
            'wheel_track_m': 0.70, 'kingpin_width_m': 0.32,
            'wheel_radius_m': 0.18, 'max_center_steering_deg': 25.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in defaults}
        self._joint_publishers = tuple(
            self.create_publisher(Float64, topic, 10) for topic in self.TOPICS)
        self.subscription = self.create_subscription(
            Twist, str(self.p['cmd_topic']), self._command, 10)
        self.invalid_latched = False
        self.get_logger().info(
            'DIRECT_JOINT_ACKERMANN_READY input=/cmd_vel outputs=6 '
            'ground_truth_odometry=true')

    def _command(self, message):
        targets = ackermann_targets(
            message.linear.x, message.angular.z,
            float(self.p['wheelbase_m']), float(self.p['wheel_track_m']),
            float(self.p['kingpin_width_m']), float(self.p['wheel_radius_m']),
            math.radians(float(self.p['max_center_steering_deg'])))
        if not targets.valid:
            if not self.invalid_latched:
                self.get_logger().error(
                    'JOINT_COMMAND_REJECTED non-finite or steering beyond 25 deg')
                self.invalid_latched = True
        else:
            self.invalid_latched = False
        values = (
            targets.front_left_steering, targets.front_right_steering,
            targets.front_left_wheel, targets.front_right_wheel,
            targets.rear_left_wheel, targets.rear_right_wheel)
        for publisher, value in zip(self._joint_publishers, values):
            publisher.publish(Float64(data=float(value)))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = None
    try:
        node = AckermannJointAdapter()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
