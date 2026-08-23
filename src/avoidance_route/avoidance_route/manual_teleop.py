"""Small terminal teleop for turtle_car's geometry_msgs/Twist contract."""

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """Keys: w/s speed, a/d steer, space stop, q quit
Commands time out automatically; keep pressing a motion key.
"""


class ManualTeleop(Node):
    def __init__(self):
        super().__init__('manual_teleop')
        for name, value in (('cmd_topic', '/cmd_vel'), ('speed_mps', 0.7),
                            ('steering_rate_rps', 0.6), ('key_timeout_s', 0.35)):
            self.declare_parameter(name, value)
        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.speed = float(self.get_parameter('speed_mps').value)
        self.steer = float(self.get_parameter('steering_rate_rps').value)
        self.timeout = float(self.get_parameter('key_timeout_s').value)
        self.last_key_ns = 0
        self.command = Twist()
        self.done = False
        self.create_timer(0.05, self._tick)

    def key(self, value):
        command = Twist()
        if value == 'w':
            command.linear.x = self.speed
        elif value == 's':
            command.linear.x = -self.speed
        elif value == 'a':
            command.linear.x = self.command.linear.x or self.speed
            command.angular.z = self.steer
        elif value == 'd':
            command.linear.x = self.command.linear.x or self.speed
            command.angular.z = -self.steer
        elif value == 'q':
            self.done = True
        self.command = command
        self.last_key_ns = self.get_clock().now().nanoseconds
        self.publisher.publish(command)

    def _tick(self):
        age = (self.get_clock().now().nanoseconds - self.last_key_ns) / 1e9
        if age > self.timeout and (self.command.linear.x or self.command.angular.z):
            self.command = Twist()
            self.publisher.publish(self.command)

    def stop(self):
        self.command = Twist()
        self.publisher.publish(self.command)


def main(args=None):
    if not sys.stdin.isatty():
        raise RuntimeError('manual_teleop requires an interactive terminal')
    rclpy.init(args=args)
    node = ManualTeleop()
    settings = termios.tcgetattr(sys.stdin)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    print(HELP)
    try:
        tty.setraw(sys.stdin.fileno())
        while rclpy.ok() and not node.done:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                node.key(sys.stdin.read(1).lower())
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=1.0)
