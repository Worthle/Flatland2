#!/usr/bin/env python3
"""Convert body Twist commands to the two generic omni steering turrets."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class OmniCommand(Node):
    def __init__(self):
        super().__init__('omni_command')
        self.declare_parameter('command_timeout', 0.5)
        self.positions = [(0.55, 0.25), (-0.55, -0.25)]
        self.publishers_ = [self.create_publisher(AckermannDriveStamped, f'drive/turret{i}/command', 10)
                            for i in [1, 2]]
        self.last = None
        self.command = Twist()
        self.last_steering = [0.0, 0.0]
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.receive, 10)
        self.timer = self.create_timer(0.02, self.publish)

    def receive(self, message):
        self.command = message
        self.last = self.get_clock().now()

    def publish(self):
        now = self.get_clock().now()
        fresh = self.last is not None and (now-self.last).nanoseconds/1e9 < self.get_parameter('command_timeout').value
        command = self.command if fresh else Twist()
        for i, ((x, y), publisher) in enumerate(zip(self.positions, self.publishers_)):
            vx = command.linear.x - command.angular.z*y
            vy = command.linear.y + command.angular.z*x
            message = AckermannDriveStamped()
            message.header.stamp = now.to_msg()
            message.drive.speed = math.hypot(vx, vy)
            if message.drive.speed > 1e-6:
                self.last_steering[i] = math.atan2(vy, vx)
            message.drive.steering_angle = self.last_steering[i]
            publisher.publish(message)


if __name__ == '__main__':
    rclpy.init()
    node = OmniCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
