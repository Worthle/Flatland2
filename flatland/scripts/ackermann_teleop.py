#!/usr/bin/env python3
"""Incremental arrow-key or WASD speed and steering control for the forklift."""
import argparse

from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from teleop_support import positive_parameter, run


HELP = """Ackermann drive
W/S or Up/Down: increase/decrease speed
A/D or Left/Right: increase/decrease steering
Each press changes one fifth of the configured limit; commands stay active.
Space: brake, keeping steering   Tab: center steering, keeping speed
q or Ctrl-C: stop, center steering, and exit.
"""


class AckermannTeleop(Node):
    exit_keys = {'q'}

    def __init__(self, max_speed, max_steering, topic):
        super().__init__('ackermann_drive_keyop_node')
        self.max_speed = positive_parameter(self, 'max_speed', max_speed)
        self.max_steering = positive_parameter(self, 'max_steering_angle', max_steering)
        self.publisher = self.create_publisher(AckermannDriveStamped, topic, 5)
        self.speed = self.steering = 0.0
        self.next_publish = 0.0

    def status(self):
        return f'Speed: {self.speed:.2f} m/s | Steering: {self.steering:.2f} rad'

    def on_key(self, key, now):
        if key in ('w', 'W', '\x1b[A'):
            self.speed = min(self.max_speed, self.speed + self.max_speed / 5)
        elif key in ('s', 'S', '\x1b[B'):
            self.speed = max(-self.max_speed, self.speed - self.max_speed / 5)
        elif key in ('a', 'A', '\x1b[D'):
            self.steering = min(self.max_steering, self.steering + self.max_steering / 5)
        elif key in ('d', 'D', '\x1b[C'):
            self.steering = max(-self.max_steering, self.steering - self.max_steering / 5)
        elif key == ' ':
            self.speed = 0.0
        elif key == '\t':
            self.steering = 0.0
        else:
            return False
        self.next_publish = now
        return True

    def update(self, dt, now):
        if now < self.next_publish:
            return
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.drive.speed = float(self.speed)
        message.drive.steering_angle = float(self.steering)
        self.publisher.publish(message)
        self.next_publish = now + 0.2

    def halt(self):
        self.speed = self.steering = 0.0
        self.next_publish = 0.0
        self.update(0.0, 0.0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('max_speed', nargs='?', type=float)
    parser.add_argument('max_steering_angle', nargs='?', type=float)
    parser.add_argument('topic', nargs='?', default='ackermann_cmd')
    args = parser.parse_args(remove_ros_args()[1:])
    speed = args.max_speed if args.max_speed is not None else 2.0
    steering = args.max_steering_angle
    if steering is None:
        steering = speed if args.max_speed is not None else 1.57
    run(lambda: AckermannTeleop(speed, steering, args.topic), HELP)
