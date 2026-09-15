#!/usr/bin/env python3
"""Differential-drive keyboard control for TurtleBot and the caster robot."""
import argparse

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from teleop_support import approach, positive_parameter, run


HELP = """Differential drive
   q  w  e       forward arcs / forward
   a  s  d       rotate left / stop / rotate right
   z  x  c       reverse arcs / reverse
t/g: both speeds +/-10%   y/h: linear +/-10%   u/j: angular +/-10%
Space or s: stop immediately. Release movement keys to slow down.
Ctrl-C: stop and exit.
"""
MOVEMENT = {'w': (1, 0), 'x': (-1, 0), 'a': (0, 1), 'd': (0, -1),
            'q': (1, 1), 'e': (1, -1), 'z': (-1, -1), 'c': (-1, 1)}
SPEED = {'t': (1.1, 1.1), 'g': (0.9, 0.9), 'y': (1.1, 1),
         'h': (0.9, 1), 'u': (1, 1.1), 'j': (1, 0.9)}


class DifferentialTeleop(Node):
    exit_keys = set()

    def __init__(self, namespace=''):
        super().__init__('diff_teleop', namespace=namespace)
        self.speed = positive_parameter(self, 'linear_speed', 0.3)
        self.turn = positive_parameter(self, 'angular_speed', 0.2)
        self.timeout = positive_parameter(self, 'key_timeout', 0.75)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 5)
        self.direction = (0, 0)
        self.current = [0.0, 0.0]
        self.last_key = float('-inf')

    def status(self):
        return f'Linear: {self.speed:.2f} m/s | Angular: {self.turn:.2f} rad/s'

    def on_key(self, key, now):
        if key in (' ', 's'):
            self.halt()
        elif key in MOVEMENT:
            self.direction = MOVEMENT[key]
            self.last_key = now
        elif key in SPEED:
            self.speed *= SPEED[key][0]
            self.turn *= SPEED[key][1]
            self.last_key = now
            return True
        return False

    def update(self, dt, now):
        if now - self.last_key > self.timeout:
            self.direction = (0, 0)
        targets = (self.speed * self.direction[0], self.turn * self.direction[1])
        self.current = [approach(value, target, rate * dt)
                        for value, target, rate in zip(self.current, targets, (0.2, 1.0))]
        message = Twist()
        message.linear.x, message.angular.z = self.current
        self.publisher.publish(message)

    def halt(self):
        self.direction = (0, 0)
        self.current = [0.0, 0.0]
        self.publisher.publish(Twist())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('namespace', nargs='?', default='')
    args = parser.parse_args(remove_ros_args()[1:])
    run(lambda: DifferentialTeleop(args.namespace), HELP)
