#!/usr/bin/env python3
"""Body-frame omni keyboard control; the launch adapter commands the turrets."""
from geometry_msgs.msg import Twist
from rclpy.node import Node

from teleop_support import approach, positive_parameter, run


HELP = """Omni drive
   q  w  e       forward-left / forward / forward-right
   a  s  d       strafe left / stop / strafe right
   z  x  c       backward-left / backward / backward-right
j/l: rotate left/right   k: stop rotation
t/g: both speeds +/-10%   y/h: linear +/-10%   u/n: angular +/-10%
Space or s: stop immediately. Release movement keys to slow down.
Ctrl-C: stop and exit.
"""
MOVEMENT = {'w': (1, 0, 0), 'x': (-1, 0, 0), 'a': (0, 1, 0), 'd': (0, -1, 0),
            'q': (1, 1, 0), 'e': (1, -1, 0), 'z': (-1, 1, 0), 'c': (-1, -1, 0),
            'j': (0, 0, 1), 'l': (0, 0, -1)}
SPEED = {'t': (1.1, 1.1), 'g': (0.9, 0.9), 'y': (1.1, 1),
         'h': (0.9, 1), 'u': (1, 1.1), 'n': (1, 0.9)}


class OmniTeleop(Node):
    exit_keys = set()

    def __init__(self):
        super().__init__('omni_teleop')
        self.speed = positive_parameter(self, 'linear_speed', 0.3)
        self.turn = positive_parameter(self, 'angular_speed', 0.5)
        self.timeout = positive_parameter(self, 'key_timeout', 0.75)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 5)
        self.direction = (0, 0, 0)
        self.current = [0.0, 0.0, 0.0]
        self.last_key = float('-inf')

    def status(self):
        return f'Linear: {self.speed:.2f} m/s | Angular: {self.turn:.2f} rad/s'

    def on_key(self, key, now):
        if key in (' ', 's'):
            self.halt()
        elif key == 'k':
            self.direction = (*self.direction[:2], 0)
            self.current[2] = 0.0
            self.last_key = now
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
            self.direction = (0, 0, 0)
        targets = [scale * direction for scale, direction in
                   zip((self.speed, self.speed, self.turn), self.direction)]
        self.current = [approach(value, target, rate * dt)
                        for value, target, rate in zip(self.current, targets, (0.5, 0.5, 1.0))]
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = self.current
        self.publisher.publish(message)

    def halt(self):
        self.direction = (0, 0, 0)
        self.current = [0.0, 0.0, 0.0]
        self.publisher.publish(Twist())


if __name__ == '__main__':
    run(OmniTeleop, HELP)
