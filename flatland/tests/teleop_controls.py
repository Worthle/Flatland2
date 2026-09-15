#!/usr/bin/env python3
"""Exercise the installed teleops through a pseudo-terminal and ROS topics."""
import math
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import tempfile
import termios
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_prefix


class Terminal:
    def __init__(self, script, args=()):
        self.master, self.slave = pty.openpty()
        self.settings = termios.tcgetattr(self.slave)
        self.log = tempfile.TemporaryFile(mode='w+')
        executable = Path(get_package_prefix('flatland'))/'lib/flatland'/script
        self.process = subprocess.Popen([sys.executable, str(executable), *args],
            stdin=self.slave, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def send(self, key):
        os.write(self.master, key.encode())

    def diagnostics(self):
        self.log.seek(0)
        return self.log.read()

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        assert termios.tcgetattr(self.slave) == self.settings, 'Terminal was not restored'
        os.close(self.master)
        os.close(self.slave)
        self.log.close()


def main():
    rclpy.init()
    node = Node('teleop_controls_check')
    latest = {}
    subscriptions = [node.create_subscription(kind, '/keys/'+topic,
        lambda msg, key=topic: latest.__setitem__(key, (time.monotonic(), msg)), 10)
        for kind, topic in [(Twist, 'cmd_vel'), (AckermannDriveStamped, 'ackermann_cmd')]]

    def wait(predicate, terminal, timeout=5, allow_exit=False):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)
            if predicate():
                return
            if not allow_exit and terminal.process.poll() is not None:
                raise AssertionError(terminal.diagnostics())
        raise AssertionError('Timed out: '+terminal.diagnostics())

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def hold(terminal, key, seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            terminal.send(key)
            spin(.08)

    def twist():
        msg = latest['cmd_vel'][1]
        return msg.linear.x, msg.linear.y, msg.angular.z

    def stopped():
        return max(abs(v) for v in twist()) < 1e-6

    def stop(terminal, key=' '):
        sent = time.monotonic()
        terminal.send(key)
        wait(lambda: latest['cmd_vel'][0] > sent and stopped(), terminal)

    def finish(terminal, topic, key='\x03'):
        sent = time.monotonic()
        if key is None:
            terminal.process.send_signal(signal.SIGTERM)
        else:
            terminal.send(key)
        def is_zero():
            if topic == 'cmd_vel':
                return stopped()
            msg = latest[topic][1].drive
            return abs(msg.speed)+abs(msg.steering_angle) < 1e-6
        wait(lambda: latest[topic][0] > sent and is_zero(), terminal, allow_exit=True)
        terminal.process.wait(timeout=5)
        assert terminal.process.returncode == 0, terminal.diagnostics()

    try:
        cases = [
            ('diff_teleop.py', ['keys'], {
                'w': (1, 0, 0), 'x': (-1, 0, 0), 'a': (0, 0, 1), 'd': (0, 0, -1),
                'q': (1, 0, 1), 'e': (1, 0, -1), 'z': (-1, 0, -1), 'c': (-1, 0, 1)}),
            ('omni_teleop.py', ['--ros-args', '-r', '__ns:=/keys'], {
                'w': (1, 0, 0), 'x': (-1, 0, 0), 'a': (0, 1, 0), 'd': (0, -1, 0),
                'q': (1, 1, 0), 'e': (1, -1, 0), 'z': (-1, 1, 0), 'c': (-1, -1, 0),
                'j': (0, 0, 1), 'l': (0, 0, -1)})]
        for script, args, bindings in cases:
            latest.clear()
            terminal = Terminal(script, args)
            try:
                wait(lambda: 'cmd_vel' in latest, terminal)
                for key, signs in bindings.items():
                    stop(terminal)
                    hold(terminal, key, .35)
                    actual = twist()
                    for value, sign in zip(actual, signs):
                        assert (abs(value) < 1e-6 if sign == 0 else value*sign > .04), (script, key, actual)
                stop(terminal, 's')
                # A single press survives the desktop's initial repeat delay.
                terminal.send('w')
                spin(.55)
                assert twist()[0] > .08, twist()
                wait(stopped, terminal, timeout=3)
                # Speed changes affect the held command without resetting it.
                hold(terminal, 'w', 1.6)
                before = twist()[0]
                terminal.send('t')
                hold(terminal, 'w', .4)
                assert math.isclose(twist()[0], before*1.1, abs_tol=.01)
                if script == 'omni_teleop.py':
                    stop(terminal)
                    hold(terminal, 'j', .4)
                    terminal.send('k')
                    wait(stopped, terminal)
                else:
                    stop(terminal)
                    hold(terminal, 'a', .4)
                    before = twist()[2]
                    terminal.send('u')
                    hold(terminal, 'a', .3)
                    assert math.isclose(twist()[2], before*1.1, abs_tol=.01)
                hold(terminal, 'w', .3)
                finish(terminal, 'cmd_vel', None if script == 'diff_teleop.py' else '\x03')
                print(f'PASS: {script}, reference keys, ramps, timeout, speed controls, final stop and namespace')
            finally:
                terminal.close()

        latest.clear()
        terminal = Terminal('ackermann_teleop.py', ['1.0', '0.5', '--ros-args', '-r', '__ns:=/keys'])
        try:
            wait(lambda: 'ackermann_cmd' in latest, terminal)
            def ack(speed, steer):
                msg = latest['ackermann_cmd'][1].drive
                return math.isclose(msg.speed, speed, abs_tol=1e-6) and math.isclose(msg.steering_angle, steer, abs_tol=1e-6)
            # Arrow escape sequences may arrive split across terminal reads.
            for part in ['\x1b', '[', 'A']:
                terminal.send(part)
                spin(.04)
            wait(lambda: ack(.2, 0), terminal)
            spin(.85)
            assert ack(.2, 0), 'Ackermann speed should persist without repeated keys'
            terminal.send('\x1b[D')
            wait(lambda: ack(.2, .1), terminal)
            terminal.send(' ')
            wait(lambda: ack(0, .1), terminal)
            terminal.send('\t')
            wait(lambda: ack(0, 0), terminal)
            terminal.send('\x1b[B\x1b[C')
            wait(lambda: ack(-.2, -.1), terminal)
            terminal.send('\x1b[A'*10+'\x1b[D'*10)
            wait(lambda: ack(1.0, .5), terminal)
            terminal.send('\t')
            wait(lambda: ack(1.0, 0), terminal)
            terminal.send(' ')
            wait(lambda: ack(0, 0), terminal)
            terminal.send('wa')
            wait(lambda: ack(.2, .1), terminal)
            terminal.send('sd')
            wait(lambda: ack(0, 0), terminal)
            terminal.send('SD')
            wait(lambda: ack(-.2, -.1), terminal)
            terminal.send('WA')
            wait(lambda: ack(0, 0), terminal)
            finish(terminal, 'ackermann_cmd', 'q')
            print('PASS: ackermann_teleop.py, arrows/WASD, persistent speed, limits, Space/Tab, namespace and final stop')
        finally:
            terminal.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
