"""Terminal handling shared by the three keyboard controllers."""
import math
import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from rclpy.signals import SignalHandlerOptions


def positive_parameter(node, name, default):
    value = node.declare_parameter(name, default).value
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def approach(current, target, step):
    return max(current - step, min(current + step, target))


class Keyboard:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.pending = ''

    def __enter__(self):
        self.settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)
        return self

    def __exit__(self, *_):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.settings)

    def read(self, timeout):
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if ready:
            data = os.read(self.fd, 4096)
            if not data:
                raise EOFError
            self.pending += data.decode('utf-8', errors='ignore')
        keys = []
        while self.pending:
            if self.pending[0] == '\x1b':
                if len(self.pending) < 2:
                    break
                if self.pending[1] in '[O':
                    if len(self.pending) < 3:
                        break
                    keys.append('\x1b[' + self.pending[2])
                    self.pending = self.pending[3:]
                    continue
            keys.append(self.pending[0])
            self.pending = self.pending[1:]
        return keys


def run(factory, help_text, ros_args=None):
    if not sys.stdin.isatty():
        raise SystemExit('An interactive terminal is required; use ./docker.sh exec flatland ...')
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_term = signal.getsignal(signal.SIGTERM)

    def interrupt(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        node = factory()
        print(help_text, flush=True)
        print(node.status(), flush=True)
        with Keyboard() as keyboard:
            previous = time.monotonic()
            while rclpy.ok():
                keys = keyboard.read(0.02)
                now = time.monotonic()
                for key in keys:
                    if key == '\x03' or key in node.exit_keys:
                        return
                    if node.on_key(key, now):
                        print('\r' + node.status() + '\x1b[K\r', end='', flush=True)
                node.update(min(now - previous, 0.1), now)
                previous = now
                rclpy.spin_once(node, timeout_sec=0)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.halt()
                # Allow the final stop to leave DDS before destroying the publisher.
                time.sleep(0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)
