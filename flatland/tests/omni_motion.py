#!/usr/bin/env python3
"""Check keyboard-driven omni strafing, turret holds, and passive caster visuals."""
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

from teleop_controls import Terminal


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    with tempfile.TemporaryDirectory() as directory:
        log = Path(directory)/'simulation.log'
        with log.open('w') as stream:
            process = subprocess.Popen(['ros2', 'launch', 'flatland', 'simulation.launch.py',
                'robot:=omni_drive_robot', 'show_viz:=false', 'objects:=false', 'yaw:=0.4'],
                stdout=stream, stderr=subprocess.STDOUT)
        rclpy.init()
        node = Node('omni_motion_check')
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        latest = {}
        subscriptions = [node.create_subscription(Odometry, '/odom',
            lambda msg: latest.__setitem__('odom', msg), 10),
            node.create_subscription(MarkerArray, '/flatland_server/debug/model/robot',
            lambda msg: latest.__setitem__('markers', {m.ns: m for m in msg.markers}), 1)]
        terminal = None

        def wait(predicate, timeout=20):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.01)
                if process.poll() is not None:
                    raise AssertionError(log.read_text())
                if terminal and terminal.process.poll() is not None:
                    raise AssertionError(terminal.diagnostics())
                if predicate():
                    return
            raise AssertionError('Timed out waiting for omni motion\n'+log.read_text()[-4000:])

        def spin(seconds, key=None):
            end = time.monotonic()+seconds
            next_key = 0
            while time.monotonic() < end:
                if key and time.monotonic() >= next_key:
                    terminal.send(key)
                    next_key = time.monotonic()+.08
                rclpy.spin_once(node, timeout_sec=.01)

        def tf(name):
            return buffer.lookup_transform('base_link', name, rclpy.time.Time()).transform

        def stop():
            terminal.send('s')
            wait(lambda: abs(latest['odom'].twist.twist.linear.x)+
                 abs(latest['odom'].twist.twist.linear.y)+
                 abs(latest['odom'].twist.twist.angular.z) < .01)
            spin(.15)

        def pose():
            p = latest['odom'].pose.pose
            return p.position.x, p.position.y, yaw(p.orientation)

        try:
            casters = {'front_right_caster': (.55, -.25), 'rear_left_caster': (-.55, .25)}
            wheels = ['turret1', 'turret2', *casters]
            wait(lambda: 'odom' in latest and 'markers' in latest and
                 all('wheel/'+name+'/tire' in latest['markers'] for name in wheels))
            wait(lambda: all(buffer.can_transform('base_link', name, rclpy.time.Time()) for name in wheels))
            for name, (x, y) in casters.items():
                transform = tf(name)
                assert math.hypot(transform.translation.x-x, transform.translation.y-y) < .005
                tire = latest['markers']['wheel/'+name+'/tire']
                assert tire.type == Marker.CYLINDER
                assert abs(tire.scale.x-.24) < 1e-6 and abs(tire.scale.z-.09) < 1e-6
                assert abs(tire.pose.position.z-.12) < 1e-6
                assert 'wheel/'+name+'/spokes' in latest['markers']
            terminal = Terminal('omni_teleop.py')
            wait(lambda: node.count_publishers('/cmd_vel') == 1)
            spin(.2)
            for key, expected in [('a', (0, 1)), ('d', (0, -1)), ('w', (1, 0)), ('x', (-1, 0))]:
                stop()
                start = pose()
                spin(1.4, key)
                current = pose()
                dx, dy = current[0]-start[0], current[1]-start[1]
                forward = math.cos(start[2])*dx+math.sin(start[2])*dy
                lateral = -math.sin(start[2])*dx+math.cos(start[2])*dy
                travel = forward*expected[0]+lateral*expected[1]
                across = forward*expected[1]-lateral*expected[0]
                assert travel > .18 and abs(across) < .04, (key, forward, lateral)
                assert abs(math.remainder(current[2]-start[2], 2*math.pi)) < .04
                target = math.atan2(-expected[1], -expected[0])
                for name in casters:
                    assert abs(math.remainder(yaw(tf(name).rotation)-target, 2*math.pi)) < .15, (key, name, yaw(tf(name).rotation))
                steering = [yaw(tf(name).rotation) for name in ['turret1', 'turret2']]
                stop()
                spin(.7)
                for name, angle in zip(['turret1', 'turret2'], steering):
                    assert abs(math.remainder(yaw(tf(name).rotation)-angle, 2*math.pi)) < .02
                print(f'PASS: key {key}, body travel {travel:.3f} m, straight heading, caster alignment and steering hold')
            for key, sign in [('j', 1), ('l', -1)]:
                stop()
                start = pose()
                spin(1.2, key)
                current = pose()
                assert math.remainder(current[2]-start[2], 2*math.pi)*sign > .2
                assert math.hypot(current[0]-start[0], current[1]-start[1]) < .05
                print(f'PASS: key {key}, rotation about the body center')
            stop()
            print('PASS: four cylindrical wheels, opposite-corner caster pivots and keyboard-driven omni motion')
        finally:
            if terminal:
                terminal.close()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
