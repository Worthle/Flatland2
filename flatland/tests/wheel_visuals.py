#!/usr/bin/env python3
"""Check cylindrical tire geometry and signed, radius-based rolling in ROS2."""
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
import yaml


def main():
    share = Path(get_package_share_directory('flatland'))
    model = yaml.safe_load((share/'robot/turtlebot.model.yaml').read_text())
    wheels = {body['name']: body for body in model['bodies'] if 'wheel_visual' in body}
    with tempfile.TemporaryDirectory() as directory:
        logfile = Path(directory)/'launch.log'
        with logfile.open('w') as output:
            process = subprocess.Popen(['ros2', 'launch', 'flatland', 'simulation.launch.py',
                'robot:=turtlebot', 'show_viz:=false', 'objects:=false'],
                stdout=output, stderr=subprocess.STDOUT)
        rclpy.init()
        node = Node('wheel_visual_check')
        latest = {}
        def receive(message):
            latest.clear()
            latest.update({marker.ns: marker for marker in message.markers})
        subscription = node.create_subscription(MarkerArray,
            '/flatland_server/debug/model/robot', receive, 1)
        publisher = node.create_publisher(Twist, '/cmd_vel', 1)
        command = Twist()
        timer = node.create_timer(.05, lambda: publisher.publish(command))
        def wait(predicate, timeout=20):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
                if process.poll() is not None:
                    raise AssertionError(logfile.read_text())
                if predicate():
                    return
            raise AssertionError('Timed out waiting for wheel animation')
        def spin_angle(marker):
            # Undo the axle's -pi/2 roll; straight travel has zero body yaw.
            q = marker.pose.orientation
            z = (q.z+q.y)/math.sqrt(2)
            w = (q.w-q.x)/math.sqrt(2)
            return 2*math.atan2(z, w)
        try:
            wait(lambda: all('wheel/'+name+'/tire' in latest for name in wheels))
            for name, body in wheels.items():
                marker = latest['wheel/'+name+'/tire']
                geometry = body['wheel_visual']
                assert marker.type == Marker.CYLINDER
                assert abs(marker.scale.x-2*geometry['radius']) < 1e-6
                assert abs(marker.scale.y-marker.scale.x) < 1e-6
                assert abs(marker.scale.z-geometry['width']) < 1e-6
                expected_z = body.get('elevation', 0)+body.get('visual_z_offset', 0)+geometry['radius']
                assert abs(marker.pose.position.z-expected_z) < 1e-6
                for part in ['rim', 'hub', 'spokes', 'tread']:
                    assert 'wheel/'+name+'/'+part in latest
            key = 'wheel/left_wheel/tire'
            radius = wheels['left_wheel']['wheel_visual']['radius']
            for speed in [.2, -.2]:
                initial = latest[key]
                command.linear.x = speed
                wait(lambda: (latest[key].pose.position.x-initial.pose.position.x)*math.copysign(1, speed) > .12)
                current = latest[key]
                distance = current.pose.position.x-initial.pose.position.x
                actual = math.remainder(spin_angle(current)-spin_angle(initial), 2*math.pi)
                expected = math.remainder(distance/radius, 2*math.pi)
                assert abs(math.remainder(actual-expected, 2*math.pi)) < .03, (actual, expected)
            command.linear.x = 0.0
            publisher.publish(command)
            print('PASS: tire/rim/spoke geometry, floor clearance, forward and reverse rolling by distance/radius')
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
