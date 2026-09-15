#!/usr/bin/env python3
"""Measure cloud freshness and simulation speed with the installed warehouse."""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--min-wall-hz', type=float, default=0,
                        help='Optional hardware-specific minimum cloud rate while moving')
    args = parser.parse_args()
    share = Path(get_package_share_directory('flatland'))
    model = yaml.safe_load((share/'robot/omni_drive_robot.model.yaml').read_text())
    lidar = next(p for p in model['plugins'] if p['type'] == 'MockLidar3D')
    expected_period = 1.0/lidar['update_rate']
    with tempfile.TemporaryFile(mode='w+') as logfile:
        process = subprocess.Popen(['ros2', 'launch', 'flatland', 'simulation.launch.py',
            'robot:=omni_drive_robot', 'show_viz:=false'], stdout=logfile, stderr=subprocess.STDOUT)
        rclpy.init()
        node = Node('lidar_cadence_check')
        records = []
        def receive(msg):
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            records.append((time.monotonic(), stamp, msg.width))
        subscription = node.create_subscription(PointCloud2, '/lidar_points', receive,
                                                qos_profile_sensor_data)
        publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        try:
            end = time.monotonic()+30
            while len(records) < 5 and time.monotonic() < end and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.01)
            if len(records) < 5:
                logfile.seek(0)
                raise AssertionError(logfile.read())
            for phase, duration in [('stationary', 3), ('moving', 8)]:
                records.clear()
                start = time.monotonic()
                last_command = 0
                while time.monotonic()-start < duration:
                    now = time.monotonic()
                    if now-last_command > .03:
                        message = Twist()
                        if phase == 'moving':
                            message.linear.x = .25*math.sin((now-start)*math.pi/2)
                        publisher.publish(message)
                        last_command = now
                    rclpy.spin_once(node, timeout_sec=.005)
                data = np.array(records)
                assert len(data) >= 5, 'Too few fresh scans'
                gaps = np.diff(data[:, 0])
                stamp_gaps = np.diff(data[:, 1])
                # Missed captures may skip periods, but scans must never be duplicated.
                assert np.all(stamp_gaps >= expected_period-1e-6)
                assert np.max(np.abs(stamp_gaps/expected_period-np.round(stamp_gaps/expected_period))) < 1e-5
                assert np.min(data[:, 2]) > 100
                wall = data[-1, 0]-data[0, 0]
                sim = data[-1, 1]-data[0, 1]
                metrics = dict(phase=phase, clouds=len(data), wall_hz=(len(data)-1)/wall,
                    sim_hz=(len(data)-1)/sim, rtf=sim/wall,
                    p95_gap_ms=float(np.percentile(gaps, 95)*1000),
                    max_gap_ms=float(gaps.max()*1000), points=int(np.median(data[:, 2])))
                print(json.dumps(metrics), flush=True)
                if phase == 'moving' and args.min_wall_hz:
                    assert metrics['wall_hz'] >= args.min_wall_hz, metrics
        finally:
            publisher.publish(Twist())
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
