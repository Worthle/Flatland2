#!/usr/bin/env python3
"""Exercise world loading, drive motion, sensing, and pause/resume."""
import math
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_srvs.srv import Empty
from rclpy.qos import qos_profile_sensor_data


def main():
    world = Path(__file__).resolve().parent/'fixtures/world.yaml'
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'flatland_server', 'flatland_server', '--ros-args',
            '-p', f'world_path:={world}', '-p', 'update_rate:=100.0',
            '-p', 'step_size:=0.01', '-p', 'show_viz:=false'],
            stdout=log, stderr=subprocess.STDOUT)
        rclpy.init()
        node = rclpy.create_node('core_smoke')
        latest = {}
        subscriptions = [node.create_subscription(kind, topic,
            lambda message, key=topic: latest.__setitem__(key, message),
            qos_profile_sensor_data)
            for topic, kind in [('/odom', Odometry), ('/scan', LaserScan),
                                ('/lidar_points', PointCloud2)]]
        publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        command = Twist()
        timer = node.create_timer(.05, lambda: publisher.publish(command))

        def wait(predicate, timeout=20):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                assert process.poll() is None, 'Server exited early'
                rclpy.spin_once(node, timeout_sec=.05)
                if predicate():
                    return
            raise AssertionError('Timed out waiting for core simulator')

        def call(name):
            client = node.create_client(Empty, name)
            assert client.wait_for_service(timeout_sec=5), name
            future = client.call_async(Empty.Request())
            wait(future.done)
            assert future.result() is not None, name
            node.destroy_client(client)

        try:
            wait(lambda: all(topic in latest for topic in ('/odom', '/scan', '/lidar_points')))
            cloud = latest['/lidar_points']
            z_offset = next(field.offset for field in cloud.fields if field.name == 'z')
            ceiling = [struct.unpack_from('<f', cloud.data, offset + z_offset)[0]
                       for offset in range(0, len(cloud.data), cloud.point_step)]
            assert sum(abs(z - 2.0) < 1e-6 for z in ceiling) == 12, ceiling
            ranges = [value for value in latest['/scan'].ranges if math.isfinite(value)]
            assert ranges and 2.8 < min(ranges) < 3.2, ranges
            assert latest['/scan'].header.frame_id == 'laser'
            command.linear.x = .3
            wait(lambda: latest['/odom'].pose.pose.position.x > .2)
            assert abs(latest['/odom'].pose.pose.position.y) < .05
            command = Twist()
            publisher.publish(command)
            call('/pause')
            # Drain messages captured before the pause response.
            end = time.monotonic()+.3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            paused = latest['/odom'].header.stamp
            end = time.monotonic()+.2
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            assert latest['/odom'].header.stamp == paused
            call('/resume')
            wait(lambda: latest['/odom'].header.stamp != paused)
            print('PASS: world loading, drive motion, laser and ceiling geometry, odometry and pause/resume')
        except Exception:
            log.seek(0)
            print(log.read()[-6000:])
            raise
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_timer(timer)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
