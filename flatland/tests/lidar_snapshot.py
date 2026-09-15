#!/usr/bin/env python3
"""Check asynchronous cloud timestamps, world coordinates, and model deletion."""
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
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from flatland_msgs.srv import DeleteModel
from ament_index_python.packages import get_package_share_directory
import yaml


def stamp(message):
    return message.header.stamp.sec*1000000000+message.header.stamp.nanosec


def main():
    share = Path(get_package_share_directory('flatland'))
    beacons = np.array([[3, 1, .6], [-3, 1, .6], [0, 4, .6], [0, -6, .6]], dtype=np.float32)
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        pcd = directory/'beacons.pcd'
        header = 'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 4\nHEIGHT 1\nPOINTS 4\nDATA binary\n'
        pcd.write_bytes(header.encode()+beacons.astype('<f4').tobytes())
        model = yaml.safe_load((share/'robot/omni_drive_robot.model.yaml').read_text())
        drive = next(p for p in model['plugins'] if p['type'] == 'OmniDrive')
        drive['pub_rate'] = 100
        lidar = next(p for p in model['plugins'] if p['type'] == 'MockLidar3D')
        lidar.update(pcd_path=str(pcd), range_noise_std_dev=0.0, elevations_deg=[0.0],
                     origin=[.2, -.1, .3], update_rate=20)
        model['plugins'] = [drive, lidar]
        model_path = directory/'robot.yaml'
        model_path.write_text(yaml.safe_dump(model))
        world = yaml.safe_load((share/'maps/warehouse/warehouse.world.yaml').read_text())
        world['layers'][0]['map'] = str(share/'maps/warehouse/warehouse.yaml')
        world['models'] = [dict(name='robot', namespace='', model=str(model_path), pose=[0, -2, .4])]
        world_path = directory/'world.yaml'
        world_path.write_text(yaml.safe_dump(world))
        with tempfile.TemporaryFile(mode='w+') as logfile:
            server = subprocess.Popen(['ros2', 'run', 'flatland_server', 'flatland_server',
                '--ros-args', '-p', f'world_path:={world_path}', '-p', 'update_rate:=100.0',
                '-p', 'step_size:=0.01'], stdout=logfile, stderr=subprocess.STDOUT)
            adapter = subprocess.Popen(['ros2', 'run', 'flatland', 'omni_command.py'],
                                       stdout=logfile, stderr=subprocess.STDOUT)
            rclpy.init()
            node = Node('lidar_snapshot_check')
            poses, clouds = {}, []
            subscriptions = [node.create_subscription(Odometry, '/ground_truth/odom',
                lambda msg: poses.__setitem__(stamp(msg), msg.pose.pose), 100),
                node.create_subscription(PointCloud2, '/lidar_points', clouds.append, qos_profile_sensor_data)]
            publisher = node.create_publisher(Twist, '/cmd_vel', 10)
            command = Twist()
            command.linear.x = .25
            command.angular.z = .12
            timer = node.create_timer(.03, lambda: publisher.publish(command))
            try:
                end = time.monotonic()+15
                checked = 0
                delayed = 0
                while checked < 60 and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=.005)
                    while clouds and stamp(clouds[0]) in poses:
                        cloud = clouds.pop(0)
                        captured = stamp(cloud)
                        pose = poses[captured]
                        q = pose.orientation
                        angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                        sensor_x = pose.position.x+math.cos(angle)*.2-math.sin(angle)*(-.1)
                        sensor_y = pose.position.y+math.sin(angle)*.2+math.cos(angle)*(-.1)
                        sensor_yaw = angle+.3
                        points = np.ndarray((cloud.width, 4), dtype='<f4', buffer=cloud.data)[:, :3]
                        world_points = np.column_stack((
                            sensor_x+math.cos(sensor_yaw)*points[:, 0]-math.sin(sensor_yaw)*points[:, 1],
                            sensor_y+math.sin(sensor_yaw)*points[:, 0]+math.cos(sensor_yaw)*points[:, 1],
                            .6+points[:, 2]))
                        error = np.linalg.norm(world_points[:, None, :]-beacons[None, :, :], axis=2).min(axis=1)
                        assert len(points) == 4 and float(error.max()) < 3e-6, (captured, error)
                        delayed += max(poses) > captured
                        checked += 1
                assert checked >= 60 and delayed > 0, (checked, delayed)
                # Delete the live sensor while its asynchronous captures are active.
                delete = node.create_client(DeleteModel, '/delete_model')
                assert delete.wait_for_service(timeout_sec=5)
                request = DeleteModel.Request()
                request.name = 'robot'
                future = delete.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=5)
                assert future.done() and future.result().success
                time.sleep(.1)
                assert server.poll() is None
                print(f'PASS: {checked} moving clouds match capture-time poses and world beacons; async sensor deletion completed')
            except Exception:
                logfile.seek(0)
                print(logfile.read()[-5000:])
                raise
            finally:
                command = Twist()
                publisher.publish(command)
                for process in [adapter, server]:
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
