#!/usr/bin/env python3
"""Exercise installed Flatland2 models, sensors, TF, motion and fork services."""
import argparse
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Empty, Trigger
from flatland_msgs.srv import Attach, DeleteModel, LiftFork, SpawnModel
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def run(args):
    with tempfile.TemporaryDirectory(prefix='flatland_smoke_') as temp:
        spawn_y = -5.0 if args.terrain else -2.0
        logfile = Path(temp)/'launch.log'
        with logfile.open('w') as output:
            process = subprocess.Popen(['ros2', 'launch', 'flatland', 'simulation.launch.py',
                f'robot:={args.robot}', f'drive_model:={args.drive_model}',
                f'robot_namespace:={args.namespace or "/"}', 'show_viz:=false', 'objects:=false',
                'x:=0.0', f'y:={spawn_y}', 'yaw:=0.4', f'terrain:={str(args.terrain).lower()}',
                f'localization:={"external" if args.external else "ground_truth"}',
                f'publish_map:={str(not args.external).lower()}'],
                stdout=output, stderr=subprocess.STDOUT)
        rclpy.init()
        node = Node('release_smoke')
        latest = {}
        prefix = '/' + args.namespace.strip('/') if args.namespace else ''
        frame_prefix = args.namespace.strip('/')+'_' if args.namespace else ''
        subscriptions = []
        for topic, kind in [('odom', Odometry), ('ground_truth/odom', Odometry),
                ('ground_truth/pose', PoseWithCovarianceStamped), ('scan', LaserScan),
                ('lidar_points', PointCloud2), ('imu/data', Imu), ('detections', PoseArray)]:
            subscriptions.append(node.create_subscription(kind, prefix+'/'+topic,
                lambda msg, key=topic: latest.__setitem__(key, msg), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(OccupancyGrid, '/map',
            lambda msg: latest.__setitem__('map', msg), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        buffer = Buffer()
        listener = TransformListener(buffer, node)

        def wait(predicate, timeout=45):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
                if process.poll() is not None:
                    raise RuntimeError('Launch exited early')
                if predicate():
                    return
            raise AssertionError('Timed out waiting for simulation condition')

        def call(kind, name, request):
            client = node.create_client(kind, name)
            assert client.wait_for_service(timeout_sec=15), name
            future = client.call_async(request)
            wait(future.done)
            response = future.result()
            node.destroy_client(client)
            if hasattr(response, 'success'):
                assert response.success, response.message
            return response

        try:
            required = {'odom', 'ground_truth/odom', 'ground_truth/pose', 'scan', 'lidar_points', 'imu/data', 'map'}
            if args.external:
                required.remove('map')
            wait(lambda: required <= latest.keys())
            if not args.external:
                assert latest['map'].info.resolution > 0 and latest['map'].info.width == 300
            assert latest['scan'].header.frame_id == frame_prefix+'laser'
            assert latest['imu/data'].header.frame_id == frame_prefix+'imu_link'
            assert latest['lidar_points'].width > 100
            assert any(math.isfinite(x) for x in latest['scan'].ranges)
            assert abs(latest['ground_truth/odom'].pose.pose.position.y-spawn_y) < .15
            assert abs(latest['odom'].pose.pose.position.x) < .15
            if args.external:
                wait(lambda: buffer.can_transform(frame_prefix+'odom', frame_prefix+'base_link', rclpy.time.Time()))
                assert not buffer.can_transform('map', frame_prefix+'base_link', rclpy.time.Time())
                assert 'map' not in latest
            else:
                wait(lambda: buffer.can_transform('map', frame_prefix+'base_link', rclpy.time.Time()))
            wait(lambda: buffer.can_transform(frame_prefix+'base_link', frame_prefix+'lidar3d', rclpy.time.Time()))
            sensor_tf = buffer.lookup_transform(frame_prefix+'base_link', frame_prefix+'lidar3d', rclpy.time.Time())
            assert sensor_tf.transform.translation.z > .5
            start = latest['ground_truth/pose'].pose.pose.position
            start_xy = (start.x, start.y)
            forklift = args.robot == 'forklift'
            kind = AckermannDriveStamped if forklift else Twist
            publisher = node.create_publisher(kind, prefix+('/ackermann_cmd' if forklift else '/cmd_vel'), 10)
            command = kind()
            if forklift:
                command.drive.speed = .35
            elif args.robot == 'omni_drive_robot':
                command.linear.y = .35
            else:
                command.linear.x = .35
            timer = node.create_timer(.05, lambda: publisher.publish(command))
            wait(lambda: math.hypot(latest['ground_truth/pose'].pose.pose.position.x-start_xy[0],
                                   latest['ground_truth/pose'].pose.pose.position.y-start_xy[1]) > .18)
            command = kind()
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.05)
                publisher.publish(command)
            odom = latest['odom'].pose.pose
            ground = latest['ground_truth/odom'].pose.pose
            predicted_x = math.cos(.4)*odom.position.x-math.sin(.4)*odom.position.y
            predicted_y = spawn_y+math.sin(.4)*odom.position.x+math.cos(.4)*odom.position.y
            assert math.hypot(ground.position.x-predicted_x, ground.position.y-predicted_y) < .08
            if args.robot == 'omni_drive_robot':
                assert odom.position.y > .15 and abs(odom.position.x) < .10
            if args.robot == 'castor_wheeled_differential_robot':
                print(f'Caster start transient: lateral displacement {odom.position.y:.3f} m, yaw {yaw(odom.orientation):.3f} rad')
                assert abs(odom.position.y) > .005 or abs(yaw(odom.orientation)) > .03
            if args.terrain and args.robot == 'turtlebot':
                assert abs(odom.position.y) > .001 or abs(yaw(odom.orientation)) > .005
            if args.spawn_probe:
                subscriptions.append(node.create_subscription(LaserScan, '/probe/scan',
                    lambda msg: latest.__setitem__('probe_scan', msg), qos_profile_sensor_data))
                spawn = SpawnModel.Request()
                spawn.name = 'probe'; spawn.ns = 'probe'
                spawn.yaml_path = str(Path(get_package_share_directory('flatland'))/'robot/turtlebot.model.yaml')
                spawn.pose.y = -6.0
                call(SpawnModel, '/spawn_model', spawn)
                wait(lambda: 'probe_scan' in latest)
                assert latest['probe_scan'].header.frame_id == 'probe_laser'
                wait(lambda: buffer.can_transform('map', 'probe_base_link', rclpy.time.Time()))
                delete = DeleteModel.Request(); delete.name = 'probe'
                call(DeleteModel, '/delete_model', delete)
            if forklift:
                for topic, msg in [('fork/height', Float32), ('fork/loaded', Bool), ('fork/attached_model', String)]:
                    subscriptions.append(node.create_subscription(msg, prefix+'/'+topic,
                        lambda value, key=topic: latest.__setitem__(key, value), 10))
                current = latest['ground_truth/pose'].pose.pose
                a = yaw(current.orientation)
                spawn = SpawnModel.Request()
                spawn.name = 'pallet_test'
                spawn.ns = 'pallet_test'
                spawn.yaml_path = str(Path(get_package_share_directory('flatland'))/'objects/pallet.model.yaml')
                spawn.pose.x = current.position.x-.90*math.cos(a)
                spawn.pose.y = current.position.y-.90*math.sin(a)
                spawn.pose.theta = a
                call(SpawnModel, '/spawn_model', spawn)
                wait(lambda: 'detections' in latest and len(latest['detections'].poses) > 0)
                attach = Attach.Request(); attach.model_name = 'pallet_test'
                call(Attach, prefix+'/fork/attach', attach)
                lift = LiftFork.Request(); lift.fraction = .3
                call(LiftFork, prefix+'/fork/lift', lift)
                wait(lambda: 'fork/height' in latest and latest['fork/height'].data > .85)
                wait(lambda: 'fork/loaded' in latest and latest['fork/loaded'].data)
                assert latest['fork/attached_model'].data == 'pallet_test'
                call(Trigger, prefix+'/fork/detach', Trigger.Request())
                wait(lambda: not latest['fork/loaded'].data)
                delete = DeleteModel.Request(); delete.name = 'pallet_test'
                call(DeleteModel, '/delete_model', delete)
            call(Empty, '/pause', Empty.Request())
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.05)
            paused_stamp = latest['odom'].header.stamp
            end = time.monotonic()+.3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.05)
            assert latest['odom'].header.stamp == paused_stamp
            call(Empty, '/resume', Empty.Request())
            wait(lambda: latest['odom'].header.stamp != paused_stamp)
            node.destroy_timer(timer)
            print(f'PASS: {args.robot}, drive={args.drive_model}, namespace={args.namespace or "root"}; sensors, map, TF, motion, odometry, services')
        except Exception:
            print(logfile.read_text()[-16000:])
            raise
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill(); process.wait()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='turtlebot')
    parser.add_argument('--drive-model', default='physics')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--terrain', action='store_true')
    parser.add_argument('--external', action='store_true')
    parser.add_argument('--spawn-probe', action='store_true')
    run(parser.parse_args())
