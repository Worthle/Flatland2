#!/usr/bin/env python3
"""Publish odom -> base TF; optionally anchor odom at the world spawn pose."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdometryTF(Node):
    def __init__(self):
        super().__init__('odometry_tf')
        for name, default in [('publish_map_tf', True), ('x', 0.0), ('y', -2.0), ('yaw', 0.0)]:
            self.declare_parameter(name, default)
        self.broadcaster = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        namespace = self.get_namespace().strip('/')
        self.odom_frame = f'{namespace}_odom' if namespace else 'odom'
        if self.get_parameter('publish_map_tf').value:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = self.odom_frame
            transform.transform.translation.x = self.get_parameter('x').value
            transform.transform.translation.y = self.get_parameter('y').value
            yaw = self.get_parameter('yaw').value
            transform.transform.rotation.z = math.sin(yaw/2)
            transform.transform.rotation.w = math.cos(yaw/2)
            self.static.sendTransform(transform)
        self.subscription = self.create_subscription(Odometry, 'odom', self.publish_tf, 10)

    def publish_tf(self, odom):
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.rotation = odom.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdometryTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
