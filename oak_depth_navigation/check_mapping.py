"""Check live sensor, odometry, cloud and occupancy-grid output with a deadline."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener


class MappingCheck(Node):
    def __init__(self, base_frame, odom_topic='/oak/odom'):
        super().__init__('oak_mapping_check')
        self.counts = Counter()
        self.received_at = {}
        self.details = {}
        self.first_pose = None
        self.base_frame = base_frame
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.subscriptions_list = []
        for label, topic, kind in [
            ('rgb', '/oak/rgb/image_rect', Image),
            ('depth', '/oak/stereo/image_raw', Image),
            ('camera_info', '/oak/rgb/camera_info', CameraInfo),
            ('imu_raw', '/oak/imu/data', Imu),
            ('imu_filtered', '/oak/imu/filtered', Imu),
            ('odom', odom_topic, Odometry),
            ('cloud', '/oak/points', PointCloud2),
        ]:
            self.subscriptions_list.append(self.create_subscription(
                kind, topic, lambda msg, label=label: self.receive(label, msg), qos_profile_sensor_data))
        self.subscriptions_list.append(self.create_subscription(OccupancyGrid, '/map',
            lambda msg: self.receive('map', msg), QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)))

    def receive(self, label, msg):
        self.counts[label] += 1
        self.received_at[label] = time.monotonic()
        if label in ('rgb', 'depth'):
            self.details[label] = {'width': msg.width, 'height': msg.height,
                                  'encoding': msg.encoding, 'frame': msg.header.frame_id}
            if label == 'depth' and msg.encoding in ('16UC1', '32FC1'):
                dtype = np.dtype('u2' if msg.encoding == '16UC1' else 'f4')
                dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
                a = np.frombuffer(bytes(msg.data), dtype=dtype)
                self.details[label]['valid_fraction'] = float(np.mean(np.isfinite(a) & (a > 0)))
        elif label == 'camera_info':
            self.details[label] = {'fx': msg.k[0], 'fy': msg.k[4], 'frame': msg.header.frame_id}
        elif label.startswith('imu'):
            q = msg.orientation
            self.details[label] = {'frame': msg.header.frame_id,
                'orientation_norm': math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w),
                'orientation_available': bool(msg.orientation_covariance[0] >= 0)}
        elif label == 'odom':
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            current = (p.x, p.y, p.z)
            if self.first_pose is None:
                self.first_pose = current
            self.details[label] = {'position': current, 'frame': msg.header.frame_id,
                'child_frame': msg.child_frame_id,
                'max_displacement_m': max(self.details.get(label, {}).get('max_displacement_m', 0),
                    math.dist(current, self.first_pose)),
                'valid': bool(all(math.isfinite(x) for x in current) and
                         abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) < 0.01 and
                         0 <= msg.pose.covariance[0] < 1000)}
        elif label == 'cloud':
            self.details[label] = {'points': msg.width * msg.height, 'frame': msg.header.frame_id}
        elif label == 'map':
            a = np.array(msg.data)
            self.details[label] = {'width': msg.info.width, 'height': msg.info.height,
                'resolution': msg.info.resolution, 'known_cells': int(np.sum(a >= 0)),
                'occupied_cells': int(np.sum(a > 50)), 'free_cells': int(np.sum((a >= 0) & (a < 50)))}

    def report(self):
        required = ('rgb', 'depth', 'camera_info', 'imu_raw', 'imu_filtered', 'odom', 'cloud', 'map')
        # The occupancy grid is latched and need not republish while stationary.
        checks = {label: self.counts[label] >= (1 if label == 'map' else 2)
                  for label in required}
        checks['sensor_and_odom_streams_fresh'] = all(
            time.monotonic() - self.received_at.get(label, 0) < 2.0
            for label in required if label != 'map')
        checks['valid_depth'] = self.details.get('depth', {}).get('valid_fraction', 0) > 0.02
        checks['valid_odom'] = self.details.get('odom', {}).get('valid', False)
        checks['map_has_obstacles'] = self.details.get('map', {}).get('occupied_cells', 0) > 10
        checks['map_has_free_space'] = self.details.get('map', {}).get('free_cells', 0) > 10
        checks['cloud_has_points'] = self.details.get('cloud', {}).get('points', 0) > 10
        checks['map_to_base_tf'] = bool(self.tf.can_transform('map', self.base_frame, rclpy.time.Time()))
        return {'passed': all(checks.values()), 'checks': checks,
                'messages': dict(self.counts), 'details': self.details}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--base-frame', default='oak_mount')
    parser.add_argument('--odom-topic', default='/oak/odom')
    parser.add_argument('--output', help='New report path (will not overwrite)')
    args = parser.parse_args()
    rclpy.init()
    node = MappingCheck(args.base_frame, args.odom_topic)
    end = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = node.report()
        result = json.dumps(report, indent=2)
        print(result)
        if args.output:
            with Path(args.output).expanduser().open('x') as stream:
                stream.write(result + '\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)
