#!/usr/bin/env python3
"""Integrate signed wheel RPM into planar odometry using constant-twist arcs.

Positive RPM means forward. Covariances are starting estimates requiring
calibration. Disable publish_tf when the EKF owns odom -> base_footprint."""

import math

import rclpy
from rclpy.node import Node


from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion


def yaw_to_quaternion(yaw):
    """2D yaw (rotation about Z only) -> geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdom(Node):

    def __init__(self):
        super().__init__('wheel_odom')

        # ------------------------------------------------------------
        # PARAMETERS
        #
        # wheel_radius/wheel_separation defaults match the same
        # measured values used in diff_drive_node.py -- keep these in
        # sync if you ever re-measure/re-tune the robot's geometry.
        # ------------------------------------------------------------
        self.declare_parameter('wheel_radius', 0.095)       # m
        self.declare_parameter('wheel_separation', 0.430)   # m

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')

        # If no /wheel_rpm arrives within this long, assume the robot
        # has stopped (serial_hub_node/firmware went away) rather
        # than integrating a stale nonzero velocity forever.
        self.declare_parameter('wheel_rpm_timeout', 0.5)

        # Whether this node broadcasts the odom -> base_footprint TF
        # itself. Set to false if robot_localization's ekf_node is
        # configured to publish that TF instead (the usual setup when
        # fusing with IMU) -- having both broadcast the same TF at
        # once causes TF conflicts.
        self.declare_parameter('publish_tf', True)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.wheel_rpm_timeout = self.get_parameter('wheel_rpm_timeout').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # ------------------------------------------------------------
        # STATE
        # ------------------------------------------------------------
        for value in (self.wheel_radius, self.wheel_separation, self.wheel_rpm_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Geometry and RPM timeout must be positive')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.wheel_positions = [0.0, 0.0]
        self.last_rpm_msg_time = self.get_clock().now()

        self.last_update_time = self.get_clock().now()

        # ------------------------------------------------------------
        # SUBSCRIBER
        # ------------------------------------------------------------
        self.create_subscription(
            Float32MultiArray, 'wheel_rpm', self._wheel_rpm_callback, 10)

        # ------------------------------------------------------------
        # PUBLISHER + TF BROADCASTER
        # ------------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # ------------------------------------------------------------
        # UPDATE LOOP
        # ------------------------------------------------------------
        # Matches the ~20 Hz rate wheel_rpm arrives at (see firmware's
        # WHEEL_PUBLISH_INTERVAL) -- no point integrating faster than
        # the input actually updates.
        self.create_timer(0.05, self._update_loop)

        self.get_logger().info(
            'wheel_odom starting. wheel_radius={} m, '
            'wheel_separation={} m, publish_tf={}'
            .format(self.wheel_radius, self.wheel_separation,
                    self.publish_tf)
        )

    # ==================================================================
    # CALLBACK
    # ==================================================================
    def _wheel_rpm_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn(
                'wheel_rpm expected [left, right], got {} values'
                .format(len(msg.data)))
            return

        if not all(math.isfinite(v) for v in msg.data):
            self.left_rpm = self.right_rpm = 0.0
            return
        self.left_rpm = float(msg.data[0])
        self.right_rpm = float(msg.data[1])
        self.last_rpm_msg_time = self.get_clock().now()

    # ==================================================================
    # UPDATE LOOP: INTEGRATE + PUBLISH
    # ==================================================================
    def _update_loop(self):
        now = self.get_clock().now()

        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        if dt <= 0.0 or dt > self.wheel_rpm_timeout:
            # Clock hasn't advanced (can happen right at startup) --
            # nothing meaningful to integrate this tick.
            return

        age = (now - self.last_rpm_msg_time).nanoseconds / 1e9

        if age < 0.0 or age > self.wheel_rpm_timeout:
            left_rpm = 0.0
            right_rpm = 0.0
        else:
            left_rpm = self.left_rpm
            right_rpm = self.right_rpm

        # RPM -> wheel linear speed, m/s.
        v_left = left_rpm * 2.0 * math.pi * self.wheel_radius / 60.0
        v_right = right_rpm * 2.0 * math.pi * self.wheel_radius / 60.0

        # Differential drive inverse kinematics.
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_separation

        # Exact constant-twist arc integration, stable near zero yaw rate.
        angle = omega * dt
        half_angle = angle / 2.0
        sinc = math.sin(half_angle) / half_angle if abs(half_angle) > 1e-9 else 1.0
        distance = v * dt * sinc
        self.x += distance * math.cos(self.theta + half_angle)
        self.y += distance * math.sin(self.theta + half_angle)
        self.theta = math.atan2(math.sin(self.theta + angle), math.cos(self.theta + angle))

        self._publish_odometry(now, v, omega)
        self._publish_joint_states(now, dt, left_rpm, right_rpm)

        if self.publish_tf:
            self._broadcast_tf(now)

    def _publish_joint_states(self, stamp, dt, left_rpm, right_rpm):
        """Integrate measured wheel speeds, including the odometry timeout."""
        velocities = [rpm * 2.0 * math.pi / 60.0 for rpm in (left_rpm, right_rpm)]
        self.wheel_positions = [
            position + velocity * dt
            for position, velocity in zip(self.wheel_positions, velocities)
        ]
        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.name = ['left_track_joint', 'right_track_joint']
        msg.position = self.wheel_positions
        msg.velocity = velocities
        self.joint_pub.publish(msg)

    # ==================================================================
    # PUBLISH: nav_msgs/Odometry
    # ==================================================================
    def _publish_odometry(self, stamp, v, omega):
        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self.theta)

        msg.twist.twist.linear.x = v
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = omega

        # Fixed placeholder covariances -- not zero (zero claims
        # perfect certainty, which wheel odometry never has), not
        # measured/tuned for this specific robot either. Order per
        # ROS convention: [x, y, z, rot_x, rot_y, rot_z], 6x6
        # row-major, only diagonal populated here.
        pose_cov = [0.0] * 36
        pose_cov[0] = 0.05    # x
        pose_cov[7] = 0.05    # y
        pose_cov[14] = 1e6    # z (unobservable for a ground robot)
        pose_cov[21] = 1e6    # rot_x (unobservable)
        pose_cov[28] = 1e6    # rot_y (unobservable)
        pose_cov[35] = 0.10   # rot_z (yaw)
        msg.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[0] = 0.05   # linear x
        twist_cov[7] = 1e6    # linear y (unobservable, no side-slip model)
        twist_cov[14] = 1e6   # linear z (unobservable)
        twist_cov[21] = 1e6   # angular x (unobservable)
        twist_cov[28] = 1e6   # angular y (unobservable)
        twist_cov[35] = 0.10  # angular z (yaw rate)
        msg.twist.covariance = twist_cov

        self.odom_pub.publish(msg)

    # ==================================================================
    # BROADCAST: odom -> base_footprint TF
    # ==================================================================
    def _broadcast_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quaternion(self.theta)

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()

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
