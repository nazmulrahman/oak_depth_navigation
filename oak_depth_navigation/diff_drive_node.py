#!/usr/bin/env python3
"""Convert Twist commands to bounded wheel PWM with kick-start and a steady-time watchdog.

Negative PWM is forward for the existing robot wiring. Saturation scales both
wheels together to preserve curvature before applying the low-speed kick."""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class DiffDrive(Node):

    def __init__(self):
        super().__init__('diff_drive')

        # ------------------------------------------------------------
        # PARAMETERS (unchanged values/meaning from the old combined node)
        # ------------------------------------------------------------
        self.declare_parameter('wheel_radius', 0.095)       # m
        self.declare_parameter('wheel_separation', 0.430)   # m
        self.declare_parameter('max_wheel_rpm', 94.5)        # rpm
        self.declare_parameter('max_pwm', 255)

        self.declare_parameter('min_pwm', 25)
        self.declare_parameter('kick_pwm', 70)
        self.declare_parameter('kick_duration', 0.15)   # seconds
        self.declare_parameter('zero_pwm_deadband', 3)

        # How often to (re)publish the current wheel command, in seconds.
        # Must stay well under the firmware's 1000 ms COMMAND_TIMEOUT --
        # serial_hub_node forwards whatever it last received from here
        # at its own heartbeat rate, but this node needs to actually
        # keep publishing fresh values while cmd_vel is active.
        self.declare_parameter('publish_period', 0.1)

        # If no /cmd_vel arrives within this long, publish (0, 0)
        # ourselves, independent of (and in addition to) the firmware
        # failsafe and the hub's own serial-level watchdog.
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_wheel_rpm = self.get_parameter('max_wheel_rpm').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.kick_pwm = self.get_parameter('kick_pwm').value
        self.kick_duration = self.get_parameter('kick_duration').value
        self.zero_pwm_deadband = self.get_parameter('zero_pwm_deadband').value
        self.publish_period = self.get_parameter('publish_period').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        for value in (self.wheel_radius, self.wheel_separation,
                      self.max_wheel_rpm, self.publish_period, self.cmd_vel_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Geometry, RPM, periods and timeouts must be positive')
        if not (1 <= self.zero_pwm_deadband <= self.min_pwm <= self.kick_pwm
                <= self.max_pwm <= 255) or not math.isfinite(self.kick_duration) or self.kick_duration < 0:
            raise ValueError('Invalid PWM limits or kick duration')

        # Per-wheel kick-start state: whether a wheel is currently in
        # its kick window, and when that window ends.
        self._kick_state = {
            'left': {'kicking': False, 'until': 0.0},
            'right': {'kicking': False, 'until': 0.0},
        }

        # Derived: real-world linear speed at full PWM.
        self.max_wheel_speed = (
            self.max_wheel_rpm * 2.0 * math.pi * self.wheel_radius / 60.0
        )

        # ------------------------------------------------------------
        # STATE
        # ------------------------------------------------------------
        self._lock = threading.Lock()
        self._linear_x = 0.0
        self._angular_z = 0.0
        self._last_cmd_time = time.monotonic()

        # ------------------------------------------------------------
        # SUBSCRIBER
        # ------------------------------------------------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            qos,
        )

        # ------------------------------------------------------------
        # PUBLISHER
        # ------------------------------------------------------------
        self.wheel_cmd_pub = self.create_publisher(
            Int32MultiArray, 'wheel_cmd_pwm', 10)

        # ------------------------------------------------------------
        # PUBLISH LOOP (fixed-rate, independent of /cmd_vel rate)
        # ------------------------------------------------------------
        self.create_timer(self.publish_period, self._publish_loop,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            'diff_drive starting. wheel_radius={} m, '
            'wheel_separation={} m, max_wheel_rpm={} rpm '
            '-> max_wheel_speed={:.3f} m/s, max_pwm={}, min_pwm={}'
            .format(self.wheel_radius, self.wheel_separation,
                    self.max_wheel_rpm, self.max_wheel_speed,
                    self.max_pwm, self.min_pwm)
        )

    # ==================================================================
    # CALLBACK
    # ==================================================================
    def _cmd_vel_callback(self, msg):
        with self._lock:
            valid = math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)
            self._linear_x = msg.linear.x if valid else 0.0
            self._angular_z = msg.angular.z if valid else 0.0
            self._last_cmd_time = time.monotonic()

    # ==================================================================
    # KINEMATICS: Twist -> (left_pwm, right_pwm)
    # ==================================================================
    def _twist_to_pwm(self, linear_x, angular_z):
        half_track = self.wheel_separation / 2.0

        # Real-world wheel speeds, positive = forward (REP103 convention).
        w_left = linear_x - angular_z * half_track
        w_right = linear_x + angular_z * half_track

        scale = max(1.0, abs(w_left) / self.max_wheel_speed,
                    abs(w_right) / self.max_wheel_speed)
        w_left /= scale
        w_right /= scale

        def to_pwm(speed):
            if self.max_wheel_speed <= 0:
                return 0
            pwm = (speed / self.max_wheel_speed) * self.max_pwm
            return max(-self.max_pwm, min(self.max_pwm, int(round(pwm))))

        pwm_left = to_pwm(w_left)
        pwm_right = to_pwm(w_right)

        # Flip sign to match this codebase's forward=negative convention.
        return -pwm_left, -pwm_right

    # ==================================================================
    # KICK-START: break static friction on low, sustained targets
    # ==================================================================
    def _apply_kickstart(self, wheel, desired_pwm, now):
        state = self._kick_state[wheel]

        # Genuinely zero -- reset and stop.
        if abs(desired_pwm) < self.zero_pwm_deadband:
            state['kicking'] = False
            return 0

        # Fast enough to move on its own -- no kick needed.
        if abs(desired_pwm) >= self.min_pwm:
            state['kicking'] = False
            return desired_pwm

        # Below min_pwm and nonzero: needs a kick to get moving.
        if not state['kicking']:
            state['kicking'] = True
            state['until'] = now + self.kick_duration

        if now < state['until']:
            sign = 1 if desired_pwm > 0 else -1
            return sign * min(self.kick_pwm, self.max_pwm)

        # Kick window over -- settle to the real (low) target so the
        # wheel keeps turning slowly instead of stopping.
        return desired_pwm

    # ==================================================================
    # PUBLISH LOOP
    # ==================================================================
    def _publish_loop(self):
        with self._lock:
            linear_x = self._linear_x
            angular_z = self._angular_z
            age = time.monotonic() - self._last_cmd_time

        # Independent watchdog: stop if /cmd_vel has gone stale.
        if age > self.cmd_vel_timeout:
            linear_x = 0.0
            angular_z = 0.0

        left, right = self._twist_to_pwm(linear_x, angular_z)

        now = time.monotonic()
        left = self._apply_kickstart('left', left, now)
        right = self._apply_kickstart('right', right, now)

        msg = Int32MultiArray()
        msg.data = [left, right]
        self.wheel_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDrive()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a final stop so the hub (and firmware) see zero
        # before this node goes away.
        stop_msg = Int32MultiArray()
        stop_msg.data = [0, 0]
        if rclpy.ok():
            node.wheel_cmd_pub.publish(stop_msg)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
