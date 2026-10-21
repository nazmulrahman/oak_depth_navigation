"""Command arbitration and deadmen before the existing DWM PWM/serial bridge."""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


class CommandGate:
    def __init__(self, mode='manual', max_linear=0.15, max_angular=0.4):
        self.mode = mode
        self.estop = False
        self.max_linear, self.max_angular = max_linear, max_angular
        self.commands = {'manual': (0.0, 0.0, -math.inf),
                         'navigation': (0.0, 0.0, -math.inf)}
        self.wheels_at = self.depth_at = self.odom_at = -math.inf

    def command(self, source, linear, angular, now):
        if not (math.isfinite(linear) and math.isfinite(angular)):
            linear = angular = 0.0
        if source == 'manual':
            self.mode = 'manual'  # Explicitly re-enable navigation after taking over.
        self.commands[source] = (linear, angular, now)

    def stop(self, active):
        self.estop = bool(active)
        self.mode = 'manual'
        self.commands = {k: (0.0, 0.0, -math.inf) for k in self.commands}

    def output(self, now):
        if self.estop:
            return 0.0, 0.0, 'emergency stop'
        if now - self.wheels_at > 0.5:
            return 0.0, 0.0, 'wheel telemetry stale'
        if self.mode == 'navigation' and (
                now - self.depth_at > 0.5 or now - self.odom_at > 0.5):
            return 0.0, 0.0, 'navigation sensors stale'
        linear, angular, stamp = self.commands[self.mode]
        if now - stamp > 0.35:
            return 0.0, 0.0, 'command timeout'
        return (max(-self.max_linear, min(self.max_linear, linear)),
                max(-self.max_angular, min(self.max_angular, angular)), self.mode)


class CommandMux(Node):
    def __init__(self):
        super().__init__('oak_command_mux')
        self.declare_parameter('navigation_enabled', False)
        self.declare_parameter('allow_navigation', True)
        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.4)
        limits = [float(self.get_parameter(n).value) for n in
                  ('max_linear_speed', 'max_angular_speed')]
        if not all(math.isfinite(v) and v > 0 for v in limits):
            raise ValueError('Speed limits must be finite and positive')
        self.gate = CommandGate('navigation' if self.get_parameter('navigation_enabled').value
                                else 'manual', *limits)
        self.pub = self.create_publisher(Twist, '/motor/cmd_vel', 10)
        self.status = self.create_publisher(String, '/oak/control_status', 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.receive('manual', m), 10)
        self.create_subscription(Twist, '/oak/nav_cmd_vel', lambda m: self.receive('navigation', m), 10)
        self.create_subscription(Bool, '/emergency_stop', lambda m: self.gate.stop(m.data), 10)
        self.create_subscription(Float32MultiArray, '/wheel_rpm', self.wheels, 10)
        self.create_subscription(PointCloud2, '/oak/points', self.depth, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_service(SetBool, '/oak/navigation_enabled', self.mode)
        self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, source, msg):
        self.gate.command(source, msg.linear.x, msg.angular.z, time.monotonic())

    def wheels(self, msg):
        if len(msg.data) == 2 and all(math.isfinite(v) for v in msg.data):
            self.gate.wheels_at = time.monotonic()

    def depth(self, msg):
        if msg.width * msg.height > 0:
            self.gate.depth_at = time.monotonic()

    def odom(self, msg):
        if math.isfinite(msg.twist.twist.linear.x) and math.isfinite(msg.twist.twist.angular.z):
            self.gate.odom_at = time.monotonic()

    def mode(self, req, res):
        if req.data and not self.get_parameter('allow_navigation').value:
            res.success, res.message = False, 'This launch allows manual control only'
        elif req.data and self.gate.estop:
            res.success, res.message = False, 'Release emergency stop first'
        else:
            self.gate.mode = 'navigation' if req.data else 'manual'
            self.gate.commands[self.gate.mode] = (0.0, 0.0, -math.inf)
            res.success, res.message = True, self.gate.mode
        return res

    def tick(self):
        linear, angular, reason = self.gate.output(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.pub.publish(msg)
        self.status.publish(String(data=json.dumps({'mode': self.gate.mode,
            'estop': self.gate.estop, 'reason': reason, 'linear': linear, 'angular': angular})))


def main():
    rclpy.init()
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
