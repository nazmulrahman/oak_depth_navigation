"""LAN dashboard with normal RGB video, map, teleop and Nav2 goals; no CDN assets."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from queue import Queue, Empty
import json
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, Imu, BatteryState
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener


class Dashboard(Node):
    def __init__(self):
        super().__init__('oak_dashboard')
        for name, default in [('host', '0.0.0.0'), ('port', 8080),
                              ('rgb_topic', '/oak/rgb/image_rect'),
                              ('imu_topic', '/oak/imu/filtered'), ('session_mode', 'standalone'), ('base_frame', 'base_footprint')]:
            self.declare_parameter(name, default)
        self.lock = threading.Lock()
        self.state = {'control': {}, 'navigation': 'idle', 'map': None,
                      'session_mode': self.get_parameter('session_mode').value}
        self.images = {}
        self.last_image = 0.0
        self.commands = Queue(maxsize=32)
        self.bridge = CvBridge()
        self.camera_pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 1)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.mode_client = self.create_client(SetBool, '/oak/navigation_enabled')
        self.goal_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_handle = None
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(Image, self.get_parameter('rgb_topic').value,
                                 self.rgb, qos_profile_sensor_data)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value, self.imu_pub.publish, qos_profile_sensor_data)
        self.create_subscription(String, '/oak/control_status', self.control, 10)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(BatteryState, '/battery_state', self.battery, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.05, self.tick)
        self.html = (Path(get_package_share_directory('oak_depth_navigation')) /
                     'dashboard/dwm_dashboard.html').read_bytes()
        self.navigation_html = (Path(get_package_share_directory('oak_depth_navigation')) /
                                'dashboard/index.html').read_bytes()
        share = Path(get_package_share_directory('oak_depth_navigation')) / 'dashboard'
        self.developer_html = (share / 'dwm_dashboard_developer.html').read_bytes()
        self.depth_html = (share / 'depth_navigation.html').read_bytes()
        dashboard = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def reply(self, code, content, kind='application/json'):
                self.send_response(code)
                self.send_header('Content-Type', kind)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(content)))
                self.end_headers()
                try:
                    self.wfile.write(content)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_GET(self):
                path = self.path.split('?')[0]
                if path in ('/', '/developer', '/dwm_dashboard_developer.html'):
                    self.reply(200, dashboard.developer_html, 'text/html; charset=utf-8')
                elif path == '/classic':
                    self.reply(200, dashboard.html, 'text/html; charset=utf-8')
                elif path == '/depth-navigation':
                    self.reply(200, dashboard.depth_html, 'text/html; charset=utf-8')
                elif path in ('/navigation', '/navigation/'):
                    self.reply(200, dashboard.navigation_html, 'text/html; charset=utf-8')
                elif path == '/api/state':
                    with dashboard.lock:
                        data = dict(dashboard.state)
                        data['camera_age'] = time.monotonic() - dashboard.last_image
                    self.reply(200, json.dumps(data, allow_nan=False).encode())
                elif path in ('/camera.jpg', '/map.png'):
                    with dashboard.lock:
                        data = dashboard.images.get(path)
                    self.reply(200 if data else 503, data or b'',
                               'image/jpeg' if path.endswith('jpg') else 'image/png')
                else:
                    self.reply(404, b'{}')

            def do_POST(self):
                # Browser commands must originate from this dashboard, not another website.
                origin = self.headers.get('Origin')
                if origin and origin != 'http://' + self.headers.get('Host', ''):
                    self.reply(403, b'{"error":"origin rejected"}')
                    return
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if size <= 0 or size > 2048:
                        raise ValueError('invalid body size')
                    payload = json.loads(self.rfile.read(size))
                    if self.path not in ('/api/cmd', '/api/stop', '/api/mode', '/api/goal'):
                        raise ValueError('unknown command')
                    if self.path in ('/api/cmd', '/api/goal'):
                        keys = ('linear', 'angular') if self.path == '/api/cmd' else ('x', 'y', 'yaw')
                        if not all(math.isfinite(float(payload[k])) for k in keys):
                            raise ValueError('nonfinite command')
                    elif type(payload.get('enabled')) is not bool:
                        raise ValueError('enabled must be boolean')
                    dashboard.commands.put_nowait((self.path, payload))
                    self.reply(202, b'{"queued":true}')
                except Exception as error:
                    self.reply(400, json.dumps({'error': str(error)}).encode())

        self.server = ThreadingHTTPServer((self.get_parameter('host').value,
                                           self.get_parameter('port').value), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Dashboard: http://JETSON_IP:{self.server.server_port}')

    def control(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.state['control'] = data

    def odom(self, msg):
        with self.lock:
            self.state['speed'] = {'linear': msg.twist.twist.linear.x,
                                   'angular': msg.twist.twist.angular.z}

    def battery(self, msg):
        with self.lock:
            self.state['battery'] = {'voltage': msg.voltage if math.isfinite(msg.voltage) else None,
                'percent': msg.percentage * 100 if math.isfinite(msg.percentage) else None}

    def rgb(self, msg):
        now = time.monotonic()
        if now - self.last_image < 0.125:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            scale = min(1.0, 800.0 / frame.shape[1])
            if scale < 1:
                frame = cv2.resize(frame, None, fx=scale, fy=scale)
            ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                return
        except Exception as error:
            self.get_logger().warning(str(error))
            return
        jpeg = data.tobytes()
        with self.lock:
            self.images['/camera.jpg'], self.last_image = jpeg, now
        self.camera_pub.publish(CompressedImage(header=msg.header, format='jpeg', data=jpeg))

    def map(self, msg):
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        pixels = np.full(data.shape, 160, np.uint8)
        pixels[data >= 0] = (255 * (1 - data[data >= 0] / 100)).astype(np.uint8)
        ok, png = cv2.imencode('.png', np.flipud(pixels))
        if ok:
            q = msg.info.origin.orientation
            yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y+q.z*q.z))
            with self.lock:
                self.images['/map.png'] = png.tobytes()
                self.state['map'] = {'width': msg.info.width, 'height': msg.info.height,
                    'resolution': msg.info.resolution, 'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y, 'yaw': yaw}

    def navigation_status(self, text):
        with self.lock:
            self.state['navigation'] = text

    def goal_accepted(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.navigation_status('Goal rejected')
                return
            self.navigation_status('Navigating')
            self.goal_handle.get_result_async().add_done_callback(
                lambda f: self.navigation_status('Goal result status: ' + str(f.result().status)))
        except Exception as error:
            self.navigation_status(str(error))

    def set_mode(self, enabled, goal=None):
        if enabled and self.get_parameter('session_mode').value == 'manual':
            self.navigation_status('Manual session: start robot_navigation.launch.py for navigation')
            return
        if not self.mode_client.service_is_ready():
            self.navigation_status('Motor command mux is unavailable')
            return
        def done(future):
            response = future.result()
            self.navigation_status(response.message)
            if goal is not None and response.success:
                self.goal_client.send_goal_async(goal).add_done_callback(self.goal_accepted)
        self.mode_client.call_async(SetBool.Request(data=enabled)).add_done_callback(done)

    def tick(self):
        try:
            t = self.tf.lookup_transform('map', self.get_parameter('base_frame').value,
                                         rclpy.time.Time())
            with self.lock:
                self.state['robot'] = {'x': t.transform.translation.x, 'y': t.transform.translation.y}
        except Exception:
            pass
        for _ in range(32):
            try:
                path, payload = self.commands.get_nowait()
            except Empty:
                break
            if path == '/api/cmd':
                msg = Twist()
                msg.linear.x = float(payload['linear'])
                msg.angular.z = float(payload['angular'])
                self.cmd_pub.publish(msg)
            elif path == '/api/stop':
                self.stop_pub.publish(Bool(data=payload['enabled']))
                if payload['enabled'] and self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
            elif path == '/api/mode':
                self.set_mode(payload['enabled'])
            elif path == '/api/goal':
                if not self.goal_client.server_is_ready():
                    self.navigation_status('Nav2 is unavailable; start robot_navigation.launch.py')
                    continue
                goal = NavigateToPose.Goal()
                goal.pose = PoseStamped()
                goal.pose.header.frame_id = 'map'
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x = float(payload['x'])
                goal.pose.pose.position.y = float(payload['y'])
                goal.pose.pose.orientation.z = math.sin(float(payload['yaw'])/2)
                goal.pose.pose.orientation.w = math.cos(float(payload['yaw'])/2)
                self.set_mode(True, goal)

    def close(self):
        self.server.shutdown()
        self.server.server_close()


def main():
    rclpy.init()
    node = Dashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
