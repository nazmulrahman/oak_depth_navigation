"""Hardware-free regression tests using real ROS message types and mocked I/O."""
import math
import threading
import time
from unittest.mock import Mock, patch

import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool, Int32MultiArray

from oak_depth_navigation.diff_drive_node import DiffDrive
from oak_depth_navigation.serial_hub_node import SerialHub
from oak_depth_navigation.wheel_odom_node import WheelOdom


@pytest.fixture(scope='module', autouse=True)
def context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def drive():
    node = DiffDrive()
    yield node
    node.destroy_node()


@pytest.fixture
def hub():
    # Never open any device, including during failed tests.
    with patch.object(threading.Thread, 'start'):
        node = SerialHub()
    node.imu_pub = Mock()
    node.battery_pub = Mock()
    node.wheel_rpm_pub = Mock()
    yield node
    node.destroy_node()


def test_drive_sign_and_saturation_preserve_curvature(drive):
    left, right = drive._twist_to_pwm(2.0, 3.0)
    assert right == -255
    assert left / right == pytest.approx((2 - 3 * .215) / (2 + 3 * .215), abs=.005)
    assert drive._twist_to_pwm(0, 0) == (0, 0)


def test_nonfinite_command_stops(drive):
    msg = Twist()
    msg.linear.x = float('nan')
    drive._cmd_vel_callback(msg)
    assert drive._linear_x == drive._angular_z == 0


def test_expired_command_stops(drive):
    drive.wheel_cmd_pub = Mock()
    drive._linear_x = .5
    drive._last_cmd_time = time.monotonic() - 2
    drive._publish_loop()
    assert list(drive.wheel_cmd_pub.publish.call_args.args[0].data) == [0, 0]


def test_kick_expires_and_resets(drive):
    assert drive._apply_kickstart('left', 10, 1) == 70
    assert drive._apply_kickstart('left', 10, 2) == 10
    assert drive._apply_kickstart('left', 0, 3) == 0
    assert drive._apply_kickstart('left', -10, 4) == -70


def test_imu_available_normalized_and_finite(hub):
    hub._parse_and_publish_imu('I,2,0,0,0,1,2,3,4,5,6')
    msg = hub.imu_pub.publish.call_args.args[0]
    assert msg.orientation.w == 1
    assert list(msg.orientation_covariance) == [0.03, 0, 0, 0, 0.03, 0, 0, 0, 0.03]
    assert msg.angular_velocity_covariance[0] == .0025
    assert msg.linear_acceleration_covariance[8] == .1
    hub.imu_pub.reset_mock()
    for line in ('I,nan,0,0,0,1,2,3,4,5,6', 'I,0,0,0,0,1,2,3,4,5,6', 'I,bad'):
        hub._parse_and_publish_imu(line)
    hub.imu_pub.publish.assert_not_called()


def test_battery_and_wheel_protocol(hub):
    hub._parse_and_publish_battery('B,24,75')
    assert hub.battery_pub.publish.call_args.args[0].percentage == .75
    hub._parse_and_publish_wheel_rpm('W,-10,-20')
    assert list(hub.wheel_rpm_pub.publish.call_args.args[0].data) == [10, 20]
    hub.wheel_rpm_pub.reset_mock()
    hub._parse_and_publish_wheel_rpm('W,inf,2')
    hub.wheel_rpm_pub.publish.assert_not_called()


def test_stop_and_relay_off_retried_every_heartbeat(hub):
    hub._write_raw = Mock()
    hub._wheel_left = hub._wheel_right = 100
    hub._last_wheel_cmd_time = time.monotonic() - 2
    hub._last_relay_state = False
    hub._heartbeat_loop()
    hub._heartbeat_loop()
    assert [call.args[0] for call in hub._write_raw.call_args_list] == [
        'M,0,0\n', 'R,0\n', 'M,0,0\n', 'R,0\n']


def test_serial_emergency_stop_blocks_pwm_and_relay(hub):
    hub._write_raw = Mock()
    hub._estop_callback(Bool(data=True))
    hub._wheel_cmd_callback(Int32MultiArray(data=[255, 255]))
    hub._relay_cmd_callback(Bool(data=True))
    hub._heartbeat_loop()
    commands = [call.args[0] for call in hub._write_raw.call_args_list]
    assert 'M,255,255\n' not in commands
    assert 'R,1\n' not in commands
    assert commands[-2:] == ['M,0,0\n', 'R,0\n']
    hub._estop_callback(Bool(data=False))
    assert hub._wheel_left == hub._wheel_right == 0


def test_odometry_arc_and_invalid_input():
    node = WheelOdom()
    try:
        node._publish_odometry = Mock()
        node._broadcast_tf = Mock()
        from rclpy.time import Time
        start = Time(seconds=10)
        end = Time(seconds=10.1)
        node.last_update_time = start
        node.last_rpm_msg_time = start
        node.left_rpm, node.right_rpm = 0, 60
        with patch.object(node, 'get_clock') as clock:
            clock.return_value.now.return_value = end
            node._update_loop()
        speed = math.pi * .095
        omega = 2 * speed / .430
        assert node.x == pytest.approx(speed / omega * math.sin(omega * .1))
        assert node.y == pytest.approx(speed / omega * (1 - math.cos(omega * .1)))
        node._wheel_rpm_callback(Float32MultiArray(data=[float('nan'), 2.0]))
        assert node.left_rpm == node.right_rpm == 0
        previous = node.x
        with patch.object(node, 'get_clock') as clock:
            clock.return_value.now.return_value = Time(seconds=20)
            node._update_loop()
        assert node.x == previous
    finally:
        node.destroy_node()


def test_wheel_joint_feedback_reverse_and_timeout():
    from rclpy.time import Time
    node = WheelOdom()
    node.joint_pub = Mock()
    try:
        node.last_update_time = node.last_rpm_msg_time = Time(seconds=10)
        node.left_rpm, node.right_rpm = 60.0, -30.0
        with patch.object(node, 'get_clock') as clock:
            clock.return_value.now.return_value = Time(seconds=10.1)
            node._update_loop()
        msg = node.joint_pub.publish.call_args.args[0]
        assert msg.name == ['left_track_joint', 'right_track_joint']
        assert list(msg.position) == pytest.approx([.2 * math.pi, -.1 * math.pi])
        assert list(msg.velocity) == pytest.approx([2 * math.pi, -math.pi])
        positions = list(msg.position)
        node.last_update_time = Time(seconds=10.6)
        with patch.object(node, 'get_clock') as clock:
            clock.return_value.now.return_value = Time(seconds=10.7)
            node._update_loop()
        stopped = node.joint_pub.publish.call_args.args[0]
        assert list(stopped.position) == positions
        assert list(stopped.velocity) == [0.0, 0.0]
    finally:
        node.destroy_node()
