#!/usr/bin/env python3
"""Own the Nano serial connection and bridge commands and telemetry to ROS.

Protocol: M,left,right and R,0/1 commands; I,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,
B,voltage,percentage and W,left,right telemetry. Gyro units must be rad/s.
wheel_rpm_sign defaults to -1 to match this package's negative-forward PWM.
Use a stable device path; auto discovery is opt-in and can select another sensor."""

import glob
import math
import threading
import time

import serial

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType

from std_msgs.msg import Bool, Int32MultiArray, Float32MultiArray
from sensor_msgs.msg import Imu, BatteryState


class SerialHub(Node):

    def __init__(self):
        super().__init__('serial_hub')

        # ------------------------------------------------------------
        # PARAMETERS
        # ------------------------------------------------------------
        # Serial port. 'auto' scans common Arduino device paths;
        # set explicitly (e.g. '/dev/ttyUSB0') if autodetect is flaky.
        self.declare_parameter('serial_port', '/dev/dwm_nano')
        self.declare_parameter('baud_rate', 115200)

        # How often to re-send the last-known wheel command and relay
        # state, in seconds. This is what keeps the firmware's
        # COMMAND_TIMEOUT from firing -- must stay well under its
        # 1000 ms window. Independent of how often diff_drive_node
        # actually publishes new /wheel_cmd_pwm values.
        self.declare_parameter('heartbeat_period', 0.1)

        # If no /wheel_cmd_pwm arrives within this long, treat the
        # wheel command as stale and heartbeat (0, 0) instead of
        # whatever was last received. Independent of (and in addition
        # to) the firmware's own COMMAND_TIMEOUT failsafe and
        # diff_drive_node's cmd_vel watchdog -- this one specifically
        # guards against diff_drive_node itself dying or hanging while
        # this hub keeps running.
        self.declare_parameter('wheel_cmd_timeout', 0.5)

        self.serial_port_param = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.heartbeat_period = self.get_parameter('heartbeat_period').value
        self.wheel_cmd_timeout = self.get_parameter('wheel_cmd_timeout').value

        # ------------------------------------------------------------
        # STATE
        # ------------------------------------------------------------
        if not (0 < self.heartbeat_period < self.wheel_cmd_timeout < 1.0):
            raise ValueError('Require 0 < heartbeat_period < wheel_cmd_timeout < 1s')
        if self.baud_rate <= 0:
            raise ValueError('baud_rate must be positive')
        self.declare_parameter('wheel_rpm_sign', -1.0)
        self.wheel_rpm_sign = self.get_parameter('wheel_rpm_sign').value
        if self.wheel_rpm_sign not in (-1.0, 1.0):
            raise ValueError('wheel_rpm_sign must be -1 or 1')
        self._lock = threading.Lock()
        self._wheel_left = 0
        self._wheel_right = 0
        self._emergency_stop = False
        self._last_wheel_cmd_time = time.monotonic()

        self._serial_lock = threading.Lock()
        self._ser = None
        self._stop_event = threading.Event()

        # ------------------------------------------------------------
        # SUBSCRIBERS
        # ------------------------------------------------------------
        # Wheel PWM targets from diff_drive_node.py. Reliable QoS --
        # this is now the ONLY path motor commands take to reach
        # serial, so a dropped message here means a dropped command,
        # unlike the old best-effort cmd_vel subscription (which was
        # fine to drop since it was a continuous stream, not discrete
        # commands).
        self.create_subscription(
            Int32MultiArray,
            'wheel_cmd_pwm',
            self._wheel_cmd_callback,
            10,
        )

        # Relay control. Discrete on/off command, so reliable QoS --
        # we want it delivered even if it arrives during a brief
        # subscription hiccup, not just "best effort, most recent".
        self._last_relay_state = None  # None = unknown yet, not sent
        self.create_subscription(Bool, 'emergency_stop', self._estop_callback, 10)

        self.create_subscription(
            Bool,
            'relay_cmd',
            self._relay_cmd_callback,
            10,
        )

        # ------------------------------------------------------------
        # IMU PUBLISHER
        # ------------------------------------------------------------
        # Sensor frame. Represent physical mounting rotation in the URDF;
        # changing this string does not rotate sensor measurements.
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        # Provisional variances, not measured calibration: rad^2,
        # (rad/s)^2, and (m/s^2)^2 respectively. Tune from sensor logs.
        self.imu_variances = []
        for name, default in (
            ('imu_orientation_variance', 0.03),
            ('imu_angular_velocity_variance', 0.0025),
            ('imu_linear_acceleration_variance', 0.1),
        ):
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            self.imu_variances.append(value)

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # ------------------------------------------------------------
        # BATTERY PUBLISHER
        # ------------------------------------------------------------
        # power_supply_technology is cosmetic metadata on the message
        # (BatteryState.POWER_SUPPLY_TECHNOLOGY_LIION = 3) -- doesn't
        # affect any math here, just what a consumer sees if it reads
        # that field.
        self.battery_pub = self.create_publisher(
            BatteryState, 'battery_state', 10)

        # ------------------------------------------------------------
        # WHEEL RPM PUBLISHER
        # ------------------------------------------------------------
        # Raw FG-tachometer-derived RPM, pass-through only -- no
        # kinematics here. wheel_odom_node.py subscribes this and
        # does the actual integration into /odom.
        self.wheel_rpm_pub = self.create_publisher(
            Float32MultiArray, 'wheel_rpm', 10)

        # ------------------------------------------------------------
        # HEARTBEAT LOOP
        # ------------------------------------------------------------
        self.create_timer(self.heartbeat_period, self._heartbeat_loop,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

        self._connect_thread = threading.Thread(target=self._connect_loop, daemon=True)
        self._connect_thread.start()

        self.get_logger().info(
            'serial_hub starting. heartbeat_period={}s, '
            'wheel_cmd_timeout={}s, imu_frame_id={}'
            .format(self.heartbeat_period, self.wheel_cmd_timeout,
                    self.imu_frame_id)
        )

    # ==================================================================
    # SERIAL: FIND PORT
    # ==================================================================
    def _find_port(self):
        if self.serial_port_param != 'auto':
            return self.serial_port_param

        candidates = (
            sorted(glob.glob('/dev/ttyACM*')) +
            sorted(glob.glob('/dev/ttyUSB*'))
        )
        return candidates[0] if candidates else None

    # ==================================================================
    # SERIAL: CONNECT (runs in background thread, retries forever)
    # ==================================================================
    def _connect_loop(self):
        while not self._stop_event.is_set():
            port = self._find_port()

            if not port:
                self.get_logger().warn(
                    'No Arduino serial port found, retrying...')
                self._stop_event.wait(2.0)
                continue

            ser = None
            try:
                ser = serial.Serial(
                    port,
                    self.baud_rate,
                    timeout=0.2,
                    write_timeout=0.2,
                    exclusive=True,
                )

                # Nano commonly resets when the serial port opens.
                self._stop_event.wait(2.0)
                if self._stop_event.is_set():
                    ser.close()
                    break
                ser.reset_input_buffer()
                ser.write(b'M,0,0\nR,0\n')

                with self._serial_lock:
                    self._ser = ser

                self.get_logger().info('Connected to Arduino on {}'.format(port))

                # Block here reading incoming lines so the OS-side RX
                # buffer doesn't fill up. "I,..." lines are IMU
                # samples and get parsed + published; everything else
                # (telemetry block, "READY", "FAILSAFE,STOP", etc.) is
                # just logged at debug level for now. If the
                # connection drops, fall through and reconnect.
                pending = b''
                while not self._stop_event.is_set():
                    try:
                        line = ser.read_until(b'\n', 512)
                        if not line:
                            continue
                    except (serial.SerialException, OSError) as error:
                        self.get_logger().warn(
                            'Serial read error: {}'.format(error))
                        break

                    pending += line
                    if len(pending) > 512:
                        pending = b''
                    elif pending.endswith(b'\n'):
                        self._handle_incoming_line(pending)
                        pending = b''

            except (serial.SerialException, OSError) as error:
                self.get_logger().warn(
                    'Could not open {}: {}'.format(port, error))

            if ser is not None:
                ser.close()
            with self._serial_lock:
                if self._ser is not None:
                    try:
                        self._ser.close()
                    except Exception:
                        pass
                self._ser = None

            self._stop_event.wait(2.0)

    # ==================================================================
    # SERIAL: INCOMING LINE DISPATCH (runs on the reader thread)
    # ==================================================================
    def _handle_incoming_line(self, raw_line):
        try:
            text = raw_line.decode('utf-8', errors='replace').strip()
        except Exception:
            return

        if not text:
            return

        if text.startswith('I,'):
            self._parse_and_publish_imu(text)
            return

        if text.startswith('B,'):
            self._parse_and_publish_battery(text)
            return

        if text.startswith('W,'):
            self._parse_and_publish_wheel_rpm(text)
            return

        # Everything else (telemetry block, "READY", "FAILSAFE,STOP")
        # -- not currently consumed by anything. Left at debug level
        # so it doesn't spam normal logs but is still inspectable
        # with --log-level debug if you need to see it.
        self.get_logger().debug('Nano: {}'.format(text))

    # ==================================================================
    # IMU: PARSE "I,qw,qx,qy,qz,gx,gy,gz,ax,ay,az" AND PUBLISH
    # ==================================================================
    def _parse_and_publish_imu(self, line):
        fields = line.split(',')

        # "I" + 10 numeric fields = 11 tokens total.
        if len(fields) != 11:
            self.get_logger().warn(
                'Malformed IMU line (expected 11 fields, got {}): {}'
                .format(len(fields), line))
            return

        try:
            qw, qx, qy, qz, gx, gy, gz, ax, ay, az = (
                float(v) for v in fields[1:]
            )
        except ValueError:
            self.get_logger().warn(
                'Malformed IMU line (non-numeric field): {}'.format(line))
            return

        if not all(math.isfinite(v) for v in (qw, qx, qy, qz, gx, gy, gz, ax, ay, az)):
            return
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if norm < 1e-6:
            return
        qw, qx, qy, qz = (v / norm for v in (qw, qx, qy, qz))
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id

        msg.orientation.w = qw
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        for field, variance in zip((
            'orientation_covariance', 'angular_velocity_covariance',
            'linear_acceleration_covariance',
        ), self.imu_variances):
            setattr(msg, field, [variance, 0.0, 0.0, 0.0, variance, 0.0,
                                 0.0, 0.0, variance])

        self.imu_pub.publish(msg)

    # ==================================================================
    # BATTERY: PARSE "B,voltage,percentage" AND PUBLISH
    # ==================================================================
    def _parse_and_publish_battery(self, line):
        fields = line.split(',')

        # "B" + 2 numeric fields = 3 tokens total.
        if len(fields) != 3:
            self.get_logger().warn(
                'Malformed battery line (expected 3 fields, got {}): {}'
                .format(len(fields), line))
            return

        try:
            voltage, percentage = (float(v) for v in fields[1:])
        except ValueError:
            self.get_logger().warn(
                'Malformed battery line (non-numeric field): {}'.format(line))
            return

        if not (math.isfinite(voltage) and voltage >= 0 and
                math.isfinite(percentage) and 0 <= percentage <= 100):
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.voltage = voltage

        # Firmware sends 0-100; sensor_msgs/BatteryState.percentage is
        # documented as a 0.0-1.0 fraction. Converting here so this
        # node's ROS output actually matches the message definition,
        # rather than pushing that conversion onto every consumer.
        msg.percentage = percentage / 100.0

        # Not measured (no current sensing implemented yet) -- NaN is
        # the correct way to say "unknown" for these fields, same
        # spirit as the -1 covariance convention used for IMU above.
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')

        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        msg.present = True

        self.battery_pub.publish(msg)

    # ==================================================================
    # WHEEL RPM: PARSE "W,left_wheel_rpm,right_wheel_rpm" AND PUBLISH
    # ==================================================================
    def _parse_and_publish_wheel_rpm(self, line):
        fields = line.split(',')

        # "W" + 2 numeric fields = 3 tokens total.
        if len(fields) != 3:
            self.get_logger().warn(
                'Malformed wheel RPM line (expected 3 fields, got {}): {}'
                .format(len(fields), line))
            return

        try:
            left_rpm, right_rpm = (float(v) for v in fields[1:])
        except ValueError:
            self.get_logger().warn(
                'Malformed wheel RPM line (non-numeric field): {}'
                .format(line))
            return

        if not all(math.isfinite(v) and abs(v) < 1e6 for v in (left_rpm, right_rpm)):
            return
        left_rpm *= self.wheel_rpm_sign
        right_rpm *= self.wheel_rpm_sign
        msg = Float32MultiArray()
        msg.data = [left_rpm, right_rpm]
        self.wheel_rpm_pub.publish(msg)

        # Encoder feedback visibility - throttled to 1/sec so it's
        # readable in the terminal instead of scrolling at ~20 Hz.
        self.get_logger().info(
            'Encoder: left={:.1f} rpm, right={:.1f} rpm'
            .format(left_rpm, right_rpm),
            throttle_duration_sec=1.0,
        )

    # ==================================================================
    # WHEEL COMMAND CALLBACK
    # ==================================================================
    def _wheel_cmd_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warn(
                'wheel_cmd_pwm expected [left, right], got {} values'
                .format(len(msg.data)))
            return

        with self._lock:
            self._wheel_left = 0 if self._emergency_stop else max(-255, min(255, int(msg.data[0])))
            self._wheel_right = 0 if self._emergency_stop else max(-255, min(255, int(msg.data[1])))
            self._last_wheel_cmd_time = time.monotonic()

    def _estop_callback(self, msg):
        with self._lock:
            self._emergency_stop = bool(msg.data)
            self._wheel_left = self._wheel_right = 0
        self._last_relay_state = False
        self._write_command(0, 0)
        self._write_raw('R,0\n')

    # ==================================================================
    # RELAY CALLBACK
    # ==================================================================
    def _relay_cmd_callback(self, msg):
        # Note: this only ever sends "R,1" / "R,0" (topic-controlled
        # mode). "R,A" (always-on, ignores firmware failsafe) is
        # intentionally NOT exposed here -- if you need always-on,
        # send it directly over serial/a terminal, not from ROS,
        # since it's meant to bypass this node's normal control path.
        desired = bool(msg.data) and not self._emergency_stop

        if desired == self._last_relay_state:
            # No change -- avoid spamming the serial line on repeated
            # publishes of the same state (e.g. latched/retained topics).
            return

        self._last_relay_state = desired

        command = 'R,1\n' if desired else 'R,0\n'
        self._write_raw(command)

        self.get_logger().info(
            'Relay -> {}'.format('ON' if desired else 'OFF')
        )

    # ==================================================================
    # HEARTBEAT LOOP
    # ==================================================================
    def _heartbeat_loop(self):
        self._send_motor_heartbeat()
        self._send_relay_heartbeat()

    # ==================================================================
    # MOTOR HEARTBEAT
    #
    # Pure forwarding -- no kinematics here, that's diff_drive_node's
    # job. This just takes whatever [left, right] it was last told
    # and keeps re-sending it often enough to satisfy the firmware's
    # COMMAND_TIMEOUT, same heartbeat behavior as before the split.
    # ==================================================================
    def _send_motor_heartbeat(self):
        with self._lock:
            left = self._wheel_left
            right = self._wheel_right
            age = time.monotonic() - self._last_wheel_cmd_time

        # Independent watchdog: if diff_drive_node has gone stale
        # (crashed, hung, network partition on a multi-machine setup),
        # stop instead of repeating a possibly-old nonzero command.
        if age > self.wheel_cmd_timeout or self._emergency_stop:
            left = 0
            right = 0

        self._write_command(left, right)

    # ==================================================================
    # Repeat both ON and OFF so a failed transition is retried.
    # Relay state is latched until another relay command or hub shutdown.
    # ==================================================================
    def _send_relay_heartbeat(self):
        self._write_raw('R,1\n' if self._last_relay_state is True else 'R,0\n')

    # ==================================================================
    # SERIAL WRITE
    # ==================================================================
    def _write_command(self, left, right):
        self._write_raw('M,{},{}\n'.format(left, right))

    def _write_raw(self, command):
        with self._serial_lock:
            if self._ser is None or not self._ser.is_open:
                return
            try:
                self._ser.write(command.encode('utf-8'))

            except (serial.SerialException, OSError) as error:
                self.get_logger().warn('Serial write error: {}'.format(error))

    # ==================================================================
    # SHUTDOWN
    # ==================================================================
    def stop_and_shutdown(self):
        self._stop_event.set()
        self._write_command(0, 0)
        self._write_raw('R,0\n')
        self._connect_thread.join(timeout=3.0)
        with self._serial_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = SerialHub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_and_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
