"""Exercise the real serial reader/writer against a pseudo-terminal, never USB."""
import os
import time
import tty

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from oak_depth_navigation.serial_hub_node import SerialHub


def test_real_serial_protocol_watchdog_and_telemetry():
    master, slave = os.openpty()
    tty.setraw(master)
    os.set_blocking(master, False)
    rclpy.init(args=['--ros-args', '-p', 'serial_port:=' + os.ttyname(slave)])
    hub = SerialHub()
    observer = Node('serial_test_observer')
    rpm = []
    sub = observer.create_subscription(Float32MultiArray, '/wheel_rpm',
                                       lambda m: rpm.append(list(m.data)), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(hub)
    executor.add_node(observer)
    received = bytearray()

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            os.write(master, b'W,-10,-20\n')
            try:
                received.extend(os.read(master, 4096))
            except BlockingIOError:
                pass

    try:
        pump(2.5)  # Real Nano-open reset delay is retained in the implementation.
        assert b'M,0,0\n' in received
        assert rpm and rpm[-1] == [10.0, 20.0]
        received.clear()
        hub._wheel_cmd_callback(Int32MultiArray(data=[-20, -30]))
        pump(0.25)
        assert b'M,-20,-30\n' in received
        received.clear()
        pump(0.65)
        assert b'M,0,0\n' in received
    finally:
        hub.stop_and_shutdown()
        executor.shutdown()
        observer.destroy_node()
        hub.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
