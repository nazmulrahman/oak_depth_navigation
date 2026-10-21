"""Manual motors, wheel odometry, telemetry and native web dashboard only."""
from oak_depth_navigation.robot_bringup import robot_description


def generate_launch_description():
    return robot_description(manual=True)
