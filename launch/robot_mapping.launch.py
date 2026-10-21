"""DWM motors, wheel/OAK-IMU odometry, camera, dashboard and depth mapping."""
from oak_depth_navigation.robot_bringup import robot_description


def generate_launch_description():
    return robot_description(navigation=False)
