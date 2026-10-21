"""DWM motors, camera localization, Nav2 and dashboard for a saved map."""
from oak_depth_navigation.robot_bringup import robot_description


def generate_launch_description():
    return robot_description(navigation=True)
