"""RTAB-Map localization and Nav2 for a mounted OAK-D Pro."""
from oak_depth_navigation.bringup import navigation_description


def generate_launch_description():
    return navigation_description()
