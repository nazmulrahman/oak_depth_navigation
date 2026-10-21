"""OAK-D Pro RGB-D + IMU mapping; starts no robot motor nodes."""
from oak_depth_navigation.bringup import mapping_description


def generate_launch_description():
    return mapping_description()
