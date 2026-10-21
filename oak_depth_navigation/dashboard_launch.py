"""Shared bridge actions for the DWM dashboard; no motor nodes are started here."""
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bridge_arguments():
    return [DeclareLaunchArgument('rosbridge', default_value='true',
            description='Start the DWM dashboard bridge; false if one is already running')]


def bridge_actions():
    return [
        Node(package='rosbridge_server', executable='rosbridge_websocket',
             name='oak_rosbridge', parameters=[{'port': 9090}],
             condition=IfCondition(LaunchConfiguration('rosbridge')), output='screen'),
        Node(package='rosapi', executable='rosapi_node', name='oak_rosapi',
             condition=IfCondition(LaunchConfiguration('rosbridge')), output='screen'),
    ]
