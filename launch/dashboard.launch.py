"""Use with an already running OAK/robot stack."""
from oak_depth_navigation.dashboard_launch import bridge_arguments, bridge_actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(bridge_arguments() + bridge_actions() + [
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        Node(package='oak_depth_navigation', executable='dashboard',
             parameters=[{'host': LaunchConfiguration('host'),
                          'port': ParameterValue(LaunchConfiguration('port'), value_type=int)}]),
    ])
