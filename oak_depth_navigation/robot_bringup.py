"""Full independent DWM + OAK robot bringup for Jazzy and Humble."""
from datetime import datetime
from pathlib import Path
import re
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from .dashboard_launch import bridge_arguments, bridge_actions
from .bringup import mapping_arguments, mapping_actions, navigation_actions, flag, value


def robot_actions(context, navigation=False, manual=False):
    share = Path(get_package_share_directory('oak_depth_navigation'))
    config = yaml.safe_load(Path(value(context, 'robot_config')).expanduser().read_text())
    serial_port = value(context, 'serial_port') or config['serial_port']
    if serial_port == 'auto':
        raise RuntimeError('Choose the Arduino serial device explicitly; auto is not used on the robot.')
    db = '(manual mode: no map database)'
    perception = []
    if not manual:
        name = value(context, 'map_name')
        explicit_db = value(context, 'database')
        if not name and not explicit_db:
            if navigation:
                raise RuntimeError('Select a saved map with map_name:=room_01 or database:=/path/to/rtabmap.db')
            name = datetime.now().strftime('map_%Y%m%d_%H%M%S')
        if name and not re.fullmatch(r'[A-Za-z0-9_-]+', name):
            raise RuntimeError('map_name may contain only letters, digits, underscore and hyphen.')
        db = explicit_db or str(Path(config['map_root']).expanduser() / name / 'rtabmap.db')
        settings = {'database': db, 'base_frame': 'base_footprint', 'planar': 'true',
                    'visual_odometry': 'false', 'odom_topic': '/odom',
                    'localization': 'true' if navigation else 'false',
                    'start_localization': 'true', 'cmd_vel_topic': '/oak/nav_cmd_vel'}
        mount = config['camera_mount']
        for key in ('x', 'y', 'z'):
            settings['cam_pos_' + key] = str(mount[key])
        for key in ('roll', 'pitch', 'yaw'):
            settings['cam_' + key] = str(mount[key])
        context.launch_configurations.update(settings)
        # Validate the selected map and camera stack before any serial node can start.
        perception = navigation_actions(context) if navigation else mapping_actions(context)
    geometry = {k: float(config[k]) for k in ('wheel_radius', 'wheel_separation')}
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher',
        name='dwm_robot_state_publisher', output='screen',
        parameters=[{'robot_description': (share / 'urdf/dwm01.urdf').read_text()}])]
    if flag(context, 'enable_motors'):
        actions.append(Node(package='oak_depth_navigation', executable='serial_hub',
            name='serial_hub', output='screen', parameters=[{'serial_port': serial_port,
                'baud_rate': int(config['baud_rate']), 'wheel_rpm_sign': float(config['wheel_rpm_sign']),
                'heartbeat_period': 0.1, 'wheel_cmd_timeout': 0.5}],
            remappings=[('imu/data', '/dwm/imu/data')]))
    drive_params = dict(geometry)
    for key in ('max_wheel_rpm', 'max_pwm', 'min_pwm', 'kick_pwm', 'kick_duration'):
        drive_params[key] = config[key]
    actions.extend([
        Node(package='oak_depth_navigation', executable='command_mux',
             parameters=[{'navigation_enabled': navigation, 'allow_navigation': not manual,
                'max_linear_speed': float(config['max_linear_speed']),
                'max_angular_speed': float(config['max_angular_speed'])}], output='screen'),
        Node(package='oak_depth_navigation', executable='diff_drive', name='diff_drive',
             parameters=[drive_params], remappings=[('cmd_vel', '/motor/cmd_vel')], output='screen'),
        Node(package='oak_depth_navigation', executable='wheel_odom', name='wheel_odom',
             parameters=[geometry, {'publish_tf': manual, 'base_frame_id': 'base_footprint'}],
             remappings=[('odom', '/odom' if manual else '/wheel/odom')], output='screen'),
    ])
    if not manual:
        actions.append(Node(package='robot_localization', executable='ekf_node', name='oak_ekf',
             parameters=[str(share / 'config/ekf.yaml')],
             remappings=[('odometry/filtered', '/odom')], output='screen'))
    if flag(context, 'dashboard'):
        actions.extend(bridge_actions())
        actions.append(Node(package='oak_depth_navigation', executable='dashboard',
            parameters=[{'host': config['dashboard_host'], 'port': int(config['dashboard_port']),
                         'base_frame': 'base_footprint',
                         'session_mode': 'manual' if manual else 'navigation' if navigation else 'mapping',
                         'imu_topic': '/dwm/imu/data' if manual else '/oak/imu/filtered'}], output='screen'))
    print(f'\nDWM robot map: {db}\nDashboard: http://JETSON_IP:{config["dashboard_port"]}\n'
          f'Motor serial bridge enabled: {flag(context, "enable_motors")}\n')
    return actions + perception


def robot_description(navigation=False, manual=False):
    share = Path(get_package_share_directory('oak_depth_navigation'))
    common = bridge_arguments() + [
        DeclareLaunchArgument('robot_config', default_value=str(share / 'config/robot.yaml')),
        DeclareLaunchArgument('serial_port', default_value=''),
        DeclareLaunchArgument('enable_motors', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        OpaqueFunction(function=lambda context: robot_actions(context, navigation, manual)),
    ]
    if manual:
        return LaunchDescription(common)
    return LaunchDescription(mapping_arguments({'rviz': 'false', 'visual_odometry': 'false',
        'responsive_view': 'false', 'base_frame': 'base_footprint',
        'odom_topic': '/odom', 'planar': 'true'}) + [
        DeclareLaunchArgument('map_name', default_value=''),
    ] + common)
