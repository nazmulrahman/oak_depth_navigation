"""Launch helpers. Only this package's config and new session files are written."""
from datetime import datetime
from pathlib import Path
import shutil
import uuid
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def value(context, name):
    return LaunchConfiguration(name).perform(context)


def flag(context, name):
    return value(context, name).lower() == 'true'


def session_database(database='', localization=False):
    """Never erase/overwrite an existing map, including during localization."""
    root = Path.home() / '.ros' / 'oak_depth_navigation' / 'sessions'
    if localization:
        source = Path(database).expanduser().resolve()
        if not database or not source.is_file():
            raise RuntimeError('localization requires database:=/absolute/path/to/saved.db')
        root.mkdir(parents=True, exist_ok=True)
        target = root / ('localize_' + uuid.uuid4().hex + '.db')
        shutil.copyfile(source, target)
        return str(target)
    target = Path(database).expanduser().resolve() if database else root / (
        datetime.now().strftime('map_%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8] + '.db')
    if target.exists():
        raise RuntimeError(f'Refusing to overwrite existing map: {target}. Choose a new database path.')
    target.parent.mkdir(parents=True, exist_ok=True)
    return str(target)


def mapping_actions(context):
    share = Path(get_package_share_directory('oak_depth_navigation'))
    localization = flag(context, 'localization')
    database = session_database(value(context, 'database'), localization)
    print(f'\nOAK session database: {database}\nClose mapping with Ctrl+C to flush it.\n')
    sim = flag(context, 'use_sim_time')
    visual = flag(context, 'visual_odometry')
    use_imu = flag(context, 'use_imu')
    base = value(context, 'base_frame')
    if flag(context, 'start_camera') and base == 'oak':
        raise RuntimeError('Use base_frame:=oak_mount for handheld mapping. The driver owns '
                           'oak_mount -> oak; odometry must not also publish a parent for oak.')
    common = {'use_sim_time': sim, 'frame_id': base,
              'Reg/Force3DoF': 'true' if flag(context, 'planar') else 'false'}
    if flag(context, 'responsive_view'):
        # Handheld preview: avoid long image queues and extrapolation after abrupt motion.
        common.update({'topic_queue_size': 5, 'sync_queue_size': 5,
                       'Rtabmap/DetectionRate': '4.0', 'Odom/GuessMotion': 'false'})
    remaps = [('rgb/image', value(context, 'rgb_topic')),
              ('depth/image', value(context, 'depth_topic')),
              ('rgb/camera_info', value(context, 'camera_info_topic')),
              ('odom', value(context, 'odom_topic')),
              ('imu', '/oak/imu/filtered')]
    actions = []
    if flag(context, 'start_camera'):
        driver = Path(get_package_share_directory('depthai_ros_driver'))
        camera_args = {'name': 'oak', 'params_file': str(share / 'config/oak.yaml'),
                       'camera_model': 'OAK-D-PRO', 'override_cam_model': 'true',
                       'publish_tf_from_calibration': 'true', 'imu_from_descr': 'true',
                       'parent_frame': base,
                       'rectify_rgb': 'true', 'use_rviz': 'false'}
        for arg in ('cam_pos_x', 'cam_pos_y', 'cam_pos_z', 'cam_roll', 'cam_pitch', 'cam_yaw'):
            camera_args[arg] = value(context, arg)
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver / 'launch/camera.launch.py')),
            launch_arguments=camera_args.items()))
    if use_imu:
        actions.append(Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
            name='oak_imu_filter', output='screen',
            parameters=[{'use_sim_time': sim, 'use_mag': False, 'publish_tf': False,
                         'world_frame': 'enu', 'remove_gravity_vector': False}],
            remappings=[('imu/data_raw', value(context, 'imu_topic')),
                        ('imu/data', '/oak/imu/filtered')]))
    if visual:
        odometry = Node(
            package='rtabmap_odom', executable='rgbd_odometry', name='oak_rgbd_odometry',
            output='screen', parameters=[str(share / 'config/rtabmap.yaml'), common,
                {'odom_frame_id': 'odom', 'publish_tf': True, 'wait_imu_to_init': use_imu}],
            remappings=remaps + [('odom_info', '/oak/odom_info')])
        # Let USB boot, exposure, focus and IMU filtering settle before the first keyframe.
        actions.append(TimerAction(period=float(value(context, 'startup_delay')),
                                   actions=[odometry]))
    slam = Node(
        package='rtabmap_slam', executable='rtabmap', name='oak_rtabmap', output='screen',
        parameters=[str(share / 'config/rtabmap.yaml'), common,
            {'database_path': database, 'map_frame_id': 'map', 'publish_tf': True,
             'subscribe_depth': True, 'subscribe_rgb': True, 'subscribe_odom_info': False,
             'Mem/IncrementalMemory': 'false' if localization else 'true',
             'Mem/InitWMWithAllNodes': 'true' if localization else 'false'}],
        remappings=remaps + [('odom_info', '/oak/odom_info'), ('map', '/map'),
                            ('cloud_map', '/oak/cloud_map')])
    actions.append(TimerAction(period=float(value(context, 'startup_delay')) + 1.0,
                               actions=[slam]))
    actions.append(Node(
        package='rtabmap_util', executable='point_cloud_xyz', name='oak_depth_cloud',
        output='screen', parameters=[{'use_sim_time': sim, 'qos': 2,
            'qos_camera_info': 2, 'approx_sync': True, 'approx_sync_max_interval': 0.04,
            'decimation': 2, 'min_depth': 0.3, 'max_depth': 5.0, 'voxel_size': 0.04}],
        remappings=[('depth/image', value(context, 'depth_topic')),
                    ('depth/camera_info', value(context, 'camera_info_topic')),
                    ('cloud', '/oak/points')]))
    if flag(context, 'responsive_view'):
        actions.append(Node(
            package='rtabmap_util', executable='point_cloud_xyzrgb', name='oak_live_color_cloud',
            output='screen', parameters=[{'use_sim_time': sim, 'qos': 2,
                'qos_camera_info': 2, 'approx_sync': True, 'approx_sync_max_interval': 0.04,
                'topic_queue_size': 2, 'sync_queue_size': 5, 'decimation': 4,
                'min_depth': 0.3, 'max_depth': 5.0, 'voxel_size': 0.025}],
            remappings=[('rgb/image', value(context, 'rgb_topic')),
                        ('depth/image', value(context, 'depth_topic')),
                        ('rgb/camera_info', value(context, 'camera_info_topic')),
                        ('cloud', '/oak/live_color_cloud')]))
    if flag(context, 'rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='oak_rviz',
                            arguments=['-d', str(share / 'rviz/mapping.rviz')],
                            parameters=[{'use_sim_time': sim}], output='log'))
    return actions


def mapping_arguments(overrides=None):
    defaults = dict(start_camera='true', use_imu='true', visual_odometry='true',
                    localization='false', database='', base_frame='oak_mount', planar='false',
                    use_sim_time='false', rviz='true', startup_delay='8.0', responsive_view='true',
                    rgb_topic='/oak/rgb/image_rect',
                    depth_topic='/oak/stereo/image_raw', camera_info_topic='/oak/rgb/camera_info',
                    imu_topic='/oak/imu/data', odom_topic='/oak/odom',
                    cam_pos_x='0.0', cam_pos_y='0.0', cam_pos_z='0.0',
                    cam_roll='0.0', cam_pitch='0.0', cam_yaw='0.0')
    defaults.update(overrides or {})
    return [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]


def mapping_description():
    return LaunchDescription(mapping_arguments() + [OpaqueFunction(function=mapping_actions)])


def navigation_actions(context):
    start_localization = flag(context, 'start_localization')
    if start_localization and not value(context, 'database'):
        raise RuntimeError('Pass database:=/path/to/finished_mapping.db for localization.')
    if value(context, 'base_frame') == 'oak':
        raise RuntimeError('Navigation needs the robot base frame, e.g. base_frame:=base_link.')
    actions = mapping_actions(context) if start_localization else []
    share = Path(get_package_share_directory('oak_depth_navigation'))
    filename = 'nav2_jazzy.yaml' if os.environ.get('ROS_DISTRO') == 'jazzy' else 'nav2.yaml'
    params = RewrittenYaml(source_file=str(share / 'config' / filename),
        param_rewrites={'robot_base_frame': value(context, 'base_frame'),
                        'odom_topic': value(context, 'odom_topic'),
                        'use_sim_time': value(context, 'use_sim_time')}, convert_types=True)
    nodes = [('nav2_controller', 'controller_server'), ('nav2_smoother', 'smoother_server'),
             ('nav2_planner', 'planner_server'), ('nav2_behaviors', 'behavior_server'),
             ('nav2_bt_navigator', 'bt_navigator'), ('nav2_velocity_smoother', 'velocity_smoother')]
    for package, name in nodes:
        remaps = [('cmd_vel', '/oak/cmd_vel_raw')]
        if name == 'velocity_smoother':
            remaps.append(('cmd_vel_smoothed', value(context, 'cmd_vel_topic')))
        actions.append(Node(package=package, executable=name, name=name,
                            parameters=[params], remappings=remaps, output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='oak_navigation_lifecycle', parameters=[{'autostart': True,
            'use_sim_time': flag(context, 'use_sim_time'), 'node_names': [n for _, n in nodes]}]))
    return actions


def navigation_description():
    return LaunchDescription(mapping_arguments({'base_frame': 'base_link', 'planar': 'true',
        'localization': 'true'}) + [
        DeclareLaunchArgument('start_localization', default_value='true',
            description='False to use an already running mapping/localization stack.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/oak/cmd_vel',
            description='Set to the robot command input only after checking footprint/extrinsics.'),
        OpaqueFunction(function=navigation_actions)])
