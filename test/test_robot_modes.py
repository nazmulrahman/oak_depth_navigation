"""Inspect launch construction without executing any node or touching devices."""
from pathlib import Path
from launch import LaunchContext
import pytest
from oak_depth_navigation import robot_bringup as robot


@pytest.fixture
def construction(monkeypatch):
    share = Path(__file__).resolve().parents[1]
    monkeypatch.setattr(robot, 'get_package_share_directory', lambda _: str(share))
    monkeypatch.setattr(robot, 'Node', lambda **kwargs: kwargs)
    monkeypatch.setattr(robot, 'bridge_actions', lambda: [{'bridge': True}])
    context = LaunchContext()
    context.launch_configurations.update(robot_config=str(share / 'config/robot.yaml'),
        serial_port='/dev/test-only', enable_motors='true', dashboard='true',
        map_name='test_room', database='')
    return context


def test_manual_never_constructs_perception_or_map(construction, monkeypatch):
    def forbidden(_):
        pytest.fail('Manual mode must not construct SLAM or Nav2')
    monkeypatch.setattr(robot, 'mapping_actions', forbidden)
    monkeypatch.setattr(robot, 'navigation_actions', forbidden)
    actions = robot.robot_actions(construction, manual=True)
    nodes = {a['executable']: a for a in actions if 'executable' in a}
    assert set(nodes) == {'robot_state_publisher', 'serial_hub', 'command_mux',
                          'diff_drive', 'wheel_odom', 'dashboard'}
    assert nodes['wheel_odom']['parameters'][1]['publish_tf'] is True
    assert nodes['wheel_odom']['remappings'] == [('odom', '/odom')]
    assert nodes['command_mux']['parameters'][0]['allow_navigation'] is False
    assert nodes['dashboard']['parameters'][0]['session_mode'] == 'manual'
    assert nodes['serial_hub']['parameters'][0]['serial_port'] == '/dev/test-only'
    assert sum('bridge' in a for a in actions) == 1


@pytest.mark.parametrize('navigation', [False, True])
def test_robot_modes_keep_ekf_and_single_dashboard(construction, monkeypatch, navigation):
    seen = []
    monkeypatch.setattr(robot, 'mapping_actions', lambda _: seen.append('mapping') or [])
    monkeypatch.setattr(robot, 'navigation_actions', lambda _: seen.append('navigation') or [])
    actions = robot.robot_actions(construction, navigation=navigation)
    assert seen == ['navigation' if navigation else 'mapping']
    nodes = {a['executable']: a for a in actions if 'executable' in a}
    assert 'ekf_node' in nodes
    assert nodes['wheel_odom']['parameters'][1]['publish_tf'] is False
    assert nodes['wheel_odom']['remappings'] == [('odom', '/wheel/odom')]
    assert nodes['dashboard']['parameters'][0]['session_mode'] == seen[0]
    assert sum(a.get('executable') == 'dashboard' for a in actions) == 1
    assert sum('bridge' in a for a in actions) == 1


def test_manual_bench_mode_omits_serial(construction):
    construction.launch_configurations['enable_motors'] = 'false'
    assert all(a.get('executable') != 'serial_hub'
               for a in robot.robot_actions(construction, manual=True))
