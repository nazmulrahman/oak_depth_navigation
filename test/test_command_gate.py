"""Motor command policy is tested without any hardware or device access."""
import math
from oak_depth_navigation.command_mux import CommandGate


def ready(mode='manual'):
    gate = CommandGate(mode)
    gate.wheels_at = gate.depth_at = gate.odom_at = 10.0
    return gate


def test_manual_takes_over_and_is_bounded():
    g = ready('navigation')
    g.command('navigation', .1, .2, 10)
    g.command('manual', 10, -10, 10)
    assert g.output(10.1) == (.15, -.4, 'manual')
    g.command('navigation', .1, .2, 10.3)
    assert g.mode == 'manual'
    assert g.output(10.4) == (0, 0, 'command timeout')


def test_emergency_release_never_replays_old_commands():
    g = ready('navigation')
    g.command('navigation', .1, 0, 10)
    g.stop(True)
    assert g.output(10.1) == (0, 0, 'emergency stop')
    g.stop(False)
    assert g.mode == 'manual'
    assert g.output(10.2) == (0, 0, 'command timeout')


def test_missing_feedback_and_depth_stop_motors():
    g = ready('navigation')
    g.command('navigation', .1, 0, 10)
    g.depth_at = 0
    assert g.output(10.1) == (0, 0, 'navigation sensors stale')
    g.command('manual', .1, 0, 10.2)
    assert g.output(10.3) == (.1, 0, 'manual')
    assert g.output(10.6) == (0, 0, 'wheel telemetry stale')


def test_nonfinite_command_is_stopped():
    g = ready()
    g.command('manual', math.nan, 0, 10)
    assert g.output(10.1)[:2] == (0, 0)


def test_manual_launch_rejects_navigation_service_without_changing_mode():
    from types import SimpleNamespace
    from oak_depth_navigation.command_mux import CommandMux
    node = SimpleNamespace(gate=ready(), get_parameter=lambda _: SimpleNamespace(value=False))
    response = SimpleNamespace(success=None, message='')
    CommandMux.mode(node, SimpleNamespace(data=True), response)
    assert response.success is False
    assert node.gate.mode == 'manual'
