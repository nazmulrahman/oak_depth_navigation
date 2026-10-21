"""No root, USB writes or udev reloads: rule generation and mutation boundaries."""
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

spec = importlib.util.spec_from_file_location('usb_setup', Path(__file__).parents[1] / 'scripts/setup_usb_udev.py')
usb = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = usb
spec.loader.exec_module(usb)


def example(subsystem='tty', serial='ABC123'):
    ep = usb.Endpoint(Path('/sys/devices/test/endpoint'), '/dev/ttyUSB0', subsystem,
                      {'ID_PATH': 'pci-0000:00:14.0-usb-0:2:1.0', 'ID_USB_INTERFACE_NUM': '00'}, '00')
    device = usb.Device(Path('/sys/devices/test'), '0403', '6001', serial, 'Arduino', [ep])
    return device, ep


def test_serial_rule_does_not_depend_on_tty_number():
    d, e = example()
    rule, group = usb.make_rule(d, e, 'dwm_nano', 'serial')
    assert 'ATTRS{serial}=="ABC123"' in rule
    assert 'ENV{ID_USB_INTERFACE_NUM}=="00"' in rule
    assert 'ttyUSB0' not in rule
    assert 'SYMLINK+="dwm_nano"' in rule and group == 'dialout'


def test_no_serial_and_duplicate_serial_require_port():
    d, e = example(serial='')
    assert not usb.serial_usable(d, e, [d])
    d, e = example()
    twin, _ = example()
    assert not usb.serial_usable(d, e, [d, twin])
    rule, _ = usb.make_rule(d, e, 'controller_left', 'port')
    assert 'ENV{ID_PATH}==' in rule and 'ATTRS{serial}' not in rule


def test_camera_indices_remain_distinct_in_both_modes():
    d, e = example('video4linux')
    e.index = '0'
    for mode in ['serial', 'port']:
        rule, group = usb.make_rule(d, e, 'front_camera', mode)
        assert 'ATTR{index}=="0"' in rule and group == 'video'


def test_multiport_ambiguous_endpoint_cannot_use_serial():
    d, e = example()
    _, second = example()
    d.endpoints.append(second)
    assert not usb.serial_usable(d, e, [d])


@pytest.mark.parametrize('name', ['../bad', 'dwm"', 'dwm name', 'ttyUSB0', 'video4', 'bad\nRUN', ''])
def test_reject_unsafe_or_kernel_alias_names(name):
    with pytest.raises(ValueError):
        usb.validate_name(name)


def test_reject_rule_injection_and_patterns():
    d, e = example(serial='abc", RUN+="bad')
    assert not usb.serial_usable(d, e, [d])
    with pytest.raises(ValueError):
        usb.make_rule(d, e, 'motor', 'serial')
    for value in ['*', 'a|b', 'bad\nvalue', 'bad\\value', '%k']:
        with pytest.raises(ValueError):
            usb.literal(value)


def test_preserve_existing_rules_and_symlinks(tmp_path):
    path = tmp_path / 'robot.rules'
    assert usb.save_rule(path, '# original\n')
    assert not usb.save_rule(path, '# original\n')
    with pytest.raises(ValueError):
        usb.save_rule(path, '# changed\n')
    link = tmp_path / 'alias.rules'
    link.symlink_to(path)
    with pytest.raises(ValueError):
        usb.save_rule(link, '# changed\n')
    assert path.read_text() == '# original\n'


def test_conflicts_include_unplugged_aliases_and_wildcards(tmp_path):
    rules = tmp_path / 'rules';rules.mkdir()
    (rules / 'old.rules').write_text('SUBSYSTEM=="tty", SYMLINK+="dwm_* spare"\n')
    with pytest.raises(ValueError, match='claimed'):
        usb.check_conflicts('dwm_nano', '# new\n', [rules], tmp_path / 'dev')
    with pytest.raises(ValueError, match='claimed'):
        usb.check_conflicts('spare', '# new\n', [rules], tmp_path / 'dev')
    assert usb.check_conflicts('front_camera', '# new\n', [rules], tmp_path / 'dev').name == '99-oak-usb-front_camera.rules'


def test_oak_has_separate_boot_permissions():
    d, e = example('usb');d.vid = '03e7';e.interface = ''
    rule, group = usb.make_rule(d, e, 'oak_camera', 'serial', True)
    boot = rule.splitlines()[-1]
    assert 'ATTR{idVendor}=="03e7"' in boot
    assert 'idProduct' not in boot and 'SYMLINK' not in boot
    assert group == 'plugdev'


def test_reload_is_targeted_and_verified(tmp_path, monkeypatch):
    d, e = example();node = tmp_path / 'ttyUSB0';node.touch();e.node = str(node)
    (tmp_path / 'dwm_nano').symlink_to(node)
    calls = []
    monkeypatch.setattr(usb.subprocess, 'run', lambda args, **kw: calls.append(args))
    usb.activate(d, e, 'dwm_nano', False, tmp_path)
    assert calls[0] == ['udevadm', 'control', '--reload-rules']
    assert calls[1:3] == [['udevadm', 'trigger', '--action=change', str(p)] for p in [d.path, e.path]]
    assert calls[-1] == ['udevadm', 'settle', '--timeout=10']


def test_reload_failure_is_reported(monkeypatch):
    d, e = example()
    def fail(args, **kwargs):
        raise subprocess.CalledProcessError(1, args)
    monkeypatch.setattr(usb.subprocess, 'run', fail)
    with pytest.raises(subprocess.CalledProcessError):
        usb.activate(d, e, 'dwm_nano', False)


def test_dry_run_never_saves_reloads_or_escalates(monkeypatch, tmp_path, capsys):
    d, e = example()
    monkeypatch.setattr(usb, 'discover', lambda: [d])
    monkeypatch.setattr(usb, 'check_conflicts', lambda *args: tmp_path / 'rule.rules')
    answers = iter(['1', '', 'dwm_nano', 'n'])
    monkeypatch.setattr('builtins.input', lambda prompt: next(answers))
    def forbidden(*args, **kwargs):
        pytest.fail('Dry-run attempted a system mutation')
    for function in ['save_rule', 'activate']:
        monkeypatch.setattr(usb, function, forbidden)
    monkeypatch.setattr(usb.subprocess, 'call', forbidden)
    assert usb.main(['--dry-run']) == 0
    assert 'DRY RUN' in capsys.readouterr().out
    assert not list(tmp_path.iterdir())
