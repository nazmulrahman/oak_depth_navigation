#!/usr/bin/env python3
"""Interactive USB aliases for Ubuntu; standard library only, no ROS required."""
import argparse
from dataclasses import dataclass, field
import fnmatch
import grp
import os
from pathlib import Path
import pwd
import re
import shutil
import subprocess
import sys

RULE_DIRS = [Path(p) for p in ('/etc/udev/rules.d', '/run/udev/rules.d',
                              '/usr/local/lib/udev/rules.d', '/usr/lib/udev/rules.d')]


def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return ''


def properties(path):
    result = subprocess.run(['udevadm', 'info', '--query=property', '--path', str(path)],
                            capture_output=True, text=True, timeout=5)
    return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)


def literal(value):
    # Reject udev patterns, substitutions and syntax, rather than interpolating them.
    if not value or any(c in value for c in '\\"*?[]|$%') or any(ord(c) < 32 for c in value):
        raise ValueError('Device identity contains special characters; use USB-port matching.')
    return '"' + value + '"'


@dataclass
class Endpoint:
    path: Path
    node: str
    subsystem: str
    props: dict = field(default_factory=dict)
    interface: str = ''
    index: str = ''

    def signature(self):
        return self.subsystem, self.interface, self.index


@dataclass
class Device:
    path: Path
    vid: str
    pid: str
    serial: str
    label: str
    endpoints: list = field(default_factory=list)


def discover(sysroot=Path('/sys')):
    devices = []
    for link in sorted((sysroot / 'bus/usb/devices').glob('*')):
        path = link.resolve()
        vid, pid = read(path / 'idVendor'), read(path / 'idProduct')
        if not vid or not pid or read(path / 'bDeviceClass') == '09':
            continue  # Hubs and root hubs are not useful robot aliases.
        device = Device(path, vid, pid, read(path / 'serial'),
                        ' '.join(filter(None, (read(path / 'manufacturer'), read(path / 'product')))))
        for subsystem in ('tty', 'video4linux'):
            for entry in sorted((sysroot / 'class' / subsystem).glob('*')):
                ep = entry.resolve()
                if path not in ep.parents:
                    continue
                props = properties(ep)
                if not props.get('DEVNAME'):
                    continue
                interface = next((read(p / 'bInterfaceNumber') for p in [ep, *ep.parents]
                                  if read(p / 'bInterfaceNumber')), '')
                device.endpoints.append(Endpoint(ep, props['DEVNAME'], subsystem, props,
                                                 interface, read(ep / 'index')))
        props = properties(path)
        if props.get('DEVNAME'):
            device.endpoints.append(Endpoint(path, props['DEVNAME'], 'usb', props))
        if device.endpoints:
            devices.append(device)
    return devices


def serial_usable(device, endpoint, devices):
    try:
        literal(device.serial)
    except ValueError:
        return False
    same_id = [d for d in devices if (d.vid, d.pid, d.serial) ==
               (device.vid, device.pid, device.serial)]
    peers = [e for e in device.endpoints if e.signature() == endpoint.signature()]
    # A composite endpoint must expose its interface in udev as well as sysfs.
    interface_ok = not endpoint.interface or endpoint.props.get('ID_USB_INTERFACE_NUM') == endpoint.interface
    return len(same_id) == 1 and len(peers) == 1 and interface_ok


def validate_name(name):
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]{0,47}', name):
        raise ValueError('Use 1–48 letters, digits, underscores or hyphens, starting with a letter.')
    if re.match(r'^(tty|video|hidraw|sd[a-z]|nvme|loop|usb|bus$|disk$|serial$)', name):
        raise ValueError('Use a custom name such as dwm_nano, front_camera or oak_camera, not a kernel name.')


def make_rule(device, endpoint, name, identity, oak_access=False):
    validate_name(name)
    usb = endpoint.subsystem == 'usb'
    attr = 'ATTR' if usb else 'ATTRS'
    matches = [f'SUBSYSTEM=={literal(endpoint.subsystem)}']
    if usb:
        matches.append('ENV{DEVTYPE}=="usb_device"')
    matches += [f'{attr}{{idVendor}}=={literal(device.vid)}',
                f'{attr}{{idProduct}}=={literal(device.pid)}']
    preamble = []
    if identity == 'serial':
        matches.append(f'{attr}{{serial}}=={literal(device.serial)}')
        if endpoint.interface:
            preamble.append(f'SUBSYSTEM=={literal(endpoint.subsystem)}, IMPORT{{builtin}}="usb_id"')
            matches.append(f'ENV{{ID_USB_INTERFACE_NUM}}=={literal(endpoint.interface)}')
    elif identity == 'port':
        path = endpoint.props.get('ID_PATH', '')
        preamble.append(f'SUBSYSTEM=={literal(endpoint.subsystem)}, IMPORT{{builtin}}="path_id"')
        matches.append(f'ENV{{ID_PATH}}=={literal(path)}')
    else:
        raise ValueError('Unknown identity mode')
    if endpoint.index:
        matches.append(f'ATTR{{index}}=={literal(endpoint.index)}')
    group = {'tty': 'dialout', 'video4linux': 'video', 'usb': 'plugdev'}[endpoint.subsystem]
    line = ', '.join(['ACTION!="remove"', *matches, f'SYMLINK+="{name}"',
                      'MODE="0660"', f'GROUP="{group}"'])
    lines = ['# Created by oak_depth_navigation/scripts/setup_usb_udev.py',
             '# Identity: ' + identity + '; alias: /dev/' + name, *preamble, line]
    if oak_access:
        # OAK changes USB product ID during boot; permissions must cover both states.
        lines += ['# OAK boot/runtime access, for every connected Luxonis/Movidius USB device.',
                  'ACTION!="remove", SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device", '
                  'ATTR{idVendor}=="03e7", MODE="0660", GROUP="plugdev"']
    return '\n'.join(lines) + '\n', group


def check_conflicts(name, content, rule_dirs=RULE_DIRS, devroot=Path('/dev')):
    target = rule_dirs[0] / f'99-oak-usb-{name}.rules'
    if target.is_symlink() or (target.exists() and read(target) != content.strip()):
        raise ValueError(f'Existing rule would be overwritten: {target}. Choose another name.')
    for directory in rule_dirs:
        for rule in directory.glob('*.rules'):
            if rule == target:
                continue
            if rule.name == target.name:
                raise ValueError(f'Rule filename already exists: {rule}')
            # Detect literal and wildcard aliases, including an unplugged device's rule.
            for line in read(rule).splitlines():
                if line.lstrip().startswith('#'):
                    continue
                for names in re.findall(r'\bSYMLINK\s*[:+]?=\s*"([^"]*)"', line):
                    if any(fnmatch.fnmatchcase(name, pattern) for pattern in names.split()):
                        raise ValueError(f'/dev/{name} is already claimed by {rule}')
    node = devroot / name
    if os.path.lexists(node) and not (target.exists() and node.is_symlink()):
        raise ValueError(f'/dev/{name} already exists; choose another name.')
    return target


def save_rule(target, content):
    if target.exists() or target.is_symlink():
        if target.is_symlink() or target.read_text() != content:
            raise ValueError(f'Refusing to overwrite {target}')
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o644)
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
        handle.flush()
        os.fsync(handle.fileno())
    return True


def activate(device, endpoint, name, oak_access, devroot=Path('/dev')):
    subprocess.run(['udevadm', 'control', '--reload-rules'], check=True, timeout=15)
    paths = list(dict.fromkeys([str(device.path), str(endpoint.path)]))
    if oak_access:
        subprocess.run(['udevadm', 'trigger', '--action=change', '--subsystem-match=usb',
                        '--attr-match=idVendor=03e7'], check=True, timeout=15)
    for path in paths:
        subprocess.run(['udevadm', 'trigger', '--action=change', path], check=True, timeout=15)
    subprocess.run(['udevadm', 'settle', '--timeout=10'], check=True, timeout=15)
    alias = devroot / name
    if alias.is_symlink() and alias.resolve() == Path(endpoint.node).resolve():
        print(f'Active: {alias} -> {alias.resolve()}')
    else:
        raise RuntimeError(f'Rule saved and reloaded, but /dev/{name} was not verified. '
                           'Reconnect the device and check again; other rules may conflict.')


def choose(prompt, options):
    while True:
        answer = input(prompt).strip()
        if answer.lower() == 'q':
            return None
        if answer.isdigit() and 1 <= int(answer) <= len(options):
            return options[int(answer) - 1]
        print('Enter a listed number, or q to quit.')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true', help='List devices without prompts or changes')
    parser.add_argument('--dry-run', action='store_true', help='Interactively preview rules without sudo or changes')
    args = parser.parse_args(argv)
    if not shutil.which('udevadm'):
        parser.error('udevadm is required (Ubuntu package: udev).')
    if not args.list and not args.dry_run and os.geteuid() != 0:
        print('Administrator access is needed to save /etc/udev/rules.d and reload udev.', flush=True)
        return subprocess.call(['sudo', sys.executable, str(Path(__file__).resolve()), *(argv or sys.argv[1:])])
    failed = False
    while True:
        devices = discover()
        if not devices:
            print('No USB devices with supported device nodes detected. Reconnect and try again.')
            return 0
        print('\nDetected USB devices (hubs excluded):')
        for i, device in enumerate(devices, 1):
            print(f'[{i}] {device.label or "Unknown device"}  VID:PID={device.vid}:{device.pid}')
            print(f'    Serial: {device.serial or "(none)"} | USB port: {device.path.name}')
            print('    Nodes: ' + ', '.join(e.node for e in device.endpoints))
        if args.list:
            return 0
        device = choose('\nDevice number (q to finish): ', devices)
        if device is None:
            return int(failed)
        for i, endpoint in enumerate(device.endpoints, 1):
            print(f'  [{i}] {endpoint.node} ({endpoint.subsystem})')
        endpoint = device.endpoints[0] if len(device.endpoints) == 1 else choose('Node to name (q to finish): ', device.endpoints)
        if endpoint is None:
            return int(failed)
        if endpoint.subsystem == 'usb':
            print('This is a raw USB alias, not a serial port or /dev/video stream.')
        usable = serial_usable(device, endpoint, devices)
        identity = 'serial' if usable else 'port'
        if usable and endpoint.props.get('ID_PATH'):
            if input('Match by [s]erial (any port) or USB [p]ort? [s]: ').strip().lower() == 'p':
                identity = 'port'
        if identity == 'port':
            print('Using physical USB-port identity. Keep this device in the same USB port/hub.')
        try:
            if identity == 'port':
                path = endpoint.props.get('ID_PATH')
                peers = [e for d in devices for e in d.endpoints
                         if e.subsystem == endpoint.subsystem and e.index == endpoint.index
                         and e.props.get('ID_PATH') == path]
                if not path or len(peers) != 1:
                    raise ValueError('No unique udev ID_PATH for this endpoint; cannot create an unambiguous port rule.')
            name = input('Custom name (e.g. dwm_nano or oak_camera; no /dev/ prefix): ').strip()
            oak_access = device.vid == '03e7'
            if oak_access:
                print('OAK access permissions will also cover all 03e7 USB boot/runtime devices.\n'
                      'The alias itself matches this USB identity; it may disappear during firmware boot.\n'
                      'DepthAI discovers the camera over USB and does not require the alias.')
            content, group = make_rule(device, endpoint, name, identity, oak_access)
            target = check_conflicts(name, content)
            grp.getgrnam(group)
            print(f'\n{target}\n{content}')
            if args.dry_run:
                print('DRY RUN: no files, groups or device settings changed.')
            elif input('Save this rule and reload now? [Y/n]: ').strip().lower() not in ('n', 'no'):
                # Confirm the selected physical identity still exists before writing.
                current = discover()
                if not any(d.path == device.path and (d.vid, d.pid, d.serial) ==
                           (device.vid, device.pid, device.serial) and
                           any(e.path == endpoint.path and e.node == endpoint.node for e in d.endpoints)
                           for d in current):
                    raise ValueError('Device changed or disconnected. Select it again.')
                save_rule(target, content)
                print(f'Saved: {target} (existing rules preserved).')
                user = os.environ.get('SUDO_USER', '')
                if user and user != 'root':
                    account = pwd.getpwnam(user)
                    if grp.getgrnam(group).gr_gid not in os.getgrouplist(user, account.pw_gid):
                        subprocess.run(['usermod', '-aG', group, user], check=True, timeout=15)
                        print(f'Added {user} to {group}. Log out/in before running ROS as that user.')
                activate(device, endpoint, name, oak_access)
        except (ValueError, KeyError, OSError, RuntimeError, subprocess.SubprocessError) as error:
            failed = True
            print(f'Could not complete setup: {error}\nAny rule already saved is retained; nothing was deleted.', file=sys.stderr)
        if input('\nConfigure another device? [y/N]: ').strip().lower() not in ('y', 'yes'):
            return int(failed)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print('\nCancelled. Existing files are preserved.')
        sys.exit(130)
