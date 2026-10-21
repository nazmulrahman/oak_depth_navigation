"""Export the live occupancy map into a fresh directory without replacing files."""
import argparse
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', help='NEW output directory; must not already exist')
    args = parser.parse_args()
    target = Path(args.directory).expanduser().resolve()
    target.mkdir(parents=True, exist_ok=False)
    result = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-f', str(target / 'map'), '--ros-args', '-p', 'map_subscribe_transient_local:=true',
        '-p', 'save_map_timeout:=15.0'])
    if result.returncode:
        raise SystemExit(result.returncode)
    print(f'Map saved to {target}. Retain the RTAB-Map .db for camera-only localization.')


if __name__ == '__main__':
    main()
