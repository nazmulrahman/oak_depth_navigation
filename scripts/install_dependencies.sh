#!/usr/bin/env bash
# Run on the Jetson as your normal user. No workspace or existing robot files are removed.
set -euo pipefail
source /etc/os-release
case "${VERSION_ID:-}" in
  24.04) oak_distro=jazzy ;;
  22.04) oak_distro=humble ;;
  *) echo 'Supported OS: Ubuntu 24.04 (Jazzy) or 22.04 (Humble).' >&2; exit 1 ;;
esac
echo "Installing ROS 2 $oak_distro dependencies for $(dpkg --print-architecture)."
sudo apt-get update
sudo apt-get install -y curl ca-certificates python3 software-properties-common
sudo add-apt-repository -y universe
if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  oak_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
  oak_apt_dir="$(mktemp -d /tmp/oak-ros-apt.XXXXXX)"
  curl -fL -o "$oak_apt_dir/ros2-apt-source.deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${oak_apt_version}/ros2-apt-source_${oak_apt_version}.${UBUNTU_CODENAME}_all.deb"
  sudo apt-get install -y "$oak_apt_dir/ros2-apt-source.deb"
fi
sudo apt-get update
sudo apt-get install -y \
  "ros-$oak_distro-ros-base" "ros-$oak_distro-depthai-ros-driver" \
  "ros-$oak_distro-rtabmap-odom" "ros-$oak_distro-rtabmap-slam" "ros-$oak_distro-rtabmap-util" \
  "ros-$oak_distro-imu-filter-madgwick" "ros-$oak_distro-robot-localization" \
  "ros-$oak_distro-navigation2" "ros-$oak_distro-nav2-bringup" \
  "ros-$oak_distro-robot-state-publisher" "ros-$oak_distro-cv-bridge" \
  "ros-$oak_distro-rosbridge-suite" "ros-$oak_distro-teleop-twist-keyboard" "ros-$oak_distro-rviz2" \
  python3-colcon-common-extensions python3-rosdep python3-serial python3-yaml \
  python3-numpy python3-opencv python3-pytest
# DepthAI v2 is intentional: the *_v3 driver has a different API.
if [ ! -e /etc/udev/rules.d/99-oak-depth-navigation.rules ]; then
  printf '%s\n' 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-oak-depth-navigation.rules >/dev/null
else
  echo 'Existing OAK udev rules file preserved.'
fi
sudo usermod -aG dialout,plugdev "$(id -un)"
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
echo "Dependencies installed. Log out/in for group access, reconnect OAK, then source /opt/ros/$oak_distro/setup.bash and run scripts/build.sh."
