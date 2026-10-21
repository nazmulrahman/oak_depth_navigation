#!/usr/bin/env bash
set -e
cat /etc/os-release
uname -m
if [ -r /proc/device-tree/model ]; then tr -d '\0' < /proc/device-tree/model; printf '\n'; fi
if [ -r /etc/nv_tegra_release ]; then cat /etc/nv_tegra_release; fi
dpkg-query -W nvidia-jetpack 2>/dev/null || true
ls /opt/ros 2>/dev/null || true
ls -l /dev/serial/by-id 2>/dev/null || true
lsusb
