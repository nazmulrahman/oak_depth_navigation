#!/usr/bin/env bash
# Source this file. Uses only the package's private build and dependency directories.
_oak_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_oak_distro="${OAK_ROS_DISTRO:-${ROS_DISTRO:-}}"
if [ -z "$_oak_distro" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then _oak_distro=jazzy; else _oak_distro=humble; fi
fi
if [ "$_oak_distro" != jazzy ] && [ "$_oak_distro" != humble ]; then
    echo 'This package supports ROS 2 Jazzy and Humble.' >&2
    return 1
fi
source "/opt/ros/$_oak_distro/setup.bash" || return 1
if [ "$_oak_distro" = humble ] && [ "$(uname -m)" = x86_64 ] && [ -d "$_oak_root/.local/deps/opt/ros/humble" ]; then
    _oak_deps="$_oak_root/.local/deps/opt/ros/humble"
    export AMENT_PREFIX_PATH="$_oak_deps:${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="$_oak_deps:${CMAKE_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$_oak_deps/lib:$_oak_deps/lib/x86_64-linux-gnu:$_oak_root/.local/deps/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
    export PYTHONPATH="$_oak_deps/local/lib/python3.10/dist-packages:$_oak_deps/lib/python3.10/site-packages:$_oak_root/.local/deps/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
    export PATH="$_oak_deps/bin:$PATH"
fi
if [ -f "$_oak_root/.local/install/local_setup.bash" ]; then
    source "$_oak_root/.local/install/local_setup.bash"
fi
unset _oak_root _oak_deps _oak_distro
