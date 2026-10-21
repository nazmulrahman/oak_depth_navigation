#!/usr/bin/env bash
set -e
_oak_package="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$_oak_package/scripts/env.sh"
mkdir -p "$_oak_package/.local"
touch "$_oak_package/.local/COLCON_IGNORE"
cd "$_oak_package"
colcon --log-base .local/log build --base-paths . --packages-select oak_depth_navigation \
    --build-base .local/build --install-base .local/install --symlink-install
