#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/activate.sh"
cd "$ROOT/ros2_ws"
"$ROOT/.venv/bin/python" -m colcon build --symlink-install --packages-select arm_sorter arm_simulation
echo "In new terminals: source $ROOT/activate.sh"
