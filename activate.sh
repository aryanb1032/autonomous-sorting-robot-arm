#!/usr/bin/env bash
# Source this file in a bash terminal; it does not start or move hardware.
ROBOT_PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "$ROBOT_PROJECT/.venv/bin/activate" ]]; then
  source "$ROBOT_PROJECT/.venv/bin/activate"
fi
if [[ -f "$ROBOT_PROJECT/ros2_ws/install/setup.bash" ]]; then
  source "$ROBOT_PROJECT/ros2_ws/install/setup.bash"
fi
export PATH="$ROBOT_PROJECT/tools:$PATH"
export ARDUINO_CONFIG_FILE="$ROBOT_PROJECT/arduino-cli.yaml"
export YOLO_CONFIG_DIR="$ROBOT_PROJECT/.yolo"
cd "$ROBOT_PROJECT"
echo 'Robot environment ready. No hardware commands sent.'
