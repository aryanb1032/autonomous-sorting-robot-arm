#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/hardware_env.sh"
"$ROOT/tools/arduino-cli" --config-file "$ROOT/arduino-cli.yaml" core list
"$ROOT/tools/arduino-cli" --config-file "$ROOT/arduino-cli.yaml" board list
python - <<'PY'
import cv2,serial,numpy,rclpy,xacro
from serial.tools import list_ports
print('Vision, serial and ROS Python imports: OK')
ports=list(list_ports.comports())
print('Serial ports:',[(p.device,p.description) for p in ports])
print('No INIT, MOVE, PWM or upload command was sent.')
PY
id -nG
