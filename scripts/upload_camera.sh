#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
port="${1:?Usage: upload_camera.sh PORT EXACT_FQBN}"
fqbn="${2:?Provide the verified board FQBN/options; ESP32 versus ESP32-S3 must not be guessed}"
python3 - "$ROOT" <<'PY'
import re,sys
from pathlib import Path
root=Path(sys.argv[1]);text=(root/'firmware/CameraWebServer/CameraWebServer.ino').read_text()
if not re.search(r'CAMERA_BOARD_CONFIRMED\s*=\s*true',text):raise SystemExit('Camera PCB/pin map is unconfirmed; upload stopped.')
ap = bool(re.search(r'CAMERA_USE_ACCESS_POINT\s*=\s*true', text))
if not ap and '"**********"' in text:raise SystemExit('Enter your local Wi-Fi credentials in the camera sketch first.')
pins=(root/'firmware/CameraWebServer/board_config.h').read_text()
if len(re.findall(r'^\s*#define\s+CAMERA_MODEL_',pins,re.M))!=1:raise SystemExit('Select exactly one correct camera model in board_config.h')
PY
"$ROOT/tools/arduino-cli" --config-file "$ROOT/arduino-cli.yaml" compile --fqbn "$fqbn" "$ROOT/firmware/CameraWebServer"
"$ROOT/tools/arduino-cli" --config-file "$ROOT/arduino-cli.yaml" upload --fqbn "$fqbn" --port "$port" "$ROOT/firmware/CameraWebServer"
