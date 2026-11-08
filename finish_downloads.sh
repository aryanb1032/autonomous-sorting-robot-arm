#!/usr/bin/env bash
# Invoked once by setup supervisor. Never flashes a board or moves a servo.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export YOLO_CONFIG_DIR="$ROOT/.yolo"
STATUS="$ROOT/download-status.txt"
echo 'Waiting for downloads and installations.' > "$STATUS"
vision_pid="$1"
arduino_pid="$2"
while kill -0 "$vision_pid" 2>/dev/null; do sleep 10; done
if .venv/bin/python -c 'import ultralytics, torch, cv2, serial; print(ultralytics.__version__, torch.__version__)'; then
  echo 'Vision Python imports: PASS' >> "$STATUS"
  .venv/bin/python -m pip freeze > requirements-installed.txt
  mkdir -p models
  (
    cd models
    ../.venv/bin/python -c 'from ultralytics import YOLO; import numpy as np; m=YOLO("yolov8n-seg.pt"); m.predict(np.zeros((320,320,3),dtype=np.uint8),imgsz=320,device="cpu",verbose=False); print("YOLOv8 CPU smoke test passed")'
  ) && echo 'YOLOv8 model download and CPU inference: PASS' >> "$STATUS" || echo 'YOLOv8 model download/inference: FAILED; see finish-downloads.log' >> "$STATUS"
else
  echo 'Vision install: FAILED; see vision-install.log in work' >> "$STATUS"
fi
while kill -0 "$arduino_pid" 2>/dev/null; do sleep 10; done
if tools/arduino-cli --config-file arduino-cli.yaml core list | grep -q '^esp32:esp32'; then
  echo 'ESP32 core installed: PASS' >> "$STATUS"
  tools/arduino-cli --config-file arduino-cli.yaml compile --fqbn esp32:esp32:esp32 firmware/serial_check && echo 'ESP32 communication sketch compile: PASS' >> "$STATUS" || echo 'ESP32 compile: FAILED; see finish-downloads.log' >> "$STATUS"
else
  echo 'ESP32 core installation: FAILED; see arduino-setup.log in work' >> "$STATUS"
fi
bash check_pc.sh > pc-check-result.txt 2>&1 && echo 'PC dependency check: PASS' >> "$STATUS" || echo 'PC dependency check: incomplete; see pc-check-result.txt' >> "$STATUS"
echo 'Automatic checks finished. No firmware flashed or motors moved.' >> "$STATUS"
