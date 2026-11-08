#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cli="$ROOT/tools/arduino-cli"
for sketch in serial_check pca_diagnostics arm_controller; do
  "$cli" --config-file "$ROOT/arduino-cli.yaml" compile --fqbn esp32:esp32:esp32 --output-dir "$ROOT/firmware/build/$sketch" "$ROOT/firmware/$sketch"
done
# Camera profile identified from user front/back photos; physical startup remains to be tested.
"$cli" --config-file "$ROOT/arduino-cli.yaml" compile --fqbn 'esp32:esp32:esp32cam' --output-dir "$ROOT/firmware/build/CameraWebServer" "$ROOT/firmware/CameraWebServer"
python3 - "$ROOT" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1]);items=[]
for file in (root/'firmware/build').rglob('*.bin'):
    items.append({'file':str(file.relative_to(root)),'sha256':hashlib.sha256(file.read_bytes()).hexdigest(),'bytes':file.stat().st_size})
(root/'firmware/build/manifest.json').write_text(json.dumps({'artifacts':items,'camera_profile':'AI-Thinker-style ESP32-CAM identified from user photos; enabled hotspot firmware'},indent=2)+'\n')
PY
