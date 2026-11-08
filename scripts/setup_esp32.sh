#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$ROOT/tools/arduino-cli" --config-file "$ROOT/arduino-cli.yaml" core update-index
"$ROOT/tools/arduino-cli" --config-file "$ROOT/arduino-cli.yaml" core install esp32:esp32@3.3.12
for sketch in serial_check arm_controller; do
  "$ROOT/tools/arduino-cli" --config-file "$ROOT/arduino-cli.yaml" compile --fqbn esp32:esp32:esp32 "$ROOT/firmware/$sketch"
done
