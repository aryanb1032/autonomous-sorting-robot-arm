#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sketch="${1:?Usage: bash scripts/upload.sh serial_check /dev/serial/by-id/DEVICE}"
port="${2:?Provide the actual USB serial port}"
case "$sketch" in
  live_trial|virtual_pick|serial_check|pca_diagnostics|arm_controller|single_channel_trial|single_channel_adafruit|channel_sweep_5deg) path="$ROOT/firmware/$sketch";;
  motor_trial)
    path="$ROOT/hardware/motor_trial/arm_controller"
    test -f "$path/hardware_config.h" || { echo "Generate motor trial firmware first."; exit 1; };;
  calibrated_arm)
    path="$ROOT/hardware/generated/arm_controller"
    test -f "$path/hardware_config.h" || { echo 'Generate the measured hardware profile first.'; exit 1; };;
  *) echo 'Use serial_check, pca_diagnostics, arm_controller (locked), calibrated_arm, or motor_trial'; exit 1;;
esac
if [[ "$sketch" == live_trial || "$sketch" == virtual_pick ]]; then
  python3 "$ROOT/tests/check_home90.py"
fi
cli="$ROOT/tools/arduino-cli"
"$cli" --config-file "$ROOT/arduino-cli.yaml" compile --fqbn esp32:esp32:esp32 "$path"
"$cli" --config-file "$ROOT/arduino-cli.yaml" upload --fqbn esp32:esp32:esp32 --port "$port" "$path"
