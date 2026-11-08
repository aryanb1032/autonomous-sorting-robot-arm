#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
if [[ $# -lt 1 ]]; then
  echo 'Usage: bash run_phone_camera.sh http://PHONE_IP:8080/video [extra detector options]'
  exit 2
fi
STREAM_URL="$1"
shift
exec "$ROOT/.venv/bin/python" "$HERE/box_vision.py" --stream "$STREAM_URL" \
  --settings "$HERE/settings.json" --output "$ROOT/../../work/phone-camera-live" "$@"
