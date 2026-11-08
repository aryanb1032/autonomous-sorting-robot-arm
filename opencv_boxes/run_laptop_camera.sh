#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
mkdir -p "$ROOT/../../work/laptop-camera-live"
exec "$ROOT/.venv/bin/python" "$HERE/box_vision.py" \
  --camera "${1:-0}" --settings "$HERE/settings.json" \
  --output "$ROOT/../../work/laptop-camera-live"
