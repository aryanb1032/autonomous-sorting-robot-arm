#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Requires python3-venv from setup_pc.sh. CPU baseline; no GPU driver changes.
python3 -m venv --system-site-packages "$ROOT/.venv"
"$ROOT/.venv/bin/python" -m pip install 'setuptools>=77,<80'
"$ROOT/.venv/bin/python" -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
"$ROOT/.venv/bin/python" -m pip install 'numpy<2' 'opencv-python<4.12' 'ultralytics==8.4.189' 'setuptools>=77,<80'
"$ROOT/.venv/bin/python" -m pip check
