#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
touch "$HERE/STOP"
printf '%s\n' 'Autonomous simulation STOP requested. This does not address physical hardware.'
