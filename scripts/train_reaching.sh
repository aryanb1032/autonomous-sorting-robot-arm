#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/simulation_env.sh"
exec python "$ROOT/reinforcement_learning/train.py" "$@"
