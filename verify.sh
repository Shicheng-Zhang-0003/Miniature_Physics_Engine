#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
export TMPDIR="$ROOT/temp"
export MPE_GAMEPAD_DEVICE=disabled
mkdir -p "$TMPDIR"

echo "MPE verification entry point; reports are written below temp/qa_runs/"
exec python3 tools/test_runner.py "$@"
