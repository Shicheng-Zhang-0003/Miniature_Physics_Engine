#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"
export TMPDIR="$ROOT/temp"
export MPE_GAMEPAD_DEVICE=disabled
mkdir -p "$TMPDIR"

echo "Running the full MPE verification profile."
exec python3 tools/test_runner.py --profile full "$@"
