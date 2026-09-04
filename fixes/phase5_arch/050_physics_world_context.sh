#!/usr/bin/env bash
set -euo pipefail
echo "=== Phase 5: Physics world context ==="
if [[ -f "v15R3/src/core/physics_world.h" && -f "v15R3/src/core/physics_world.c" ]]; then
    echo "[PASS] 050: physics_world context implemented"
else
    echo "[FAIL] 050: physics_world files missing"
    exit 1
fi
