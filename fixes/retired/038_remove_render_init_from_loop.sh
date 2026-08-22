#!/usr/bin/env bash
# ============================================================
# FIX 038 — REND-010: remove render_init() call from render loop
# Phase:   phase2_bugs
# Files:   v15R2/src/render/new_render.c
# Depends: 037
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/render/new_render.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

# Check if render_init() is called inside render_scene_current
# The pattern is: render_init () on its own line inside render_scene_current
if ! grep -A3 'void render_scene_current' "$TARGET" | grep -q 'render_init'; then
    echo "[SKIP] render_init() not called in render_scene_current"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_038"

# Remove the render_init() call from render_scene_current
# It appears as "    render_init ();" right after the function opening
# We need to be careful to only remove it from render_scene_current, not from when_realised
# The pattern in render_scene_current is: render_init (); followed by if (render_init_status
sed -i '/^    render_init ();$/{
    N
    /if (render_init_status/{
        s/^    render_init ();\n//
    }
}' "$TARGET"

# Postflight: render_init should still exist in when_realised
if ! grep -q 'render_init ();' "$TARGET"; then
    echo "[FAIL] render_init() removed from all locations — when_realised needs it"
    exit 1
fi

echo "[PASS] 038: REND-010 fixed — render_init() removed from per-frame path"
