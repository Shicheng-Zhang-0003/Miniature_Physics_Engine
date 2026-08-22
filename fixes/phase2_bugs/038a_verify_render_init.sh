#!/usr/bin/env bash
# ============================================================
# FIX 038a — Verify render_init() removal was correct;
#             the surviving call lives in root_gtk.c, not new_render.c
# Phase:   phase2_bugs
# Files:   v15R2/src/render/new_render.c, v15R2/src/root_gtk.c
# Depends: 037, 038
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

RENDER="v15R2/src/render/new_render.c"
GTKROOT="v15R2/src/root_gtk.c"

if [[ ! -f "$RENDER" ]]; then
    echo "[SKIP] $RENDER not found"
    exit 0
fi
if [[ ! -f "$GTKROOT" ]]; then
    echo "[SKIP] $GTKROOT not found"
    exit 0
fi

# 1. The function definition must still exist in new_render.c
if ! grep -q 'void render_init' "$RENDER"; then
    echo "[FAIL] render_init function definition missing from new_render.c"
    exit 1
fi

# 2. render_scene_current must NOT call render_init() any more
if sed -n '/void render_scene_current/,/^}/p' "$RENDER" | grep -q 'render_init ()'; then
    echo "  render_init() still called in render_scene_current — removing"
    cp "$RENDER" "${RENDER}.pre_038a"
    sed -i '/void render_scene_current/,/^}/ {
        /^    render_init ();$/d
    }' "$RENDER"
fi

# 3. when_realised in root_gtk.c MUST still call render_init()
if ! grep -q 'render_init ()' "$GTKROOT"; then
    echo "[FAIL] render_init() call missing from root_gtk.c (when_realised)"
    exit 1
fi

echo "[PASS] 038a: render_init() removed from per-frame path, kept in when_realised"
