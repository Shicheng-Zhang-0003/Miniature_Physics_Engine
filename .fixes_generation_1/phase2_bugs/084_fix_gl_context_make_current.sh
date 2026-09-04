#!/usr/bin/env bash
# ============================================================
# FIX 084 — Repair: ensure GL context is current before render_init()
#   GTK3's GtkGLArea does not automatically make the GL context
#   current during the "realize" signal. Calling OpenGL functions
#   (via libepoxy) without a current context causes an assertion
#   failure in epoxy_get_proc_address.
#   This script ensures gtk_gl_area_make_current() is called
#   before render_init() in the when_realised callback.
#
# Phase:   phase2_bugs
# Files:   v15R3/src/root_gtk.c
# Depends: 038b
# Risk:    low
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/root_gtk.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }

# Check if already fixed
if grep -q 'gtk_gl_area_make_current' "$TARGET"; then
    echo "[SKIP] gtk_gl_area_make_current already present in root_gtk.c"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_084"

# Insert gtk_gl_area_make_current immediately before render_init()
# inside the when_realised callback.
awk '
/(when_realised|on_realize|on_window_realize)/ { in_realise = 1 }
in_realise && /render_init\s*\(\s*\)/ {
    print "    gtk_gl_area_make_current (GTK_GL_AREA (widget)); /* FIX_084: make GL context current */"
    in_realise = 0
}
{ print }
' "$TARGET" > "${TARGET}.tmp" && mv "${TARGET}.tmp" "$TARGET"

# Postflight
if ! grep -q 'gtk_gl_area_make_current' "$TARGET"; then
    echo "[FAIL] gtk_gl_area_make_current not added"
    exit 1
fi

# Verify build
cd v15R3/src
if make > /tmp/build_084.log 2>&1; then
    echo "[PASS] 084: GL context fix applied, build verified"
else
    tail -10 /tmp/build_084.log
    echo "[FAIL] 084: build failed"
    exit 1
fi
