#!/usr/bin/env bash
# ============================================================
# FIX 076a — Repair: physics_world_init must zero the world struct
#
# Fixes the 077 crash: "double free or corruption (out)".
# Root cause: `physics_world world;` is uninitialized on the stack.
# If `bodies` holds non-null garbage, physics_world_init's
# `if (!world->bodies)` guard skips the malloc, and cleanup later
# frees that garbage pointer. Zeroing the struct in init guarantees
# bodies == NULL so the malloc always runs.
#
# Phase:   phase2_robotics
# Files:   v15R2/src/core/physics_world.c
# Depends: 059c
# Risk:    low (defensive init hardening)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R2/src/core/physics_world.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] $TARGET not found"; exit 0; }
grep -q 'MPE_FTC_076a' "$TARGET" && { echo "[SKIP] already applied"; exit 0; }
cp "$TARGET" "${TARGET}.pre_076a"

# 1. Ensure <string.h> is present for memset
if ! grep -q '#include <string.h>' "$TARGET"; then
  sed -i '/#include <math.h>/a #include <string.h> /* MPE_FTC_076a */' "$TARGET"
fi

# 2. Zero the struct at the top of physics_world_init (init function only)
awk '
/^void physics_world_init/ { in_init=1; print; next }
in_init && /if \(!world\) \{return;\}/ {
    print
    print "    memset (world, 0, sizeof (physics_world)); /* MPE_FTC_076a */"
    in_init=0
    next
}
{ print }
' "$TARGET" > "${TARGET}.tmp" && mv "${TARGET}.tmp" "$TARGET"

# Postflight
grep -q 'memset (world, 0, sizeof (physics_world));' "$TARGET" \
  || { echo "[FAIL] memset not inserted into physics_world_init"; exit 1; }
grep -q '#include <string.h>' "$TARGET" \
  || { echo "[FAIL] string.h not included"; exit 1; }

# Verify by rebuilding + running the teleop test if its make target exists
cd v15R2/src
if grep -q '^test_teleop_drive:' makefile 2>/dev/null; then
  if make test_teleop_drive > /tmp/teleop_test.log 2>&1; then
    tail -6 /tmp/teleop_test.log
    echo "[PASS] 076a: physics_world_init hardened; teleop test passes"
  else
    tail -12 /tmp/teleop_test.log
    echo "[FAIL] 076a: teleop test still failing after hardening"
    exit 1
  fi
else
  echo "[PASS] 076a: physics_world_init hardened (teleop target absent; 077 will verify)"
fi
