#!/usr/bin/env bash
# ============================================================
# FIX 059a — Repair: test_two_world link target missing constraint objects
#
# Root cause: 067 wired constraint_solve_all()/constraint_apply_motors()
#   into physics_world.c, but the test_two_world target (created earlier
#   by 059d) doesn't list physics/constraint.c, so the rebuild fails with
#   "undefined reference to constraint_solve_all".
# Fix: add physics/constraint.c + physics/revolute_joint.c to the
#   TWO_WORLD_SOURCES line (the one ending in `broadphase.c \`).
# Numbered 059a so it runs before 059d in run_all.sh.
#
# Phase:   phase0_foundation
# Files:   v15R3/src/makefile
# Depends: 059d (target exists), 063 + 067 (constraint sources present)
# Risk:    low (one targeted makefile line edit)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
MAKEFILE="v15R3/src/makefile"

[[ -f "$MAKEFILE" ]] || { echo "[SKIP] $MAKEFILE not found"; exit 0; }

# Only applies once the two-world target exists.
if ! grep -q 'TWO_WORLD_SOURCES' "$MAKEFILE"; then
  echo "[SKIP] TWO_WORLD_SOURCES not present yet (fresh tree)"
  exit 0
fi

# The unfixed line ends `... physics/broadphase.c \`. If that pattern is
# gone, the target already links the constraint sources.
if ! grep -qE 'physics/broadphase\.c[[:space:]]*\\[[:space:]]*$' "$MAKEFILE"; then
  echo "[SKIP] test_two_world target already links constraint sources"
  exit 0
fi

cp "$MAKEFILE" "${MAKEFILE}.pre_059a"

sed -E -i 's|physics/broadphase\.c[[:space:]]*\\[[:space:]]*$|physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\|' "$MAKEFILE"

# Postflight: stale pattern gone, constraint sources present.
if grep -qE 'physics/broadphase\.c[[:space:]]*\\[[:space:]]*$' "$MAKEFILE"; then
  echo "[FAIL] TWO_WORLD_SOURCES still ends at broadphase.c"
  exit 1
fi
if ! grep -q 'physics/constraint.c physics/revolute_joint.c' "$MAKEFILE"; then
  echo "[FAIL] constraint sources not present in makefile"
  exit 1
fi

echo "[PASS] 059a: test_two_world target now links constraint.c + revolute_joint.c"
