#!/usr/bin/env bash
# ============================================================
# FIX 067 — ARCH: run constraints inside physics_world_step
#   constraint_apply_motors() before velocity integration (torque
#   accumulator), constraint_solve_all() after the contact solver and
#   before position integration. Adds the constraint.h include.
# Phase:   phase1_constraints
# Files:   v15R3/src/core/physics_world.c
# Depends: 059c, 063
# Risk:    medium (three targeted inserts; compile-gated)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R3/src/core/physics_world.c"
[[ -f "$TARGET" ]] || { echo "[SKIP] physics_world.c not found"; exit 0; }
grep -q 'MPE_FTC_067' "$TARGET" && { echo "[SKIP] constraints already wired"; exit 0; }
grep -q 'contact_cache_save (world_manifolds, manifold_count);' "$TARGET" || { echo "[SKIP] solver anchor not found"; exit 0; }
grep -q 'float angular_damping = powf (g_cfg.world.drag \* 0.97f, dt);' "$TARGET" || { echo "[SKIP] damping anchor not found"; exit 0; }
cp "$TARGET" "${TARGET}.pre_067"

# 1. include constraint.h
sed -i '/#include "..\/physics\/broadphase.h"/a #include "../physics/constraint.h" /* MPE_FTC_067 */' "$TARGET"
# 2. motors before the velocity-integration loop
sed -i '/float angular_damping = powf (g_cfg.world.drag \* 0.97f, dt);/a\    constraint_apply_motors (world->bodies, world->body_count, dt); /* MPE_FTC_067 */' "$TARGET"
# 3. positional/axis solve after the contact solver, before position integration
sed -i '/contact_cache_save (world_manifolds, manifold_count);/a\    constraint_solve_all (world->bodies, world->body_count, dt); /* MPE_FTC_067 */' "$TARGET"

# Postflight
grep -q '#include "../physics/constraint.h"' "$TARGET" || { echo "[FAIL] include not added"; exit 1; }
grep -q 'constraint_apply_motors (world->bodies, world->body_count, dt);' "$TARGET" || { echo "[FAIL] motor pass not inserted"; exit 1; }
grep -q 'constraint_solve_all (world->bodies, world->body_count, dt);' "$TARGET" || { echo "[FAIL] solve pass not inserted"; exit 1; }
echo "[PASS] 067: physics_world_step now runs constraints + motors"
