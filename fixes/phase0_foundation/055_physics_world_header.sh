#!/usr/bin/env bash
# ============================================================
# FIX 055 — ARCH-001 (real): physics_world header, pure sim state
#   Replaces the dead scaffold. No camera/input/UI coupling.
# Phase:   phase0_foundation
# Files:   v15R2/src/core/physics_world.h
# Depends: none
# Risk:    low (whole-file overwrite; nothing reads old fields)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R2/src/core/physics_world.h"
grep -q 'MPE_FTC_055' "$TARGET" 2>/dev/null && { echo "[SKIP] real header already present"; exit 0; }
[[ -f "$TARGET" ]] && cp "$TARGET" "${TARGET}.pre_055"

cat > "$TARGET" <<'EOF'
/* MPE_FTC_055: Real physics world — pure simulation state. No camera/input/UI. */
#ifndef physics_world_h
#define physics_world_h

#include "rigidbody.h"
#include <stdint.h>

typedef struct {
    rigidbody *bodies;
    int body_count;
    int body_capacity;
    uint32_t next_object_id;
} physics_world;

void physics_world_init (physics_world *world);
void physics_world_cleanup (physics_world *world);
int  physics_world_add_sphere (physics_world *world, float radius, float mass, vector3 position);
int  physics_world_add_cube (physics_world *world, vector3 position, vector3 half_extensions, float mass);
void physics_world_clear (physics_world *world);
void physics_world_step (physics_world *world, float dt);
physics_world *physics_world_get_primary (void);

#endif
EOF

grep -q 'physics_world_step' "$TARGET" || { echo "[FAIL] header not written"; exit 1; }
if grep -q 'camera main_camera;' "$TARGET"; then echo "[FAIL] camera coupling still present"; exit 1; fi
echo "[PASS] 055: real physics_world.h installed"
