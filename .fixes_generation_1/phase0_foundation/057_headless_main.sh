#!/usr/bin/env bash
# ============================================================
# FIX 057 — TEST-001: headless entry point (additive)
#   Wrapped in MPE_HEADLESS so the default build is unaffected.
#   Avoids mpe_engine.h -> no GTK dependency.
# Phase:   phase0_foundation
# Files:   v15R3/src/headless_main.c
# Depends: 055, 056
# Risk:    low (new file only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
TARGET="v15R3/src/headless_main.c"
grep -q 'MPE_FTC_057' "$TARGET" 2>/dev/null && { echo "[SKIP] headless_main already present"; exit 0; }

cat > "$TARGET" <<'EOF'
/* MPE_FTC_057: headless entry. Build with -DMPE_HEADLESS via `make headless`. */
#ifdef MPE_HEADLESS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "config/mpe_config.h"
#include "core/physics_world.h"

int main (int argc, char *argv []) {
    int ticks = (argc > 1) ? atoi (argv [1]) : 3600;
    mpe_config_init ();
    physics_world world;
    physics_world_init (&world);
    physics_world_add_sphere (&world, 0.5f, 1.0f, (vector3) {0.0f, 5.0f, 0.0f});
    physics_world_add_cube (&world, (vector3) {2.0f, 5.0f, 0.0f}, (vector3) {0.5f, 0.5f, 0.5f}, 2.0f);
    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < ticks; t++) { physics_world_step (&world, dt); }
    int invalid = 0;
    for (int i = 0; i < world.body_count; i++) {
        rigidbody *rb = &world.bodies [i];
        if ((!isfinite (rb->position.x)) || (!isfinite (rb->position.y)) || (!isfinite (rb->position.z))) {invalid++;}
    }
    printf ("[headless] ticks=%d bodies=%d invalid=%d result=%s\n",
            ticks, world.body_count, invalid, (invalid == 0) ? "PASS" : "FAIL");
    physics_world_cleanup (&world);
    return (invalid == 0) ? 0 : 1;
}
#endif /* MPE_HEADLESS */
EOF

grep -q 'MPE_HEADLESS' "$TARGET" || { echo "[FAIL] headless_main not written"; exit 1; }
echo "[PASS] 057: headless_main.c added"
