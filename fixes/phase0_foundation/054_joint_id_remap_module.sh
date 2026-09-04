#!/usr/bin/env bash
# ============================================================
# FIX 054 — SAVE: joint/object ID remap module (additive)
#   scene_load reassigns IDs; saved joints reference old IDs and are
#   silently dropped on first tick. Adds a remap table to rewrite IDs.
#   NOTE: wiring into scene_saving.c/scene_load.c is a follow-up
#   compile-gated step (intentionally NOT blind sed).
# Phase:   phase0_foundation
# Files:   v15R3/src/scene/scene_id_remap.h, scene_id_remap.c
# Depends: none
# Risk:    low (new files only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
DIR="v15R3/src/scene"
H="$DIR/scene_id_remap.h"; C="$DIR/scene_id_remap.c"
[[ -d "$DIR" ]] || { echo "[SKIP] $DIR not found"; exit 0; }
if [[ -f "$H" ]] && grep -q 'MPE_FTC_054' "$H"; then
  echo "[SKIP] scene_id_remap already present"; exit 0
fi

cat > "$H" <<'EOF'
/* MPE_FTC_054 */
#ifndef scene_id_remap_h
#define scene_id_remap_h
#include <stdint.h>
void scene_id_remap_reset (void);
void scene_id_remap_add (uint32_t old_id, uint32_t new_id);
uint32_t scene_id_remap_resolve (uint32_t old_id);
#endif
EOF

cat > "$C" <<'EOF'
/* MPE_FTC_054 */
#include "scene_id_remap.h"
#include "../config/mpe_constants.h"
typedef struct { uint32_t old_id; uint32_t new_id; } id_remap_entry;
static id_remap_entry remap_table [mpe_max_bodies];
static int remap_count = 0;
void scene_id_remap_reset (void) { remap_count = 0; }
void scene_id_remap_add (uint32_t old_id, uint32_t new_id) {
    if (remap_count >= mpe_max_bodies) {return;}
    remap_table [remap_count].old_id = old_id;
    remap_table [remap_count].new_id = new_id;
    remap_count++;
}
uint32_t scene_id_remap_resolve (uint32_t old_id) {
    for (int i = 0; i < remap_count; i++) {
        if (remap_table [i].old_id == old_id) {return remap_table [i].new_id;}
    }
    return old_id;
}
EOF

grep -q 'scene_id_remap_resolve' "$H" || { echo "[FAIL] header not written"; exit 1; }
grep -q 'scene_id_remap_resolve' "$C" || { echo "[FAIL] impl not written"; exit 1; }
echo "[PASS] 054: scene_id_remap module added"
