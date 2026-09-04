#!/usr/bin/env bash
# ============================================================
# FIX 059 — ARCH-001 (partial): context-parameterize broadphase
#   broadphase_generate_pairing() and broadphase_update_cell_size()
#   stop reading obj_per_scene/object_count globals; they receive
#   (rigidbody *bodies, int body_count) explicitly. Legacy call
#   sites pass the globals, so behaviour is unchanged. This is the
#   seam that lets physics_world_step() drive collision for any
#   body array — including independent worlds.
# Phase:   phase0_foundation
# Files:   v15R3/src/physics/broadphase.c, broadphase.h
#          v15R3/src/simulation.c (2 call sites)
# Depends: none
# Risk:    medium (rename guarded by exact occurrence counts)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
BP="v15R3/src/physics/broadphase.c"
BH="v15R3/src/physics/broadphase.h"
SIM="v15R3/src/simulation.c"
for f in "$BP" "$BH" "$SIM"; do
  [[ -f "$f" ]] || { echo "[SKIP] $f not found"; exit 0; }
done
grep -q 'MPE_FTC_059' "$BP" && { echo "[SKIP] 059 already applied"; exit 0; }

OC=$(grep -c '\bobject_count\b' "$BP")
OS=$(grep -c '\bobj_per_scene\b' "$BP")
[[ "$OC" -eq 5 ]] || { echo "[SKIP] expected 5 object_count uses in broadphase.c, found $OC"; exit 0; }
[[ "$OS" -eq 4 ]] || { echo "[SKIP] expected 4 obj_per_scene uses in broadphase.c, found $OS"; exit 0; }
grep -q 'static void broadphase_update_cell_size (void) {' "$BP" || { echo "[SKIP] cell-size signature not found"; exit 0; }
grep -q 'int broadphase_generate_pairing (broadphase_pair \*collision_pairs_output_array, int maximum_pairs_allowed) {' "$BP" || { echo "[SKIP] pairing signature not found"; exit 0; }

cp "$BP"  "${BP}.pre_059"
cp "$BH"  "${BH}.pre_059"
cp "$SIM" "${SIM}.pre_059"

# 1. Signatures take explicit context
sed -i 's|static void broadphase_update_cell_size (void) {|static void broadphase_update_cell_size (rigidbody *bodies, int body_count) { /* MPE_FTC_059 */|' "$BP"
sed -i 's|    broadphase_update_cell_size ();|    broadphase_update_cell_size (bodies, body_count);|' "$BP"
sed -i 's|int broadphase_generate_pairing (broadphase_pair \*collision_pairs_output_array, int maximum_pairs_allowed) {|int broadphase_generate_pairing (rigidbody *bodies, int body_count, broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed) { /* MPE_FTC_059 */|' "$BP"

# 2. Replace global reads with parameters (safe: preflight verified scope)
sed -i 's|\bobject_count\b|body_count|g' "$BP"
sed -i 's|\bobj_per_scene\b|bodies|g' "$BP"

# 3. Header declaration
sed -i 's|int broadphase_generate_pairing (broadphase_pair \*collision_pairs_output_array, int maximum_pairs_allowed);|int broadphase_generate_pairing (rigidbody *bodies, int body_count, broadphase_pair *collision_pairs_output_array, int maximum_pairs_allowed); /* MPE_FTC_059 */|' "$BH"

# 4. Legacy call sites pass the legacy globals (behaviour unchanged)
sed -i 's|broadphase_generate_pairing (persistent_collision_pairs, mpe_max_broadphase_pairs)|broadphase_generate_pairing (obj_per_scene, object_count, persistent_collision_pairs, mpe_max_broadphase_pairs)|' "$SIM"
sed -i 's|broadphase_generate_pairing (pair_buffer, mpe_max_broadphase_pairs)|broadphase_generate_pairing (obj_per_scene, object_count, pair_buffer, mpe_max_broadphase_pairs)|' "$SIM"

# Postflight
[[ $(grep -c '\bobject_count\b' "$BP") -eq 0 ]] || { echo "[FAIL] object_count still in broadphase.c"; exit 1; }
[[ $(grep -c '\bobj_per_scene\b' "$BP") -eq 0 ]] || { echo "[FAIL] obj_per_scene still in broadphase.c"; exit 1; }
grep -q 'rigidbody \*bodies, int body_count' "$BH" || { echo "[FAIL] header not updated"; exit 1; }
CALLS=$(grep -c 'broadphase_generate_pairing (obj_per_scene, object_count,' "$SIM")
[[ "$CALLS" -eq 2 ]] || { echo "[FAIL] expected 2 updated call sites in simulation.c, found $CALLS"; exit 1; }
echo "[PASS] 059: broadphase now takes explicit (bodies, count) context"
