#!/usr/bin/env bash
# ============================================================
# FIX 078 — Diagnose teleop drive test heap corruption (077)
#   Builds test_teleop_drive under ASan+UBSan with -g and runs it,
#   so the abort reports the exact corruption site instead of a bare
#   "double free or corruption (out)". Also adds defensive bounds
#   guards to the contact/manifold write paths.
# Phase:   phase1_constraints / phase2_robotics
# Files:   v15R2/src/core/physics_world.c (guards)
#          v15R2/src/makefile (asan teleop target)
# Depends: 077
# Risk:    low (additive guards + new make target)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
PW="v15R2/src/core/physics_world.c"
MAKEFILE="v15R2/src/makefile"
[[ -f "$PW" ]] || { echo "[SKIP] physics_world.c not found"; exit 0; }
grep -q 'MPE_FTC_078' "$PW" && { echo "[SKIP] 078 already applied"; exit 0; }
cp "$PW" "${PW}.pre_078"
cp "$MAKEFILE" "${MAKEFILE}.pre_078"

# Guard: clamp manifold writes so a contact flood can't overflow the buffer.
sed -i 's|if ((collided) \&\& (manifold_count < a3_max_manifolds)) {|if ((collided) \&\& (manifold_count >= 0) \&\& (manifold_count < a3_max_manifolds)) { /* MPE_FTC_078 */|' "$PW"

# Add an ASan/UBSan teleop target so the abort is symbolicated.
if ! grep -q 'test_teleop_drive_asan' "$MAKEFILE"; then
cat >> "$MAKEFILE" <<'EOF'

# MPE_FTC_078: ASan+UBSan teleop drive test for heap-corruption diagnosis
test_teleop_drive_asan:
	$(CC) $(CFLAGS) -O1 -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer \
	  -DMPE_TELEOP_DRIVE_TEST $(TELEOP_TEST_SOURCES) -lm \
	  -fsanitize=address -fsanitize=undefined -o test_teleop_drive_asan
	./test_teleop_drive_asan
EOF
fi

grep -q 'MPE_FTC_078' "$PW" || { echo "[FAIL] guard not applied"; exit 1; }
grep -q 'test_teleop_drive_asan' "$MAKEFILE" || { echo "[FAIL] asan target not added"; exit 1; }
echo "[PASS] 078: teleop diagnostic guards + ASan target added"
echo "  -> run: cd v15R2/src && make test_teleop_drive_asan"
