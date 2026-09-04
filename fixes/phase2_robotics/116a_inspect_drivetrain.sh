#!/usr/bin/env bash
# ============================================================
# 116a — READ-ONLY inspector for Path A (real traction fix)
# Prints everything fix 116 needs to know about the current
# drivetrain state. Modifies nothing.
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
RB="v15R2/src/robotics/robot.c"
DT="v15R2/src/robotics/drivetrain.c"

echo "================ 116a: DRIVETRAIN INSPECTION ================"
echo ""
echo "--- [1] Wheel geometry defines (robot.c) ---"
grep -nE '#define[[:space:]]+(WHEEL|CHASSIS)[A-Z_]*' "$RB" || echo "  (none — inline values?)"
echo ""
echo "--- [2] How wheels are added: cylinder or sphere? ---"
grep -n 'physics_world_add_\(cylinder\|sphere\|cube\)' "$RB" | head -8
echo ""
echo "--- [3] Motor torque application in ftc_robot_update ---"
grep -n -B2 -A6 'output_torque' "$RB" | head -30
echo ""
echo "--- [4] chassis_force assignment (drivetrain.c) ---"
grep -n 'mecanum_chassis_force[[:space:]]*=' "$DT"
echo ""
echo "--- [5] chassis_torque assignment + torque scale ---"
grep -n 'mecanum_chassis_torque[[:space:]]*=' "$DT"
grep -nE 'torque_scale|TORQUE_SCALE|rotate[[:space:]]*\*[[:space:]]*[0-9]' "$DT" | head -5
echo ""
echo "--- [6] force scale value ---"
grep -nE 'force_scale|FORCE_SCALE' "$DT" | head -5
echo ""
echo "--- [7] drivetrain_update body (first 20 lines) ---"
awk '/^void drivetrain_update/{f=1} f{print NR": "$0; if(++c>20) exit}' "$DT"
echo ""
echo "--- [8] math3D functions available ---"
M="v15R2/src/core/math3D.h"
for fn in vector3_cross_product vector3_normalisation vector3_scaling vector3_dot vector3_addition vector3_subtraction; do
    if grep -q "$fn" "$M"; then echo "  [OK]   $fn"; else echo "  [MISS] $fn"; fi
done
echo ""
echo "--- [9] force/torque accumulator fields (rigidbody.h) ---"
grep -nE 'force_accumulator|torque_accumulator' v15R2/src/core/rigidbody.h | head -4
echo ""
echo "--- [10] gravity constant available? ---"
grep -rn 'world.gravity' v15R2/src/config/mpe_config_schema.c | head -2
echo ""
echo "================ INSPECTION DONE ================"
