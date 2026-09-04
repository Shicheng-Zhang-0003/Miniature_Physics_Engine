set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

RB_H="v15R2/src/core/rigidbody.h"
PW_H="v15R2/src/core/physics_world.h"

for f in "$RB_H" "$PW_H"; do
    [[ -f "$f" ]] || { echo "[SKIP] $f not found"; exit 0; }
done

# Check if the fix is already applied (no trailing '/' after MPE_FTC_090 comments)
if grep -q 'MPE_FTC_090 \*//' "$RB_H"; then
    echo "[SKIP] 092 already applied"
    exit 0
fi

cp "$RB_H" "${RB_H}.pre_092"
cp "$PW_H" "${PW_H}.pre_092"

# Fix rigidbody.h: remove trailing '/' after MPE_FTC_090 comments
sed -i 's|/\* MPE_FTC_090 \*/|/* MPE_FTC_090 */|g' "$RB_H"
sed -i 's|/\* MPE_FTC_090: half-length along axle (X) \*/|/* MPE_FTC_090: half-length along axle (X) */|g' "$RB_H"

# Fix physics_world.h: remove trailing '/' after MPE_FTC_090 comments
sed -i 's|/\* MPE_FTC_090 \*/|/* MPE_FTC_090 */|g' "$PW_H"

# Postflight: verify no trailing '/' after MPE_FTC_090 comments
if grep -q 'MPE_FTC_090 \*//' "$RB_H"; then
    echo "[FAIL] trailing '/' still present in rigidbody.h"
    exit 1
fi
if grep -q 'MPE_FTC_090 \*//' "$PW_H"; then
    echo "[FAIL] trailing '/' still present in physics_world.h"
    exit 1
fi

# Verify build
cd v15R2/src
if make > /tmp/build_092.log 2>&1; then
    echo "[PASS] 092: cylinder headers fixed, build passes"
else
    tail -20 /tmp/build_092.log
    echo "[FAIL] 092: build still failing after fix"
    exit 1
fi
