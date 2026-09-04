#!/usr/bin/env bash
# ============================================================
# FIX 093f — Diagnose: math3_inverse epsilon threshold
#   Prints the math3_inverse source and sweeps diagonal magnitudes
#   to find the exact epsilon where it starts returning zero.
#   READ-ONLY. This is the last diagnostic before the fix.
#
# Phase:   phase3_sensors (cylinder keystone)
# Files:   none modified
# Depends: 093e2
# Risk:    none (read-only)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

M3D="v15R2/src/core/math3D.h"
SRC="v15R2/src"

echo "============================================================"
echo "[1] math3_inverse source (from math3D.h)"
echo "============================================================"
awk '
/math3_inverse[ \t]*\(/ && !in_func { in_func = 1 }
in_func {
    print
    if ($0 ~ /^\}/) { in_func = 0; exit }
}
' "$M3D"

echo ""
echo "[1b] grep line numbers for math3_inverse (fallback reference):"
grep -n 'math3_inverse' "$M3D" | head -8

echo ""
echo "============================================================"
echo "[2] magnitude sweep: where does math3_inverse break?"
echo "============================================================"

cat > /tmp/math3_sweep_093f.c << 'SWEEP_EOF'
#include <stdio.h>
#include <math.h>
#include "core/math3D.h"

int main(void) {
    float vals[] = {1.0f, 0.1f, 0.01f, 1e-3f, 1e-4f, 1e-5f, 1e-6f, 1e-7f, 1e-8f, 1e-10f};
    for (int i = 0; i < 10; i++) {
        float v = vals[i];
        math3 m = (math3){{{v, 0, 0}, {0, v, 0}, {0, 0, v}}};
        math3 inv = math3_inverse(m);
        float expected = 1.0f / v;
        int ok = (fabsf(inv.matrix[0][0] - expected) < fabsf(expected) * 0.01f);
        printf("v=%.0e  det=%.2e  inv[0][0]=%.4e  expect=%.4e  %s\n",
               (double)v, (double)(v * v * v), (double)inv.matrix[0][0],
               (double)expected, ok ? "OK" : "BROKEN");
    }
    return 0;
}
SWEEP_EOF

cd "$SRC"
if gcc -I. -O2 -o /tmp/math3_sweep_093f /tmp/math3_sweep_093f.c -lm 2>/tmp/sweep_build_093f.log; then
    /tmp/math3_sweep_093f
else
    echo "[DIAG] sweep failed to compile — math3D.h needs more includes:"
    cat /tmp/sweep_build_093f.log
fi

echo ""
echo "[DIAG] paste the math3_inverse source + sweep output back."
