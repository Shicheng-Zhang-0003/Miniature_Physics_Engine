#!/usr/bin/env bash
# ============================================================
# FIX 002 — BUG-002: compile_asan script is broken
# Phase:   phase0_trivial
# Files:   v15R3/src/compile_asan
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/src/compile_asan"

# --- Preflight ---
if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

if ! grep -q '\.\.\.' "$TARGET"; then
    echo "[SKIP] No literal '...' found — already fixed"
    exit 0
fi

# --- Backup ---
cp "$TARGET" "${TARGET}.pre_002"

# --- Fix: replace entire file with a working script ---
cat > "$TARGET" << 'SCRIPT_EOF'
#!/usr/bin/env bash
# AddressSanitizer + UndefinedBehaviorSanitizer build
set -euo pipefail
cd "$(dirname "$0")"

make clean

SAN_CFLAGS="$(pkg-config --cflags gtk+-3.0 epoxy) -I. -O1 -g -Wall -Wextra \
  -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer"

SAN_LIBS="$(pkg-config --libs gtk+-3.0 epoxy) -lm \
  -fsanitize=address -fsanitize=undefined"

make -j"$(nproc)" CFLAGS="$SAN_CFLAGS" LIBS="$SAN_LIBS"

echo ""
echo "=== ASan+UBSan build complete ==="
echo "Run: ./engine"
echo "Sanitizer reports print to stderr on first violation."
SCRIPT_EOF

chmod +x "$TARGET"

# --- Postflight ---
if grep -q '\.\.\.' "$TARGET"; then
    echo "[FAIL] Literal '...' still present"
    exit 1
fi

echo "[PASS] 002: BUG-002 fixed — compile_asan now uses real pkg-config flags"
