#!/usr/bin/env bash
# V-01: Build with AddressSanitizer + UndefinedBehaviorSanitizer.
# The F5/F6/F7/F8/F10 torture run itself needs the GUI (manual).
set -euo pipefail
SRC="v15R3/src"
[[ -d "$SRC" ]] || { echo "ERROR: $SRC not found." >&2; exit 1; }

echo "=== V-01: Sanitizer build (ASan + UBSan) ==="
cd "$SRC"
make clean > /dev/null 2>&1 || true

SAN_CFLAGS="$(pkg-config --cflags gtk+-3.0 epoxy) -I. -O1 -g -Wall -Wextra \
 -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer"
SAN_LIBS="$(pkg-config --libs gtk+-3.0 epoxy) -lm \
 -fsanitize=address -fsanitize=undefined"

if make CFLAGS="$SAN_CFLAGS" LIBS="$SAN_LIBS" 2>&1 | tee /tmp/v01_build.log; then
    echo "[PASS] Sanitizer build succeeded -> $SRC/engine"
else
    echo "[FAIL] Sanitizer build failed." >&2
    exit 1
fi

echo ""
echo "=== MANUAL STEP: torture test under sanitizers ==="
echo "  cd $SRC && ./engine"
echo "  Press F5, F6, F7, F8, F10, and F11, then exercise normal use."
echo "  ASan/UBSan reports print to stderr the moment they fire."
echo "Pass criterion: no severe sanitizer errors."
echo "NOTE: this leaves a sanitizer binary; rerun V-02 to rebuild normal."
