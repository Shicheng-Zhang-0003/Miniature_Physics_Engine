#!/usr/bin/env bash
# ============================================================
# FIX 022a — Repair: script 022 deleted the closing } of main()
# Phase:   phase0_trivial
# Files:   v15R2/src/root_gtk.c
# Depends: 022
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/root_gtk.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

# Check if the file ends with 'return 0;' (missing closing brace)
LAST_NONBLANK=$(tail -20 "$TARGET" | grep -v '^$' | tail -1)

if [[ "$LAST_NONBLANK" == *"return 0;"* ]]; then
    echo "  Detected missing closing brace — appending }"
    printf '}\n' >> "$TARGET"
elif [[ "$LAST_NONBLANK" == *"}"* ]]; then
    echo "[SKIP] File already ends with }"
    exit 0
else
    echo "[SKIP] Unexpected file ending — manual review needed"
    tail -5 "$TARGET"
    exit 0
fi

# Verify the file now compiles structurally (count braces)
OPEN_BRACES=$(grep -o '{' "$TARGET" | wc -l)
CLOSE_BRACES=$(grep -o '}' "$TARGET" | wc -l)

if [[ "$OPEN_BRACES" -ne "$CLOSE_BRACES" ]]; then
    echo "[FAIL] Brace mismatch: $OPEN_BRACES open vs $CLOSE_BRACES close"
    exit 1
fi

echo "[PASS] 022a: root_gtk.c repaired — closing brace restored"
