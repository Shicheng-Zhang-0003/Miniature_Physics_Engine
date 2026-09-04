#!/usr/bin/env bash
# ============================================================
# FIX 012 — DOC-001: delete duplicate config sections in how_to_use.md
# Phase:   phase0_trivial
# Files:   v15R3/how_to_use.md
# Depends: none
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R3/how_to_use.md"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

# Count occurrences of the section header
COUNT=$(grep -c '^## Configuration System (Key 6)' "$TARGET" || true)
if [[ "$COUNT" -le 1 ]]; then
    echo "[SKIP] Only $COUNT occurrence(s) found — already fixed"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_012"

# Find line number of 2nd occurrence and the "## Validation Tests" line
SECOND_LINE=$(grep -n '^## Configuration System (Key 6)' "$TARGET" | sed -n '2p' | cut -d: -f1)
VALIDATION_LINE=$(grep -n '^## Validation Tests' "$TARGET" | head -1 | cut -d: -f1)

if [[ -z "$SECOND_LINE" || -z "$VALIDATION_LINE" ]]; then
    echo "[SKIP] Could not locate section boundaries"
    exit 0
fi

if [[ "$SECOND_LINE" -ge "$VALIDATION_LINE" ]]; then
    echo "[SKIP] Section boundaries inverted — manual review needed"
    exit 0
fi

# Delete from 2nd occurrence up to (but not including) Validation Tests
DELETE_END=$((VALIDATION_LINE - 1))
sed -i "${SECOND_LINE},${DELETE_END}d" "$TARGET"

# Verify only one occurrence remains
REMAINING=$(grep -c '^## Configuration System (Key 6)' "$TARGET" || true)
if [[ "$REMAINING" -ne 1 ]]; then
    echo "[FAIL] Expected 1 occurrence, found $REMAINING"
    exit 1
fi

echo "[PASS] 012: DOC-001 fixed — duplicate config sections removed ($COUNT → 1)"
