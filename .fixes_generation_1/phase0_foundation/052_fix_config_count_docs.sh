#!/usr/bin/env bash
# ============================================================
# FIX 052 — DOC: config count drift (57 -> 69) in v15R3 docs/gates
#   Registry has 69 params; v15R3 gates/notes still say 57.
# Phase:   phase0_foundation
# Files:   v15R3/RELEASE_GATES.md, v15R3/release_notes_v15R3.md
# Depends: none
# Risk:    low (exact-phrase replacements)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
GATES="v15R3/RELEASE_GATES.md"
NOTES="v15R3/release_notes_v15R3.md"
CHANGED=0

if [[ -f "$GATES" ]] && grep -q 'All 57 tunable parameters' "$GATES"; then
  cp "$GATES" "${GATES}.pre_052"
  sed -i 's/All 57 tunable parameters/All 69 tunable parameters/' "$GATES"
  CHANGED=$((CHANGED+1))
fi
if [[ -f "$NOTES" ]] && grep -q '57 live tunables' "$NOTES"; then
  cp "$NOTES" "${NOTES}.pre_052"
  sed -i 's/57 live tunables/69 live tunables/' "$NOTES"
  CHANGED=$((CHANGED+1))
fi

if [[ $CHANGED -eq 0 ]]; then echo "[SKIP] No 57-count references found"; exit 0; fi
if grep -q 'All 57 tunable parameters' "$GATES" 2>/dev/null; then echo "[FAIL] gates still say 57"; exit 1; fi
if grep -q '57 live tunables' "$NOTES" 2>/dev/null; then echo "[FAIL] notes still say 57"; exit 1; fi
echo "[PASS] 052: v15R3 docs now reference 69 tunables"
