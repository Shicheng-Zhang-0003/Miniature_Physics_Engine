#!/usr/bin/env bash
# ============================================================
# FIX 023 — SAVE-010: remove status/ backup files from disk
# Phase:   phase0_trivial
# Files:   v15R2/src/status/
# Depends: none (gitignore fix already ran)
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

STATUS_DIR="v15R2/src/status"
REMOVED=0

if [[ ! -d "$STATUS_DIR" ]]; then
    echo "[SKIP] $STATUS_DIR not found"
    exit 0
fi

# Remove .bak and .backup files (engine.cfg itself stays)
for f in "$STATUS_DIR"/*.bak "$STATUS_DIR"/*.backup; do
    if [[ -f "$f" ]]; then
        rm "$f"
        REMOVED=$((REMOVED + 1))
    fi
done

# Verify engine.cfg still exists (the live config)
if [[ ! -f "$STATUS_DIR/engine.cfg" ]]; then
    echo "[WARN] engine.cfg missing — engine will use defaults on next start"
fi

echo "[PASS] 023: SAVE-010 fixed — removed $REMOVED backup file(s) from status/"
