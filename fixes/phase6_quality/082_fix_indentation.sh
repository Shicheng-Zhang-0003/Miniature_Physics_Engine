#!/usr/bin/env bash
# ============================================================
# FIX 082 — CODE QUALITY: Normalize indentation across codebase
#
#   Uses clang-format with the project .clang-format config to
#   normalize indentation in all .c and .h files under v15R3/src/.
#   Creates .pre_082 backups before modifying each file.
#
#   Falls back to a manual sed-based approach if clang-format
#   is not installed.
#
# Phase:   phase6_quality
# Files:   v15R3/src/**/*.c, v15R3/src/**/*.h
# Depends: none
# Risk:    low (formatting only, no logic changes)
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

SRC_DIR="v15R3/src"
CLANG_FORMAT_CONFIG="$ROOT/.clang-format"

# --- Preflight ---
if [[ ! -d "$SRC_DIR" ]]; then
    echo "[SKIP] $SRC_DIR not found"
    exit 0
fi

# Check if already applied (marker file)
MARKER="$ROOT/$SRC_DIR/.indentation_fixed_082"
if [[ -f "$MARKER" ]]; then
    echo "[SKIP] Indentation already normalized (marker exists)"
    exit 0
fi

# Count files to process
FILE_COUNT=$(find "$SRC_DIR" \( -name '*.c' -o -name '*.h' \) | wc -l)
echo "  Found $FILE_COUNT source files to check"

# --- Strategy 1: clang-format (preferred) ---
if command -v clang-format &>/dev/null; then
    echo "  Using clang-format: $(clang-format --version)"

    if [[ ! -f "$CLANG_FORMAT_CONFIG" ]]; then
        echo "[FAIL] .clang-format not found at project root"
        echo "       Create it first (see setup instructions)"
        exit 1
    fi

    CHANGED=0
    SKIPPED=0

    while IFS= read -r -d '' file; do
        # Skip files that are already well-formatted
        if clang-format --dry-run --Werror "$file" &>/dev/null; then
            SKIPPED=$((SKIPPED + 1))
            continue
        fi

        # Backup
        cp "$file" "${file}.pre_082"

        # Format in-place
        clang-format -i --style=file "$file"

        CHANGED=$((CHANGED + 1))
    done < <(find "$SRC_DIR" \( -name '*.c' -o -name '*.h' \) -print0 | sort -z)

    echo "  Formatted: $CHANGED files"
    echo "  Already clean: $SKIPPED files"

# --- Strategy 2: Fallback (basic tab-to-space + trim) ---
else
    echo "  clang-format not found. Using basic fallback."
    echo "  (Install clang-format for full normalization: sudo apt install clang-format)"

    CHANGED=0

    while IFS= read -r -d '' file; do
        NEEDS_FIX=false

        # Check for tabs
        if grep -qP '\t' "$file" 2>/dev/null; then
            NEEDS_FIX=true
        fi

        # Check for trailing whitespace
        if grep -qE ' +$' "$file" 2>/dev/null; then
            NEEDS_FIX=true
        fi

        if [[ "$NEEDS_FIX" == false ]]; then
            continue
        fi

        # Backup
        cp "$file" "${file}.pre_082"

        # Convert tabs to 4 spaces
        sed -i 's/\t/    /g' "$file"

        # Remove trailing whitespace
        sed -i 's/[[:space:]]*$//' "$file"

        # Ensure file ends with newline
        if [[ -s "$file" ]] && [[ "$(tail -c 1 "$file")" != "" ]]; then
            echo "" >> "$file"
        fi

        CHANGED=$((CHANGED + 1))
    done < <(find "$SRC_DIR" \( -name '*.c' -o -name '*.h' \) -print0 | sort -z)

    echo "  Fixed: $CHANGED files (tabs->spaces, trailing whitespace)"
fi

# --- Postflight ---
# Verify no tabs remain
TAB_COUNT=$(find "$SRC_DIR" \( -name '*.c' -o -name '*.h' \) -exec grep -lP '\t' {} \; 2>/dev/null | wc -l)
if [[ "$TAB_COUNT" -gt 0 ]]; then
    echo "  WARNING: $TAB_COUNT files still contain tabs"
fi

# Verify build still works
cd "$SRC_DIR"
if make clean >/dev/null 2>&1 && make >/dev/null 2>&1; then
    echo "  Build verification: PASS"
else
    echo "  Build verification: FAIL"
    echo "  Restoring backups..."
    find "$SRC_DIR" -name '*.pre_082' | while read -r backup; do
        original="${backup%.pre_082}"
        cp "$backup" "$original"
    done
    echo "[FAIL] Build failed after formatting. All files restored."
    exit 1
fi

# Mark as complete
touch "$MARKER"

echo "[PASS] 082: Indentation normalized across $FILE_COUNT files"
