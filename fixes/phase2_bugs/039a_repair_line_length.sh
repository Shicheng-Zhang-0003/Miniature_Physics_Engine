#!/usr/bin/env bash
# ============================================================
# FIX 039a — Insert line-length guard into mv_insert_char
#             using awk for reliable multi-line context matching
# Phase:   phase2_bugs
# Files:   v15R2/src/ui_input/microvim.c
# Depends: 039
# Risk:    low
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"

TARGET="v15R2/src/ui_input/microvim.c"

if [[ ! -f "$TARGET" ]]; then
    echo "[SKIP] $TARGET not found"
    exit 0
fi

# Already guarded?
if grep -q 'mv_max_line_len - 1' "$TARGET"; then
    echo "[SKIP] Line-length guard already present"
    exit 0
fi

cp "$TARGET" "${TARGET}.pre_039a"

# Insert the guard inside mv_insert_char, right after
#   int len = mv_line_len (row);
# but ONLY when the next line is
#   char *line = mv.lines [row];
# which uniquely identifies mv_insert_char.
awk '
/int len = mv_line_len \(row\);/ {
    print
    if ((getline nxt) > 0) {
        if (nxt ~ /char \*line = mv\.lines \[row\];/) {
            print "    if (len >= mv_max_line_len - 1) {return;} /* FIX_039: prevent overflow */"
        }
        print nxt
    }
    next
}
{ print }
' "$TARGET" > "${TARGET}.tmp" && mv "${TARGET}.tmp" "$TARGET"

# Postflight
if ! grep -q 'mv_max_line_len - 1' "$TARGET"; then
    echo "[FAIL] Line-length guard not added"
    exit 1
fi

# Make sure it landed inside mv_insert_char, not somewhere else
if ! sed -n '/static void mv_insert_char/,/^}/p' "$TARGET" | grep -q 'mv_max_line_len - 1'; then
    echo "[FAIL] Guard not inside mv_insert_char"
    exit 1
fi

echo "[PASS] 039a: line-length guard added to mv_insert_char"
