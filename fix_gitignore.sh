#!/usr/bin/env bash
# ============================================================
# FLASH FIX: Rewrite .gitignore for new repo root
#
# Context: .git has been moved up one level so the repo root
#          now contains v15R2/ AND fixes/. The old .gitignore
#          was written for when v15R2/ WAS the root.
#
# This script:
#   1. Writes a new root-level .gitignore covering everything
#   2. Deletes the old v15R2/.gitignore (now redundant)
#   3. Purges any already-cached .o / .d / build junk from git
#   4. Stages the result
#
# Run from: the folder that contains v15R2/ and .git/
# ============================================================
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

# --- Safety: make sure we're in the right place ---
if [[ ! -d "v15R2" ]]; then
    echo "[FAIL] v15R2/ not found here. Run this from the repo root."
    exit 1
fi
if [[ ! -d ".git" ]]; then
    echo "[FAIL] .git/ not found here. Run this from the repo root."
    exit 1
fi

echo "--- Writing new root .gitignore ---"

cat > .gitignore << 'GITIGNORE_EOF'
# ============================================================
# MPE root .gitignore
# Repo root contains: v15R2/  fixes/  run_all.sh  verify.sh
# ============================================================

# ============ Build output (matches anywhere in tree) ============
*.o
*.d
*.obj
*.a
*.so
*.exe
engine
build/

# ============ Fix-script backups ============
# Every fix script creates .pre_NNN backups before mutating
*.pre_*

# ============ Backup / scratch files ============
*.bak
*.bak_*
*.orig
*.rej
*~

# ============ Editor swap files ============
*.swp
*.swo
.*.swp

# ============ Editors / IDEs ============
.vscode/
.idea/
*.sublime-project
*.sublime-workspace
compile_commands.json
.cache/

# ============ OS cruft ============
.DS_Store
Thumbs.db
desktop.ini

# ============ Runtime state (engine writes at startup) ============
# status/ dirs contain engine.cfg, scene.dat — regenerated
status/

# ============ Logs ============
# fix_log.txt IS tracked (execution history). Only stray logs ignored.
*.log

# ============ Sanitizer / temp build artifacts ============
/tmp/
*.san
GITIGNORE_EOF

echo "[OK]   .gitignore written ($(wc -l < .gitignore) lines)"

# --- Remove the old nested .gitignore ---
if [[ -f "v15R2/.gitignore" ]]; then
    rm "v15R2/.gitignore"
    echo "[OK]   v15R2/.gitignore removed (now covered by root)"
else
    echo "[SKIP] v15R2/.gitignore already gone"
fi

# --- Purge cached junk that git might already be tracking ---
echo ""
echo "--- Purging cached build artifacts from git index ---"

PURGED=0

# .o files
if git ls-files --cached '*.o' | grep -q .; then
    git rm -r --cached --quiet '*.o' 2>/dev/null || true
    PURGED=$((PURGED + 1))
    echo "  purged *.o from index"
fi

# .d files
if git ls-files --cached '*.d' | grep -q .; then
    git rm -r --cached --quiet '*.d' 2>/dev/null || true
    PURGED=$((PURGED + 1))
    echo "  purged *.d from index"
fi

# build/ directories
if git ls-files --cached 'build/' | grep -q .; then
    git rm -r --cached --quiet 'build/' 2>/dev/null || true
    PURGED=$((PURGED + 1))
    echo "  purged build/ from index"
fi

# engine binary
if git ls-files --cached 'engine' | grep -q .; then
    git rm -r --cached --quiet 'engine' 2>/dev/null || true
    PURGED=$((PURGED + 1))
    echo "  purged engine binary from index"
fi

# status/ dirs
if git ls-files --cached 'status/' | grep -q .; then
    git rm -r --cached --quiet 'status/' 2>/dev/null || true
    PURGED=$((PURGED + 1))
    echo "  purged status/ from index"
fi

# .bak files
if git ls-files --cached '*.bak' | grep -q .; then
    git rm -r --cached --quiet '*.bak' 2>/dev/null || true
    PURGED=$((PURGED + 1))
    echo "  purged *.bak from index"
fi

# .pre_* backup files (in case any exist already)
if git ls-files --cached '*.pre_*' | grep -q .; then
    git rm -r --cached --quiet '*.pre_*' 2>/dev/null || true
    PURGED=$((PURGED + 1))
    echo "  purged *.pre_* from index"
fi

if [[ $PURGED -eq 0 ]]; then
    echo "  (nothing cached that needs purging)"
fi

# --- Stage the new .gitignore ---
git add .gitignore
echo ""
echo "[OK]   .gitignore staged"

# --- Quick verification ---
echo ""
echo "--- Verification ---"
echo "  .gitignore exists:       $(test -f .gitignore && echo YES || echo NO)"
echo "  v15R2/.gitignore gone:   $(test ! -f v15R2/.gitignore && echo YES || echo NO)"
echo "  *.o ignored:             $(git check-ignore -q v15R2/src/test.o 2>/dev/null && echo YES || echo NO)"
echo "  *.d ignored:             $(git check-ignore -q v15R2/src/test.d 2>/dev/null && echo YES || echo NO)"
echo "  build/ ignored:          $(git check-ignore -q v15R2/src/build/ 2>/dev/null && echo YES || echo NO)"
echo "  engine ignored:          $(git check-ignore -q v15R2/src/engine 2>/dev/null && echo YES || echo NO)"
echo "  status/ ignored:         $(git check-ignore -q v15R2/src/status/ 2>/dev/null && echo YES || echo NO)"
echo "  *.pre_* ignored:         $(git check-ignore -q v15R2/src/foo.pre_001 2>/dev/null && echo YES || echo NO)"
echo "  fixes/ NOT ignored:      $(git check-ignore -q fixes/test.sh 2>/dev/null && echo NO || echo YES)"
echo "  fix_log.txt NOT ignored: $(git check-ignore -q fix_log.txt 2>/dev/null && echo NO || echo YES)"

echo ""
echo "=== FLASH FIX COMPLETE ==="
echo "Next: git commit -m 'repo: relocate .gitignore to new root, purge build artifacts'"
