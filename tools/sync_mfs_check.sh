#!/usr/bin/env bash
# DESPOT-2026-09-26: MFS twin-tree drift guard.
# 461-MFS (standalone repo) and v15S/src/ecosystem/mfs (vendored copy) must
# stay source-identical. Build artifacts (*.o/*.d/*.so/*.dll/*.exe/*.a,
# build/, plugins/) and .git are ignored. Exit 1 on drift, listing files.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
EMBED="$ROOT/v15S/src/ecosystem/mfs"
STAND="$HOME/Desktop/work/projects/461-MFS"
# Allow override: SYNC_MFS_STAND=/path/to/461-MFS
if [ -n "${SYNC_MFS_STAND:-}" ]; then STAND="$SYNC_MFS_STAND"; fi
# Fallback: sibling of project root (…/projects/461-MFS)
if [ ! -d "$STAND" ]; then STAND="$(cd "$ROOT/.." && pwd)/461-MFS"; fi
if [ ! -d "$STAND" ]; then echo "[sync-mfs] standalone not found, skipping ($STAND)"; exit 0; fi
if [ ! -d "$EMBED" ]; then echo "[sync-mfs] embedded not found: $EMBED"; exit 1; fi
DRIFT="$(diff -rq "$STAND" "$EMBED" --exclude=.git --exclude=build --exclude=plugins --exclude='*.o' --exclude='*.d' --exclude='*.so' --exclude='*.dll' --exclude='*.exe' --exclude='*.a' --exclude='*.lib' --exclude='sync_mfs_check.sh' 2>&1 || true)"
if [ -n "$DRIFT" ]; then
  echo "[sync-mfs] DRIFT DETECTED (standalone vs embedded):"
  echo "$DRIFT"
  echo "Fix: cp v15S/src/ecosystem/mfs/<file> <461-MFS>/<file> (or reverse if standalone is newer), then re-run."
  exit 1
fi
echo "[sync-mfs] in sync (source-identical, artifacts ignored)"
