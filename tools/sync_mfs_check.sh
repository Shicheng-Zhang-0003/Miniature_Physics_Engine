#!/usr/bin/env bash
# DESPOT-2026-09-26: MFS twin-tree drift guard.
# 461-MFS (standalone repo) and v15S/src/ecosystem/mfs (vendored copy) must
# stay source-identical. Build artifacts (*.o/*.d/*.so/*.dll/*.exe/*.a,
# build/, plugins/), test scratch (temp/), and .git are ignored.
# Exit 1 on drift, listing files.
#
# DESPOT-2026-10-02 (operational lie: the guard could not run from the tree
# it was documented for). README/docs/SYNC_CONTRACT.md advertised "a local
# copy also lives at this repo's root (./sync_mfs_check.sh) for standalone
# use". It could never work there. The path resolution was:
#
#     HERE="$(dirname "$0")";  ROOT="$HERE/..";  EMBED="$ROOT/v15S/src/..."
#
# That is correct when the script lives at <repo>/tools/sync_mfs_check.sh
# (the 475 layout: ROOT is the 475 repo root, and v15S/ is under it). But
# the standalone copy lives at the 461 ROOT, so HERE/.. is the shared
# *parent* directory (…/projects), and the script went looking for
#
#     …/projects/v15S/src/ecosystem/mfs      <- does not exist
#
# while the real vendored copy is at
#
#     …/projects/475-MPE/v15S/src/ecosystem/mfs
#
# Result: every standalone run died with "embedded not found" and exit 1,
# which reads like a misconfiguration, not like "you have 1.5k lines of
# undetected drift". The drift it existed to catch was in fact present and
# undetected for the whole life of the contract.
#
# Fix below: search a list of candidate locations instead of assuming one,
# and — the part that actually matters — make "could not locate the twin"
# a DISTINCT, loud, non-zero-status outcome that can never be mistaken for
# "in sync" or for "drift found".
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"

# What is compared: shared SOURCE only, per docs/SYNC_CONTRACT.md
# (".c/.h/.md/.sh/.mk source and documentation, including tests/ and
# docs/"). Repo-level metadata is excluded because it is per-repo by
# nature: 461-MFS has its own .gitignore (added 2026-10-02 with the build
# artifacts it untracks) while 475-MPE has a different one at its own root
# covering its own layout. Requiring those two to match would be requiring
# two repositories to be the same repository.

# ---- locate the embedded (475) copy --------------------------------------
# Ordered candidates: explicit override, then the two known layouts, then a
# shallow search under the sibling project directory.
EMBED=""
if [ -n "${SYNC_MFS_EMBED:-}" ] && [ -d "${SYNC_MFS_EMBED}" ]; then
    EMBED="$SYNC_MFS_EMBED"
elif [ -f "$HERE/../../core/physics_world.c" ] && [ -d "$HERE/../../ecosystem/mfs/modules/ftc" ]; then
    # script at <475>/tools/  ->  <475>/v15S/src/ecosystem/mfs
    EMBED="$(cd "$HERE/../../ecosystem/mfs" 2>/dev/null && pwd || true)"
elif [ -d "$HERE/../475-MPE/v15S/src/ecosystem/mfs" ]; then
    # script at the standalone 461 root -> sibling 475-MPE
    EMBED="$(cd "$HERE/../475-MPE/v15S/src/ecosystem/mfs" && pwd)"
elif [ -d "$HERE/../v15S/src/ecosystem/mfs" ]; then
    # script at a 461-like root that is itself under a v15S parent
    EMBED="$(cd "$HERE/../v15S/src/ecosystem/mfs" && pwd)"
else
    CAND="$(cd "$HERE/../.." 2>/dev/null && pwd || true)"
    for p in "$CAND"/*/v15S/src/ecosystem/mfs "$CAND"/v15S/src/ecosystem/mfs; do
        if [ -d "$p" ] && [ -d "$p/modules/ftc" ]; then EMBED="$(cd "$p" && pwd)"; break; fi
    done
fi

# ---- locate the standalone (461) copy ------------------------------------
STAND=""
if [ -n "${SYNC_MFS_STAND:-}" ] && [ -d "${SYNC_MFS_STAND}" ]; then
    STAND="$SYNC_MFS_STAND"
elif [ -d "$HERE" ] && [ -d "$HERE/modules/ftc" ] && [ -f "$HERE/mfs_internal.c" ]; then
    STAND="$HERE"          # running from the standalone tree itself
else
    for p in "$HERE/../461-MFS" "$HERE/../.."/*/461-MFS "$HOME/Desktop/work/projects/461-MFS"; do
        if [ -d "$p" ] && [ -d "$p/modules/ftc" ]; then STAND="$(cd "$p" && pwd)"; break; fi
    done
fi

# ---- fail loudly and distinguishably -------------------------------------
if [ -z "$EMBED" ]; then
    cat >&2 <<EOF
[sync-mfs] COULD NOT LOCATE THE EMBEDDED TWIN — drift check did NOT run.

  searched relative to: $HERE
  overrides: SYNC_MFS_EMBED=<path to 475-MPE/v15S/src/ecosystem/mfs>

  This is NOT "in sync" and NOT "drift found". It means the guard was
  blind. Exit 2 keeps it distinguishable from exit 1 (drift).
EOF
    exit 2
fi
if [ -z "$STAND" ]; then
    echo "[sync-mfs] standalone not found, skipping ($STAND)" >&2
    exit 0
fi
if [ "$STAND" = "$EMBED" ]; then
    echo "[sync-mfs] both sides resolved to the same tree ($EMBED); nothing to compare" >&2
    exit 2
fi

DRIFT="$(diff -rq "$STAND" "$EMBED" --exclude=.git --exclude=build --exclude=temp --exclude=plugins --exclude='*.o' --exclude='*.d' --exclude='*.so' --exclude='*.dll' --exclude='*.exe' --exclude='*.a' --exclude='*.lib' --exclude=sync_mfs_check.sh --exclude=.gitignore 2>&1 || true)"
if [ -n "$DRIFT" ]; then
  echo "[sync-mfs] DRIFT DETECTED (standalone vs embedded):"
  echo "[sync-mfs]   standalone: $STAND"
  echo "[sync-mfs]   embedded:   $EMBED"
  echo "$DRIFT"
  echo "Fix: cp v15S/src/ecosystem/mfs/<file> <461-MFS>/<file> (or reverse if standalone is newer), then re-run."
  exit 1
fi
echo "[sync-mfs] in sync (source-identical, artifacts ignored)"
echo "[sync-mfs]   standalone: $STAND"
echo "[sync-mfs]   embedded:   $EMBED"