# MFS Sync Contract (standalone 461 ↔ embedded 475 copy)

This repo (`461-MFS`) and the vendored copy at
`475-MPE/v15S/src/ecosystem/mfs/` are the **same source tree twice**.
There is no subtree, submodule, or merge mechanism between them — only
this contract plus a checker script. Read this before editing either side.

## Direction of truth

Neither side is declared upstream. The rule is: **fix wherever you are,
then mirror immediately, then prove it.** The checker enforces
source-identity; it does not pick a winner.

## What is synced

All `.c` / `.h` / `.md` / `.sh` / `.mk` source and documentation, including
`tests/` and `docs/`. Explicitly **excluded** (build artifacts, never
synced):

- `build/` objects, `*.o`, `*.d`, `*.so`, `plugins/*.so`
- `.git/` (each repo has its own history)
- `sync_mfs_check.sh` itself (lives at repo root here, in `tools/` there)

## Checking

From the 475 side (canonical invocation):

```
475-MPE/tools/sync_mfs_check.sh
# override: SYNC_MFS_STAND=/path/to/461-MFS
```

Exit 0 = source-identical (prints confirmation). Exit 1 = drift, listing
every differing file. A local copy also lives at this repo's root
(`./sync_mfs_check.sh`) for standalone use.

## Mirroring (when the checker fails)

```bash
SRC=.../475-MPE/v15S/src/ecosystem/mfs
DST=.../461-MFS
for f in $(cd $SRC && git status --short . | awk '{print $2}'); do
  cp "$SRC/$f" "$DST/$f"
done
```

(or the reverse direction if the standalone side is newer). Mirror the
**complete** change set — partial mirrors are how the trees silently fork.
Then re-run the checker, rebuild both sides, and re-run the MFS suite from
the engine tree (see `docs/TESTING.md`).

## History and rationale

The trees were unified on 2026-09-26 (despot audit): six MFS files had
drifted (roller/teleport honesty, motor thermal path, odometry
quantization, gamepad triggers, README counts), plus the suite harness
(hotload SIGSEGV, module_1 command schedule, seven physics-truth rigs).
All were mirrored and the checker was added. Known divergence risks that
this contract guards:

- Fixing a bug in one tree and forgetting the other (the pre-contract
  steady state — assume it will happen again without the checker).
- Editing build artifacts instead of sources (`*.so`/`build/` are outputs;
  never copy them across).
- "Temporary" local tweaks during debugging that never get mirrored —
  either commit them to both sides or revert before leaving.
