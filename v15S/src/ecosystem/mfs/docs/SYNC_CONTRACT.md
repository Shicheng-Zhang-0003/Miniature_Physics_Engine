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

From either side:

```
475-MPE/tools/sync_mfs_check.sh          # 475 side (canonical)
461-MFS/sync_mfs_check.sh                # standalone side
SYNC_MFS_STAND=/path/to/461-MFS 475-MPE/tools/sync_mfs_check.sh
SYNC_MFS_EMBED=/path/to/embedded/mfs 461-MFS/sync_mfs_check.sh
```

Both copies resolve the other tree by searching a list of candidate
locations rather than assuming one layout, and both print the two resolved
paths so a green result is attributable to a specific pair of trees.

**Exit codes are three-valued, deliberately:**

| code | meaning |
|---|---|
| 0 | in sync — source-identical, artifacts ignored |
| 1 | **drift found** — every differing file is listed |
| 2 | **the guard did not run** — a tree could not be located, or both sides resolved to the same tree |

Exit 2 exists because of the failure this contract failed to catch for its
whole life (DESPOT-2026-10-02). The standalone copy originally resolved the
embedded tree as `$HERE/../v15S/src/ecosystem/mfs`, which is correct from
`<475>/tools/` but wrong from the 461 root — there `HERE/..` is the shared
parent directory, so it looked for `…/projects/v15S/…` instead of
`…/projects/475-MPE/v15S/…`. Every standalone run exited 1 with
"embedded not found", a status indistinguishable from real drift, and the
drift was in fact present and undetected: this tree had drifted 11 files
behind its twin, including four gated tests that did not exist here. **A
guard that cannot tell "no drift" from "did not look" is not a guard.**

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
