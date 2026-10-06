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
- Fixing a bug in the engine tree's copy and not in the standalone one.
  This is the direction the drift actually runs in. The embedded copy is
  built and exercised by the engine's own verification runs, so it is
  where work lands; the standalone copy is only built when someone
  remembers to.

  **SUPERSEDED 2026-10-06.** "Every drift recorded above was twin ->
  standalone, never the reverse" held for both prior recurrences and is now
  false. The third recurrence ran the OTHER way: 461 was *ahead* of the twin
  on `drivetrain.c` and `robot.h`, carrying a commit literally titled
  "UNSTABLE: Midrefactor ... DO NOT USE". A direction rule inferred from two
  observations is not a rule. The rule is the one at the top of this file:
  fix wherever you are, mirror immediately, prove it. A tree that commits
  "DO NOT USE" state and does not mirror is the failure this contract is
  for, and it happened in the direction the contract claimed was impossible.

## Second recurrence: 2026-10-05

The 2026-10-02 drift happened again, and this time the guard was working
and nobody acted on it. It is recorded here because the mechanism
differs from the first one and the contract needed a new clause for it.

WHAT THE CHECKER SAID. Byte-level drift on 47 files, exit 1.

WHAT THE DRIFT ACTUALLY WAS. 19 files. The other 28 were the engine
tree's tree-wide clang-format pass (space before paren, stacked if-else,
blank lines removed, bracket spacing), which had been applied to the
embedded copy on 2026-10-04 and never to the standalone one. A byte
comparison cannot tell a reformat from a rewrite, and on this
codebase the reformat was larger than the change.

DIRECTION. Established rather than assumed. Comments and whitespace were
normalised away and the token streams compared: every one of the 19
files differed only by insertions on the embedded side. Not one hunk
added content in the standalone direction. The embedded copy was
therefore a strict superset, the mirror direction was unambiguous, and
mirroring could not lose work.

WHAT WAS MIRRORED. All 54 shared source and doc files, embedded ->
standalone. No file was added and none removed; the two file sets were
already identical apart from \`sync_mfs_check.sh\`, which this contract
excludes because it lives at the root here and in \`tools/\` there.
461-only metadata (\`.gitignore\`, \`temp/\`) was left alone.

VERIFICATION, so the green result is attributable rather than assumed:

- \`sync_mfs_check.sh\` exits 0 from both roots.
- Standalone suite 15/15, up from 14; the fifteenth gate is
  \`release_settle\`, which is the gate for the idle-tire work.
- Standalone suite 15/15 again under ASan+UBSan with \`detect_leaks=1\`,
  run through the existing \`MFS_TEST_CFLAGS\` hook, which the standalone
  tree did not previously have a mode for.
- Engine \`--profile full\`: 234 checks, 0 failed, 2 xfailed,
  0 blocking.
- A git worktree at the pre-mirror tag reproduced the ASan ODR abort
  identically, establishing that abort as an artifact of instrumenting
  both the suite binary and the plugin rather than anything the mirror
  introduced.

CLAUSE ADDED. A reformat sweeping one tree makes byte drift useless as a
signal, so a large byte count is not by itself evidence of large
behavioural drift. On this pair of trees, resolve the count before
reporting it: normalise, compare tokens, and state which direction the
insertions run. A drift report that says "47 files" when 28 of them are
formatting trains the reader to ignore drift reports.

## Third recurrence: 2026-10-06

**A RENAME BROKE THE CONTRACT'S CENTRAL INVARIANT, and the drift guard
caught it correctly.** The 2026-10-06 commits renamed `Makefile` ->
`makefile` and `README_MFS.md` -> `readme.md` in the standalone tree ONLY.
Renaming is not an in-place edit: it makes each file "Only in" one side, so
the checker reported drift on four paths at once -- two content diffs plus two
pure renames -- and the contract's promise of source identity was simply no
longer true. The guard was right and the tree was wrong.

Resolution, and the reasoning, since both halves were defensible:

- **`README_MFS.md` -> `readme.md` was kept** in the standalone tree and
  mirrored to the twin. It is a pure documentation rename, every in-tree
  reference was updated in the same pass, and it matches the lowercase
  convention the rest of this project's docs already use.
- **`Makefile` -> `makefile` was REVERTED.** Lowercase `makefile` is found by
  GNU make on Linux but NOT by every `make` on a case-insensitive filesystem,
  and it diverges from the engine tree's documented entry point. A cosmetic
  rename that can break the build on macOS/Windows is a bad trade for the
  contract's sake, and the contract is not worth that. Restored in the
  standalone tree, no reference churn.

CLAUSE ADDED. A rename is a contract event, not a local edit. Before renaming
anything that both sides share, mirror the rename in the same commit that
makes it, or the guard will (correctly) report drift on paths that have no
content difference at all -- which is a confusing report and trains people to
ignore it.

KNOWN LIMIT OF THE GUARD, recorded rather than fixed. Path resolution
ends in a shallow glob over \`*/v15S/src/ecosystem/mfs\`, which will bind
to any stale scratch tree that happens to match. At least seven exist
under \`/tmp/opencode\` (\`orig/\`, \`mpe/\`, \`v15base/\`, \`v15chk/\`,
\`v15fmt/\`, \`v9a/\`, \`v9b/\`, \`v9fmt/\`). The explicit sibling candidate
is tried first, so the real pair resolves correctly in both layouts
today — this is a latent trap, not a live fault. Worth pinning the
candidate list or requiring an explicit override.
