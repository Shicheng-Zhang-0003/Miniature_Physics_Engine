# MIG Build Plan — execution playbook for the v16S train

> Companion to [`NON_UNIFORM_BODY.md`](NON_UNIFORM_BODY.md) (the design
> charter, frozen). Charter freezes **what and why**; this document carries
> **how, when, in what order, and how to resume after a lost session**.
> Every remaining work package is a numbered, self-contained step list with
> exact file touch-points, per-step verification commands, and the bitwise
> contract each step must keep. Written 2026-10-30 after WP0 landed.
>
> ⏸ STATUS LINE (keep updated at every step): **WP0 landed and pushed
> (181c2c0 charter, 0882935 WP0 code, 636b10e playbook — 2026-10-30).
> WP1 step 1.1 landed 2026-10-30 (`b0e1cbb`): `mig_pendulum_physical`
> first-run green on the UNMODIFIED engine — measured period 1.8188 s vs
> the corrected closed form 1.8184 s, max_drift 0.0003 m, Steiner term
> 74% of I_pivot; zero pre-existing debt surfaced. RULINGS RECORDED
> 2026-10-30 (§2): P1 RENAME NUF→MIG (MPE-Infinity-Generator); P2 cap 16
> counted confirmed with the hybrid/GPU future-work note; P3 hulls in
> scope as incremental WP2.5 (§8.5); P4 the 1.4 stub lands. THE P1
> RENAME LANDED standalone 2026-10-30 (`84f8c8d`): files/identifiers/
> gates are mig_*, this playbook is MIG_BUILD_PLAN.md. WP1 STEPS 1.2+1.3+
> 1.4 LANDED TOGETHER 2026-10-31 (`df383f0`, one green engine-touching
> commit): the 1.2 `mig_bridge`/`mig_make_bridge` handoff in the pure
> module, the 1.3 `mpe_clamp_composite_events` counter + the
> grows-once body fields (`mass_is_composed`, `composite_part_count`,
> zeroed at all three initializer sites), and the 1.4
> `physics_world_add_composite` stub (composed mass + full authored
> tensor on a sphere-backed body; policy pre-validated, zero clamp
> events; parts dead weight until WP2). ENGINE TRUTH FROM 2026-10-31:
> the engine binary CHANGES BITS for the first time since v15S+MFS-v10S
> (struct grows once, one world entry point, physics_world.c calls
> mig_compose); the plain-scene numeric path is certified unchanged by
> the headless byte-probe A/B (bit-identical) and the in-suite
> determinism twin, not by binary identity. 1.5 verified in the same
> landing (suite 49/47+2, quick 63/63, ASan ×3 mig gates + determinism
> green). The `fix:` follow-up landed 2026-10-31 (`b99d4fc`): the three
> downstream link lists that compile the engine core directly
> (ecosystem/mgb/makefile, ecosystem/mfs/build_tests.sh,
> ecosystem/mfs/mfs_sources.mk) gained `core/mig_mass.c` after the
> author's `make everything` exposed the mgb link failure; mgb tier A
> 44/0 against the modified engine, everything rescanned clean, and the
> standing rule recorded — an engine-touching landing must scan
> `make everything`, not only build_suite. 1.6 landed 2026-10-31:
> PHYSICS.md carries the one-line MIG pointer (charter + build plan +
> the honest WP1 intermediate behavior). MFS INTERRUPTION LANDED
> 2026-11-01 (`2e02c97`, twin `22a8c76`): the v10S aim_solve red gate was
> closed as a test-harness double step (explicit `ftc_fleet_step_all` plus
> the attached module's `pre_step` hook, so two ball-aero passes per tick
> against live's one), suite 32/32 gated pass=5 fail=0 on both trees, sync
> guard green after the twin also gained the b99d4fc core/mig_mass.c link
> lists. WP2 STEP 2.1 LANDED 2026-11-01 (`98debd9`): the fifth object
> type `object_composite` with every earlier numeric value stable, the
> append-only internal `part_base` after the WP1 pair (zeroed at all
> three initializer sites), the per-world contiguous part pool with
> write-on-success `physics_world_composite_parts_append`, per-body
> monotonic indices, inactive slots never reused until clear, and the P2
> cap `composite.max_parts` default 16 (config can only lower; a foreign
> 99 is frozen to 16) counted once through `mpe_clamp_composite_events`;
> gate `mig_composite_pool` is the 50th blocking entry, suite 50/48+2,
> quick 64/64, make everything exit 0. WP2 STEP 2.2 LANDED 2026-11-01
> (`364428d`): `physics_world_add_composite` is the real application
> point — whole-assembly validation including any over-cap tail, the
> body composed over the STORED slice only through `mig_make_bridge`,
> `mig_rebase_parts` COM framing into a per-world pool slice, per-part
> effective kilograms via `mig_part_mass`, the P2 cap counted once only
> after a successful append, and the explicit composed-inertia guard in
> sanitize/wake/set_static with all three `rigidbody_update_inertia_*`
> entry points refusing to overwrite authored tensors; gate
> `mig_composite_world` is the 51st registered blocking entry, suite
> 51/49+2, quick 65/65, make everything exit 0. NEXT: WP2 2.3
> broadphase part dispatch, 2.4 ghost-view narrowphase — each its own
> green commit.**

---

## §0 Resume protocol (read this first, always)

If a session (or its tool budget) died mid-stream, re-establish ground
truth in this order and do not write a new line of code until green:

```bash
cd 475-MPE/v16R1/src
make build_suite            # zero complaints expected (-Werror is default)
./test_mpe_suite --all      # expect: Total: 49 | Blocking failures: 0
cd ../..
python3 tools/test_runner.py --profile quick   # expect Checks: 63 | passed: 63
git status --short          # manifest must match §1 exactly — no half-written files
```

Baseline verdict rules:

- **All green and manifest matches §1** → the tree is at the documented
  baseline. Pick up the next unchecked box in the current WP of §3–§8.
- **Red suite, manifest has half-written files** → the crash interrupted a
  step mid-edit. Revert the half-written file(s) to their last known-good
  state (git for tracked files, §1 manifest numbers for untracked ones),
  re-run the baseline, re-execute the step from its first checkbox.
- **Red and no half-written files** → a real regression. STOP. Diagnose
  before anything new; file it in the ledger in §9; never build new gates
  on red.

Checkpoint rule (applies to every WP below): **an engine-touching change
is committed green, or not committed at all.** Never leave an intermediate
contract state uncommitted at the end of a session. This is the
throttle/crash countermeasure: the reset wall must find the tree either
before or after a step, never inside one.

The session's working state lives in exactly three things: this document
(kept current), the git tree, and the suite numbers. Nothing else needs to
survive a context loss.

---

## §1 Baseline ledger (as of 2026-11-01 — update at every landing)

Landed and pushed (WP0 block: commits `181c2c0` charter, `0882935` code,
`636b10e` playbook; WP1 1.1 in `b0e1cbb`; rulings `8385b95`; P1 rename
`84f8c8d`; WP1 1.2–1.4 in `df383f0` — 2026-10-31; WP2 2.1 in `98debd9`,
WP2 2.2 in `364428d` — 2026-11-01):

| Artifact | Nature |
|---|---|
| `v16R1/docs/NON_UNIFORM_BODY.md` | design charter (frozen decisions) |
| `v16R1/src/core/mig_mass.h` | composition contract + error enums + the 1.2 `mig_bridge` handoff struct + 2.2 `mig_part_mass` (effective per-part kg) |
| `v16R1/src/core/mig_mass.c` | exact composition: mass/COM/inertia, double accumulation, definition-order deterministic; + `mig_make_bridge` + `mig_part_mass` |
| `v16R1/src/core/rigidbody.h/.c` | 1.3 counter + accessor + reset; grows-once body fields `mass_is_composed`/`composite_part_count` (zeroed at all three initializer sites); 2.1 appends `object_composite` + `OBJECT_COMPOSITE`, the engine-internal `part_base` slice anchor, and `mpe_note_composite_clamp`; 2.2 `rigidbody_restore_composed_inertia` + sanitize composite type/geometry branches and composed restores in sanitize/wake/`set_static(false)`, with all three `rigidbody_update_inertia_*` early-returning for composed bodies |
| `v16R1/src/core/physics_world.h/.c` | 1.4 `physics_world_add_composite` stub (composed mass, sphere bounds, authored tensor); 2.1 `composite_part` + per-world pool (`composite_parts`/count/capacity), grower, write-on-success `physics_world_composite_parts_append`, cleanup free + clear compaction; 2.2 completes `physics_world_add_composite` (whole-assembly validation, stored-slice composition under the cap, COM-framed pool slice, effective per-part kg, authored-tensor precondition) with `physics_world_composite_cap` shared by append |
| `v16R1/src/config/mpe_config.h/.c, mpe_config_schema.c` | 2.1 appended `cat_composite` + `composite.max_parts` (default 16, schema 1–16, save loop reaches the new category) |
| `v16R1/src/tests/mpe_suite_mig.c` | gates `mig_mass_composite` + `mig_pendulum_physical` + `mig_composite_stub` + `mig_composite_pool` + `mig_composite_world` (51st, blocking) |
| `v16R1/src/tests/mpe_suite_main.c` | externs + registry entries (51st run blocking) |
| `v16R1/src/makefile` | `MPE_TEST_CORE += core/mig_mass.c`, `MPE_SUITE_SRCS += tests/mpe_suite_mig.c`, help-note renumber |
| `tools/test_runner.py` | floor 46 → 47 → 48 → 49 → 50 → 51 with dated history comments |

Names in this table are the PRESENT (post-P1-rename) spellings; at their
landing commits the files were `nu_mass.{h,c}` and the gates were
`nuf_*` — see §2's mapping and the rename line in §9. The rename landing
also renamed this playbook file itself (`NUF_BUILD_PLAN.md` →
`MIG_BUILD_PLAN.md`).

Measured baseline (numbers a resume session must reproduce):

- Suite: 51 registered, **49/49 blocking green**, 2 diag-informational
  (WP0 landing was 47/45; 1.1 made 48/46; 1.2–1.4 made 49/47; 2.1 made
  50/48; 2.2 made 51/49).
- Runner quick: **65/65** (WP0 was 61/61; 1.1 was 62/62; 1.2–1.4 was
  63/63; 2.1 was 64/64).
- ASan/UBSan suite build: the MIG gates are clean —
  `mig_composite_world`, `mig_composite_pool`, `mig_composite_stub`,
  `mig_mass_composite`, `mig_pendulum_physical`, `determinism` all pass
  with no sanitizer report (run total 51, 47/49 blocking: the asan
  target itself still fails `loader_lifecycle` and `ftc_ecosystem` on
  the pre-existing missing `-rdynamic` export of `g_cfg`, unrelated to
  MIG and recorded in §9).
- Engine binary: **code changes, the plain-scene numeric path does not**
  (`364428d`: no struct grows — `rigidbody` stays 396 bytes — no config
  key moves, and every new branch is gated on `mass_is_composed` or
  `type == object_composite`; `physics_world_add_composite` itself now
  produces the real composite body instead of the WP1 sphere-backed
  stub, and no scene path can construct one yet). Binary identity with
  2.1 is not claimed; the plain-scene numeric path is certified by the
  headless byte-probe against `13ccf9c`, byte-identical
  (ticks=3600 bodies=2 invalid=0 fallback_pow=0 fallback_trig=0
  result=PASS), and the in-suite `determinism` twin stays 600 ticks
  bitwise identical. The WP1 (`df383f0`) and 2.1 (`98debd9`) byte-probe
  entries remain the record of the two bits-changing landings.

WP0's verdict is on record: the one red in its history was the test
author's own oracle (missing cylinder dz-Steiner in the triple's Ixx), not
the implementation — the ledger keeps the story so nobody re-litigates it.

Charter decisions in force (from charter §2, §4, §11), summarized so a
fresh session does not need to re-read the whole charter: **(A) position
stays the COM; no `com_local` field — an off-COM primitive is a one-part
composite. Composites are ONE rigid body with a parts payload, never a
 welded chain (joints remain the connector system). Part indices are
 per-body monotonic, never reused; deletion deactivates. Floor/CCD rotation
 frontier is unchanged. Convex hulls and mesh import are NOT v16S.**

---

## §2 Rulings — recorded resolutions (ruled 2026-10-30, defaults void)

The three pending defaults were put to the author 2026-10-30 and each
came back resolved; they are recorded here exactly as decided, BEFORE the
affected WP's first commit (this section's own protocol). The
identifiers map lives here; the ledger (§9) records the mapping line at
the rename landing.

- **P1 — the name: RENAMED.** Resolution (verbatim intent): *name it
  MIG (MPE-Infinity-Generator), fits the bill as we should be able to
  create near infinite types of objects now.*
  Ride-along amendment (author directive, 2026-10-30): the rename lands
  FIRST as a standalone green commit — *massive changes leave till
  later* — and WP1's body-contract steps (1.2/1.3/1.4) follow in their
  own landings. Binding consequence: the FIRST engine-facing commit is
  born with `mig_` names and carries no rename (a rename after that
  would be grep surgery); the standalone rename touches only test-core
  files (`mig_mass.c` stays outside the engine link list), so the engine
  binary is bit-unchanged through both.
  Mapping as applied at the rename landing:
  `core/nu_mass.{h,c}` became `core/mig_mass.{h,c}`; `nu_compose` →
  `mig_compose`, `nu_rebase_parts` → `mig_rebase_parts`, `nu_part` →
  `mig_part`, `nu_totals` → `mig_totals`, `nu_compose_*` errors →
  `mig_compose_*` (header guard `nu_mass_h` → `mig_mass_h`); gates
  `nuf_mass_composite` → `mig_mass_composite` and
  `nuf_pendulum_physical` → `mig_pendulum_physical` (test functions
  `mpe_t_mig_*`, registry name strings, externs; `tests/mpe_suite_nuf.c`
  became `tests/mpe_suite_mig.c`); spec verb `nuf <name>` →
  `mig <name>`; scene record `NUF1` → `MIG1`;
  `docs/NUF_BUILD_PLAN.md` became `docs/MIG_BUILD_PLAN.md` (charter §9
  cross-link updated); future gate names carry the `mig_` prefix
  (`mig_composite_stack`, `mig_floor_ccd_composite`,
  `mig_pendulum_com_offset`, `mig_scene_roundtrip`). The SCENE TYPE name
  stays `object_composite` (descriptive, not brand). Commit history
  keeps the historical names — the ledger line makes the archaeological
  mapping unambiguous.
- **P2 — part cap: CONFIRMED at 16, counted.** Author note (verbatim):
  *16 for now, hybrid arrangement with chance to change with potential
  gpu possible layer in future.* Consequences committed to:
  `composite.max_parts = 16`, config-schema key in the ship registry,
  counted clamp (keep the FIRST 16 parts, report through
  `mpe_clamp_composite_events`, never a load veto); and the note is
  recorded as a named FUTURE-WORK row, not v16S scope: the parts backing
  store may later grow a GPU-side layer in a hybrid CPU+GPU arrangement —
  the config key is exactly what keeps that from becoming rename-style
  surgery if it arrives.
- **P3 — convex hulls: AMENDED into scope, incremental.** Resolution
  (verbatim intent): *option 2, slowly and incrementally.* Hulls enter as
  **WP2.5** (§8.5, appended below) — foundation-first staged sub-steps,
  each its own green-or-rollback commit, NOT one SAT-charged block. The
  v16S cut carries whichever increments are green at cut time; the rest
  stay flag-OFF with ledger evidence. The original scope-risk framing
  still applies (SAT algebra + divergence-theorem inertia are the two
  real inventions), which is precisely why the staging exists.
- **P4 — the §3 step 1.4 decision (was open): LAND THE STUB.**
  `physics_world_add_composite` enters WP1 with the documented
  intermediate behavior — composed mass + full authored inertia tensor
  on a sphere-backed body, parts recorded inert until WP2. The plan's
  binding caveat applies from day one: the composed tensor is NEVER
  regenerated from type defaults by sanitize (the existing custom-tensor
  guard pattern), so the stub cannot silently replace the very thing WP0
  certified.

---

## §3 WP1 — the mass contract touches the body (first engine change)

**Scope:** let a body's mass tensor arrive from `mig_compose` instead of
the three hardcoded type formulas; wire the counted-clamp bridge. STILL
no composite type, no parts pool (that is WP2). The WP0 suite stays
exactly as green as it was.

- [x] **1.1 Pre-change probe: `mig_pendulum_physical`** in
  `tests/mpe_suite_mig.c` — revolute pendulum with a CUBE bob pivoted at a
  top-face anchor. Measured period vs the full physical-pendulum closed
  form `T = 2π·√(I_pivot/(M·g·d))` with `I_pivot = I_com + M·d²` (the
  setup steps must narrate that these ARE the parallel-axis numbers WP0's
  composition is about to certify). This gate runs on the UNMODIFIED
  engine and SHOULD pass today: it certifies that the existing
  revolute+tensor machinery already behaves physically, so WP2's composite
  gets a green pre-decline. If it is red: file in §9's ledger —
  pre-existing debt, discovered by WP1's near-zero-cost side probe.
  LANDED 2026-10-30 (`b0e1cbb`), first run green, zero debt surfaced:
  measured 1.8188 s vs the amplitude-corrected oracle 1.8184 s (0.02%),
  17 crossings, max_drift 0.0003 m against the 0.02 house bound,
  amplitude decay 0.3% over ~8 periods, ASan clean. The bob is the
  non-cubic cuboid half 0.15×0.6×0.25 (m=2), Steiner term 74% of
  I_pivot = 0.975 — the red clause above stays on the books for future
  probes.
- [x] **1.2 Bridge struct** in `core/mig_mass.h`: the handoff contract WP2
  will consume — `mig_compose` totals + re-anchored part list, marked
  "caller owns; world copies in". No engine include added; keep the
  dependency-clean rule.
  LANDED 2026-10-31 (`df383f0`): `mig_bridge {totals, parts, count}` +
  `mig_make_bridge(src, count, out_parts, out)` — compose once into
  scratch, copy into the caller's buffer, re-anchor the COPY by
  `-totals.com` via `mig_rebase_parts` verbatim; `src` read-only, aliasing
  rejected, write-on-success preserved, named compose errors pass
  through verbatim. The P2 cap is deliberately absent here: exact
  composition over any count, the cap belongs to WP2's stored pool.
- [x] **1.3 Clamp counter:** `mpe_clamp_composite_events` (+ accessor) in
  `core/rigidbody.h` beside the existing block, atomic-per the
  DESPOT-2026-10-12 M4 note (this counter is shared with WP2's cap and
  WP3's load-sum gate).
  LANDED 2026-10-31 (`df383f0`): extern at `rigidbody.h:261`,
  accessor `mpe_clamp_composite_total()` at `:273`, definition at
  `rigidbody.c:60`, reset wired at `:1420`. Zero increments in WP1 BY
  DESIGN — the stub pre-validates and rejects instead of substituting.
  Same commit carries the grows-once body fields (`mass_is_composed`,
  `composite_part_count`) zeroed at all three initializer sites.
- [x] **1.4 Application point:** a NEW world entry point
  `physics_world_add_composite` (this is the WP1 stub that WP2 completes:
  in WP1 it creates a sphere-backed body with the composed
  mass/inertia/local tensor and records parts as DEAD WEIGHT to be
  attached in WP2 — the stage-once pattern the loader already uses; the
  documented intermediate behavior is "composed mass, sphere bounds"). If
  the stub shape feels wrong on implementation day, the alternative is
  WP1 = 1.1–1.3 only and the entry point moves wholly into WP2 — decide
  AT that step, record the choice here, and keep the commit boundary
  green either way.
  LANDED 2026-10-31 (`df383f0`, P4's choice recorded: the stub landed).
  Decl `physics_world.h:209`, implementation `physics_world.c:1384`.
  Whole-assembly validation first; policy PRE-validated at the engine's
  own windows (mass [1e-4, 1e6], bound [0.01, 100], equality legal);
  bound = max_i(|com−p_i| + B_i), B_i sphere r / cube √(hx²+hy²+hz²) /
  cylinder √(hl²+r²) — COM-relative, so the cap case is spread, not a
  moved single part; tensors authored BEFORE the construction sanitize
  pass so no clamp can fire and nothing regenerates (the WP1 form of
  the WP2 2.2 guard). Gate `mig_composite_stub` (49th, blocking) pins:
  dumbbell truth (mass 4, Ixx 0.1 / Iyy=Izz 1.1 vs the 0.9 sphere
  default), multiply-back via the gate's own float algebra, 27-entry
  bitwise tensor survival over 120 free-fall ticks, free-fall vs a
  plain-sphere control within 1e-7 (dy -19.6199 m both), zero-clamp
  sweep over all five counters, nine rejections each with body_count
  unchanged, plus a bridge sub-test on an asymmetric closed form.
  Two gate-side incidents, both fixed gate-side: the bridge-Ixx oracle
  first said 0.2 (WP0-style self-term mistake; true value 0.1 =
  0.025 + 0.075) and the first free-fall assembled at y=0 on the
  default floor (the 0.75-bound body depenetrated up 0.75 m; fixture
  moved to y=30). ASan clean.
- [x] **1.5 Verify:** suite `--all` green 45/45+1(1.1) blocking; quick
  profile (floor moves 47→48 in the same commit as 1.1); ASan gate rerun;
  engine bitwise contract: a plain scene with zero composite bodies must
  be bit-identical — run the existing `determinism` twin unchanged.
  LANDED 2026-10-31 (`df383f0`): `--all` Total 49, 47/47 blocking + 2
  diag green; quick 63/63 (floor moved 48→49 for the new gate); ASan
  rerun on all three mig gates + `determinism`, all clean; the bitwise
  contract holds TWICE — in-suite determinism twin (600 ticks bitwise
  identical) and the headless byte-probe (pre-edit capture
  /tmp/opencode/headless_before.txt vs post-edit build diff:
  byte-identical, ticks=3600 bodies=2 invalid=0 fallback_pow=0
  fallback_trig=0 result=PASS). The engine-binary era changes: v15S
  bit-identity ends by design, replaced by the byte-probe contract.
- [x] **1.6 Docs:** PHYSICS.md gains a one-line pointer to the charter
  (full §9 phase at WP6); ledger notes.
  LANDED 2026-10-31 (docs commit): the pointer sits in PHYSICS.md's top
  blockquote and states all three truths at once — the charter path, the
  build-plan path, and the honest WP1 intermediate behavior (composed
  mass/tensor on a sphere-backed body, parts inert) with the full
  composite equations deferred to WP6. No numbers moved anywhere.

## §4 WP2 — composite type, parts pool, narrowphase part dispatch (the heavy one)

**Scope:** `object_composite` becomes a real first-class type: parts as a
per-world pool, ghost-view narrowphase, part-scoped cache identity,
broadphase/floor/CCD hooks, editor-invisible (UI is WP4). Estimated
1800–2400 LOC. Every step ends green or does not commit.

- [x] **2.1 Type + pool.** Append `object_composite` to the `object_type`
  enum + `OBJECT_COMPOSITE` alias (`core/rigidbody.h`); new per-world
  composite part pool in `core/physics_world.h/.c` (slice
  `part_base/part_count` on the body; monotonic part indices; inactive
  flag; cap + counted clamp from P2). Body append-only fields land in the
  SAME commit as the WP1 bridge so the struct grows once, abi-document
  style — rigidbody is engine-internal and NOT the external plugin ABI
  (`mpe_module.h` is; that bump belongs to 2.4).
  LANDED 2026-11-01 (`98debd9`). Enum/alias at `rigidbody.h:31/:40`
  (values 0–4, every earlier value stable); one additional append-only
  internal field `part_base` at `:174` after the WP1 pair, zeroed at all
  three initializer sites `rigidbody.c:533/:1118/:1349` (the WP1 bridge
  carried no base, so this is the one extra struct append 2.1 requires;
  the plugin ABI is untouched). `composite_part` at
  `physics_world.h:47-60`; world fields at `:71-73`; grower at
  `physics_world.c:889`, append at `:902` — write-on-success, validates
  the stored slice as sphere/cube/cylinder only, authors
  `part_index = first_index + i` and `active = 1`, never mutates the
  source; cleanup frees at `:836`, clear is the sole compaction point at
  `:1506` (count 0, capacity kept). P2 cap: `composite.max_parts` in the
  appended `cat_composite` category (`mpe_config.h:35/:184`;
  `mpe_config_schema.c:279-281`, default 16, schema 1–16;
  `mpe_config.c:260` save loop reaches it); an over-cap request keeps
  the first cap entries, fires `mpe_note_composite_clamp`
  (`rigidbody.c:84`) exactly once through `mpe_clamp_composite_events`,
  and a foreign cap above 16 freezes to 16 in the append. Gate
  `mig_composite_pool` (50th, blocking, `mpe_suite_mig.c:634`,
  registry `mpe_suite_main.c:216`) pins enum values, zeroed body fields,
  20→16 with one counted clamp and no mass-counter leak, source
  untouched, second slice base 16 with indices restarting at 0, inactive
  slot 2 preserved while the next append continues at `first_index` 2,
  four bad-input no-ops (NULL, count 0, negative first_index,
  `object_custom`), cap 4→4 and foreign 99→16 each with one more clamp,
  clear keeping capacity 64, cleanup NULLing pointer/count/capacity. One
  gate-side build incident, engine untouched: the alias equality line
  compared two anonymous enums and `-Werror=enum-compare` rejected it;
  fixed with an int cast in the gate. ASan runs the new gate clean; the
  asan target's two plugin-loader gates stay red on its pre-existing
  missing `-rdynamic` export of `g_cfg` (see §9).
- [x] **2.2 World API.** `physics_world_add_composite` completes the WP1
  stub: `mig_compose` bridge (validate every part, whole-assembly
  invalidation semantics preserved — the composed body refuses nothing a
  caller can count), `mig_rebase_parts` about the COM into the pool slice,
  body mass/inverse/authored `inertia_tensor_local` written, sanitize
  guard extended so a composed tensor is never regenerated from type
  defaults (mirror of the existing custom-type guard),
  `rigidbody_update_axes`-equivalent bounds.
  LANDED 2026-11-01 (`364428d`). `physics_world_add_composite` at
  `physics_world.c:1506` validates the WHOLE assembly with `mig_compose`
  whenever `count` exceeds the cap (a bad part at index 16 of 17 rejects
  with no body, pool slot, or counter movement), then composes only the
  STORED slice through `mig_make_bridge`: 20 one-kilogram parts store the
  first 16, the body weighs 16 kg not 20, bound 0.425, and
  `mpe_note_composite_clamp` fires exactly once after the append
  succeeds; a lowered cap of 4 stores 4 and weighs 4. The body is
  `object_composite` at the assembly COM with `part_base`/
  `composite_part_count` pointing at a COM-framed slice (dumbbell parts
  stay in definition order at +0.5/-0.5), per-part materials copied from
  the config defaults for each part's own primitive type, and per-part
  effective kilograms from `mig_part_mass` (`mig_mass.h:94`,
  `mig_mass.c:184`; density cube 2 kg/m^3 over 1 m^3 lands 2 kg,
  I = 1/3, bound 0.8660254). The cap helper
  `physics_world_composite_cap` (`physics_world.c:888`) is shared with
  the append at `:914` (default 16, frozen ceiling 16, non-positive
  falls back to 16). Pre-append windows reject with -1 and mutate
  nothing: mass [1e-4,1e6], bound [0.01,100], and a finite non-zero
  authored inverse (`physics_world_composite_inverse` at `:1452`);
  `physics_world_composite_part_from_mig` at `:1475` stages the pool
  records before world storage is touched. The explicit guard lives in
  `rigidbody.c`: `rigidbody_restore_composed_inertia` at `:205`
  rebuilds both inverses from `inertia_tensor_local`; `object_composite`
  passes the corrupt-type check at `:240`; the composite geometry branch
  at `:379` keeps the authored radius and never regenerates a tensor;
  the `needs_inertia_recalc` site at `:525`, `rigidbody_wake` at `:1270`
  and `rigidbody_set_static(false)` at `:1310` all restore first; and
  `rigidbody_update_inertia_sphere/cube/cylinder` early-return at
  `:683/:699/:1379` for composed bodies. `physics_world.h:226` documents
  the full order. Gate `mig_composite_world` (51st, blocking,
  `mpe_suite_mig.c:768`, registry `mpe_suite_main.c:222`, runner floor
  51 at `test_runner.py:144`) pins the literals above plus
  `rigidbody_set_static(true)` keeping the authored tensor while zeroing
  inverses, set_static(false)/set_kinematic(true/false)/a direct
  zero-inverse sanitize restoring inv*I within 2e-4 without a type
  formula, ten gravity ticks preserving type/slice/tensor bitwise, and
  the caller's source arrays never mutated. Two incidents, both
  gate-side and engine-untouched: the drafted multi-condition checks
  passed a label as a third argument to the two-argument `MPE_CHECK`
  macro and failed to compile until the labels were removed, and the
  first run failed two part-position checks because the oracle assumed
  the -0.5 part was stored first when `mig_rebase_parts` preserves
  definition order and stores +0.5 first — the engine was right and the
  oracle was fixed. No struct or config change this step (`rigidbody`
  stays 396 bytes); ASan runs the new gate clean while the target's two
  pre-existing plugin-loader gates stay red as recorded in §9.
- [ ] **2.3 Broadphase.** `broadphase_bounding_radius` (~`
  physics/broadphase.c:139`) composites = max over parts of
  `|d_part,+ part bounding radius|` (exact, conservative); the AABB path
  (~:306) gains the composite branch vs per-part axis slicing; the
  spatial hash itself is untouched.
- [ ] **2.4 Narrowphase — ghost-view dispatch (the WP2 crux).** Design,
  verbatim from the charter's "reuses the EXISTING builtin primitive pair
  functions against part-translated poses": per-world scratch `rigidbody`
  ghost views (memcpy of the parent body + part-local pose override,
  refreshed per pair — no malloc, no signature surgery across the 261
  call sites, existing sphere/box/cylinder pair functions byte-for-byte
  untouched). Worker discipline: ghosts are PER-WORKER scratch under
  `MPE_THREADS > 1` (pair-local writes stay disjoint; ghosts are
  read-only to the pair functions after their refresh). Contact emission
  maps back to PARENT identities with the part index stamped per contact.
  Dispatch entry: `mpe_shape_dispatch` branch (`core/physics_world.c` ~:1226,
  also the legacy loop's call at ~:1405).
- [ ] **2.5 Contact identity ≥ day one.** Append-only fields on
  `contact_point_data` and `cached_contact`: `part_a`, `part_b`
  (`physics/collision_mechanics.h` + world cache); cache key becomes
  (id_a, part_a, id_b, part_b). `MPE_MODULE_ABI` 1 → 2 in
  `core/mpe_module.h`, bump documented with the small-struct-read rule the
  stage-destructor append already established; ABI-1 `.so` files keep
  loading and cannot see the new fields. THE ORDER MATTERS: 2.5 lands in
  the SAME commit as 2.4's first manifold map — a composite touching two
  parts with unscoped keys is the silent-aliasing bug this feature must
  never ship with (the TASK_05 both-side-match repair is the precedent
  to cite in the commit).
- [ ] **2.6 Floor + CCD.** Per-part floor manifolds under the existing
  plane path (gate `mig_floor_ccd_composite`: tumbling L-composite
  clamps before tunneling at canonical dt, no landing pump); floor CCD
  lowest-point scan over parts; volume sweeps stay bounding-sphere.
  Rotation-only tunneling stays unsupported — same frontier as v15S,
  now named once for composites in the same sentence.
- [ ] **2.7 Renderer.** composite draws per part through the existing
  cube/sphere/cylinder meshing+instancing paths with part poses;
  body-level `no_collide` respected. (Mesh instance count guard: parts
  are bounded by P2's cap — no renderer unboundedness.)
- [ ] **2.8 Gates** (each its own suite entry, floor bump same commit):
  `mig_composite_stack` (10-primitive stack vs one 10-part composite,
  same mass/materials: both settle asleep); `mig_floor_ccd_composite`;
  part-scoped cache probe (two resting parts → distinct cache entries,
  warm starts never cross); `mig_pendulum_com_offset` — the charter end-
  to-end gate now becomes possible: composite bob with off-COM mass,
  period vs parallel-axis closed form (this gate certifying the whole
  contract through the solver is the thesis of the train); determinism
  twins with composites at threads 1/4/8 (charter §4.5).
- [ ] **2.9 Verify:** quick profile green; `make tsan` (exists) green
  over the suite (ghost scratch is the worker-race surface the TSan
  profile exists for); suite `--all` green at the bumped floor;
  documented A2 statement: zero-composite scenes byte-identical to the
  pre-WP2 run (keep an un-touched `temp/qa_runs` summary from the
  pre-composite commit for the diff).

## §5 WP3 — scene persistence (v200 `MIG1` record)

Charter §5 is the contract; the plan adds touch-points only.

- [ ] **3.1 Writer/reader two-pass:** body rows as v200 today, then a
  `MIG1` extension record per composite (part tuples per charter layout),
  referenced by body id (the id-remap pass already exists —
  `scene/scene_id_remap.*`); CRC covers the stream unchanged.
- [ ] **3.2 Loader kindness (degrade-not-veto):** missing blob →
  bounding-cube placeholder + counted diag (the custom-body precedent in
  `scene/scene_load.c` ~:228 is the template); unknown part type inside a
  blob → that part degrades, scene continues; load-time sum-consistency
  gate (recompose vs stored mass, mismatch = diag + clamp, never a veto).
- [ ] **3.3 Gate:** `mig_scene_roundtrip` (save → load → bitwise pose +
  mass identity) in the same commit as the writer.

## §6 WP4 — terminal verbs + spec format + the tiny CAD editor

Headless before GTK, always. The spec text is the single source of truth;
the editor is a window onto it; no editor-only truth exists.

- [ ] **4.1 Spec grammar v1** (freeze AT this step, diagram becomes the
  reader's spec):

  ```
  # MIG object spec — one file, one object. Reader rejects unknown
  # keywords LOUDLY (named diag), never guesses. `frame assembly` means
  # the part poses below are in the assembly frame and the COM is computed
  # on save (the body is always born COM-framed).
  mig <name>
    frame assembly
    part <index> type sphere|cube|cylinder
      dims radius r [half hl] [hx hx hy hy hz hz]     # per-type subset
      pose px py pz qw qx qy qz
      material mu_s F mu_k F rest F
      mass kg | density rho
      colour r g b
  ```

- [ ] **4.2 Headless surface first.** Parser that fills `mig_part` arrays
  (pure, suite-gated for determinism/reject codes); terminal verbs in
  `ui_input/term_obj.c`: `mig load`, `mig show`, `mig save`, `mig list`
  (spec store is a per-session named table, not a world write). Includes
  the spec-round-trip gate (read + write must reproduce byte-identical
  normalized output) BEFORE any button exists.
- [ ] **4.3 microvim is the editor.** The in-app editing path is literally
  `ui_input/microvim.c` (it already exists, 1390 lines) — `mig edit`
  opens the spec in the microvim buffer; saving re-parses and stages the
  body. "Tiny CAD" step one costs almost nothing and is native.
- [ ] **4.4 GTK panel** (`ui_input/mig_editor.c`, new): part-list panel +
  per-part fields (type, dims, pose, material, mass-or-density, colour)
  in the `config_menu`/`editor_dialog` construction style; two-axis
  translate gizmo first, rotate fields numeric; live COM marker rendered
  in the 3D view plus total-mass readout; principal-axis display is a
  named TODO, not v16S scope. Grammar frozen at 4.1 governs both paths.
- [ ] **4.5 Gate:** editor panel edits → spec save → load from disk →
  bitwise identity, PLUS the WP2 physics gates re-run with editor-created
  bodies (editor and terminal create the same objects through the same
  parser).

## §7 WP5 — MFS adoption (toggle default OFF, A/B or the gate doesn't flip)

Charter §8 is the contract; the plan adds the risk discipline.

- [ ] **5.1** `mfs_robot_build_composite()` in
  `ecosystem/mfs/modules/ftc/submodules/robot.c`: chassis + wheel parts
  (per-part anisotropic trio; the DESPOT mu_along/mu_across naming and the
  anisotropy-frame truth transfer to part level verbatim — a wheel part's
  material frame is its own part-local cross-frame; the frame id pointing
  at the composite body means the body-local frame) + battery/ballast
  parts for honest inertia. Motor drive stays
  `rb_apply_forces_localised` (already COM-correct), forces now applied at
  wheel-part world points.
- [ ] **5.2** Config toggle `mfs robot.composite 0|1`; default 0 for the
  entire WP.
- [ ] **5.3** A/B battery (the acceptance bar): straight, strafe, pivot,
  settle — composite robot vs the child-body robot within the
  documented drivetrain bands. No default flip until every band matches.
- [ ] **5.4** Odometry: θ stays ∫ω_y·dt (frame-invariant); document the
  origin-vs-COM sweep artifact once in `v16R1/FTC.md` as the real-team
  artifact (tracking wheels orbit the COM; so does the sim's origin now).
- [ ] **5.5** Contract hygiene: the MFS suite keeps its current num-
  bers (31 gated, `aim_solve` open red stays "reported, not touched" —
  a landing that incidentally fixed or broke it would be an untraceable
  change); suite gate additions for composite robots are SMOKES only
  until 5.3 green, real gates after.

## §8 WP6 — docs, policy amendment, release cadence (the v16S cut)

- [ ] **6.1** Single-commit doc sweep: `docs/PHYSICS.md` §9-composition
  (the equation block from `mig_mass.c`, the (A)-decision, part-scoped
  keys, ghost/vector-free view, kernel payload row);
  ARCHITECTURE kernel section (parts pool + part-pair iteration);
  GLOSSARY (composite, part, rebase, totals, ghost view);
  docs/INDEX rows; BUILD (the WP0 makefile line); TESTING command paths;
  REAMINING_WORK slot move; verison-every touched doc. — link check 0
  broken per house rule.
- [ ] **6.2** RELEASE_POLICY amendment (charter §12): the freeze-line
  note gains the v16S charter entry (same move as v15S→v16R1); version
  macros stay `v16R1-dev` until the cut; `mpe_engine.h` duplicate moved at
  the same commit (the :105/:108 pair the v16R1 cutover documented).
- [ ] **6.3** Release notes skeleton at the cutover commit (sections:
  twin-train continuation, the mass contract, composites + editor, robotics
  A/B numbers, open items incl. hull timing + rotation-CCD frontier).
- [ ] **6.4** Suite floor lands at its final v16S value in one dated
  entry; quick profile re-baseline with the new count on BOTH trees.
- [ ] **6.5** The 461 mirror: contract before any engine cut —
  `tools/sync_mfs_check.sh` green, per the twin discipline (engine files
  ride with the sync script's existing rules; the MFS deltas of WP5 mirror
  twin→both).

## §8.5 WP2.5 — convex hulls (P3 ruling: in scope, incremental)

Appended by the 2026-10-30 P3 ruling (option 2, *slowly and
incrementally*). NOT a single SAT-charged block: every sub-step below is
its own commit, green-or-rollback, and the v16S cut checklist (§10)
counts only the green increments — the rest stay config-flag OFF with a
ledger line saying so. Work files live in `core/` beside the MIG mass
module; nothing here duplicates an existing primitive path until the
reference gates force it.

- [ ] **2.5.0 Flag.** `composite.hull 0|1` default 0, config-schema key;
  NOT the load path — every hull increment ships behind this flag.
- [ ] **2.5.1 Point collection (no physics).** Import or build hull
  point sets as data; unit: exact tetra/box point sets, no solver
  involvement, suite-gated garbage rejection (non-finite points).
- [ ] **2.5.2 The 2D proof.** Divergence-theorem area/moment on polygon
  cross-sections vs closed forms (box slice exact) — the mathematical
  foundation of 2.5.3, proven on the lowest dimension first.
- [ ] **2.5.3 Hull inertia, standalone.** Divergence-theorem mass
  properties for closed triangle meshes as a pure math module (mig_mass
  discipline: double accumulation, write-on-success, named errors);
  gates: box hull reproduces the box closed form, tetra exact.
- [ ] **2.5.4 SAT reference pairs.** hull-vs-primitive narrowphase
  behind `composite.hull 1`, reference-pair gates only (box-hull vs box,
  sphere-hull vs sphere); no scene-format default flips.
- [ ] **2.5.5 Cut note.** Whichever of 2.5.1–2.5.4 are green at the v16S
  cut ride to release; a ledger line names the shipped increment set.

## §9 Risk register + the standing ledger

- **Cross-part warm-start aliasing** — highest. Mitigations: part-scoped
  keys in the SAME commit as the first manifold map (2.4+2.5); the
  TASK_05 probe pattern as `mig` cache gate; no part-index reuse ever.
- **Floor churn (F10-class) through composites.** Mitigation: the
  `mig_composite_stack` sleep gate is named BEFORE 2.6 lands; pair-
  novelty wake should cover by construction; if it does not, the debugging
  path is the F10 root-cause record and the solver islands' sleep gates,
  documented in that order.
- **Ghost-view determinism** — memcpy order fixed = parent index order;
  ghost refresh per pair is deterministic; TSan + 1/4/8 twins are the
  always-runs, not the once-runs.
- **MFS regression onto the 31/32** — toggle stays OFF through the WP;
  A/B gates decide the flip; bundle numbers reported, not promised.
- **Editor/physics ownership drift** — spec text is the source of truth;
  the editor parses the same grammar the suite uses; no hand-poked state
  around staged swaps.
- **ABI bump (2) breaking old `.so`** — append-only small-struct read
  keeps ABI-1 plugins loading; the loader test `module` +
  `loader_lifecycle` must stay green at every WP2 commit.
- **Hull scope creep** — P3 ruled hulls in scope as INCREMENTAL WP2.5
  (§8.5): every sub-step is green-or-rollback, the cut ships only green
  increments behind `composite.hull 0` default, and anything charging
  ahead of the staged list requires an explicit amendment here, not
  momentum.

Ledger (new entries get a line, never an edit of a past line):

- 2026-10-30 — WP0 landed. One incident: gate author's triple-Ixx oracle
  missed the cylinder dz-Steiner term (3·cz²); implementation was correct;
  the gate fixed the mathematician. ASan clean. Nothing pre-existing
  surfaced.
- 2026-10-30 — WP1 step 1.1 landed (`b0e1cbb`): first-run green, zero
  incidents — measured 1.8188 s vs the corrected closed form 1.8184 s
  (0.02%), max_drift 0.0003 m, ASan clean. Verdict recorded: the engine's
  revolute + hand-authored tensor satisfies the physical-pendulum closed
  form with the Steiner term at 74% of I_pivot, so WP2's composite
  inherits a certified solver on that path (narrowphase claims nothing
  from this landing — only the tensor path is pinned).
- 2026-10-31 — WP1 steps 1.2–1.4 landed (`df383f0`), the FIRST
  engine-touching MIG commit. Scope: bridge struct + counter + the
  add_composite stub; grows-once body fields; gate 49th. Incidents: two,
  both gate-oracle side and fixed gate-side (bridge-Ixx 0.2→0.1
  self-term mistake, machine corrected the mathematician again; free-fall
  spawn at y=0 hit the default floor — 0.75-bound body depenetrated up,
  fixture moved to y=30). Engine truth: binary changes bits from this
  landing (struct +8 bytes, one world entry point, physics_world.c →
  mig_compose); plain-scene numeric identity re-certified by the
  headless byte-probe A/B (byte-identical) + determinism twin, and the
  partial-truth note carried forward: WP1's stub does NOT yet extend the
  explicit sanitize guard (a future rigidbody_set_static(rb, false) on a
  composed body WILL regenerate the tensor from the sphere formula — the
  full guard is WP2 2.2's first move; the WP1 gate does not exercise
  that path and the stub is documented intermediate behavior). The
  counter discipline held: mpe_clamp_composite_events stayed 0 across
  construction, 120 ticks, and every rejection — substitution-counting
  starts with WP2's cap exactly as P2 planned.
- 2026-10-31 — WP1 1.6 landed: PHYSICS.md's top blockquote gains the
  one-line MIG pointer citing the charter
  (`docs/NON_UNIFORM_BODY.md`), this playbook, and the honest WP1
  intermediate behavior (composed mass/tensor on a sphere-backed body,
  parts inert until WP2), with the full composite equations deferred to
  WP6 as the charter's §9 always planned; zero incidents, zero numbers
  moved.
- 2026-10-31 — `fix:` follow-up to `df383f0` — the first engine-touching
  landing missed its DOWNSTREAM consumers: `ecosystem/mgb/makefile`
  (`CORE` :18, linked by both `mgb_mpe_test` and `mgb_probe`) and
  `ecosystem/mfs/build_tests.sh` (`CORE` :95) compile the engine core
  DIRECTLY instead of through MPE_TEST_CORE, so physics_world.c's new
  `mig_compose` reference found no definition when the author ran
  `make everything` (the main makefile's `mgb_bridge` target recurses
  into the mgb sub-tree's default `all`). All three lists gained
  `core/mig_mass.c` right after `core/mpe_primary.c` — the main
  makefile's MPE_TEST_CORE order — with the third being the canonical
  `ecosystem/mfs/mfs_sources.mk` (its own header says "update both files
  together"). Battery-gap honesty: build_suite + the quick profile never
  touch the ecosystem targets, so the landing's verification had not run
  `make everything`; the author's own build found the hole. Post-fix
  evidence: default `make` green, `make mgb_bridge` builds the full mgb
  `all` (mgb_mpe_test 1.9 MB, mgb_probe 1.9 MB, mgb_bridge.so loads and
  exports mpe_module_desc), `make test` tier A = PASS 44 FAIL 0
  (tier B skips by its own s2tui contract), `make everything` scanned
  with zero error lines, and the engine battery re-run green (suite 49/
  47+2, quick 63/63, docs-consistency OK).
- 2026-10-30 — P1 rename landed (standalone, before 1.2+ per the
  author's directive: massive changes later): every landed `nuf_*` name
  on the chart is now spelled `mig_*` — `core/nu_mass.{h,c}` →
  `core/mig_mass.{h,c}` (later re-listed in §1 annotations as was/now),
  gates `nuf_mass_composite`/`nuf_pendulum_physical` (renamed at
  landing) → `mig_mass_composite`/`mig_pendulum_physical`, playbook file
  renamed `MIG_BUILD_PLAN.md`; the mapping bullets above date the
  historical spellings. The two 2026-10-30 lines above had their
  spellings updated by that rename — their content, verdicts, and hashes
  are untouched and their gate/filenames are in the mapping bullets.
- 2026-11-01 — WP2 step 2.1 landed (`98debd9`), the second
  engine-touching MIG commit and the first WP2 one. Scope: the fifth
  object type + alias, the per-world contiguous part pool with
  write-on-success append and per-body monotonic indices, inactive slots
  never reused, the P2 cap as `composite.max_parts` default 16 with a
  counted clamp, clear as the sole compaction point, cleanup free. One
  deliberate deviation from the "grows once" wish, recorded rather than
  hidden: `part_base` is one additional append after the WP1 pair
  because the WP1 bridge carried no pool base; `rigidbody` is
  engine-internal (not `mpe_module.h`), so no plugin ABI moved and 2.4
  still owns that bump. Incidents: one, gate-side and engine-untouched —
  `-Werror=enum-compare` rejected the gate's alias equality line (two
  anonymous enums), fixed with an int cast. Engine truth: the binary
  changes bits again (internal struct +4 bytes, new config category/key,
  new pool entry point) while the zero-composite numeric path is
  certified unchanged twice on this tree — the headless byte-probe
  against `4ffb4c1` is byte-identical (ticks=3600 bodies=2 invalid=0
  fallback_pow=0 fallback_trig=0 result=PASS) and the in-suite
  `determinism` twin stays 600 ticks bitwise identical. Battery: suite
  50 total, 48/48 blocking + 2 diag; quick 64/64; docs consistency 7/7;
  `make everything` exit 0 (log /tmp/opencode/make_everything_21.log);
  MFS sync guard in sync. ASan/UBSan: the new gate and the other four
  MIG/determinism gates are clean with no sanitizer report, but the asan
  target as written fails `loader_lifecycle` and `ftc_ecosystem` with
  undefined symbol `g_cfg` because it omits the `-rdynamic` the normal
  build passes — pre-existing target gap, unchanged by this commit and
  outside 2.1, now on record so a later green-ASan claim must either fix
  the target or name those two gates. No pre-existing engine defect
  surfaced; no numbers moved outside the new gate's own counter.
- 2026-11-01 — WP2 step 2.2 landed (`364428d`), the third
  engine-touching MIG commit and the second WP2 one. Scope: the world API
  completion — `physics_world_add_composite` now joins an assembly into
  ONE `object_composite` body at the COM, validating the whole caller
  list (including parts beyond the P2 cap, so a bad discarded tail refuses
  everything), composing mass/inertia over the stored slice only (20
  one-kilogram parts become a 16 kg body with bound 0.425, never 20),
  re-anchoring parts into the pool through `mig_make_bridge`/
  `mig_rebase_parts` in definition order, storing effective per-part
  kilograms via the new `mig_part_mass`, copying per-part materials from
  each part's own primitive defaults, counting exactly one
  `mpe_clamp_composite_events` substitute only after a successful append,
  and sharing the frozen cap helper between add and append. The explicit
  guard: `rigidbody_restore_composed_inertia` rebuilds both inverses from
  the authored tensor, sanitize accepts `object_composite`, keeps its
  authored radius, and restores instead of substituting a type formula at
  the recalc site, `rigidbody_wake` and `rigidbody_set_static(false)` use
  the same restore, and the three `rigidbody_update_inertia_*` functions
  early-return for composed bodies. Incidents: two, both gate-side and
  engine-untouched — the drafted multi-condition checks passed a label as
  a third argument to the two-argument `MPE_CHECK` macro (compile
  failure, labels removed), and the first run caught the gate's own
  part-order oracle assuming the -0.5 part was stored first when
  definition order stores +0.5 first (engine right, oracle fixed). Engine
  truth: no struct or config change (the internal `rigidbody` stays 396
  bytes, no key moves), the binary differs from 2.1 only in the new
  composite-gated code, and binary identity is not claimed; the
  plain-scene numeric path is certified by the headless byte-probe
  against `13ccf9c`, byte-identical (ticks=3600 bodies=2 invalid=0
  fallback_pow=0 fallback_trig=0 result=PASS), with the in-suite
  `determinism` twin bitwise identical. Battery: suite 51 total, 49/49
  blocking + 2 diag; quick 65/65; docs consistency 7/7; `make everything`
  exit 0 (log /tmp/opencode/make_everything_22.log); MFS sync guard in
  sync. ASan/UBSan: `mig_composite_world` and the other five
  MIG/determinism gates are clean with no sanitizer report, while the
  asan target as written still fails only `loader_lifecycle` and
  `ftc_ecosystem` on undefined symbol `g_cfg` from its missing
  `-rdynamic` (47/49, pre-existing, unchanged, outside 2.2). No
  pre-existing engine defect surfaced; no numbers moved outside the new
  gate's own assertions.

## §10 Definition of done for v16S (the cut checklist)

All gates green at the final floor; policy amendment committed; release
notes skeleton committed at the cutover; POSIX+Windows build wrappers
(`run_all.sh`/`run_all.bat`) verified; 461 mirror synced green; the one
IRreversible-contract item (MPE_MODULE_ABI→2) documented in
`core/mpe_module.h` with the smaller-read rule; KNOWN_LIMITATIONS gains
the composite rows (part cap, rotation-CCD frontier unchanged, principal-
axis readout TODO); VERSION_HISTORY row written from the release-notes
numbers, not before.
