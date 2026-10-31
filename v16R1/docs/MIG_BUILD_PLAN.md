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
> RENAME LANDED standalone 2026-10-30 (this commit) per the author's
> rename-first directive: files/identifiers/gates are mig_* and this
> playbook is MIG_BUILD_PLAN.md; the engine touched NOTHING —
> `mig_mass.c` remains outside the engine link list, engine binary
> bit-unchanged since v15S+MFS-v10S. NEXT: the massive changes WP1
> 1.2–1.4 (bridge struct, clamp counter, add_composite stub) in their
> own green landing, engine still untouched until then; WP2–6 not
> started.**

---

## §0 Resume protocol (read this first, always)

If a session (or its tool budget) died mid-stream, re-establish ground
truth in this order and do not write a new line of code until green:

```bash
cd 475-MPE/v16R1/src
make build_suite            # zero complaints expected (-Werror is default)
./test_mpe_suite --all      # expect: Total: 48 | Blocking failures: 0
cd ../..
python3 tools/test_runner.py --profile quick   # expect Checks: 62 | passed: 62
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

## §1 Baseline ledger (as of 2026-10-30 — update at every landing)

Landed and pushed 2026-10-30 (WP0 block: commits `181c2c0` charter,
`0882935` code, `636b10e` playbook; WP1 step 1.1 followed in `b0e1cbb`):

| Artifact | Nature |
|---|---|
| `v16R1/docs/NON_UNIFORM_BODY.md` | design charter (frozen decisions) |
| `v16R1/src/core/mig_mass.h` | composition contract + all type/error enums |
| `v16R1/src/core/mig_mass.c` | exact composition: mass/COM/inertia, double accumulation, definition-order deterministic |
| `v16R1/src/tests/mpe_suite_mig.c` | gates `mig_mass_composite` (7 probe families, oracle independence) + `mig_pendulum_physical` (WP1 1.1 probe, hand-written oracle) |
| `v16R1/src/tests/mpe_suite_main.c` | externs + registry entries (48th run blocking) |
| `v16R1/src/makefile` | `MPE_TEST_CORE += core/mig_mass.c`, `MPE_SUITE_SRCS += tests/mpe_suite_mig.c`, help-note renumber |
| `tools/test_runner.py` | floor 46 → 47 → 48 with dated history comments |

Names in this table are the PRESENT (post-P1-rename) spellings; at their
landing commits the files were `nu_mass.{h,c}` and the gates were
`nuf_*` — see §2's mapping and the rename line in §9. The rename landing
also renamed this playbook file itself (`NUF_BUILD_PLAN.md` →
`MIG_BUILD_PLAN.md`).

Measured baseline (numbers a resume session must reproduce):

- Suite: 48 registered, **46/46 blocking green**, 2 diag-informational
  (WP0 landing was 47/45; step 1.1 added the 48th blocking gate).
- Runner quick: **62/62** (WP0 landing was 61/61).
- ASan/UBSan suite build: green, both `mig_*` gates clean.
- Engine binary: unchanged bits (`mig_mass.c` outside the engine link
  list; step 1.1 touched no engine file).

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
- [ ] **1.2 Bridge struct** in `core/mig_mass.h`: the handoff contract WP2
  will consume — `mig_compose` totals + re-anchored part list, marked
  "caller owns; world copies in". No engine include added; keep the
  dependency-clean rule.
- [ ] **1.3 Clamp counter:** `mpe_clamp_composite_events` (+ accessor) in
  `core/rigidbody.h` beside the existing block, atomic-per the
  DESPOT-2026-10-12 M4 note (this counter is shared with WP2's cap and
  WP3's load-sum gate).
- [ ] **1.4 Application point:** a NEW world entry point
  `physics_world_add_composite` (this is the WP1 stub that WP2 completes:
  in WP1 it creates a sphere-backed body with the composed
  mass/inertia/local tensor and records parts as DEAD WEIGHT to be
  attached in WP2 — the stage-once pattern the loader already uses; the
  documented intermediate behavior is "composed mass, sphere bounds"). If
  the stub shape feels wrong on implementation day, the alternative is
  WP1 = 1.1–1.3 only and the entry point moves wholly into WP2 — decide
  AT that step, record the choice here, and keep the commit boundary
  green either way.
- [ ] **1.5 Verify:** suite `--all` green 45/45+1(1.1) blocking; quick
  profile (floor moves 47→48 in the same commit as 1.1); ASan gate rerun;
  engine bitwise contract: a plain scene with zero composite bodies must
  be bit-identical — run the existing `determinism` twin unchanged.
- [ ] **1.6 Docs:** PHYSICS.md gains a one-line pointer to the charter
  (full §9 phase at WP6); ledger notes.

## §4 WP2 — composite type, parts pool, narrowphase part dispatch (the heavy one)

**Scope:** `object_composite` becomes a real first-class type: parts as a
per-world pool, ghost-view narrowphase, part-scoped cache identity,
broadphase/floor/CCD hooks, editor-invisible (UI is WP4). Estimated
1800–2400 LOC. Every step ends green or does not commit.

- [ ] **2.1 Type + pool.** Append `object_composite` to the `object_type`
  enum + `OBJECT_COMPOSITE` alias (`core/rigidbody.h`); new per-world
  composite part pool in `core/physics_world.h/.c` (slice
  `part_base/part_count` on the body; monotonic part indices; inactive
  flag; cap + counted clamp from P2). Body append-only fields land in the
  SAME commit as the WP1 bridge so the struct grows once, abi-document
  style — rigidbody is engine-internal and NOT the external plugin ABI
  (`mpe_module.h` is; that bump belongs to 2.4).
- [ ] **2.2 World API.** `physics_world_add_composite` completes the WP1
  stub: `mig_compose` bridge (validate every part, whole-assembly
  invalidation semantics preserved — the composed body refuses nothing a
  caller can count), `mig_rebase_parts` about the COM into the pool slice,
  body mass/inverse/authored `inertia_tensor_local` written, sanitize
  guard extended so a composed tensor is never regenerated from type
  defaults (mirror of the existing custom-type guard),
  `rigidbody_update_axes`-equivalent bounds.
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

## §10 Definition of done for v16S (the cut checklist)

All gates green at the final floor; policy amendment committed; release
notes skeleton committed at the cutover; POSIX+Windows build wrappers
(`run_all.sh`/`run_all.bat`) verified; 461 mirror synced green; the one
IRreversible-contract item (MPE_MODULE_ABI→2) documented in
`core/mpe_module.h` with the smaller-read rule; KNOWN_LIMITATIONS gains
the composite rows (part cap, rotation-CCD frontier unchanged, principal-
axis readout TODO); VERSION_HISTORY row written from the release-notes
numbers, not before.
