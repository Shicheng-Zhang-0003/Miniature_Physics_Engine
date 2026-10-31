# MIG — Non-Uniform Body Framework (design record, v16R1 → v16S)

> Design record for the non-uniform object physics surface: mass composition
> (off-center COM, composed inertia), welded composite bodies (multi-part
> geometry on one rigid frame), and the native CAD object editor that creates
> them. Target train: **v16S**. MFS (FTC robotics) is the first consumer;
> MPE ships the framework by default.
>
> Status: DESIGN RECORD, nothing implemented. Commit as the train-opening
> document on the v16R1 head, in the same move the v15S → v16R1 cutover
> opened shape work. The RELEASE_POLICY v15R3 feature-freeze line is
> superseded by this charter (see §11).

---

## 1. Ground truth today (what cannot be expressed)

The body contract (`core/rigidbody.h`) currently admits four types —
sphere, cube, cylinder, custom — with three physics-hard walls:

- **COM == position, always.** No center-of-mass offset exists as a field.
  Every lever arm in the solver is `r = contact − position`; free flight,
  CCD lowest-point, `rb_apply_forces_localised`, and every diagnostic
  assume `position` is the COM. A mass distribution that is not symmetric
  about the frame origin cannot be modeled. There is no field to even lie
  about it with.
- **Inertia comes from three hardcoded formulas** (sphere ⅖mr², box
  m/12(h²+d²), cylinder ½mr² + m/12(3r²+L²)). A user tensor cannot be
  supplied; a tensor that is physically correct for an off-center or
  composed object cannot exist.
- **Geometry is one primitive per body.** `object_custom` is the foreign
  land: sphere-backed (`physics_world_add_custom` initializes a sphere and
  stamps the type), broadphase sees only `radius`, scene v200 has no
  plugin blob (a saved custom body reloads as a capsule placeholder,
  `scene_load.c` "custom body: no plugin blob in format"). It is a
  narrowphase probe, not a shape system.

What is actually good enough to build on (and this design leans on it):

- `inverse_inertia_system` is recomputed generically as `R·I⁻¹·Rᵀ` from
  `inverse_inertia_tensor_local` — arbitrary symmetric tensors ride this
  path today. Nothing in the integrator or solver is shape-formula-bound.
- `rb_apply_forces_localised` computes torque about `position` — which is
  exactly COM-relative torque, valid for any composed object as-is.
- The anisotropic friction trio is an existing append-only precedent with
  an explicit bitwise-neutrality contract ("default false => …
  bit-for-bit unaffected").
- MFS already models robots as child-body assemblies (analytic mecanum:
  6 bodies / 4 joints) and pays the joint-solve slop for it.

---

## 2. The governing decision: `position` stays the COM

Two conventions were considered for non-uniform mass:

- **(A) `position` remains the COM.** Composition, not fields, carries the
  asymmetry: a composite body's parts are stored in COM-relative frames,
  computed once at creation/edit. The physics columns (integrators,
  solver, free flight, CCD quadratic, floor gate) are untouched —
  position was always the COM and still is.
- **(B) `position` becomes the CAD/object origin** with an added
  `com_local` offset, and every consumer derives the COM. Physics-correct
  but invasive: the origin's world position is no longer integrated by
  `x += v·dt` at all — the COM is. Lever arms, support depth,
  CCD, sleep gating, scene semantics, MFS odometry — dozens of call
  sites, every one a bitwise regression risk.

**Decision: (A).** It is the lower-blast choice, it is the append-only
house discipline (default-composition-absent keeps legacy bodies
bit-for-bit identical — the same move as the anisotropy fields), and it
matches real CAD→physics pipelines (URDF/Gazebo compute link COM and
re-express geometry about it at construction). The cost is an editor
convention: when an asymmetric part is added, the editor re-baselines
part coordinates so the *material* stays put while the physics origin
re-anchors to the new COM. That re-baseline is editor bookkeeping, not
physics.

Consequence: **there is no `com_local` field on `rigidbody`.** Off-COM
mass on a single primitive shape is expressed as a one-part composite.
The composite is the concept, not an optional offset bolted to old
shapes.

---

## 3. The mass contract (Axis 1)

### 3.1 New core unit: `core/mig_mass.c`

Ownership split away from `rigidbody.c` so the hardcoded primitive
formulas stay the primitives' truth and composition gets its own audited
surface:

- **Input contract:** per part — type (sphere/cube/cylinder), local pose
  (position + unit quaternion, engine convention `(w,x,y,z)`), dimensions,
  and a mass source. Per-part mass source is one of: `mass M` or
  `density ρ` (mutually exclusive per part). Exact composite volumes:
  sphere (4/3)πr³, cube 8·h_x·h_y·h_z, cylinder πr²·(2·half_length),
  axle X per the house cylinder convention.
- **Output contract:** total mass M, COM (in the assembly frame supplied
  by the caller), and inertia tensor I_com composed by exact per-type
  formulas + Steiner (parallel-axis) translation, accumulated in
  **double**, iterated in **part definition order** (fixed order is part
  of the determinism contract — no reordering, no per-call parallelism
  inside the composition).
- **Sanity gates, counted per the DESPOT discipline:** total mass into
  the existing mass clamp band; composed matrix diagonalized where
  supplied as off-diagonal (not in v16S — parts compose diagonal tensors
  only, structure below); smallest principal inertia floored at a
  percentage of the largest (a zero-thickness flange must not produce a
  singular axis — clamp event counted through `mpe_note_clamp`-class
  counters, new `mpe_clamp_composite_events`).
- **Sanitize guard:** `rigidbody_sanitize` must never regenerate a
  composite body's tensor from its type defaults (the same class of
  guard that already protects `object_custom` foreign types). Composite
  tensors are authoritative-only: written at composition, re-written
  only on part edit, re-expressed world-side by the existing generic
  `R·I⁻¹·Rᵀ` machinery — nothing new in the hot path.

### 3.2 Primitive-path truth stays truth

`rigidbody_update_inertia_{sphere,cube,cylinder}` are unchanged and keep
the "verified, do not fix" comment in `rigidbody.h`. Composition math
derives from the same formulas and must **duplicate, not call, the
primitive functions if the call order would change their semantics**
(cf. the audit rule: shared-stage behavior differences must be named).
In practice composition calls the same per-part exact formulas in
`mig_mass.c` and the `rigidbody.h` comment block gets a pointer to this
doc.

---

## 4. The composite payload (Axis 2)

### 4.1 Type and layout

- New `object_composite` appended to `object_type` (fifth value;
  `OBJECT_COMPOSITE` alias). `object_custom` stays what it is: the
  foreign/plugin lane. Composite is core-owned, its parts are core
  primitives (sphere, cube, cylinder in v16S).
- Per-world `composite_part` pool (contiguous, pool discipline as
  everywhere else in the kernel): `(type, local_pos, local_quat, dims,
  material fields, mass fields, part_index)`. Body→parts via
  `part_base, part_count` slice on the body. Cap `composite.max_parts`
  (default 16) — a substitute above the cap is COUNTED, bounded stderr,
  per house clamp observability.
- **Part indices are per-body monotonic and never reused:** deleting a
  part deactivates it (systems skip inactive parts); new parts take the
  next index; the pool only compacts on `physics_world_clear`. Rationale:
  contact-cache keys embed part indices (§4.3); reusing them across
  edits would alias warm-start impulses across unrelated parts — the
  exact aliasing class the engine already fixed once at body granularity
  (`MPE_TASK_05_CACHE_MATCH_BEGIN` both-side material-point matching;
  the "two B bodies sharing one A" repair).
- **Edit timing truth:** part edits (mass, pose, dims, material) or
  their deletion take effect at the next step's broadphase rebuild.
  Mid-tick edits queue. An edit bumps the body's revision and the
  body-property stamp so warm-start caches and inertia caches treat the
  body as a new physical object (existing `a3_task05_body_property_stamp`
  contract, extended).
- **Sleep:** a composite is one body — one wake, one island. A part edit
  wakes the body. This is the payoff that child-body robots cannot
  match.
- Composite is strictly rigid, NOT a chain. Articulated mechanisms stay
  on joints (two bodies + revolute), per house truth. No nested
  sub-physics inside a body. (Named non-goal, §8.)
- `no_collide`, static, kinematic, sleep properties are body-level —
  exactly as for every other type.

### 4.2 Mass accounting contract on the body

At composition the body receives: `mass`, `inverse_mass`,
`inertia_tensor_local` and inverse, all generic fields — written directly
(no per-type init call), then `rigidbody_sanitize` runs under the
composite guard, then `rigidbody_update_axes`-equivalent bounds. The
body's `broadphase_bounding_radius` becomes the max part bounding
radius + COM-offset magnitude (exact: max over parts of
`|com_part_offset_world| + part_radius`), conservative and cheap.

### 4.3 Narrowphase: part-pair iteration

`mpe_shape_dispatch` (core/physics_world.c) gains a composite branch:
for (part_a in A) × (part_b in B), AABB-in-AABB prefilter in the parent
pair frame, then reuses the EXISTING builtin primitive pair functions
against part-translated poses. Contact emission collects across all
part pairs into the manifold list.

- **Contact identity gains the part indices, unconditionally and on day
  one:** `contact_point_data` and `cached_contact` gain
  `uint8_t/part_a, part_b` (append-only struct extension + MPE_MODULE_ABI
  bump to 2; loader reads ABI-1 .so structs as smaller, fields default 0
  — the documented append-only pattern already used for the stage-state
  destructor). Cache keys become (id_a, part_a, id_b, part_b). Without
  part-scoped keys, a composite resting on two parts would share warm
  start memory across parts — the one bug this feature must never ship
  with.
- **Friction material at contact time:** the contact resolves materials
  from the *part* touching (per-part µs/µk/restitution, per-part
  anisotropic trio). The anisotropy frame contract (`friction_anisotropy_frame`,
  the DESPOT mu_along/mu_across naming) transfers to part level with the
  same directional truth; a wheel part's material frame is its own
  part-local axle cross-frame, and `friction_anisotropy_frame` pointing
  at the composite body means the composite's local frame.
- Combined-µ rule unchanged (`min`), applied per contact.

### 4.4 Floor, CCD, insulation

- **Floor:** per-part floor manifolds under the existing static-plane
  path; support depth derived per part. Gate: composite resting on floor
  settles into sleep within the documented slop band, no churn — this is
  a suite gate (§7), because the F10 runaway's root cause was
  never-sleeping resting contacts and composites multiply their surface.
- **CCD:** unchanged algorithms. Floor quadratic uses the lowest part
  support point (K-part scan, exact per house parts types). Volume-style
  sweeps continue on the bounding sphere. Rotation-only tunneling stays
  unsupported (§8), explicitly including composites: substep stays the
  user answer.
- **Islands/sleep/warm-start caches:** body-level, no structural change
  beyond the part-scoped keys.

### 4.5 Determinism contract for composites

- Part composition order = definition order; sums in double.
- Part iteration order in narrowphase = part index order per body,
  then the pair's (A,B) role order — no parallel writes to shared
  contact slots (the current pair-local narrowphase discipline extends
  to part-local slots, written disjointly then merged after the barrier
  in pair order, exactly as the 2026-10-20 audit fixed for pair-local
  collision).
- Composite bodies enter the existing determinism twin battery
  (§7): 600-tick bitwise identity at MPE_THREADS 1/4/8.

---

## 5. Scene persistence: v200 extension

Append-only, in the spirit of the loader's kindness rules:

- Per composite body, a `MIG1` extension record after the body row:
  `part_count`, then per part: `(part_index u16, type u8, pose
  quad+f32×3, dims f32×3, µs, µk, restitution f32, mass_mode u8, mass
  f32, colour f32×3)`. CRC already covers the stream.
- Loader rules:
  - No blob (legacy file): composite-degraded — the body loads as its
    bounding-cube placeholder with a counted diag, scene is NOT vetoed
    (the existing custom-body kindness precedent).
  - Unknown part type inside a blob: that part degrades to a
    bounding-cube placeholder at its bounding volume; diag counted;
    scene continues.
  - Sum consistency gate on load: mass/>COM recomputed and compared to
    the stored body mass; mismatch above a documented tolerance is a
    diag + clamp (never a scene veto).
- Save always writes the full blob; round-trip identity (bitwise) is a
  suite gate (§7).

---

## 6. The CAD editor (tiny by explicit scope)

**Ordering rule: the headless surface lands BEFORE the GTK surface.**
The framework must be create-able, step-able, save-able, and
gate-locked with no UI running, then the editor becomes a window onto
it. This is how the rest of the project already rehearses
(`mpe-tui` verbs, per scene; F5–F11 validation scenes).

### 6.1 Language surface (also the file format for sharing objects)

A declarative object spec (text), the single source of truth the editor
mutates and scenes serialize to — sketched (exact grammar frozen at
implementation):

```
mig battery_box
  part 0 type cylinder            # wheels ride per-part later; fields below
    dims r 0.05 half 0.02 pose (0.0, -0.02, 0.03) qw..qx..qy..qz 1 0 0 0
    material mu_s 0.9 mu_k 0.7 rest 0.1
    density 2200
    colour (0.15, 0.15, 0.18)
  part 1 type cube ...
```

Terminal verbs in `term_obj.c` (new `mig` family under the house verb
taxonomy): `mig load <file>`, `mig spawn <name>`, `mig show <name>`,
`mig edit` surface via the same spec. These make the object format
read-them-once-and-be-done forever, and give the suite a full
creation→step→save→load→step gate with zero GUI.

### 6.2 GTK surface (`ui_input/mig_editor.c`, new)

Built out of existing in-tree muscle (config_menu + editor_dialog +
object_spawner patterns, GTK4 controllers, `mouse_lock` gestures):

- Spawn target "MIG object"; part list panel (add per part: sphere /
  cube / cylinder; fields: dims, pose, material, mass-or-density,
  colour).
- Two-axis translate gizmo + rotate fields; grid snap toggle (config
  key). Numeric pose fields first — gizmo polishing is post-v16S.
- Live readouts: total mass, computed **COM marker rendered in the 3D
  view** (the thing this feature is about, impossible to tune blind),
  inertia printed numerically. Principal-axis display (Jacobi,
  fixed-sweep, deterministic) is a documented TODO, not v16S scope.
- The COM re-anchor: when the part list changes, the editor re-expresses
  all parts about the new COM and moves `position` to keep the world
  placement of material unchanged (the §2 decision, made concrete).
- ECS-free, pool-backed: the editor edits the spec, regenerates the
  body atomically (staged body build then swap), never hand-pokes
  mid-step state.
- Scene roundtrip and spec recomputation are the same code path — no
  editor-only truth.

### 6.3 Explicitly tiny (out of editor scope, documented)

Boolean/dissolve ops, mesh vertex editing, sketch-and-extrude, mirrored
assemblies. The editor builds bodies from primitive parts with real
physics; hull-from-mesh is the NEXT train's geometry feature (§8).

---

## 7. Gates (names and pass conditions, written before code)

Per house audit style — the closed-form probes exist before the
features they certify:

- `mig_mass_composite` — exact composed mass/COM/I against
  hand-closed-form parts (two-sphere dumbbell, cube+cube L, mixed
  triple). Gates the composition math in isolation.
- `mig_pendulum_com_offset` — revolute pendulum whose bob's COM is
  offset from the anchor: measured period vs the parallel-axis shifted
  closed form within the pendulum ≤1.5% house band. **This one gate
  proves the mass contract end-to-end through the solver**, because the
  lever arm, revolute constraint, and angular draw all read the COM.
- `mig_composite_stack` — a 10-primitive stack vs one 10-part composite,
  same total mass/materials: both settle asleep, composite stack top-
  block drift within the documented slop band of the primitive stack.
- `mig_floor_ccd_composite` — tumbling L-composite dropped from height:
  floor clamp before tunneling at canonical dt, no landing pump.
- `mig_scene_roundtrip` — composite scene save → load → bitwise pose +
  mass identity; degraded-load diag path green.
- `mig_determinism_twins` — composite scenes at MPE_THREADS 1/4/8
  bitwise identical over 600 ticks.
- Contact-cache part gate (inside the suite's builder): two parts of
  one composite resting — warm-start IDs distinct per part (probe used
  to kill any residual cross-part aliasing, mirroring the TASK_05
  probe discipline).
- Suite floor bump: registered entries floor raised in
  `tools/test_runner.py` in the same commit that registers the gates
  (the runner enforces this rule already).
- Existing gates must not move: quick profile stays green, full stays
  green, ASan/UBSan/TSan stays clean, MFS suite stays at its current
  numbers (31 gated with aim_solve's named open red — landing MIG must
  not "incidently" fix or break it; its state is reported, not touched).

---

## 8. MFS adoption (first consumer; parked on the toggle)

- `robot.c` gains `mfs_robot_build_composite()`: the analytic-mode robot
  becomes one composite — chassis + 4 wheel parts (mecanum, anisotropic
  per part) + battery/mass-ballast parts for honest inertia. This
  collapses 6 bodies / 4 joints → 1 body / 0 joints; the mecanum-model
  reads are all per-part, and motor torque application stays
  `rb_apply_forces_localised` (already COM-correct, §1).
- **Gated default OFF** (`mfs robot.composite 0|1` in the existing robot
  config) with FTC gate re-baseline as the acceptance condition: drive
  A/B (straight, strafe, pivot, settle) must match the child-body
  robot within the documented accuracy of the drivetrain gates before
  the default flips. Physics-first pivot: the composite robot must
  survive the SAME gates, not become a new reference.
- Odometry remains θ = ∫ω_y·dt (frame-invariant). Position trace now
  reads the COM; origin-vs-COM sweep artifact is described once in the
  FTC doc as the known real-team artifact (tracking wheels orbit the
  COM — the sim reproduces what real logs show).
- BioBuzz props stay primitives this train (the hive and pieces are
  pneumatic physics targets, not the motivating asymmetry).

---

## 9. Work packages (in landing order)

| WP | Content | Size estimate | Bitwise contract |
|---|---|---|---|
| WP0 | Gates-first probes (per §7, math-only harness, no contract change) | ~400 LOC tests | none needed |
| WP1 | `core/mig_mass.c` + composition + clamps + WP0 gates green | ~700 LOC + tests | legacy untouched (bitwise) |
| WP2 | `object_composite` + parts pool + narrowphase part dispatch + part-scoped cache keys + broadphase/floor/CCD hooks | ~1800–2400 LOC | default-clean scenes bitwise |
| WP3 | v200 `MIG1` blob + loader kindness + roundtrip gate | ~400 LOC | old scenes load unchanged |
| WP4 | Terminal `mig` verbs + spec format + (then) GTK editor | ~1400 LOC | new surface only |
| WP5 | MFS composite robot behind the toggle + FTC A/B | ~500 LOC | MFS gate numbers reported A/B |
| WP6 | Docs (PHYSICS §9-composition, ARCHITECTURE kernel row, GLOSSARY entries, VERIFICATION commands), RELEASE_POLICY charter amendment, release-notes skeleton | docs | — |

Order notes: WP1 and WP2 are the physics surface; WP4's terminal half
comes before any GTK (headless-first rule); WP5 last so the robotics
stop is gated every time by the already-green engine.

Execution playbook: docs/MIG_BUILD_PLAN.md — per-package step lists,
exact file touch-points, per-step verification commands, pending
rulings (P1–P3) with defaults, the resume protocol for a lost session,
and the baseline ledger. The charter freezes decisions; the plan carries
the how-and-when.

## 10. Risks (named with their mitigations)

- **Cross-part warm-start aliasing** — the single most likely bug. Day-
  one part-scoped keys + the TASK_05-style probe + the part-index
  no-reuse contract. Killed before it can exist, not after.
- **Floor churn (F10-class)** — composites raise floor contact surface.
  The `mig_composite_stack` sleep gate is named before implementation;
  if churn appears, it reproduces a known bug class through a new door,
  and the F10 pair-novelty wake design covers it by construction.
- **Editor/physics ownership drift** — spec files are the single truth;
  the editor is a window. No hand-poking around staged swaps.
- **MFS regression risk to the 31/32 gating** — toggle-default-off, A/B
  gates, the aim_solve red stays reported-not-touched.
- **ABI surface** — contact struct extension follows the documented
  append-only pattern with MPE_MODULE_ABI bump; ABI-1 .so continues
  loading by documented smaller-read semantics.
- **Solver iteration cost on stacked composites** — 10-part bodies in
  stacks solve identically to 10 bodies at the same manifolds count;
  the composite stack gate measures against the primitive stack and
  reports the number rather than promising it.

## 11. Named non-goals for v16S (so scope does not drift)

- Convex hull shape family (SAT-algebra, hull inertia by divergence
  theorem: the NEXT geometry train; the plugin lane meanwhile carries
  any special exact pair).
- Mesh import (OBJ/glTF) in any form.
- Articulated sub-physics inside a body (joints remain the
  connecting-tissue system).
- Boolean/shear editor operations, sketch-and-extrude.
- Per-part velocity fields (composites are one rigid object; that IS
  the feature).
- Rotation-only CCD beyond what v15S shipped (unchanged frontier).

## 12. Release-policy note

The v15R3 freeze line forbids new features on the v16R1 head; v16S is
the next Stable and this design record is its charter. On the commit
that opens WP1, RELEASE_POLICY.md gains the charter entry (the same
move v15S → v16R1 used to reopen the tree), and this doc moves to the
train-record slot. Nothing in §1–§10 is frozen until then.
