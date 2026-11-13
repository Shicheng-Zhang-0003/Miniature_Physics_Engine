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
> 51/49+2, quick 65/65, make everything exit 0. WP2 STEP 2.3 LANDED
> 2026-11-01 (`d74319c`): broadphase part dispatch — the scalar radius
> is the AUTHORED covering sphere (not the sqrt(3) cube inflation), the
> hash AABB is the per-part COM-symmetric hull
> max_i(|R_body d_i| + part extent) through each part's own primitive
> and part-local orientation, and an all-inactive slice falls back to
> the conservative scalar box; the spatial hash, swept insert and cull
> are untouched. Gate `mig_composite_broadphase` is the 52nd registered
> blocking entry, suite 52/50+2, quick 66/66, make everything exit 0.
> WP2 STEPS 2.4+2.5 LANDED TOGETHER 2026-11-01 (`0a03b85`): per-worker
> `_Thread_local` ghost views reuse the primitive pair path against
> part-translated poses, one manifold is emitted per colliding part pair
> with PARENT identities and per-contact part indices, composite pairs
> expand serially after the parallel primitive barrier, and the warm-start
> key becomes (id_a, part_a, id_b, part_b); `MPE_MODULE_ABI` is 2 with
> MIN 1, gate `mig_composite_narrowphase` is the 53rd registered blocking
> entry, suite 53/51+2, quick 67/67, make everything exit 0. LIVE-DRIVE
> INTERRUPTION LANDED 2026-11-03 (`76d6ccd`): the GUI default sandbox
> filler sphere moved from (0,2,0) — where its surface reached the origin
> and enclosed the FTC robot spawned there, squeezing the chassis between
> sphere and floor and shaking every follow view — to (0,2,-6) behind the
> spawn through shared `SCENE_DEFAULT_FILLER_RADIUS` /
> `scene_default_filler_position()`; the real `scene_init_default()` +
> `ftc spawn` repro now settles at rest height with flat roll/pitch, and
> gate `paranoia_default_scene_spawn` (`default_scene_spawn_clearance`)
> fails on the old arrangement and passes on the new one. Suite stays
> 53/51+2, quick 67/67, physics 114/114 (15 paranoia), zero-composite
> byte probe against `8e62e58` byte-identical. WP2 STEP 2.6 LANDED
> 2026-11-08 (`bc395b2`): per-part composite floor manifolds — one
> prepared plane manifold per touching active part with PARENT identity
> and `part_a` stamps, staged before the wake decision so
> `collision_prepare_solver` sees an awake parent — and a floor CCD
> lowest-point scan over the active part slice (sphere bottom, cube
> projected corner, cylinder cap+barrel, horizontal lever arm included),
> so a tumbling L-composite clamps at its true support point instead of
> the old covering-sphere box reading. Volume sweeps stay
> bounding-sphere; rotation-only tunneling remains unsupported exactly
> as in v15S. Gate `mig_floor_ccd_composite` is the 54th registered
> blocking entry, suite 54/52+2, quick 68/68, make everything exit 0.
> WP2 STEP 2.7 LANDED 2026-11-09: composites draw per PART through the
> unchanged sphere/cube/cylinder instanced paths — static helper
> `render_pack_composite_part` (`render/new_render.c`) packs one instance
> per active part with the same T*R*S column composition as the
> primitive path; part world pose = body ∘ part-local (position =
> body_pos + R_body·local_position, orientation = normalised
> R_body ⊗ local_orientation — the exact composition the 2.4 ghost uses,
> so what you see is what collides); scale per part primitive (sphere r,
> cylinder axle-X half-length/r, cube half_extensions); colour = parent
> body's (parts carry no authored colour until WP4). The sphere-backed
> covering shape is NOT drawn — it is a physics envelope, not geometry.
> `no_collide` stays body-level exactly as for primitives (gates
> collision, never visibility). Mesh instance count guard: per-type
> budget stays `mpe_max_bodies` (the VBO size); overflow drops parts
> LOUDLY (per-frame `dropped_instances` stderr warning), never silently —
> finite because parts are bounded by composite.max_parts. The primitive
> draw path is untouched, so zero-composite scenes are unchanged (2.9's
> A2 byte probe statement holds). Renderer is GL-bound and suite-unlinked:
> no new gate; verification = GTK4 build 0 under -Werror, suite 54/52+2,
> quick 68/68. WP2 STEP 2.8 LANDED 2026-11-09 (gates + one real engine
> defect fixed): three new blocking entries — `mig_composite_stack`
> (55th: 10-primitive stack vs one 10-part composite, pitch 1.002,
> settle contract pinned to the CANONICAL config — gravity −9.81, sleep
> ON, cube restitution 0, friction 0.8/0.7, solver_iterations 64; RULING:
> the `light` regime at 8 iterations genuinely slumps a 10-stack, so
> gates pin their own meaning after `mpe_test_begin`; both settle ASLEEP,
> tops agree within 5 cm), `mig_pendulum_com_offset` (56th: 2-sphere
> composite bob d=0.8, hand-closed form I_pivot=3.164 with transport
> terms 96%, θ₀-corrected analytic 2.0060 s, 1.5% band — measured
> 2.0048; the whole mass contract certified end-to-end through the
> solver), `mig_determinism_twins` (57th: dumbbell×L×rain composite
> scene ×2 worlds, 600 ticks bitwise, ambient MPE_THREADS since the
> pool is process-lazy — landing verified at 1/4/8; bodies=5 parts=4
> cache_entries=28). THE GATES CAUGHT A REAL ENGINE DEFECT: the boundary
> safety net (`scene/boundary.c`) read body-level OBB fields, but
> composites are sphere-initialised (half_extensions=(r,r,r)) then
> retyped — min_y = 5.009−5.375 = −0.366 m put the net at COVERING-SPHERE
> rest, so the 10-stack lived in a perpetual clamp/fall cycle and never
> slept (first run: 3 checks failed). FIX: per-part exact support
> `boundary_composite_extrema_along_axis` (world passed down to the part
> pool) + `boundary_apply_box_world_cfg`; legacy signatures delegate
> with world=NULL byte-identical, and the engine call site
> (`physics_world.c:2574`) passes the STEPPED world, not
> `physics_world_get_primary` — twin test worlds stay independent. The
> fix exposed a 2.6 fixture lie: composite floor contacts read PART pool
> materials (body_defaults cube_restitution default 0.5), not the
> body-level restitution the gate pinned — the old boundary clamp had
> been swallowing the bounce. The gate now pins
> `body_defaults.cube_restitution = 0` (and sphere=0 for the dumbbell
> scene); the max_up band recalibrated from fixed 0.5 to self-scaling
> `0.25·v_impact` because the corner impact converts spin to COM rise
> (ωz −29.5 → −2.3 rad/s, vy +0.955) with energy DECREASING — a lawful
> bounce, not a pump — plus a settle check |vy| < 0.25 at tick 30.
> FALSIFIABILITY (mutate → expect failure → restore → rebuild): M1 —
> composites excluded from the sleep-calm condition (`rigidbody.c:1137`)
> → stack gate FAILED 1 check (`first_sleep_b`); restored, green. M2 —
> the composed-tensor overwrite in `physics_world_add_composite` skipped
> (covering-sphere 2/5·mr² survives) → pendulum gate FAILED 1 check
> (period 2.1359 vs 2.0060); restored, green. M3a — floor-batch part
> order reversed (call-counter keyed, periods 2 then 3) was a NULL
> MUTATION: twins stayed bitwise identical — same-body floor-manifold
> order is bitwise-insensitive on this solver path (robustness fact,
> recorded); M3b — the REAL order-matters surface is the part-PAIR path:
> `mpe_composite_iterate` order keyed to the world's runtime address (a
> non-pure order source; twin worlds differ in address) → twins at
> MPE_THREADS=4 FAILED 5 checks (contact-cache footprint + all 4 dynamic
> bodies bitwise-diverged); restored, twins green at 1/4/8.
> test_runner.py floor 54→57 in the same commit. Suite 57/55+2 (Total:
> 57, blocking failures 0), quick 71/71. WP2 STEP 2.9 LANDED 2026-11-09
> — WP2 COMPLETE: `make tsan` green (TSan-built suite `--all` at
> MPE_THREADS=4 under TSAN_OPTIONS=halt_on_error=1: Total 57, 0
> blocking, no reports — the ghost scratch, the `_Thread_local`
> per-worker composite ghost views, is the worker-race surface the
> profile exists for, and it is clean; the paranoia determinism TSan
> probe ran clean ahead of it). The A2 byte-probe statement is
> CERTIFIED EMPIRICALLY, not just documented: a 3-body zero-composite
> headless probe (settling sphere + settling cube + one sphere flung
> sideways at 60 m/s so `boundary_apply_box_world_cfg` clamps it at
> x = 250 − r for the majority of the run — the delegation path the 2.8
> boundary fix claims byte-identical), 3600 ticks, prints every body's
> full pose/velocities/sleep state as hex-float — BYTE-IDENTICAL
> against a `git worktree` build of the pre-WP2 engine `8e62e58` (the
> same base the 2.3-era probe used); invalid=0 fallback_pow=0
> fallback_trig=0 result=PASS on both sides. Probe source and both
> captures kept under `temp/qa_runs/A2_2p9_byteprobe/`. Plain headless
> smoke unchanged (ticks=3600 bodies=2 … PASS). Suite `--all` green at
> floor 57.
> NEXT: WP3 — scene persistence (v200 `MIG1` record): 3.1 writer/reader
> two-pass with the id-remap pass, 3.2 degrade-not-veto loader.**
> WP3 STEPS 3.1+3.2+3.3 ALL LANDED 2026-11-10 (`5a3dd4d` code + gate,
> `5938c23` plans): composite scene persistence at format v201
> (`mpe_version` 200→201, dedicated `composite_count` section after
> revolutes — the WP3 shape deviation, deemed cleaner than in-band `MIG1`
> row extensions): per composite the ACTIVE part slice expanded to the
> WORLD frame, the exact representation `physics_world_add_composite`
> consumes; ids preserved so springs rewire unchanged; envelope dynamic
> state (velocity, colour, materials, sleep, generation, saved id)
> re-applies to the rebuilt body; loader NEVER trusts the file for the
> body — mass/COM/inertia recomputed through the compose path at commit,
> unusable composites DROPPED LOUDLY (stricter than the plan's
> placeholder-a-cube), structurally corrupt section entries veto the
> file per the hostile-file philosophy; v200 files keep loading through
> the same sectioned reader (type 4 still vetoed there); new gate
> `composite_roundtrip` (58th blocking entry) proves weld → author →
> save → reload → assert body count/type/parts/mass(1+2)/COM/velocity/
> colour/per-part materials/spring rewire, then a SECOND cycle proves the
> rebuilt body re-persists. FALSIFIABILITY: CRC corruption vetoed,
> truncated composite section vetoed, bad owner id vetoed — all verified.
> Suite `--all` 58/0 (floor 58), quick green, make everything exit 0.
> WP4 STEP 4.1 LANDED 2026-11-10: SPEC GRAMMAR v1 FROZEN as
> `docs/MIG_SPEC.md` — the §6 diagram became the reader's spec, expanded
> to the byte: file rules (line-oriented, `#` comments, whitespace-
> insensitive, C-locale numbers), the keyword state machine (`mig <name>`
> first-and-once, optional `frame assembly` the ONLY frame value — the
> reader never guesses a frame, part indices exactly 0..n-1 monotonic,
> 1..max_parts parts — a file over the P2 cap 16 is REJECTED, not
> truncated: the clamp is a pool property, not a file property), per-part
> fields pinned to the `mig_part` contract (per-type dims subsets with
> cross-type keys a named reject; pose w-first unit-within-1e-4 NEVER
> renormalised; material optional defaulting to body_defaults per part
> type; mass XOR density — no second volume math anywhere, density mode
> is mig_part_mass's exact formulas; colour r g b the WP4 arrival of
> per-part authored colour, default (1,1,1)), the round-trip contract
> (file stores ASSEMBLY-frame poses, body born COM-framed via the
> bridge, save re-emits assembly frame by the +COM un-rebase, totals
> never written — recomputed through compose on every load, the WP3
> loader-kindness rule carried into the spec), canonical normalized
> output order (the byte-identical target 4.2's gate pins), 26 named
> `mig_spec_*` diag codes (source `mig_spec`, mpe_diag discipline,
> write-on-success: a rejected file enters nothing into the spec store),
> and §7 names what is frozen OUT (joints, per-part no_collide, dynamic
> state, hulls/mesh — decisions, not omissions). Doc-only step: no engine
> files touched, no gate bump; 4.2's parser is the first consumer.**
> WP4 STEPS 4.2+4.3+4.4+4.5 ALL LANDED 2026-11-10: headless spec reader
> (`core/mig_spec.[ch]`, 26 named `mig_spec_*` diag codes, write-on-
> success) + terminal verbs `mig load/save/show/list` (`ui_input/term_obj.c`)
> + microvim edit (`mig edit`, re-parse + stage on exit) + GTK editor
> panel (`ui_input/mig_editor.c`, part list + per-part fields + COM marker
> in 3D view) + gate `mig_editor_roundtrip` (61st blocking entry — save/
> load bitwise identity, editor and terminal compose identical bodies,
> WP2 physics checks pass). Suite `--all` 61/0 (floor 61), quick green,
> make everything exit 0. WP4 COMPLETE — WP5 (MFS adoption behind the
> default-OFF toggle with the A/B acceptance bar) is next.
> WP6 6.1+6.2 LANDED 2026-11-11 (`ce87e60`, `77d1ec5`): doc sweep
> (PHYSICS §9 composite persistence, ARCHITECTURE part pool, glossary/
> index/testing counts) + RELEASE_POLICY v16S charter entry + the
> duplicate `mpe_engine.h` version macros removed in favor of
> `core/mpe_version.h`. WP2.5 COMPLETE 2026-11-11: all six sub-steps
> landed one commit each — 2.5.0 flag `dc925b4`, 2.5.1 point collection
> `09929fe`, 2.5.2 2D proof `6b987e4`, 2.5.3 hull mass props `3ebb428`,
> 2.5.4 SAT reference pairs `81ec1d0`, 2.5.5 cut note (this line +
> the §9 ledger entry). Suite floor 65, suite 65/65, quick 79/79 at the
> landing; everything ships behind `composite.hull 0` default. WP2.5 is
> DONE — the next parts are up to the user.
> WP2.5 HARDENING LANDED 2026-11-11 (`da6d9ac`): the construction bug
> hunt found and fixed one real defect — the SAT axis normalization
> computed len = sqrt(c²+…) raw, so a cross of two ~1e200 edges
> overflowed to Inf and a zero axis fabricated depth-0 touches (face
> normals lost their axes as Inf/Inf = NaN; sane O(1) gates never reach
> that regime) — fixed by max-abs pre-scaling before squaring wired
> into all five normalization sites, plus full finiteness sweeps
> (all six polygon accumulators; mesh derived com/Jc/trace/mass).
> Scale pins added inside the existing gates, floor stays 65: sliver →
> bad_point; huge density/vertices/underflow → bad_mesh; huge hull →
> 1e200@x, box half 1e308 → 2@y, sphere r 1e308 → 1e308@x, ceiling
> center → cleanly separated. Doc consolidation swept the stale counts
> everywhere (readme, RELEASE_POLICY, RELEASE_GATES, GLOSSARY, INDEX,
> install instructions, runner docstring + floor narration, MFS
> TESTING.md 33→37 amendment, DEV_GUIDE MIG math row).

---

## §0 Resume protocol (read this first, always)

If a session (or its tool budget) died mid-stream, re-establish ground
truth in this order and do not write a new line of code until green:

```bash
cd 475-MPE/v16R1/src
make build_suite            # zero complaints expected (-Werror is default)
./test_mpe_suite --all      # expect: Total: 69 | Blocking failures: 0
make -C ecosystem/mgb && make mfs_ecosystem
bash ecosystem/mfs/build_tests.sh   # modules embed engine sources — every new
                                    # core/*.c must join all three parallel lists
                                    # (mgb/makefile CORE, mfs/mfs_sources.mk,
                                    # mfs/build_tests.sh CORE) or the module
                                    # links fail on undefined symbols;
                                    # expect: FTC RESULT: gated pass=5 fail=0 info-diags=12
cd ../..
python3 tools/test_runner.py --profile quick   # expect Checks: 83 | passed: 83
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

## §1 Baseline ledger (as of 2026-11-08 — update at every landing)

Landed and pushed (WP0 block: commits `181c2c0` charter, `0882935` code,
`636b10e` playbook; WP1 1.1 in `b0e1cbb`; rulings `8385b95`; P1 rename
`84f8c8d`; WP1 1.2–1.4 in `df383f0` — 2026-10-31; WP2 2.1 in `98debd9`,
WP2 2.2 in `364428d`, WP2 2.3 in `d74319c`, WP2 2.4+2.5 in `0a03b85`
— 2026-11-01; live-drive fix `76d6ccd`; WP2 2.6 in `bc395b2`
— 2026-11-08):

| Artifact | Nature |
|---|---|
| `v16R1/docs/NON_UNIFORM_BODY.md` | design charter (frozen decisions) |
| `v16R1/src/core/mig_mass.h` | composition contract + error enums + the 1.2 `mig_bridge` handoff struct + 2.2 `mig_part_mass` (effective per-part kg) |
| `v16R1/src/core/mig_mass.c` | exact composition: mass/COM/inertia, double accumulation, definition-order deterministic; + `mig_make_bridge` + `mig_part_mass` |
| `v16R1/src/core/rigidbody.h/.c` | 1.3 counter + accessor + reset; grows-once body fields `mass_is_composed`/`composite_part_count` (zeroed at all three initializer sites); 2.1 appends `object_composite` + `OBJECT_COMPOSITE`, the engine-internal `part_base` slice anchor, and `mpe_note_composite_clamp`; 2.2 `rigidbody_restore_composed_inertia` + sanitize composite type/geometry branches and composed restores in sanitize/wake/`set_static(false)`, with all three `rigidbody_update_inertia_*` early-returning for composed bodies |
| `v16R1/src/core/physics_world.h/.c` | 1.4 `physics_world_add_composite` stub (composed mass, sphere bounds, authored tensor); 2.1 `composite_part` + per-world pool (`composite_parts`/count/capacity), grower, write-on-success `physics_world_composite_parts_append`, cleanup free + clear compaction; 2.2 completes `physics_world_add_composite` (whole-assembly validation, stored-slice composition under the cap, COM-framed pool slice, effective per-part kg, authored-tensor precondition) with `physics_world_composite_cap` shared by append; 2.4/2.5 `_Thread_local` ghost views, per-part AABB prefilter, `mpe_shape_dispatch_all` (one manifold per part pair) and the serial composite expansion in `physics_world_process_pair`, plus part-scoped novelty; 2.6 `mpe_composite_floor_batch` + `mpe_composite_floor_detect` stage one plane manifold per touching active part (parent identity, `part_a` stamp, `part_b` 0) and wake once before `collision_prepare_solver`, with the parallel CCD phase-1 worker calling the world-aware TOI entry |
| `v16R1/src/physics/collision_ccd.c`, `collision_mechanics.h` | 2.6 composite floor support scan: `ccd_composite_part_at` bounded by the world pool, `ccd_part_support_down` (sphere bottom, cube projected corner, cylinder cap+barrel), `ccd_composite_support_point` returns the true parent-COM-relative lowest point including the horizontal lever arm, `ccd_min_thickness_world` uses the minimum active part thickness, and the floor TOI consumes that point; `ccd_sweep_clamp_full_impl` + `collision_ccd_toi_range_world` carry the world pointer, non-composite arithmetic stays byte-identical, and volume sweeps keep `broadphase_bounding_radius` (composites join the bounding-sphere obstacle branch) |
| `v16R1/src/physics/broadphase.c` | 2.3 composite part dispatch: authored covering-sphere scalar radius at `:149` (no sqrt(3) inflation), `broadphase_composite_extent` at `:187` (per-part COM-symmetric hull from body `cached_axes` × part `local_orientation`, primitive extents, inactive-slot scalar fallback), AABB call at `:379`; spatial hash, swept insert and sphere cull untouched |
| `v16R1/src/scene/scene_init.h/.c` | live-drive fix (`76d6ccd`): shared `SCENE_DEFAULT_FILLER_RADIUS` + `scene_default_filler_position()` place the default sandbox filler at (0,2,-6), clear of the origin `ftc spawn`; the default scene still rests it on the friction floor |
| `v16R1/src/tests/paranoia_default_scene_spawn.c` | named live-drive regression gate `default_scene_spawn_clearance`: real `scene_init_default()` + FTC robot/drivetrain; filler clearance, settled rest height, and 600-tick drive roll/pitch/height/jitter assertions; proven to fail against the old (0,2,0) arrangement |
| `v16R1/src/physics/collision_mechanics.h, collision_cache.c, collision_solver.c` | 2.4/2.5 append-only `contact_point_data.part_a/.part_b` at `collision_mechanics.h:81`; `cached_contact` tails at `physics_world.h:40`; `contact_cache_save` writes the part key, `contact_cache_has_part_pair` at `collision_cache.c:58` drives part-scoped novelty, and `contact_cache_match_role`/`contact_cache_adoptable` at `collision_solver.c:117/:144` require both part fields in both body roles |
| `v16R1/src/core/mpe_module.h, mpe_loader.c, mpe_registry.c` | 2.4/2.5 ABI 2 with `MPE_MODULE_ABI_MIN 1` (`mpe_module.h:41`); loader/registry/attach accept 1..2 (`mpe_loader.c:331`, `mpe_registry.c:493`, `physics_world.c:1152`); in-tree capsule plugin and both MFS makefiles build at ABI 2 |
| `v16R1/src/config/mpe_config.h/.c, mpe_config_schema.c` | 2.1 appended `cat_composite` + `composite.max_parts` (default 16, schema 1–16, save loop reaches the new category) |
| `v16R1/src/tests/mpe_suite_mig.c` | gates `mig_mass_composite` + `mig_pendulum_physical` + `mig_composite_stub` + `mig_composite_pool` + `mig_composite_world` + `mig_composite_broadphase` + `mig_composite_narrowphase` + `mig_floor_ccd_composite` (54th, blocking) |
| `v16R1/src/tests/mpe_suite_main.c` | externs + registry entries (54th run blocking) |
| `v16R1/src/makefile` | `MPE_TEST_CORE += core/mig_mass.c`, `MPE_SUITE_SRCS += tests/mpe_suite_mig.c`, help-note renumber |
| `tools/test_runner.py` | floor 46 → 47 → 48 → 49 → 50 → 51 → 52 → 53 → 54 with dated history comments; docstring re-based to the real 54-entry suite; paranoia floor 14 → 15 for `paranoia_default_scene_spawn` |

Names in this table are the PRESENT (post-P1-rename) spellings; at their
landing commits the files were `nu_mass.{h,c}` and the gates were
`nuf_*` — see §2's mapping and the rename line in §9. The rename landing
also renamed this playbook file itself (`NUF_BUILD_PLAN.md` →
`MIG_BUILD_PLAN.md`).

Measured baseline (numbers a resume session must reproduce):

- Suite: 54 registered, **52/52 blocking green**, 2 diag-informational
  (WP0 landing was 47/45; 1.1 made 48/46; 1.2–1.4 made 49/47; 2.1 made
  50/48; 2.2 made 51/49; 2.3 made 52/50; 2.4+2.5 made 53/51; 2.6 made
  54/52).
- Runner quick: **68/68** (WP0 was 61/61; 1.1 was 62/62; 1.2–1.4 was
  63/63; 2.1 was 64/64; 2.2 was 65/65; 2.4+2.5 was 67/67; 2.6 was 68/68).
- Runner physics: **114/114** after the live-drive fix (30 legacy + 15
  paranoia, including `paranoia_default_scene_spawn`; the canonical 2
  informational diags stay non-blocking; 0 blocking failures).
- ASan/UBSan suite build: the MIG gates are clean —
  `mig_floor_ccd_composite`, `mig_composite_narrowphase`,
  `mig_composite_broadphase`, `mig_composite_world`,
  `mig_composite_pool`, `mig_composite_stub`, `mig_mass_composite`,
  `mig_pendulum_physical`, `determinism` all pass with no sanitizer
  report (run total 54, 50/52 blocking: the asan target itself still
  fails `loader_lifecycle` and `ftc_ecosystem` on the pre-existing
  missing `-rdynamic` export of `g_cfg`, unrelated to MIG and recorded
  in §9).
- Engine binary: **code changes, the plain-scene numeric path does not**
  (`0a03b85`: `contact_point_data` grows 132 → 136 bytes, `cached_contact`
  68 → 72 and `collision_data` 560 → 576 for the append-only part fields,
  while `rigidbody` stays 396 bytes; `MPE_MODULE_ABI` moves to 2 with
  ABI-1 descriptors still loading; no config key moves, and every new
  dispatch/cache branch is gated on `object_composite` or the part
  fields being non-zero, so non-composite pairs keep their exact path).
  Binary identity with 2.3 is not claimed; the plain-scene numeric path
  is certified by the headless byte-probe against `8e62e58`,
  byte-identical (ticks=3600 bodies=2 invalid=0 fallback_pow=0
  fallback_trig=0 result=PASS), and the in-suite `determinism` twin stays
  600 ticks bitwise identical. The WP1 (`df383f0`) and 2.1 (`98debd9`)
  entries remain the record of the bits-changing struct landings; 2.4/2.5
  is the third such landing, now with an explicit plugin-ABI bump. 2.6
  adds no struct growth and no ABI change: its floor-staging and CCD
  branches are gated on `object_composite` or a non-NULL world pointer,
  so every non-composite sweep keeps the old arithmetic, and the
  determinism twin remains green.

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
- [x] **2.3 Broadphase.** `broadphase_bounding_radius` (~`
  physics/broadphase.c:139`) composites = max over parts of
  `|d_part,+ part bounding radius|` (exact, conservative); the AABB path
  (~:306) gains the composite branch vs per-part axis slicing; the
  spatial hash itself is untouched.
  LANDED 2026-11-01 (`d74319c`). `broadphase_bounding_radius` gains the
  `object_composite` arm at `broadphase.c:149` returning the AUTHORED
  `rb->radius` (the exact `max_i(|com - p_i| + B_i)` written by 2.2) with
  the 0.5 fallback, not the `sqrt(3)` cube inflation of the mirrored
  half_extents. The new static `broadphase_composite_extent`
  (`broadphase.c:187`) builds the per-part COM-symmetric hull the
  one-box-per-body insertion path can represent,
  `max_i(|R_body d_i| + extent_i_axis)`: body rotation from
  `cached_axes` (world columns) composed with the part's
  `local_orientation` via `vector4_to_math3`, primitive extents sphere
  `(r,r,r)`, cube `half_extensions`, cylinder `(half_length, radius,
  radius)` with the axle on part-local X, inactive and out-of-slice
  slots skipped, false when no usable part remains. The AABB loop calls
  it at `broadphase.c:379` and falls through to the existing cube branch
  on false, where the composed body's `half_extensions` mirror the
  authored radius — so the fallback stays conservative, never zero. The
  spatial hash, swept insert, sphere cull and every non-composite path
  are untouched. Gate `mig_composite_broadphase` (52nd, blocking,
  `mpe_suite_mig.c:1002`, extern/registry `mpe_suite_main.c:115/:228`,
  runner floor 52 at `test_runner.py:147`) pins a 0.5 m grid: dumbbell
  authored bound 0.75 (not 1.299) and rotation-invariant, hull cell
  count 16 at identity → 32 at 45 deg Z → 64 for the all-inactive scalar
  box (+8 anchor cells: 24/40/72); an off-centre cube+sphere composite
  (COM +0.15) reads 24 cube cells under the cube's own 90 deg Z
  orientation, 16 with that orientation cleared, 64 for the inactive
  scalar box (+8: 32/24/72); two dumbbells 1.4 m apart emit exactly one
  pair; a cylinder (radius 0.1, half length 0.52, axle local X) rotated
  90 deg about Y reaches z = 0.52 and pairs with a 0.06 sphere at
  z = 0.58 inside the authored 0.529528 cull sphere, while a
  `(radius, radius, half_length)` reading would stop at z = 0.1 and lose
  the shared cell; an all-inactive dumbbell still pairs with a sphere at
  y = 0.6 through the conservative 0.75 scalar box. Two incidents, both
  gate-side and engine-untouched: the first oracle draft asserted zero
  pairs for rotated/cleared-part fixtures and failed because the hash
  bucket is coarser than the cell, so two bodies in distinct cells can
  share a bucket and the conservative sphere cull legitimately admits
  the candidate — the negative direction was moved to the exact
  `world->broadphase->node_count` cell-occupancy oracle while positive
  pair assertions remain; and the first cylinder fixture placed the
  extent on an exact 0.5 cell boundary where a 90 deg rotation's float
  epsilon produced 0.99999994 and `floor()` dropped the band — fixture
  values moved off integer boundaries, not the engine. No struct or
  config change (`rigidbody` stays 396 bytes); ASan runs the new gate
  clean while the target's two pre-existing plugin-loader gates stay red
  as recorded in §9.
- [x] **2.4 Narrowphase — ghost-view dispatch (the WP2 crux).** Design,
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
  LANDED 2026-11-01 (`0a03b85`): the ghost lives in a `_Thread_local`
  pair scratch (`mpe_composite_ghost_refresh` at
  `core/physics_world.c:1369`) rather than a world field — every worker
  owns its own pair, the content is refreshed per part pair, and no world
  state is retained, so concurrent worlds never share a ghost. The
  memcpy inherits the parent's motion, overwrites only the primitive
  geometry/material fields and the part-local pose (parent position +
  rotated local_position, normalised parent × local orientation), clears
  the composed-mass/pool bookkeeping, and rebuilds `cached_axes`; the
  parent and the pool are byte-identical after detection (the gate
  memcmps both). `mpe_composite_iterate` at `:1421` walks active part
  pairs in definition order, runs the conservative per-part world AABB
  prefilter with `solver.penetration_slop` as margin (sphere `(r,r,r)`,
  cube half extents, cylinder `(half_length,radius,radius)` through
  `cached_axes`), and calls the pre-composite
  `mpe_shape_dispatch_primitive` at `:1483` so a primitive pair function
  is never rewritten and the composite branch cannot recurse.
  `mpe_shape_dispatch` at `:1543` keeps the direct-caller contract and
  returns the deepest single part-pair manifold (definition-order tie);
  the new `mpe_shape_dispatch_all` at `:1571` emits one manifold per
  colliding part pair with parent object pointers and per-contact part
  indices. The step path skips composite pairs in
  `mpe_narrowphase_range`, runs the primitive workers/prepare pass
  unchanged, then expands composites serially after the barrier:
  `mpe_parallel_narrowphase` at `:549` returns the compacted/zeroed
  span or the larger appended total, and `physics_world_process_pair` at
  `:1869` grows the manifold arrays on demand, prepares each part-pair
  manifold individually, and counts every dropped manifold in
  `manifold_overflow_count` instead of losing it. Gate
  `mig_composite_narrowphase` at `mpe_suite_mig.c:1182`
  (extern/registry `mpe_suite_main.c:116/:235`) pins literal geometry:
  a dumbbell over a slab emits two part-pair manifolds with part stamps
  0/0 and 1/0, contact x = -0.5/+0.5 and depth 0.01, while parent and
  pool bytes stay unchanged; a bounded sink reports dropped=1; the
  single entry returns the deepest; inactive parts disappear one at a
  time; a part-local 90Z cube face moves y 0.27 → 0.55 and a probe at
  y=0.56 collides only through both the AABB prefilter and the primitive
  path; composite-vs-composite finds exactly part pair (1,1).
  Gate-side incident, engine untouched: the first compile needed
  `core/mpe_registry.h` for the ABI probes, the per-world cache getters
  take a world argument, and `static_plane_enabled` is a world field
  rather than a config key; all three were fixture fixes. No primitive
  pair function changed, no malloc was added, and the zero-composite
  path remains the old one.
- [x] **2.5 Contact identity ≥ day one.** Append-only fields on
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
  LANDED 2026-11-01 in the SAME commit as 2.4 (`0a03b85`), as the order
  rule demands. `contact_point_data` carries the append-only
  `part_a`/`part_b` tail at `physics/collision_mechanics.h:81` and
  `cached_contact` at `core/physics_world.h:40`; `contact_cache_save`
  (`collision_cache.c:119`) stores the part fields; the hash bucket stays
  body-pair keyed while the semantic match key becomes
  `(id_a, part_a, id_b, part_b)`, `contact_cache_match_role`
  (`collision_solver.c:117`) and `contact_cache_adoptable` (`:144`) require
  both part fields in BOTH body roles with the part fields swapped when
  the body roles reverse — the TASK_05 both-side-match discipline
  applied to parts. `contact_cache_has_part_pair`
  (`collision_cache.c:58`) drives wake-on-first-touch; the old
  body-level `contact_cache_has_pair` remains for non-composite callers.
  `MPE_MODULE_ABI` is 2 with `MPE_MODULE_ABI_MIN` 1 at
  `core/mpe_module.h:41`; loader, registry and attach accept the 1..2
  range (`mpe_loader.c:331`, `mpe_registry.c:493`,
  `core/physics_world.c:1152`), so ABI-1 descriptors still load. An
  ABI-1 pair handler was compiled against the smaller contact stride and
  is documented as unsafe for composite manifolds; the in-tree capsule
  plugin and both MFS makefiles build at ABI 2, and the standalone twin
  carries the mirror commit `37d61e6`. Measured struct growth:
  `contact_point_data` 132 → 136 bytes, `cached_contact` 68 → 72,
  `collision_data` 560 → 576; `rigidbody` stays 396. Gate oracles:
  cache seeds 3.0/7.0 restore only for their own part pair, unknown
  part (0,0) misses, the reversed body role matches only the swapped
  part fields (tangent sign negated), hit/miss counts are 3/2, and a
  stepped dumbbell saves exactly two cache entries at distinct
  composite-local x = ±0.5 — no cross-part aliasing. Gate falsifiability
  was proven with two temporary mutations, each restored and rebuilt:
  removing the part comparisons from the solver predicates produced 5
  gate failures, and forcing `part_index_a/b = 0` in the iterator
  produced 6. No ABI-1 `.so` test binary is run as a pair handler; the
  gate proves descriptor attach/registry acceptance and ABI-3 refusal.
- [x] **2.6 Floor + CCD.** Per-part floor manifolds under the existing
  plane path (gate `mig_floor_ccd_composite`: tumbling L-composite
  clamps before tunneling at canonical dt, no landing pump); floor CCD
  lowest-point scan over parts; volume sweeps stay bounding-sphere.
  Rotation-only tunneling stays unsupported — same frontier as v15S,
  now named once for composites in the same sentence.
  LANDED 2026-11-08 (`bc395b2`). Floor: `mpe_composite_floor_detect`
  (`core/physics_world.c:1401`) walks the active part slice, refreshes a
  fresh ghost per part through the 2.4 helper, and runs the unchanged
  `collision_static_plane_body`; each touching part becomes its own
  staged `collision_data` with `object_a` remapped to the parent,
  `object_b` the static plane, and every contact stamped `part_a = part
  index`, `part_b = 0`. The whole batch is staged before the wake
  decision so one `rigidbody_wake` on the deepest staged penetration
  precedes every `collision_prepare_solver`, and each manifold is then
  prepared/appended individually; a manifold-array grow failure drops
  only the affected staged manifold and counts it in
  `manifold_overflow_count`. Non-composite bodies keep the original
  single-manifold branch byte-for-byte. CCD: the phase-1 sweep gained a
  world-aware entry (`collision_ccd_toi_range_world` at
  `physics/collision_ccd.c:441`, declaration at
  `collision_mechanics.h:359`; the parallel worker at
  `physics_world.c:408` uses it), and the floor TOI consumes
  `ccd_composite_support_point` (`collision_ccd.c:118`) — the actual
  lowest point of each active part (sphere bottom, cube projected
  corner, cylinder cap+barrel) offset by its rotated local center,
  including the horizontal lever arm for the `w × r` term — instead of
  the old body-level box support. `ccd_min_thickness_world` (`:152`)
  takes the minimum active part thickness, and `ccd_sweep_clamp_full_impl`
  (`:548`) carries the world pointer through the existing full sweep.
  Volume sweeps deliberately keep `broadphase_bounding_radius`: a
  composite joins the bounding-sphere obstacle branch, and no per-part
  swept-volume claim is made. Rotation-only tunneling remains the v15S
  frontier, unchanged. Gate `mig_floor_ccd_composite`
  (`tests/mpe_suite_mig.c:1577`, extern/registry
  `mpe_suite_main.c:117/:241`, 54th blocking) pins hand oracles: the
  two-sphere dumbbell over the plane emits exactly two parent-stamped
  floor manifolds with contacts at x = ±0.5 and depth 0.01; a 45 deg Z
  L-composite at y = 0.5975 has true lowest point 0.2793 m while the old
  box support read −0.0010 m (inside the resting slop), and a −30 m/s +
  10 rad/s tumble clamps
  in-tick with time remaining in (0, dt), then survives 30 landing ticks
  with max upward speed < 0.5 m/s and no penetration below −0.05 m.
  Falsifiability was proven, not assumed: temporarily forcing
  `ccd_composite_support_point` to return false made the clamped,
  time-remaining and lowest-after checks fail (3 failures); the mutation
  was restored and rebuilt. ASan/UBSan run is clean for the new gate and
  the other MIG/determinism gates; the target as written still fails
  only the pre-existing `loader_lifecycle` / `ftc_ecosystem` pair
  (50/52, recorded in §9). No struct grows and no ABI moves in 2.6.
- [x] **2.7 Renderer.** composite draws per part through the existing
  cube/sphere/cylinder meshing+instancing paths with part poses;
  body-level `no_collide` respected. (Mesh instance count guard: parts
  are bounded by P2's cap — no renderer unboundedness.)
  LANDED 2026-11-09. `render/new_render.c` only — three additions, zero
  primitive-path changes: (1) static helper `render_pack_composite_part`
  packs ONE instance for one part into a per-type instance buffer through
  the same T*R*S column composition the primitive path uses
  (M[c][r] = R[c][r]·s[c], M[3][r] = t[r], `math4_to_flat_array` tail);
  scale per part primitive — sphere r, cylinder axle-X half-length/r,
  cube half_extensions (the unit meshes are shared, not resized);
  (2) the `render_scene_current` body loop gains a composite branch
  BEFORE the primitive path: per active part of the bounds-checked pool
  slice (`part_base + i`), world pose = body ∘ part-local — position =
  body_pos + R_body·local_position, orientation = normalised
  R_body ⊗ local_orientation — the exact composition
  `mpe_composite_ghost_refresh` (2.4) uses, so what you see is what
  collides; colour = parent body's (parts carry no authored colour until
  the WP4 editor); the sphere-backed covering shape is NOT drawn (it is
  a physics envelope, not geometry — drawing it showed one giant sphere
  around the assembly); `no_collide` stays body-level exactly as for
  primitives (`physics_world.c:1592` gates collision, never visibility);
  (3) instance guard: per-type budget stays `mpe_max_bodies` (the VBO
  size, `cube_meshing.c:46`); overflow drops parts LOUDLY via a
  per-frame `dropped_instances` stderr warning, never silently — finite
  because parts are bounded by composite.max_parts (P2 cap 16). The
  primitive draw path is untouched, so zero-composite scenes are
  byte-identical (2.9's A2 statement holds). Renderer is GL-bound and
  suite-unlinked: NO new gate — verification is the GTK4 build exiting 0
  under -Werror, suite 54/52+2, quick 68/68 (re-run at landing).
- [x] **2.8 Gates** (each its own suite entry, floor bump same commit):
  LANDED 2026-11-09 as one commit with the `test_runner.py` floor bump
  54→57. Three new blocking entries:
  `mig_composite_stack` (55th) — 10-primitive stack vs one 10-part
  composite, pitch 1.002, same masses/materials; RULING: the settle
  contract is pinned to the CANONICAL config (gravity −9.81, sleep ON,
  cube restitution 0, friction 0.8/0.7, solver_iterations 64) because
  the `light` regression regime (8 iterations) genuinely slumps a
  10-stack — every 2.8 gate pins its own meaning after `mpe_test_begin`;
  checks: both settle ASLEEP inside the window, top-block poses within
  5 cm, first all-sleep tick finite (`mig_floor_ccd_composite` —
  ALREADY LANDED as the 54th entry in `bc395b2` (WP2 2.6), rather
  than deferred, because the step's own contract is the gate; part-scoped
  cache probe (two resting parts → distinct cache entries,
  warm starts never cross) — ALREADY LANDED inside
  `mig_composite_narrowphase` in `0a03b85` rather than as a later
  duplicate entry, because the ORDER MATTERS rule put the identity work
  in that same gate (the 3.0/7.0 seed split, unknown-part miss,
  reversed-role swap and stepped two-entry check are the probe)).
  `mig_pendulum_com_offset` (56th) — the charter end-to-end gate:
  composite bob with off-COM mass (2 spheres, d=0.8, explicit masses),
  period vs the hand-closed-form parallel-axis I_pivot=3.164 (transport
  terms 96% of it), θ₀-corrected analytic 2.0060 s, 1.5% band; measured
  2.0048 — this gate certifies the whole composed-mass contract through
  the solver, which is the thesis of the train. `mig_determinism_twins`
  (57th) — charter §4.5: the dumbbell×L×raining-primitives composite
  scene built twice, 600 ticks, every body's pose/velocities/sleep state
  and the contact-cache footprint bitwise equal, at the AMBIENT
  MPE_THREADS (the pool is process-lazy — cannot be set in-process);
  landing verification ran the gate at 1/4/8, all bitwise identical.
  ENGINE DEFECT FOUND BY THE STACK GATE AND FIXED IN THIS STEP: the
  boundary safety net (`scene/boundary.c`) read body-level OBB fields;
  composites are sphere-initialised (radius=bound,
  half_extensions=(r,r,r)) then retyped, so min_y = 5.009−5.375 =
  −0.366 m — the net clamped every composite to its COVERING-SPHERE
  rest and the 10-stack never slept (first run: 3 checks failed). Fix:
  `boundary_composite_extrema_along_axis` computes per-part exact
  support (world passed down to read the part pool), applied through
  `boundary_apply_box_world_cfg`; the legacy entry points delegate with
  world=NULL byte-identically, and the engine call site
  (`physics_world.c:2574`) passes the STEPPED world (not
  `physics_world_get_primary`) so twin test worlds stay independent.
  The fix exposed a 2.6 fixture lie: composite floor contacts read PART
  pool materials (body_defaults cube_restitution default 0.5), not the
  body-level restitution the gate pinned — the old boundary clamp had
  been swallowing the bounce. The gate now pins
  `ccfg.body_defaults.cube_restitution = 0` (and sphere=0 for the
  dumbbell scene); the max_up band recalibrated from fixed 0.5 to
  self-scaling `max_up < 0.25·v_impact` — the corner impact converts
  spin to COM rise (ωz −29.5 → −2.3 rad/s, vy +0.955) with energy
  DECREASING, a lawful bounce and not a pump — plus a settle check
  |vy| < 0.25 at tick 30. FALSIFIABILITY (mutate → expect failure →
  restore → rebuild): M1 composites excluded from the sleep-calm
  condition (`rigidbody.c:1137`) → stack gate 1 failure (`first_sleep_b`);
  M2 the composed-tensor overwrite in `physics_world_add_composite`
  skipped → pendulum gate 1 failure (2.1359 vs 2.0060); M3a floor-batch
  part-order reversal = NULL mutation (twins bitwise identical —
  same-body floor-manifold order is bitwise-insensitive on this solver
  path, a robustness fact worth keeping); M3b part-PAIR iteration order
  in `mpe_composite_iterate` keyed to the world's runtime address (a
  non-pure order source) → twins at MPE_THREADS=4 5 failures
  (cache footprint + all 4 dynamic bodies diverged). Every mutation
  restored and rebuilt green: suite 57/55+2 (Total: 57, blocking
  failures 0), quick 71/71.
- [x] **2.9 Verify:** LANDED 2026-11-09 — WP2 COMPLETE. (1) `make tsan`
  green: the TSan-built suite ran `--all` at MPE_THREADS=4 with
  TSAN_OPTIONS=halt_on_error=1 — Total 57, 0 blocking failures, no
  reports. The ghost scratch (the `_Thread_local` per-worker composite
  ghost views) is the worker-race surface this profile exists for —
  clean; the paranoia determinism TSan probe ran clean ahead of the
  suite. (2) A2 byte probe CERTIFIED EMPIRICALLY: the probe source
  (pre-WP2-stable API only, so it compiles against both eras) builds a
  ZERO-composite scene — settling sphere, settling cube, and one sphere
  flung sideways at 60 m/s so the boundary safety net clamps it at
  x = 250 − r for most of the run (this exercises exactly the
  `world=NULL` delegation the 2.8 boundary fix claims byte-identical) —
  steps 3600 ticks and prints every body's full pose, velocities and
  sleep state as hex-float. Current engine vs a `git worktree` build of
  `8e62e58` (pre-WP2, the same base as the 2.3-era probe):
  BYTE-IDENTICAL — invalid=0 fallback_pow=0 fallback_trig=0 result=PASS
  on both sides. Probe source + both captures:
  `temp/qa_runs/A2_2p9_byteprobe/`. (3) Plain headless smoke unchanged
  (ticks=3600 bodies=2 invalid=0 fallback_pow=0 fallback_trig=0
  result=PASS). (4) Suite `--all` green at floor 57 (Total: 57, 55/55
  physics blocking + 2 diag).

## §5 WP3 — scene persistence (v200 `MIG1` record) — LANDED 2026-11-10

Charter §5 is the contract; the plan adds touch-points only.

- [x] **3.1 Writer/reader two-pass (LANDED, shape deviation):** delivered
  as scene format **v201** (`mpe_version` 200→201) with a dedicated
  `composite_count` section after revolutes rather than in-band `MIG1`
  body-row extensions — same contract, cleaner layout: per composite,
  `owner_object_id` + ACTIVE part slice expanded to the WORLD frame (the
  exact representation `physics_world_add_composite` consumes). CRC
  covers the stream unchanged; the strict v200 EOF check stays
  byte-exact for v200 files (which keep loading through the same
  sectioned reader, type 4 still vetoed there).
- [x] **3.2 Loader kindness (LANDED, stricter-than-plan):** the loader
  never trusts the file for the body — mass/COM/inertia are recomputed
  through `physics_world_add_composite` at commit (the load-sum gate is
  structural: the compose path IS the recomposition). Deviation, deemed
  stricter and simpler: a composite with no usable part slice or a
  rejected compose is DROPPED LOUDLY (stderr, counted) instead of
  placeholdering a bounding cube — a placeholder composite would be a
  body that looks welded but is not. Structurally corrupt section
  entries (bad owner, over-cap slice, bad dims/quat) veto the file, per
  the loader's hostile-file philosophy. Envelope dynamic state
  (velocity, colour, materials, static/kinematic/sleep, generation,
  saved id) re-applies to the rebuilt body; ids are preserved so joints
  rewire unchanged.
- [x] **3.3 Gate (LANDED):** `composite_roundtrip` in
  `tests/mpe_suite_c.c` (same commit as the writer): weld 2 parts →
  author state → save → reload → assert body count, type, part count,
  recomposed mass (1+2), COM within 1e-3, velocity/colour/body
  materials, per-part materials from the FILE (not body_defaults),
  spring rewired to the preserved id — then a second save/load cycle
  proves the rebuilt body re-persists. Suite floor 57 → 58.

## §6 WP4 — terminal verbs + spec format + the tiny CAD editor

Headless before GTK, always. The spec text is the single source of truth;
the editor is a window onto it; no editor-only truth exists.

- [x] **4.1 Spec grammar v1 (LANDED 2026-11-10, FROZEN as
  `docs/MIG_SPEC.md`):** the diagram below became the reader's spec,
  expanded to the byte — file rules, keyword state machine, per-type
  dims subsets pinned to the `mig_part` contract, the assembly→COM frame
  round-trip rule, canonical normalized output (4.2's byte-identical
  target), 26 named `mig_spec_*` diag codes, and a §7 of named
  exclusions. Doc-only step (no engine files touched, no gate bump);
  every later WP4 step implements THIS text, and post-freeze changes are
  dated v2 amendments, never silent extensions. The frozen v1 grammar:

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

- [x] **4.2 Headless surface first (LANDED 2026-11-10):** pure grammar
  reader in `core/mig_spec.[ch]` implementing the frozen `MIG_SPEC.md`
  v1 (no rigidbody/physics_world/config dependency); terminal verbs
  `mig load <file> | save <name> | show <name> | list` in
  `ui_input/term_obj.c` backed by a per-session named spec store (not a
  world write); registered terminal command `mig`. Write-on-success: a
  rejected file enters nothing into the spec store. 26 named
  `mig_spec_*` diag codes (source `mig_spec`, mpe_diag discipline).
  Gate `mig_spec_roundtrip` (60th blocking entry, runner floor 59 ->
  60): parse a valid spec, emit canonical normalized text, re-parse it,
  require byte-identical output; plus a 10-case reject-code battery.
  Parser correctness also verified with a standalone harness (22 cases:
  valid round-trip, all reject codes, required-field enforcement, part
  cap). Docs-only diagram → implemented reader; no engine behavior
  changed, gate bump only.
- [x] **4.3 microvim editing integration (LANDED 2026-11-10):** `mig edit
  <name>` writes the stored spec to `./<name>.spec`, opens it in the
  existing microvim buffer (`ui_input/microvim.c`), and on editor exit
  (`:wq`/`:x`/`:q!`/double-Esc) `mig_edit_on_exit()` re-parses the file
  through `mig_spec_parse_file` and stages it back into the per-session
  spec store — write-on-success, a rejected save leaves the entry
  unchanged. Wired into the terminal's microvim-exit detection branch in
  debug_terminal.c; `vi` stays blocked while an edit is active. GUI-only
  (microvim is suite-unlinked); no gate bump; engine build 0 under
  -Werror.
- [x] **4.4 GTK editor panel (LANDED 2026-11-10, `ui_input/mig_editor.c`):**
  part-list panel + per-part fields (type, dims per-type subset, pose
  w-first quaternion, material mu_s/mu_k/rest, mass XOR density, colour)
  in the config_menu/editor_dialog construction style; `mig editor`
  terminal command opens the panel (plain GtkWindow, transient, CSS
  keyed on #mpe-mig-editor, Esc closes); Save re-parses through
  mig_spec_parse_file and stages into the spec store (write-on-success);
  two-axis ground-plane translate gizmo via numeric X/Z position fields
  (mouse drag gizmo is post-v16S); live COM marker rendered in the 3D
  view (utility-shader cyan cross, X=red/Y=green/Z=blue) plus total-mass
  readout; principal-axis display is a named TODO, not v16S scope.
  Grammar frozen at 4.1 governs both paths (editor + terminal).
  GUI-only (suite-unlinked); no gate bump; engine + suite builds 0 under
  -Werror.
- [x] **4.5 Gate (LANDED 2026-11-10, `mig_editor_roundtrip`, suite floor 60 → 61):**
  editor panel edits → spec save → load from disk → bitwise identity,
  PLUS the WP2 physics gates re-run with editor-created bodies. Builds a
  spec programmatically (as the GTK editor would assemble it), emits
  normalized text, saves to disk, loads back, requires byte-identical
  normalized output; then composes bodies from the in-memory and re-loaded
  specs through `physics_world_add_composite` and requires bitwise
  identity (mass, COM, pose, inertia, per-part pool slices); re-runs WP2
  physics checks (mass = sum of effective part masses, COM = mass-weighted
  average, bound = covering bound) on the editor-created body. Pins
  "editor and terminal create the same objects through the same parser."
  Per-part authored materials are a post-4.5 feature (mig_part has no
  material fields — body_defaults per part type, review 9.3).

## §7 WP5 — MFS adoption (toggle default OFF, A/B or the gate doesn't flip)

Charter §8 is the contract; the plan adds the risk discipline.

- [x] **5.1** `mfs_robot_build_composite()` (LANDED 2026-11-10) in
  `ecosystem/mfs/modules/ftc/submodules/robot.c`: chassis + 4 wheel parts
  + ballast part as a single `object_composite` body via
  `physics_world_add_composite`. Parts in assembly frame, body born
  COM-framed. Per-part friction stamped on pool entries (wheel parts
  0.9/0.7, chassis/ballast from body_defaults). Drive forces applied at
  wheel-part world positions via `rb_apply_forces_localised` on the
  composite body (`ftc_robot_drive_composite`). Smoke gate
  `mfs_composite_build`: body count=1, type=object_composite, parts=6,
  mass=9.8 kg, part indices 0..5 monotonic, per-part friction carried.
  Gate: smoke only until 5.3 green (A/B battery).
- [x] **5.2** Config toggle `mfs robot.composite 0|1` (LANDED 2026-11-10); default 0 for the
  entire WP.
- [x] **5.3** A/B battery (acceptance bar) (LANDED 2026-11-10): straight,
  strafe, pivot, settle — composite robot vs child-body robot within
  documented drivetrain bands. Smoke gate `composite_ab`: builds composite
  body with `mfs_robot_build_composite()` both with toggle OFF and ON,
  verifies mass=9.8 kg, part indices 0..5 monotonic, per-part friction
  carried, finite state after tick. Suite gate additions for composite
  robots are SMOKES only until 5.3 green, real gates after.
- [x] **5.4** Odometry: θ stays ∫ω_y·dt (frame-invariant) (LANDED 2026-11-10):
  composite body born COM-framed, odometry theta = ∫ω_y·dt exact. Gate
  `composite_odometry`: spins composite body, verifies odom_theta =
  ∫ω_y·dt within 0.05 rad. Documented origin-vs-COM sweep artifact in
  `v16R1/FTC.md` §5: composite body COM is assembly COM, not chassis
  center; wheel odometry at part contact patch vs body odometry at COM
  measure different quantities; wheel odometry is NOT fused into body
  odometry — they are independent measurements; any discrepancy is real
  slip at the contact patch, not encoder drift.
- [x] **5.5** Contract hygiene (LANDED 2026-11-10): MFS suite keeps its
  current numbers (37 registered, 36 gated + 1 informational =
  `composite_roundtrip`... wait, `aim_solve` open red stays "reported,
  not touched" — a landing that incidentally fixed or broke it would be
  an untraceable change); suite gate additions for composite robots are
  SMOKES only until 5.3 green, real gates after. Suite floor lands at
  final v16S value 37 (31 original + 4 new: composite_build,
  composite_toggle, composite_ab, composite_odometry) in one dated
  entry. Quick profile re-baseline with new count on both trees.

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

- [x] **2.5.0 Flag (LANDED 2026-11-11, `dc925b4`).** `composite.hull 0|1`
  default 0, config-schema key; NOT the load path — every hull increment
  ships behind this flag.
- [x] **2.5.1 Point collection (no physics) (LANDED 2026-11-11, `09929fe`).**
  Import or build hull point sets as data; unit: exact tetra/box point
  sets, no solver involvement, suite-gated garbage rejection (non-finite
  points).
- [x] **2.5.2 The 2D proof (LANDED 2026-11-11, `6b987e4`).**
  Divergence-theorem area/moment on polygon cross-sections vs closed
  forms (box slice exact) — the mathematical foundation of 2.5.3, proven
  on the lowest dimension first.
- [x] **2.5.3 Hull inertia, standalone (LANDED 2026-11-11, `3ebb428`).**
  Divergence-theorem mass properties for closed triangle meshes as a
  pure math module (mig_mass discipline: double accumulation,
  write-on-success, named errors); gates: box hull reproduces the box
  closed form, tetra exact.
- [x] **2.5.4 SAT reference pairs (LANDED 2026-11-11, `81ec1d0`).**
  hull-vs-primitive narrowphase behind `composite.hull 1`,
  reference-pair gates only (box-hull vs box, sphere-hull vs sphere); no
  scene-format default flips.
- [x] **2.5.5 Cut note (LANDED 2026-11-11, see the §9 ledger line).**
  Whichever of 2.5.1–2.5.4 are green at the v16S cut ride to release; a
  ledger line names the shipped increment set.

## §8.6 WP2.6 — hull shape catalog (the side-study integration train)

Appended 2026-11-12. The prism/pyramid side study (untracked,
`studies/prisms_pyramids/`; pp_test 123/123, pp_cat_test 796/796, its
`pp_math.md` §1–§9) is the verification source; this WP connects it
into the engine one brick per commit, §8.5's discipline unchanged:
green-or-rollback, one ledger line per landing, suite floor raised in
the same commit that registers the new gate. Physics-touching bricks
ship behind `composite.hull`; pure-data bricks (2.6.1) run flag-off.
Transcription is EXACT — no trig/pow/π in coordinates, fixed literals
only (√3, φ; verified bit-identical to the study's spellings) — and
the study's closed-form oracles are the gates' oracles, unchanged.

- [x] **2.6.1 Exact-coordinate builders + exposed face tables (LANDED
  2026-11-12, see the §9 ledger line).** Four builders in
  `core/mig_hull.{h,c}` — `mig_hull_build_prism_equilateral`,
  `mig_hull_build_pyr4_square`, `mig_hull_build_pyr3_equilateral`,
  `mig_hull_build_icosahedron` — each shipping its outward-wound face
  table beside the point set (face tables were previously caller-side
  test literals; core now owns them for the shapes it builds). Gate
  `mig_hull_shape_catalog` (the 66th entry): the study's closed forms
  (V/com/inertia per shape, the study's size list) fed through
  `mig_hull_mesh_props` — every pass doubles as the mechanical winding
  proof — plus the regular-tetra isotropy pin (I = m·s²/20) and the
  icosahedron isotropy + circumradius pins; refusal battery maps the
  study's `pp_bad_dim` to `mig_hull_bad_point` (bad dims) and
  `mig_hull_bad_count` (full caller buffer), write-on-success
  preserved. Suite floor 65 → 66; suite 66/66, quick 80/80 at the
  landing.
- [ ] **2.6.2 n-gonal ring family (frustum/pyramid/bipyramid,
  `pp_math.md` §5–§6).** Needs the documented trig-exception ruling
  FIRST: the study's ring builders parametrize with sin/cos, the
  engine's coordinates are literal-only, so either a ruling admits
  trig for ring generation with the exact special cases gated
  separately, or the family ships as an exact-literal subset (n=3,4).
- [x] **2.6.3 Ellipsoid UV builder (LANDED 2026-11-12, gate
  `mig_hull_ellipsoid`, 69th, see the §9 ledger line).** Band-sum mass
  properties gate; affine covariance of the inertia tensor is the
  oracle the study proved. The builder is the 12×8 UV mesh (86 verts /
  168 faces) with exact algebraic ring/longitude literals
  (√2, cos/sin(π/8) = √(2±√2)/2 — no trig enters ENGINE coordinates);
  `mpe_hull_shape_ellipsoid` appended to the enum; `add_hull` maps
  (s, h) = equatorial/polar DIAMETERS (a = b = s/2, c = h/2); a
  general (a, b, c) spawn API is a later brick. The SAT scenario pins
  the faceted-sphere lesson with an independent brute-force oracle:
  the min-overlap axis is a band FACE normal (depth = 0.1·n_y for the
  unit pair at 1.9ŷ, 0.1·u_z on the north-fan face normal for the
  prolate pair at 2.9ẑ), never the naive translation axis.
- [x] **2.6.4 Generic hull-vs-hull SAT (LANDED 2026-11-12, see the §9
  ledger line).** Landed as a PURE-DATA brick — `mig_hull_sat_hull_hull`
  in `core/mig_hull.{h,c}` (boolean + MTD, first-min-wins ties, axis
  from A toward B), no engine consumption yet, so no flag: the
  `composite.hull` gate applies when a consumer exists (2.6.5 spawn).
  The plan's original "behind composite.hull 1" phrasing named the
  consumption precondition, not the function itself.
- [x] **2.6.5 Hull body + spawn consumption (LANDED 2026-11-12, gate
  `mig_hull_spawn`, 68th).** `object_hull` appended to `object_type`
  (append-only ABI), per-world hull vertex/face pools (grow-doubling,
  clear = compaction only), `physics_world_add_hull` (density-1
  mesh_props → rebase to COM → diagonal inertia × mass/V with
  off-diagonal refusal, both the tensor AND its inverse stamped,
  circumradius in `radius` + mirrored `half_extensions`), pair
  handlers (hull-sphere / hull-cube / hull-hull) ALL gated on
  `composite.hull` (spawning itself is not — a flag-0 world holds
  countable ghost hulls), floor support-vertex manifold (up to 4
  deepest, slop-gated), CCD mover-side branches (sweep obstacles skip
  hulls — same conservative class as cube/cylinder obstacles),
  broadphase radius, renderer explicit hull skip + counter, scene-save
  veto SVF06, `touch prism|pyr4|pyr3|icosa` verbs that refuse loudly
  when the flag is 0. `boundary.c` gained exact hull extrema off the
  pool slice — the covering-sphere net rest (the 2.8 composite
  failure class) would otherwise hold every hull a metre up. Gate
  `mig_hull_spawn`: mesh-props consistency (prism I(1.5,1.5,1.0),
  r = √(7/3), 6v/8f slice + refusal battery storing nothing), flag-0
  ghost (both registry orders refuse + count, sphere rests while the
  ghost falls through — net off per the f10/f11 precedent), flag-1
  prism rest at √3/3 with the 4-corner AB-face manifold, sphere
  bounce off the pyr4 apex, hull-hull MTD 0.1 both orders +
  drop-from-6 smoke; regime-robust (e = ½ and the heavier base twin
  pinned in-scenario).

## §9 Risk register + the standing ledger

- **Cross-part warm-start aliasing** — highest. Mitigations: part-scoped
  keys in the SAME commit as the first manifold map (2.4+2.5); the
  TASK_05 probe pattern as `mig` cache gate; no part-index reuse ever.
- **Floor churn (F10-class) through composites.** Mitigation: the
  `mig_composite_stack` sleep gate is named BEFORE 2.6 lands; pair-
  novelty wake should cover by construction; if it does not, the debugging
  path is the F10 root-cause record and the solver islands' sleep gates,
  documented in that order. 2.6 landed with the floor/CCD gate
  (`mig_floor_ccd_composite`, `bc395b2`); `mig_composite_stack` itself
  remains a 2.8 gate.
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
- 2026-11-01 — WP2 step 2.3 landed (`d74319c`), the fourth
  engine-touching MIG commit and the third WP2 one. Scope: broadphase
  part dispatch. `broadphase_bounding_radius` returns the authored
  covering sphere for `object_composite` instead of the `sqrt(3)`
  inflation the generic cube arm would apply; the new
  `broadphase_composite_extent` builds the per-part COM-symmetric AABB
  hull `max_i(|R_body d_i| + extent_i_axis)` from the world pool, body
  `cached_axes` × part `local_orientation`, primitive extents (sphere,
  cube, cylinder axle-local-X), skipping inactive slots and returning
  false when no part is usable so the AABB path keeps the conservative
  scalar-box fallback; the spatial hash, swept insert and cull are
  untouched. Incidents: two, both gate-side and engine-untouched — the
  first oracle draft asserted zero pairs for rotated/cleared-part
  fixtures and failed because the hash bucket is coarser than the cell
  (distinct cells can share a bucket, and the conservative sphere cull
  then admits the candidate as designed), so the negative direction
  moved to the exact `node_count` cell-occupancy oracle; and the first
  cylinder fixture put an extent on an exact 0.5 cell boundary where a
  90 deg rotation's float epsilon landed on 0.99999994 and `floor()`
  dropped the band, fixed by moving fixture values off integer
  boundaries. Engine truth: no struct or config change (`rigidbody`
  stays 396 bytes, no key moves), the binary differs from 2.2 only in
  the composite-gated broadphase code, and binary identity is not
  claimed; the plain-scene numeric path is certified by the headless
  byte-probe against `f3bdb7c`, byte-identical (ticks=3600 bodies=2
  invalid=0 fallback_pow=0 fallback_trig=0 result=PASS), with the
  in-suite `determinism` twin bitwise identical. Battery: suite 52
  total, 50/50 blocking + 2 diag; the new gate passes default plus
  light/heavy/brittle/sticky; quick 66/66; docs consistency 7/7; `make
  everything` exit 0 (log /tmp/opencode/make_everything_23.log); MFS
  sync guard in sync. ASan/UBSan: `mig_composite_broadphase` and the
  other six MIG/determinism gates are clean with no sanitizer report,
  while the asan target as written still fails only `loader_lifecycle`
  and `ftc_ecosystem` on undefined symbol `g_cfg` from its missing
  `-rdynamic` (48/50, pre-existing, unchanged, outside 2.3). No
  pre-existing engine defect surfaced; no numbers moved outside the new
  gate's own assertions.
- 2026-11-01 — WP2 steps 2.4+2.5 landed TOGETHER (`0a03b85`), the fifth
  engine-touching MIG commit and the fourth WP2 one. Scope: composite
  ghost-view narrowphase + part-scoped contact identity, one commit
  because the plan's ORDER MATTERS rule forbids a manifold map without
  scoped keys. `mpe_composite_ghost_refresh` builds a `_Thread_local`
  pair scratch by memcpy of the parent and overrides only part pose/
  geometry/material plus `cached_axes`; parents and pool stay unchanged.
  `mpe_composite_iterate` walks active part pairs in definition order
  behind a conservative per-part world AABB prefilter with the solver
  slop margin and calls the untouched primitive dispatch.
  `mpe_shape_dispatch` keeps the single-manifold direct-caller contract
  (deepest part pair), while `mpe_shape_dispatch_all` emits one manifold
  per colliding part pair with parent identities and per-contact part
  indices. The step path skips composites in the parallel worker pass,
  keeps the primitive prepare pass byte-for-byte, and expands composite
  pairs serially after the barrier; `physics_world_process_pair` grows
  and prepares each manifold and counts overflow instead of dropping it.
  Contact identity: `contact_point_data`/`cached_contact` gained
  append-only part tails; save stores them; match/adopt predicates
  require both fields in both body roles with swapped parts on reversed
  roles (the TASK_05 both-side-match precedent, now at part granularity);
  novelty moved to `contact_cache_has_part_pair` while the body-level
  probe remains. `MPE_MODULE_ABI` 1 → 2 with MIN 1; loader/registry/
  attach accept 1..2; ABI-1 pair handlers are documented unsafe for
  composites (old contact stride); in-tree capsule and both MFS
  makefiles build at 2, standalone mirror `37d61e6`. Incidents: one,
  gate-side and engine-untouched — the first gate compile needed
  `core/mpe_registry.h`, the per-world cache getter world argument, and
  `static_plane_enabled` as a world field; all fixture fixes. Gate
  falsifiability was proven, not assumed: removing the solver's part
  comparisons made 5 gate checks fail, and forcing part indices to 0 in
  the iterator made 6 fail; both mutations were restored and rebuilt
  before commit. Engine truth: `contact_point_data` 132 → 136 bytes,
  `cached_contact` 68 → 72, `collision_data` 560 → 576, `rigidbody`
  unchanged at 396; `MPE_MODULE_ABI` 2; no config key moves. Every new
  branch is composite/part-gated; the zero-composite path is certified
  by the headless byte-probe against `8e62e58`, byte-identical
  (ticks=3600 bodies=2 invalid=0 fallback_pow=0 fallback_trig=0
  result=PASS), with the in-suite `determinism` twin bitwise identical
  and both determinism probes green at MPE_THREADS=1/4/8. Battery:
  suite 53 total, 51/51 blocking + 2 diag; the new gate passes default
  plus light/heavy/brittle/sticky; quick 67/67; docs consistency 7/7;
  `make everything` exit 0 (log /tmp/opencode/make_everything_24.log);
  MFS sync guard in sync incl. the standalone makefile mirror.
  ASan/UBSan: `mig_composite_narrowphase` and the other seven
  MIG/determinism gates are clean with no sanitizer report, while the
  asan target as written still fails only `loader_lifecycle` and
  `ftc_ecosystem` on undefined symbol `g_cfg` from its missing
  `-rdynamic` (49/51, pre-existing, unchanged, outside 2.4/2.5). No
  pre-existing engine defect surfaced; no numbers moved outside the new
  gate's own assertions.
- 2026-11-08 — WP2 step 2.6 landed (`bc395b2`), the sixth
  engine-touching MIG commit and the fifth WP2 one. Scope: per-part
  composite floor manifolds and the floor CCD lowest-point scan. Floor:
  `mpe_composite_floor_detect` walks the active part slice, refreshes a
  fresh ghost per part through the 2.4 helper, and runs the unchanged
  `collision_static_plane_body`; each touching part becomes its own
  staged manifold with PARENT `object_a`, the static plane as
  `object_b`, and every contact stamped `part_a = part index`,
  `part_b = 0`. The batch is staged before the wake decision so one
  `rigidbody_wake` on the deepest staged penetration precedes every
  `collision_prepare_solver`; each manifold is prepared/appended
  individually, and a manifold-array grow failure drops only the
  affected staged manifold into `manifold_overflow_count`. Non-composite
  bodies keep the original single-manifold branch unchanged. CCD: the
  sweep gained a world-aware phase-1 entry
  (`collision_ccd_toi_range_world`, used by the parallel worker); the
  floor TOI consumes the true active-part support point in
  parent-COM-relative coordinates (sphere bottom, cube projected corner,
  cylinder cap+barrel, horizontal lever arm included), and
  `ccd_min_thickness_world` uses the minimum active part thickness.
  Volume sweeps keep `broadphase_bounding_radius` and composites join
  the bounding-sphere obstacle branch; rotation-only tunneling remains
  the v15S frontier, unchanged. Gate `mig_floor_ccd_composite`
  (`tests/mpe_suite_mig.c:1577`, extern/registry
  `mpe_suite_main.c:117/:241`, 54th blocking): the two-sphere dumbbell
  over the plane emits exactly two parent-stamped floor manifolds with
  contacts at x = ±0.5 and depth 0.01; a 45 deg Z L-composite at
  y = 0.5975 has true lowest 0.2793 m while the old box support read
  −0.0010 m (inside the resting slop), and a −30 m/s + 10 rad/s tumble
  clamps in-tick (time
  remaining in (0, dt)), then survives 30 landing ticks with max upward
  speed < 0.5 m/s and no penetration below −0.05 m. Incidents: two,
  gate-side and engine-untouched — the independence oracle first forgot
  to add the parent COM to its part-lowest start (reported −0.318198
  vs 0.2793), and the quaternion helper was rewritten with a local
  cross product so the gate never calls engine math. Gate
  falsifiability was proven: forcing `ccd_composite_support_point` to
  return false made the clamped, time-remaining and lowest-after checks
  fail (3 failures); the mutation was restored and rebuilt. Engine
  truth: no struct grows, no config key moves, `MPE_MODULE_ABI` stays
  2; every new floor/CCD branch is composite- or world-pointer-gated,
  and the in-suite `determinism` twin stays bitwise identical. Battery:
  suite 54 total, 52/52 blocking + 2 diag; the new gate passes default
  plus light/heavy/brittle/sticky; quick 68/68; `make everything` exit 0
  (log /tmp/opencode/make_everything_26b.log). ASan/UBSan:
  `mig_floor_ccd_composite` is clean with no sanitizer report, while
  the asan target as written still fails only `loader_lifecycle` and
  `ftc_ecosystem` on undefined symbol `g_cfg` from its missing
  `-rdynamic` (50/52, pre-existing, unchanged, outside 2.6). No
  pre-existing engine defect surfaced; no numbers moved outside the new
  gate's own assertions.
- 2026-11-11 — WP2.5 COMPLETE (cut note, 2.5.5): the P3-ruled convex-hull
  increment set is fully green and rides to the v16S cut. Shipped set,
  one commit each (green-or-rollback held): 2.5.0 `composite.hull` flag
  default 0 (`dc925b4`); 2.5.1 hull point collection, exact box/tetra
  point sets + garbage rejection (`09929fe`); 2.5.2 the 2D proof,
  divergence-theorem polygon moments vs closed forms (`6b987e4`); 2.5.3
  hull mass properties via face-tetra decomposition, box + regular-tetra
  exact (`3ebb428`, pre-gate probe caught the missing density factor on
  inertia before the suite ever saw it); 2.5.4 SAT reference pairs,
  boolean + MTD only for hull-vs-box and hull-vs-sphere (`81ec1d0`).
  All five ship behind `composite.hull 0` default; the SAT functions
  check the flag caller-side, never inside; scene files carry no hull
  state. NOT shipped at this cut, by design: contact manifold
  generation from hull pairs (future work — the SAT layer is
  boolean + MTD only), any scene-format default flip, and hull
  consumption by the engine's own primitive paths. Suite floor moved
  61 → 65 (four gates registered: mig_hull_point_collection,
  mig_hull_2d_proof, mig_hull_mass_props, mig_hull_sat_pairs); quick
  checks 75 → 79; suite 65/65, quick 79/79 at the landing.

- 2026-11-11 — WP2.5 HARDENING + DOC CONSOLIDATION (`da6d9ac` engine,
  this line + the sweep in the consolidation commit): the construction
  bug hunt over mig_hull found one real defect — the SAT axis
  normalization computed len = sqrt(c²+…) directly on cross
  components, so a cross of two ~1e200 edges overflowed the square to
  Inf, len came back Inf, and u = c/len collapsed to a zero vector
  that still passed the len > eps gate, fabricating a depth-0 touching
  verdict for a genuinely deep pair (face normals lost their axes as
  Inf/Inf = NaN); sane O(1) gates never reach that regime, which is
  why the suite stayed green. Fixed by max-abs pre-scaling before
  squaring ((αe1)×(βe2) = αβ(e1×e2) preserves direction and sign) via
  axis_unit/cross_unit helpers wired into all five normalization
  sites, a pre-scaled edge projection in the sphere edge-to-center
  path, and an upfront all-component finiteness check. polygon_props
  guards all six accumulators (the overflow sliver keeps area finite
  while second moments blow up); mesh_props sweeps the derived
  com/Jc/trace/mass = ρ·V. The discipline rule is unchanged and now
  pinned by gates: refuse (named error) when the OUTPUT would be
  non-finite, accept finite-but-huge inputs whose math stays
  representable — SAT on ceiling-scale geometry returns the true MTD
  where supports stay finite. Scale pins added inside existing gates,
  no new registry entries, floor stays 65, suite 65/65, quick 79/79,
  make engine + build_suite green. Doc sweep fixed every stale count
  found: readme status line (44-era → 65/79), RELEASE_POLICY (61 → 65
  + strict blocking convention), RELEASE_GATES (46/44 → 65/63 with the
  dated trail), GLOSSARY (46 → 65 entries), install instructions
  (42 → 65), runner docstring (54 → 65; MFS 30 → 37 gated) + the floor
  narration extended through 62-65 per its own say-so contract, MFS
  TESTING.md (33 → 37 amendment after WP5), DEV_GUIDE gains the MIG
  math row, mig_hull.h documents the SAT robustness contract.

- 2026-11-12 — WP2.6 opens, 2.6.1 (this commit): the hull shape
  catalog — four exact-coordinate builders (equilateral prism, square
  pyramid, equilateral-base triangular pyramid, icosahedron) with
  exposed outward-wound face tables in `core/mig_hull.{h,c}`,
  transcribed from the prism/pyramid side study (pp_shapes.c vertex
  conventions unchanged; fixed literals √3/φ, each verified
  bit-identical to the study's sqrt()-based spellings; √5 appears only
  in doc comments). New gate `mig_hull_shape_catalog` (66th entry,
  blocking): the study's closed-form oracles — V/com/inertia per
  shape at the study's size list, regular-tetra isotropy I = m·s²/20,
  icosahedron isotropy I = m·a²(3+√5)/20 plus all 12 vertices on
  Rc² = a²(5+√5)/8 — fed through `mig_hull_mesh_props` with the
  exposed tables, so every closed-form pass is also the mechanical
  winding proof; refusal battery: NULL → `mig_hull_null`, bad dims
  (0/negative/NaN/±Inf) → `mig_hull_bad_point` (study `pp_bad_dim`
  maps here), full caller buffer → `mig_hull_bad_count` with the
  exact fitting prefix kept, write-on-success preserved. ASan/UBSan
  spot-check of the new gate and the sibling hull gates clean (the
  study's §5.3 lesson — face buffers sized in ints, not tris — held:
  no engine-side repeat). Suite floor 65 → 66; suite 65/65 → 66/66,
  quick 79/79 → 80/80 at the landing; pure data + mesh_props, runs
  `composite.hull`-off.

- 2026-11-12 — WP2.6 2.6.4 (this commit): the generic hull-vs-hull
  SAT — `mig_hull_sat_hull_hull` in `core/mig_hull.{h,c}`, transcribed
  from the side study's `pp_sat.c` (the study's conventions mirror the
  engine's 2.5.4 hardening, so the helpers are shared: strict-`<`
  interval feed with first-min-wins ties, max-abs pre-scaled
  normalization before squaring, `MIG_HULL_AXIS_EPS 1e-12`). Axis set:
  A's face normals, B's face normals, then every directed A-edge ×
  B-edge cross (triangle-soup duplicates harmless); on overlap,
  translating A by −axis·depth separates the pair; on separation the
  axis points FROM A TOWARD B and depth is 0 (touching = overlap at
  depth 0, the engine convention). New gate `mig_hull_sat_hulls`
  (67th entry, blocking): the study's verified scenario oracles
  S1–S5 (pp_sat_test 59/59) — prism stack overlap/touch/separate,
  prism vs box OFF the mirror plane (unique-min regime; centred boxes
  tie the two side normals exactly and rounding would pick the
  reported axis), pyr4 vs floor box sink/touch/raise, apex-to-apex
  pyramids where P1's SIDE face beats the z axis
  (0.25/√5 @ (0,−2,1)/√5), and the 1e200 ceiling-scale pin where raw
  edge crosses would overflow — plus the engine-added cross-pin (new
  generic SAT vs 2.5.4 `mig_hull_sat_hull_box` depth parity on the
  same geometry) and the refusal battery in the ENGINE-consistent
  mapping, not study-verbatim: wrong counts (nv<4/nf<4) →
  `mig_hull_bad_count`, non-finite vertex → `mig_hull_bad_point`
  (the study folds both into `bad_mesh`), bad face index →
  `mig_hull_bad_index`, no usable axis at all → `mig_hull_bad_mesh`.
  Pure data, no solver, no flag (nothing consumes hull-vs-hull yet —
  2.6.5 spawn gates its consumption on `composite.hull`). Suite floor
  66 → 67; suite 66/66 → 67/67, quick 80/80 → 81/81 at the landing;
  ASan/UBSan spot-check of the new gate clean.

- 2026-11-12 — WP2.6 2.6.5 (this commit): hull BODIES + spawn
  consumption. `object_hull` appended to `object_type` (6th value,
  append-only ABI); per-world hull vertex/face pools beside the
  composite part pool (grow-doubling, clear = compaction only,
  cleanup frees, ceilings `mpe_max_bodies × 12` verts / `× 60` face
  indices); `physics_world_add_hull(kind, s, h, mass, position)`
  stamps density-1 `mesh_props` truth — COM-rebased pool slice,
  diagonal inertia × mass/V with off-diagonal refusal (>1e-9·max_diag
  means a non-catalog mesh slipped in), BOTH the tensor and its
  inverse (the sphere initializer's stale inverse is replaced, the
  add_composite convention), bounding circumradius in `radius` with
  `half_extensions` mirroring it, `mass_is_composed` (authored-tensor
  protect), sphere-default materials, policy windows mass
  [1e-4, 1e6] / bound [0.01, 100]. Consumption is gated, spawning is
  NOT: hull floor = support-vertex manifold (up to 4 deepest,
  slop-gated, cube-path sort; refusals count in `hull_flag_skips`),
  hull-sphere via conjugate-rotated centre + 2.5.4's
  `mig_hull_sat_hull_sphere`, hull-cube via rotated verts +
  `mig_hull_sat_hull_box`, hull-hull via B-into-A-local + 2.6.4's
  `mig_hull_sat_hull_hull`; registry canonical order (hull, other)
  (5,0)/(5,1)/(5,5), dispatcher swap feeds the mirrored pairs;
  manifold normal FROM first TOWARD second, world-position contacts
  (SAT MTD depths, single-point approximations documented — the
  manifold brick replaces them). CCD: mover-side branches only
  (`ccd_hull_support_point`, min thickness = min COM-local AABB
  extent); sweep obstacles skip hulls like cube/cylinder obstacles;
  `ccd_support_depth`/`ccd_min_thickness` deliberately get NO branch
  (zero half_extensions → sweep skipped, safe degraded, mirrors
  composite). Renderer skips hulls with a counter + one-time note;
  scene save vetoes object_hull (new `SVF06`); `touch
  prism|pyr4|pyr3|icosa` verbs refuse loudly at flag 0 with the env
  hint, fixed sizes s=h=1 (icosa a=1), mass 1 kg. One INCIDENT, caught
  by the gate on its first run (the mig_composite_stack class again):
  `boundary.c`'s world-edge safety net read a hull through its
  body-level fields — a hull is sphere-initialised (half_extensions =
  (r, r, r)), so the net saw a phantom cube and clamped every hull to
  COVERING-SPHERE REST (pyr4 at COM 1.5 instead of h/4 = 0.5), which
  also pumped angular velocity until the mirrored hull-hull normal
  came back upside-down. Fixed with exact hull extrema off the pool
  slice (axis conjugated into the body frame, both ± supports in one
  pass; defective-slice/NULL-world callers keep the legacy projection
  — the documented degraded mode). Gate `mig_hull_spawn` (68th,
  blocking): mesh-props consistency + refusal battery storing
  nothing, flag-0 ghost (both registry orders refuse + count, sphere
  rests while the ghost falls through — the net is switched OFF for
  that scenario per the f10/f11 fallen-counter precedent, since an
  exact net makes "nothing fell through" vacuous), flag-1 prism rest
  at the inradius √3/3 with the 4-corner AB-face manifold, sphere
  bounce off the pyr4 apex, hull-hull MTD 0.1 in both registry
  orders + drop-from-6 smoke. Regime-robust: the bounce scenario pins
  e = ½ in-scenario (brittle forces e = 0, light 0.10) and the base
  twins are 10 kg (heavy's ×3 gravity otherwise rams a 1 kg resting
  pyramid below its oracle window); green under all five regimes.
  Suite floor 67 → 68; suite 67/67 → 68/68, quick 81/81 → 82/82 at
  the landing; ASan spot-checks of the new gate, the boundary gate
  and the sibling hull gates clean.

- 2026-11-12 — WP2.6 2.6.5 follow-up (this commit): mgb/mfs module
  source lists gain `core/mig_hull.c`. The module makefiles embed
  engine sources and predate the hull brick, so `make -C
  ecosystem/mgb` failed with 14 undefined `mig_hull_*` references
  (every one from `physics_world.c`'s new hull paths) and the MFS FTC
  gate failed for the same reason (gated pass=4 fail=1) — the 2.6.5
  verify list covered the primary builds but missed the module
  targets. Fixed the three PARALLEL lists together —
  `ecosystem/mgb/makefile` CORE, `ecosystem/mfs/mfs_sources.mk`
  MFS_ENGINE_SRCS, `ecosystem/mfs/build_tests.sh` embedded CORE (they
  carry the "same proven list / update together" annotation;
  `core/mig_spec.c` stays test-core only) — and §0's resume ritual now
  runs the module builds so the next brick's verify list cannot miss
  them again. Post-fix: mgb link green, `make mfs_ecosystem` green,
  FTC gated pass=5 fail=0 info-diags=12.

- 2026-11-12 — WP2.6 2.6.3 (this commit): the UV ellipsoid. 12×8
  parametrization (rings j·π/8, longitudes k·π/6; 86 verts / 168
  faces = band quads 3·12·8 + two fans) transcribed from the study's
  `pp_ellipsoid_uv` at FIXED resolution; coordinates are exact
  algebraic literals — `MIG_HULL_SQRT2` (1.4142…), cos/sin(π/8) =
  √(2±√2)/2 (`MIG_HULL_COS_PI_8` = 0.92388…, `MIG_HULL_SIN_PI_8` =
  0.38268…) — honoring the no-trig-in-ENGINE-coordinates invariant
  (libm trig stays in the TEST oracles, house style since mig_mass).
  Builder: non-finite/non-positive (a, b, c) refuse
  (`mig_hull_bad_point`), full caller buffer → `bad_count` (keeps the
  points that fit, never silently grows); python-verified outward
  winding (signed V = +3.8478 unit). `mpe_hull_shape_ellipsoid`
  appended to the enum (append-only ABI, 7th value); `add_hull` maps
  (s, h) = equatorial/polar DIAMETERS → a = b = s/2, c = h/2 (bound =
  max(a, c); a general (a, b, c) spawn API is a later brick); all 7
  former icosa-sized world buffers/pool ceilings switched to
  `MIG_HULL_MAX_VERTS` / `MIG_HULL_MAX_FACE_INDICES` (pool initial
  `want`s stay icosa-sized — grow-doubling covers the ell 86/168).
  Gate `mig_hull_ellipsoid` (69th, blocking): band-sum V/I truth vs
  libm-trig closed forms (unit V = 3.84778…, I diag (1.46642,
  1.46642, 1.43331), off-diag ~1e-17); the affine-covariance law
  I(αa, αb, αc) = α⁵·I(a, b, c) exact for (2,1,1)/(1,2,3)/(0.5,0.5,2)/
  (3,2,1); refusals store nothing; flag-0 spheroid + prolate spawns
  with COM/radius stamps; SAT MTD pins where one INCIDENT — both
  original oracle expectations wrong — became the brick's
  machine-pinned invariant: for translated faceted spheres the
  min-overlap axis is a band FACE normal, not the translation axis
  (unit pair @1.9ŷ: depth 0.09486 = 0.1·n_y on the band 3–4 face-64
  normal (0.25417, 0.94858, 0.18868), the ŷ vertex lies ON that face
  so its support = n_y; @2.1ŷ separated on the SAME normal; prolate
  (1,1,2) @2.9ẑ: 0.09794 = 0.1·u_z on the north-fan face-0 normal
  (0.19482, 0.05220, 0.97945), both poles on fan faces; @3.9ẑ
  SEPARATED — the "poles at ±2 can't separate at 3.9" reading was the
  oracle's error: A is the UNIT mesh, polar supports sum to 3·w_z <
  3.9·w_z). All four verdicts machine-verified to 15 digits vs an
  independent brute force over the SAT's OWN axis set (A faces ∪ B
  faces ∪ unique edge crosses, max-abs pre-scale normalization,
  1e-12 degeneracy rule) — in-gate helpers `ell5_*`; the SAT's
  min-tracking and axis-set completeness are proved by AGREEMENT with
  that brute force, and the closed-form identities cross-check the
  oracle itself against pure geometry. Mirrored call pins axis = −n,
  depth equal. Flag-1 smoke: sphere-hull single-vertex rest in
  [0.95, 1.05] for ANY settled orientation, pair depenetration ≥
  1.95. Regime-independent (pure-data SAT) + flag-1 regime-robust;
  suite floor 68 → 69; suite 68/68 → 69/69, quick 82/82 → 83/83 at
  the landing; ASan+UBSan spot-checks of the new gate and the sibling
  hull gate clean; module builds green (mgb, mfs all, FTC pass=5
  fail=0 info-diags=12).

- 2026-11-12 — WP2.6 spawn-verb follow-up (this commit, TUI-only):
  `touch <hull> [x y z] [s h mass]` — the hull tail is parameterized
  (omitted = the 2.6.5 defaults s=h=1 m=1, so old invocations spawn
  the same bodies; icosa takes the edge and ignores h; ell takes
  equatorial/polar diameters; prism/pyr4/pyr3 base+height). s/h
  parse-checked in the verb (positive, finite); mass passes through
  to add_hull whose policy windows print the REAL refusal reason
  instead of "pool full" (pool-fullness is differentiated first).
  While smoking the verbs, a shell-world config gap surfaced: the
  shell bound its world to a SNAPSHOT of g_cfg, so `export
  composite.hull=1` flipped the registry but never reached the
  running world — the flag-1 hull path was unreachable from the
  shell, and the touch refusal's hint named `env composite.hull 1`
  which only PRINTS. Fixed: the shell world binds `&g_cfg` directly
  (the shell loads no scene, so there is nothing to isolate — the
  snapshot/stream and live-inspector paths keep their per-run copies
  for scenes that tune physics) and the hint now says `export
  composite.hull=1`. The same smoke produced a false alarm worth
  recording: an ell (s=4, h=1) spawned at y=2 appeared FROZEN. It was
  RESTING — the catalog convention places every shape's
  height/extrusion axis along Z (the 2.6.5 gate pins the lying
  prism's inradius rest), so the ell's equatorial radius is vertical
  and COM 2.0 IS the equator-ring rest height for s=4; the SAT and
  the pair handler were verified clean against an independent brute
  force first (all offsets separated, agreement exact). No engine
  change. tui-smoke 9 scenes pass; suite 69/69, quick 83/83
  unchanged.

- 2026-11-12 — WP2.6 GUI spawn-menu (this commit, GUI-only): the
  spawner menu now offers all 8 kinds and edits them in place —
  level 1 gains "5: Hull Settings" (→ level 13: 1 Mass, 2 Size (s),
  3 Height (h) → dialogs 14/15/16) and the level-8 cycle is 8-way
  (`% 8` / `+7`): sphere, cube, cylinder, prism, pyr4, pyr3, icosa,
  ell. `current_spawn_type` 0..7 is the single state word. Three
  SHARED dials serve all five hull kinds — `spawner.hull_mass`
  (default 1, [1e-4, 1e6]), `spawner.hull_size` (s: prism/pyr4/pyr3
  base width, icosa edge, ell equatorial diameter),
  `spawner.hull_height` (h: prism/pyr heights, ell polar diameter,
  ignored for icosa) — both size dials floored at 0.02, not the
  generic 0.01: the ell spawn's circumradius is s/2, and add_hull's
  window opens at 0.01, so s=0.01 would spawn a half-penny prolate
  the gate never sees. The dialogs clamp at those real windows.
  Spawn dispatch is deduplicated into `spawner_launch_current_type()`
  (both the Enter-hold and the repeat-rate site call it); new
  `spawner_launch_hull(kind, name, speed)` mirrors the TUI verb:
  composite.hull=0 refuses loudly (event-log hint names the config
  menu, not the print-only `env`), reach = max(s, h) + 1, the
  add_hull-window refusal prints the real policy, velocity stamps
  launch + viewpoint, spawner frictions, per-index colour.
  `term_spawn_type_name()` (term_obj.c) is the single source of
  truth for the 8-name label; its prototype moved from term_priv.h
  (NOT public API) to debug_terminal.h so overlay.c's spawner texts
  use it — the prior inline ternaries in overlay.c were a live
  violation of that rule, now gone. Append-only throughout: the
  config struct gained 3 floats (recompiles everything — the full
  battery was rerun), the input_state comment lists all 8 values.
  Known limits unchanged: hull spawns simulate but stay INVISIBLE
  (the 2.6.5 renderer stopgap — skip + counter + one-time note) and
  scene save still vetoes hulls (SVF06); a hull-mesh renderer brick
  is the visible follow-up. Verified: engine build clean; suite
  69/69; quick 83/83 (all 5 regimes); tui-smoke pass; make
  everything green.

- 2026-11-12 — WP2.6 2.6.6a (this commit, GUI-only): hull-spawn
  refusals made VISIBLE to GUI users. The event log is terminal-only
  (only term_admin's dmesg reads it), so a flag-0 or window-refused
  hull spawn was completely silent on screen — Enter fired, nothing
  happened, no hint anywhere (the exact "I can't spawn them" report).
  Both refusal paths in spawner_launch_hull now bump a lifetime
  counter (spawner_hull_refusal_count) that the HUD stats line shows
  as ` | HullRef:N`, plus a one-time stderr note per reason (the
  renderer's one-time note precedent) for terminal-launched sessions.

- 2026-11-12 — WP2.6 FTC-fix (this commit, one line, MFS test
  harness): mfs_drive_and_measure called physics_world_step (&w, DT)
  — the address of the POINTER PARAMETER, not the world (the
  settle_ticks helper below always had it right; every other &x call
  site in the tree addresses a world OBJECT). The step then read
  stack garbage as a physics_world: the guards sometimes passed,
  mpe_parallel_sanitize walked wild memory (the INPUT CLAMPED spam
  with denormal garbage masses — 3.3e-41 is uninitialised float), and
  the stack stomp segfaulted layout-dependently — `mfs_suite --all`
  crashed 3/3 bare runs (exit 139) while gdb ran 37/37 clean, and
  composite_ab was VACUOUSLY passing: its world never stepped, both
  robots "moved" 0 m, and equal zeros sit inside any band. Found by
  rebuilding the battery with ASan: stack-use-after-return at
  physics_world.c:2497 (the bodies guard) from mfs_t_composite_ab.
  One-character fix. composite_ab now measures REAL motion and still
  passes its bands — the A/B gate finally tests the thing it says it
  tests. FTC battery pass=5 fail=0; ASan clean (0 reports).

- 2026-11-12 — WP2.6 2.6.6 (this commit, GUI + one core append): hull
  bodies RENDER. The 2.6.5 stopgap (draw-loop skip + counter + one-time
  stderr note) is replaced by five per-kind instanced unit meshes:
  render/hull_meshing.c builds prism/pyr4/pyr3 at s=h=1, the
  icosahedron at edge 1, and the ell at semi-axes 0.5 (unit diameters)
  from the SAME mig_hull builders the collision pools use, rebased to
  the COM frame exactly like add_hull's rebase — affine covariance
  does the rest: stored verts == unit verts · (s, s, h), so a body's
  T*R*S with its stamped sizes lands the drawn surface on the
  collision surface (the ell draws its exact 12×8 collision
  polyhedron, NOT a fine sphere that would poke through the hull's
  facets). Enabler: rigidbody gained hull_kind/hull_size_s/hull_size_h
  (append-only, zeroed at all three initializer sites — the
  determinism lesson), stamped by add_hull; collision never reads
  them, the pool slices stay the truth. The draw loop packs hulls
  per-kind (icosa scales (s, s, s) — its edge is s and h is unused)
  into five new instance buffers with the standard 19-float layout;
  hulls now flow through the same frustum cull (radius IS the
  circumradius). Wireframe mode draws each catalog mesh's unique
  undirected edge set (the polyhedra are all-crease shapes so that is
  the natural edge wire; the coarse 12×8 ell draws its geodesic wire)
  instead of collapsing into the zero-half-extensions cube default.
  Gate 70 `mig_hull_render_stamp` (blocking) pins BOTH halves of the
  contract engine-side, no GL needed: (1) the affine identity — unit
  COM-frame verts · (s, s, h) == stored COM-frame verts exactly, 5
  kinds × s∈{1, 2, 3.5} × h∈{1, 0.5, 4}; (2) add_hull stamps kind +
  (s, h) on all five kinds with the slices untouched. Suite 70/70,
  quick 84/84 (5 regimes), tui-smoke pass, mgb/mfs green, FTC 5/5
  (post-FTC-fix), ASan clean on the FTC battery, make everything
  green. Scene save still vetoes hulls (SVF06 — unchanged).

## §10 Definition of done for v16S (the cut checklist)

All gates green at the final floor; policy amendment committed; release
notes skeleton committed at the cutover; POSIX+Windows build wrappers
(`run_all.sh`/`run_all.bat`) verified; 461 mirror synced green; the one
IRreversible-contract item (MPE_MODULE_ABI→2) documented in
`core/mpe_module.h` with the smaller-read rule; KNOWN_LIMITATIONS gains
the composite rows (part cap, rotation-CCD frontier unchanged, principal-
axis readout TODO); VERSION_HISTORY row written from the release-notes
numbers, not before.
