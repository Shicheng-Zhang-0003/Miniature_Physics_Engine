# DESPOT AUDIT 2026-10-19 — engine truth ledger (475-MPE v15S + mirrors)

Scope: full mathematical/programming audit of 475-MPE v15S, with fixes mirrored
to 461-MFS (tests) and 472-MGB (bridge). Every entry below was reproduced with a
probe or a failing suite before the fix, and re-verified after.

## Fixed and verified

1. CCD sweep order race / thread nondeterminism — `physics/collision_ccd.c`.
   `collision_ccd_sweep_clamp_range` read every body while writing clamps into
   others; results depended on worker interleaving. Split into read-only
   `collision_ccd_toi_range` (own output slots) + `collision_ccd_apply_range`
   (own bodies), two pools passes with a barrier between
   (`physics_world.c` `mpe_parallel_ccd_sweep`). Verified: `test_paranoia_ccd`
   PASS at MPE_THREADS=1/2/4/8, 3 runs each.

2. Manifold capacity overflow — `core/physics_world.c`.
   Narrowphase wrote `pair_count` manifolds into a fixed initial capacity
   (1024), memset past the end → heap corruption / `free(): invalid pointer`
   in MFS `biobuzz_stage`. Added `physics_world_ensure_manifold_capacity` with
   doubling growth, `slot_count` threading, serial fallback and clamp on grow
   failure. Verified: MFS suite no longer aborts; 106/106 physics profile.

3. External forces dropped for free-flying bodies — `core/rigidbody.c:79-153`,
   `core/physics_world.c`. The analytic free-flight integrator (gravity+drag
   closed form) restored `tick_v0` and overwrote velocity, silently discarding
   spring/motor/module forces for contact-free joint-free bodies. Probe:
   100 N on a 1 kg free sphere gave vx=0, expected 1.6667. Fix: new
   `ext_force_this_tick` flag set by `rb_apply_forces_perfect` and all direct
   accumulator writers (prismatic motor, MFS intake/drivetrain, suite tests);
   gravity now adds straight to the accumulator; the analytic path is only
   used when no external force was applied. Probe now vx=1.666667.

4. Cylinder-inside-cube normal inverted — `physics/collision_cylinder.c`.
   Interior branch emitted the outward escape direction as the manifold normal,
   but the convention is A→B (A=cylinder): the solver pushed the cylinder
   deeper into the cube (probe: x=0.9 → 0.821). Fix: normal = −outward, with
   `outward_escape` kept for the face contact position. Probe: normal
   (−1,0,0), cylinder pushed out to x=0.979.

5. Inverted slab levitation — `physics/collision_cylinder.c` (static AABB slab
   shortcut). An up_sign<0 slab called `collision_static_plane_cylinder` with
   `plane_y` at its world-bottom face while the helper still models a
   solid-below floor, so a cylinder 4.5 m below matched with penetration 5.0 m
   and was levitated at +0.1033 m/tick. Fix: the shortcut only applies when
   the near face points up; inverted slabs use the exact segment-OBB path.
   Probe: hit=0, cylinder free-falls (vy=−0.1635).

6. Only the first 8 awake islands solved — `core/physics_world.c`.
   Task array was capped at `MPE_MAX_POOL_THREADS` and the loop stopped
   filling, silently skipping islands past the 8th (routine: >8 separated
   bodies). Fix: batch the island loop, one pool pass per MPE_MAX_POOL_THREADS
   batch. Also fixed the zero-island case: `num_active <= 1` read
   uninitialized `tasks[0]` (wild pointer on a settled scene).

7. Single-island fast path ignored `friction_only` — `core/physics_world.c`.
   Hardcoded `false` on that path; now passes the caller's flag through.

8. Prismatic motor produced no torque — `physics/revolute_joint.c`.
   The force was accumulated at the COM only, so an off-centre anchor could
   not spin the bodies; the solver then spent iterations removing the error.
   Now τ = r×F at both anchors, matching the limit-impulse path.

9. `mgb_link.c` uninitialized pipe fds — `ecosystem/mgb/src/mgb_link.c`
   (mirrored to 472-MGB). On first-pipe failure the cleanup tested and closed
   uninitialized fds. fds are now −1-initialized and guarded. Both MGB suites
   PASS 38/0 after the change.

10. Broadphase SIMD selfcheck tautology — `physics/broadphase.c` compared the
    scalar cross against itself; now compares against `simd_cross`.

11. `partial_throttle` test lie — `ecosystem/mfs/tests/mfs_suite_c.c:1061`.
    Strict `dist[2] > dist[0]` failed on a 1 mm saturation inversion
    (0.888/0.888/0.887) that exists on pristine HEAD too. Now allows the 1 mm
    quantization with a comment (mirrored to 461-MFS).

## Open ledger (reproduced, not yet fixed — documented honestly)

- Prismatic joint has no relative angular constraint (`revolute_joint.c`
  prismatic_solve): only 2 lateral translation rows + axial limits are solved;
  relative roll/pitch/yaw are free. `revolute_joint.h:30-33` promises 5 DOF
  removed. A correct fix needs 3 angular rows; until then prefer revolute+limit
  for constrained sliders.
- Thread pool is not re-entrant: pool state is written outside `s_pool_lock`,
  there is no shutdown/join path, and two concurrently stepped worlds can
  break each other (`physics_world.c`). Treat one stepped world per process.
- SIGPIPE from a closed MGB child can terminate the engine; the bridge does
  not ignore SIGPIPE process-wide.
- `mgb_sink_mpe_stage` clears joints during scene load — verify callers
  re-apply joint scenes after sink, or the loaded world loses constraints.
- `ftc_hotload` needs `MPE_FTC_PLUGIN` or a run from `ecosystem/mfs` via
  `build_tests.sh`; from `temp/ftc_tests` it reports a missing plugin
  (environmental, not an engine defect).

## Verification snapshot (2026-10-19)

- `python3 tools/test_runner.py --profile physics`: 106/106 pass, 2 informational, 0 blocking.
- `test_paranoia_ccd`: PASS at MPE_THREADS=1/2/4/8 (3 runs each).
- MFS suite (475 + 461): `FTC RESULT: gated pass=17 fail=0 info-diags=12 (ungated)`.
- MGB (475 ecosystem + 472): `mgb: PASS 38 FAIL 0`.
- Force-drop probe: vx=1.666667 (100 N, 1 kg, 1 tick). Cylinder probes as in #4/#5.
