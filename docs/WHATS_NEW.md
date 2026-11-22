# What's New — v16R1 (active head)

`v16R1` is the active development head opened from the v15S release. It keeps
the GTK4 modular kernel and adds the MIG composite/hull/void train and CAD
editor model expansion.

## Current head highlights

- **MIG composites (WP1–WP2):** composed rigid bodies with per-part mass
  properties, pool-backed part storage, broadphase/narrowphase part dispatch,
  boundary/CCD/composite stack support, and deterministic composite gates.
- **MIG persistence (WP3):** scene v201 composite records and MIG spec grammar
  round-trip; scene v202 adds per-part `is_void` for void-bearing composites.
- **CAD editor model (WP4/WP8 first slice):** headless design model in
  `v16R1/src/cad/` with typed adds, transforms, snap/rotate, duplicate/array,
  subtract void verbs, spec persistence, build seam to `mig_make_bridge` /
  `mig_void_make_bridge`, plan view, multi-level undo/redo, groups, mirror,
  align/distribute, copy/paste, per-part color/name, and measurement.
- **CAD editor model menus (first slice, model/API):** numeric page menus on
  keys 7/8/9 (engine derived): 7 = selection/placement (next/prev/clear/
  place@0/move G/J/L/delete/duplicate/close); 8 = add/place (sphere/cube/
  cylinder adds, mirror, align min/center/max, distribute); 9 = group/
  clipboard (undo/redo, group/ungroup/members, copy/paste, measure, mass).
  GUI/TUI wiring follows.
- **Hulls (WP2.5/2.6):** convex hull point collection, 2D proof hull, mass
  properties, SAT pairs/hull-hull, shape catalog, spawn/render stamp, analytic
  ellipsoid path, ellipsoid spawner default, rotor spin stability.
- **Voids (WP7 complete):** sealed cavity subtraction (7.0a), engine
  consumption (7.0b), CAD void tool (7.0c), scene/spec persistence (7.0d),
  and quality lifts (7.0e: hull ring pieces, part cap 16→64).

## Verification snapshot (2026-11-21, WP8 first slice)

- Quick profile: 95/95 green.
- Canonical C suite: 81 total, 79 blocking green, 2 diag-informational.
- MFS: 37-gate suite, gated pass=5 fail=0, 12 info-diags.
- MGB: PASS 44 FAIL 0.
- TUI smoke: PASS.
- `make everything` audit: not run in this slice.

## Foundation inherited from v15S

- **GTK4 port** — event controllers, gestures, overlay, dialogs; GTK4 is the
  only supported toolkit.
- **Module system (MPI)** — stable C ABI (`core/mpe_module.h`): shape-pair
  handlers, broadphase, solver, per-world tick modules; hot-load `.so` plugins.
- **Per-world configuration** — hot paths take world config; no kernel global
  simulation state.
- **Multicore solver** — `MPE_THREADS` 1–8; island-based parallelism with
  bitwise-identical determinism twins at 1/4/8.
- **Data structures** — growable pools, O(1) contact-pair/id lookups, small-
  first broadphase nodes, process-wide determinism counters.
- **Diagnostics** — `mpe_diag` names failures with subsystem/code/source/values.
- **Ecosystem** — MFS robotics and MGB/S2 bridge live under `v16R1/src/ecosystem/`.

For detailed MIG history, see `v16R1/docs/MIG_BUILD_PLAN.md` §9 ledger.
