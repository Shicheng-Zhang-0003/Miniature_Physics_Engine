# 472-MGB in 475 — integration notes (this copy)

Copied verbatim from `4179/.../v9R4/472-MGB` (core: `include/`, `src/mgb_link.c`,
`src/mgb_frame.c`, `src/mgb_map.c`, `src/mgb_sink_file.c`, `tools/`, `tests/test_mgb.c`,
`scenes/`, `readme.md`, `PROTOCOL.md`), then programmed in:

- `src/mgb_sink_mpe.c` — the real sink (static spheres, zero-force spring
  joints for bond lines, stage offset, double→float once). Replaces the stub
  conceptually; the stub file is NOT copied (contract now implemented).
- `modules/s2/s2_bridge.{h,c}` — MPI `s2-bridge` descriptor (generic,
  `deterministic=false`): per-world session, spawn/handshake/configure/stage
  on attach, cadence refresh on pre_step, freeze + loud on link loss.
- `tests/golden_water.s2` + `tests/mgb_mpe_test.c` — headless tiers A (golden,
  no s2tui) and B (live petri via `MGB_S2TUI_BIN` or the sibling default,
  skipped gracefully when absent).
- `tests/mgb_load_test.c` — headless dlopen ABI check of `mgb_bridge.so`.
- `Makefile` — standalone from the engine build; `make test` runs headless
  tiers, `make loadtest` checks the .so descriptor.

Display contract: mapped mm used numerically as MPE metres ("1 unit = 1 mm
S2-display"), static bodies (solver-skipped) + k=0/c=0 joints (force-free,
rendered).

### Staging placement (revised 2026-10-09)

The stage default is **`(0,20,-60)`**, not `(0,120,0)`. Two reasons, both
learned the hard way:

- **The renderer frustum-culls silently.** The engine's default camera sits
  at `(0,20,50)`, yaw −90° (forward `0,0,-1`), pitch 0, through a 45°
  vertical FOV (22.5° half-angle). The old offset put the dish ~100 units
  *above* that axis — every atom 53–76° off-axis — so all 776 were dropped
  with no warning: attach reported success and the screen stayed empty.
- **The offset targets the frame's bounding-box centre**, not raw S2
  coordinates. Adding it to raw coordinates only means anything for a scene
  sitting at the origin, and the petri dish is not: with the offset added raw
  its centre still sat 21° off-axis and only 2 of the 8 bounding-box corners
  were on screen.

Attach now logs `STAGED_BOUNDS` with the real centre, extent and radius
range, so "staged but invisible" is never silent again. Override the offset
live with `$MGB_OX` / `$MGB_OY` / `$MGB_OZ`.

### Status

- **Live in the engine.** `mod load ecosystem/mgb/build/mgb_bridge.so` then
  `mod attach s2-bridge`; owner-confirmed rendering the staged dish in the GTK
  view on 2026-10-09.
- **Stage-once.** The dish renders but does **not move or vibrate**: bodies
  are static, bonds are zero-force, and the refresh cadence
  (`cfg->every`, default 0) is not wired into `pre_step`. The current
  `bridge_stage` *adds* bodies rather than moving existing ones, so enabling a
  cadence naively would duplicate the dish every tick — follow mode needs a
  move-don't-re-add refresh path. Tracked in `v15S/REMAINING_WORK.md`.
- `s2_bridge_stats` still returns −1 for frame/error counts; the TUI health
  surface is not wired.

### s2tui path resolution (revised 2026-10-09)

`$MGB_S2TUI_BIN` wins, and if it is set but **not executable the attach fails
loudly** — it never silently falls back, because an override exists precisely
to pin the executable. Otherwise a bounded upward search (≤14 levels) runs from
the module's own directory (`dladdr` on a *static* anchor, so a same-named
global in another plugin cannot interpose and steer it), then from the
working directory. Every candidate is checked with `access(X_OK)` and
normalised with `realpath`, so a hit is always absolute, canonical and
existing; a miss stays a miss.

> This replaced a fixed "strip 7 path components" scheme, which assumed one
> exact install depth. Reached through a symlinked working directory it emitted
> `<root>/4179-MPE/../4179-Magi/.../s2tui`, which resolves only while that
> symlink exists — otherwise the kernel returns `ENOENT` before it can even
> apply `..`, and the operator is shown "no such file" for a plausible-looking
> path. The plausible-looking path was the bug; the resolver tier now gates
> that a hit is an existing executable, absolute, canonical and `..`-free.

### Gates

`make test` → `mgb-mpe: PASS 43 FAIL 0`. Tiers: parser/mapper policy, static
+ zero-force staging, live petri through a real `s2tui`, **spawn-failure
naming** (each mode distinct, with its errno), and **resolver** (hit/miss
contract, override honoured and normalised). `make loadtest` checks the
exported `mpe_module_desc`.
