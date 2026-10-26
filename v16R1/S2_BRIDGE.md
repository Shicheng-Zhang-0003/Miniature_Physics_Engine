# MPE S2 Molecular Bridge (MGB) — v15S

> Display-only, one-way bridge from an external `s2tui` simulation into MPE
> for rendering. Positions flow S2 → MPE, never back. No physics coupling.

---

## 1. Overview

The S2 molecular bridge (`ecosystem/mgb`, MPI registry name `s2-bridge`)
spawns a live `s2tui` child, configures it through the shell S2 already has,
reads strict `S2SAVE1` frames produced by `sync`, and maps them to MPE
bodies and zero-force bond lines.

**Phase 0** stages a single snapshot — the dish renders but does not drift
under gravity or contact response (kinematic bodies, `k=0` bonds).

**Phase 1** (2026-10-15): `cfg->every > 0` is wired into `pre_step`. Each
refresh tick calls `mgb_sink_mpe_move()`, which **repositions existing
bodies in place** rather than re-spawning them. The stable S2-atom → MPE-body
mapping means atom `i` always maps to the same body; new atoms create new
bodies; removed atoms' bodies remain frozen at last position. Zero-force
bond lines (`k=0`) are updated to the current separation each refresh.

Bodies are **kinematic** (not static or dynamic): gravity and collision
impulses do not move them, and other MPE bodies treat them as immovable
collision obstacles. Refresh ticks apply S2 positions to the same mapped
bodies in place.

Physical coupling (positions flowing back to S2) remains out of scope:
**display-only, one-way, always.**

---

## 2. Architecture

Three pipes (S2's own contract):

- **Child stdin down** — commands.
- **Child stdout up** — command output (drained to a transcript file;
  content informational only).
- **Child stderr up** — banner/prompts/diagnostics (the **framing** channel).
  The `s2>` prompt is printed to stderr with `fflush`, so prompt framing
  MUST read stderr. Reading stdout for the prompt was the first bug in this
  file (fixed before first probe).

The snapshot frame travels via `sync` FILE (never via stdout) — so stdout
buffering can delay transcript text but can never corrupt a frame.

---

## 3. Module Descriptor

```c
const mpe_module_desc_t mpe_module_desc = {
    1, "s2-bridge", "0.1", "generic", false,
    bridge_attach, bridge_detach, bridge_pre_step, NULL, NULL
};
```

`deterministic = false` (display path): S2 doubles + libm + wall-clock pipes
stay outside the determinism envelope by design.

---

## 4. Attach Failure Codes

| Code | Meaning |
|---|---|
| `S2B_E_NULL` (-1) | engine called us wrong |
| `S2B_E_ALLOC` (-2) | out of memory |
| `S2B_E_NO_S2TUI` (-3) | resolved path is empty or not executable |
| `S2B_E_SPAWN` (-4) | fork/exec of s2tui failed |
| `S2B_E_SCENE` (-5) | scene name unknown |
| `S2B_E_CONFIG` (-6) | s2tui refused configure command |
| `S2B_E_MAP` (-7) | mapper allocation failed |
| `S2B_E_STAGE` (-8) | staging bodies into the world failed |
| `S2B_E_NOTHING` (-9) | no atoms parsed from the live stream |

Every failure emits a diagnostics record (`s2bridge/<code>`) with the exact
cause — no more guessing between "no such file", "no exec bit", "exec
rejected the format", and "it ran but stayed silent".

---

## 5. Path Resolution

1. `$MGB_S2TUI_BIN` — explicit override, validated with `access(X_OK)`; an
   explicit override that does not resolve is **fatal** (no fallback search —
   falling back would silently run a different binary than the one asked for).
2. Beside this `.so` image — `dladdr()` on a `static` anchor (so a same-named
   GLOBAL in another plugin cannot interpose and steer the arithmetic);
   upward search from that directory (`4179-Magi/s2tui`).
3. Upward search from CWD.

A miss stays a miss — an unvalidated guess is never installed.

---

## 6. Placement & Frustum

The stage offset targets the **frame's bounding-box centre**, not raw S2
coordinates. Default `(0, 20, -60)` puts the dish centre on the default
camera axis (`(0,20,50)`, looking down −Z, 45° vertical FOV). Override with
`$MGB_OX` / `$MGB_OY` / `$MGB_OZ`.

The renderer frustum-culls silently. If the screen looks empty, read
`STAGED_BOUNDS` (logged on attach) with the real centre, extent, and radius
range first.

> Old bug: offset `(0, 120, 0)` put the dish 100 units ABOVE the camera;
> every staged atom was 53–76° off-axis against a 22.5° half-FOV, so the
> renderer culled all 776 atoms. Attach reported success and the screen
> stayed empty.

---

## 7. Units

- **In:** Å (positions), e (charge), eV (LJ), fs (velocities, unused).
- **Out:** millimetres of display space (1 Å → `MGB_MM_PER_A` mm, default
  1.0). The scale is a display transform, not physics. It is labeled
  everywhere the mapped data lands and inverts exactly (double arithmetic,
  one rounding at the sink boundary if the sink is float).

`mgb_unmap(v_mm, mm_per_a)` inverts exactly.

---

## 8. Bonds

Zero-force spring joints (`k = 0, c = 0`, `L0` = current length). Hooke and
damping are both exactly zero, so the solver is untouched AND the renderer
still draws the magenta joint lines. Never physical springs.

---

## 9. Debug Terminal

`s2_bridge_stats()` returns frame and error counts. Phase 0 returns −1 (keeps
callers honest until the graphical phase lands). See `v16R1/REMAINING_WORK.md`
for the TUI health surface follow-up.

---

## 10. Building

```bash
cd v16R1/src/ecosystem/mgb
make              # builds mgb_bridge.so + mgb_mpe_test
make -C ecosystem/mgb   # from v16R1/src
```

`mgb_mpe_test` gates the real sink against a recorded frame (golden section).
Not part of any runner profile — run manually.
