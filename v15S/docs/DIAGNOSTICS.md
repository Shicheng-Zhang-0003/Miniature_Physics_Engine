# Engine Diagnostics & the S2 Bridge

How the engine reports its own failures, and how to read them.

This document exists because the previous failure mode was not a wrong
answer, it was an *unnameable* one: every rejection collapsed into a bare
`-1`, so the terminal printed a **guess** (`"attach failed (table full /
attach hook)"`) about a subsystem the operator could not see. Localising a
single live failure took days. Every error below is named, coded, and
traced to the line that raised it.

---

## 1. The self-report

`core/mpe_diag.h` / `core/mpe_diag.c`. One call site, four outputs.

```c
MPE_DIAG_ERROR ("attach", "E_HOOK", "attach rejected: '%s' returned %d", name, rc);
MPE_DIAG_WARN  ("loader", "E_BUSY", "'%s' is still referenced", path);
MPE_DIAG_INFO  ("attach", "OK", "attached '%s' at slot %d", name, slot);
```

Each record carries four things, and all four are load-bearing:

| Field | Why it exists |
|---|---|
| `source` | which subsystem raised it (`attach`, `detach`, `loader`, `s2bridge`, …) |
| `code` | stable, greppable, safe to match in scripts |
| `site` | engine `file:line` that decided it |
| runtime values | the numbers that caused it (paths, counts, errnos) |

Every record is fanned out **in one locked pass** to:

1. **stderr**, flushed, so it appears before the prompt returns;
2. **the event log** (overlay ring, as `log_error` / `log_warn` / `log_info`);
3. **the debug terminal**, live, when it is open (see §2);
4. the **diagnostics ring** — 128 records, plus per-source totals.

Warnings go to stdout and errors/fatals to stderr, so a harness that treats
any stderr output as failure is not blinded by chatter.

`core/` never links `ui_input/`. The debug terminal registers a *sink
callback* (`mpe_diag_set_sink`), which is why a plugin can report into the
engine's ring without a UI dependency.

### Coalescing
A fault that repeats every frame must not drown the log, and must never be
swallowed. Identical consecutive records are coalesced and re-printed when
the count hits a power of two, then every 64th. So "it happened twice" and
"it is still happening 4096 frames later" are both visible.

### Threading
Any thread may emit (a physics worker can). The ring and the per-source
totals are mutex-guarded under the same contract as `event_log`: never
hot-path, always serialised.

---

## 2. Reading it: the `diag` command

Available in the debug terminal. Open with F12 (or the terminal menu).

| Command | Effect |
|---|---|
| `diag` | per-source totals, then the last 25 records |
| `diag tail 50` | last 50 records only |
| `diag sources` | per-source error/warn totals only |
| `diag on` / `diag off` | live mirror of new records (ON when the terminal opens) |
| `diag clear` | forget every record, totals included |
| `diag status` | mirror state, ring occupancy, source count |

With the terminal open, mirroring is on by default and every new engine
error appears there as it happens — you never have to go looking for it.

The same text is available without the GUI: `core/mpe_diag.c` exposes
`mpe_diag_render (char *out, size_t cap, int max_records)`.

---

## 3. Error codes

### Attach — `physics_world_attach_module` (`MPE_ATTACH_E_*`)
All codes are `< 0`, so every pre-existing `>= 0` / `< 0` caller is
unaffected. Only the ability to *name* the cause is new.

| Code | Macro | Meaning |
|---|---|---|
| −1 | `MPE_ATTACH_E_NULL` | null world or descriptor |
| −2 | `MPE_ATTACH_E_ABI` | descriptor built against another ABI — **rebuild the plugin against this tree** |
| −3 | `MPE_ATTACH_E_COUNT` | world's tick-module count is corrupt; re-init the world |
| −4 | `MPE_ATTACH_E_TABLE` | all `MPE_MAX_TICK_MODULES` (16) slots in use — the record lists the occupants |
| −5 | `MPE_ATTACH_E_HOOK` | the module's own `attach` refused; the hook's own code is passed through verbatim |
| −6 | `MPE_ATTACH_E_NAME` | module name is NULL/empty |

`physics_world_attach_strerror (code)` returns exact text. Never widen a
code into a range — the UI switches on these values verbatim.

### Detach — `physics_world_detach_module`
`0` ok, `MPE_DETACH_E_NULL` (−1), `MPE_DETACH_E_NOTFOUND` (−2). A miss
records what *is* attached, because "not found" and "attached under a
different name" are different problems.

### Loader — `mpe_loader_load` / `mpe_loader_unload`
Every rejection is reported through one helper, so the text returned to the
caller and the text in the ring are generated from the same call and cannot
drift apart.

| Code | Meaning |
|---|---|
| `E_EMPTY` | empty path |
| `E_NOFILE` | no such file (the common case: engine not run from `v15S/src`) |
| `E_JAIL` | resolves **outside** `ecosystem/` — modular artifacts must live under `ecosystem/<member>/…` |
| `E_PATHLONG` | resolved path exceeds what the registry's origin buffer stores |
| `E_STALE` | already loaded and the file **changed on disk**: stale code is still running. Unload, then load again |
| `E_STALE_GONE` | already loaded but the file vanished — restart the engine |
| `E_TABLE` | handle table full |
| `E_DLOPEN` | `dlopen` failed (includes the `dlerror` text) |
| `E_ECO_ABI`, `E_ECO_REG` | ecosystem descriptor ABI/name mismatch, or registry full/duplicate |
| `E_ABI`, `E_REG` | module descriptor ABI/name mismatch, or registry full/duplicate |
| `E_NODESC` | exports neither `mpe_ecosystem_desc` nor `mpe_module_desc` |
| `E_BUSY` | unload refused: a live world still calls into this image |
| `E_LONGNAME` | unload refused: registered name will not fit; refuses rather than half-tear-down |
| `E_NOTFOUND` | unload: no loaded image matches that path or name |

---

## 4. The S2 bridge (`s2-bridge`)

> **Platform: POSIX (Linux) only.** The bridge spawns a child with
> `fork`/`exec` and drives it over `pipe`s; it has no MPE_WINDOWS branch and
> is not built on Windows. The engine's own diagnostics (§1–§3) are portable;
> this section is not.

Display-only, one-way: S2 `s2tui` is the simulation, MPE renders it.
Positions flow S2 → MPE and nothing is ever written back.

```bash
# from v15S/src
./engine
# in the debug terminal:
mod load ecosystem/mgb/build/mgb_bridge.so
mod attach s2-bridge
diag                 # if anything above failed, this says exactly what
```

`modinfo s2-bridge` shows the descriptor, hooks, and whether it is loaded.

### Configuration (environment)

| Variable | Meaning |
|---|---|
| `MGB_S2TUI_BIN` | absolute path to `s2tui`. **If set and not executable, attach fails loudly** — it never silently falls back to a different binary, because an override exists precisely to pin the executable |
| `MGB_OX`, `MGB_OY`, `MGB_OZ` | stage offset (default `0 20 -60`), overridable live without a rebuild |

`mm_per_a` (1 Å → 1 unit) is compile-time (`MGB_MM_PER_A`).

### How `s2tui` is located
1. `$MGB_S2TUI_BIN`, validated with `access(X_OK)`;
2. a bounded upward search (≤14 levels) from **this `.so`'s own directory**,
   found via `dladdr` on a *static* anchor — a static symbol cannot be
   interposed by another plugin exporting a same-named global;
3. the same upward search from the working directory.

Every candidate is validated *and normalised* with `realpath`, so a hit is
always an absolute, canonical, existing executable. A miss stays a miss —
the resolver never installs an unvalidated guess.

> Historical trap, kept as a gate: an earlier implementation stripped a
> *fixed* number of path components, which assumed one exact install depth.
> Reached through a symlinked working directory it emitted
> `<root>/4179-MPE/../4179-Magi/.../s2tui`, which resolves **only while that
> symlink exists** — otherwise the kernel returns `ENOENT` on the symlink
> before it can even apply `..`. The plausible-looking path was the bug.
> The gate now pins: a hit is an existing executable, is absolute, is
> canonical, and contains no unresolved `..`.

### Spawn failures are named, never guessed
`mgb_spawn` reports a `mgb_spawn_status_t` plus the OS errno. Retrieve it
with `mgb_spawn_reason (int *why_errno)`.

| Status | Meaning |
|---|---|
| `mgb_spawn_e_no_path` | empty path |
| `mgb_spawn_e_notfound` | no such file (`ENOENT`) |
| `mgb_spawn_e_noexecbit` | exists but not executable (`EACCES`) |
| `mgb_spawn_e_noexec` | `exec` itself failed (`ENOEXEC`, …) |
| `mgb_spawn_e_pipe` / `_e_fork` / `_e_alloc` | resource failure |
| `mgb_spawn_e_handshake` | ran, but never sent the expected banner/prompt |

`exec` failures are captured through a `CLOEXEC` status pipe read with
`poll()`. A non-blocking read races the child and turns every exec failure
into a bogus "handshake timeout" — the exact misdiagnosis this removes.

### Staging placement — why "attached" can still be invisible
The renderer frustum-culls **silently**. The engine's default camera sits
at `(0,20,50)`, yaw −90° (forward `0,0,−1`), pitch 0, through a **45°
vertical FOV** (22.5° half-angle), far plane 1000. Anything further than
22.5° off that axis is dropped with no warning.

So attach now emits `STAGED_BOUNDS` on success:

```
staged 776 body/bodies in slots 0..775 | centre (0.00, 20.00, -60.00)
  | extent 43.02 x 42.11 x 41.41 | radius 0.320..1.380
```

and `STAGED_BOUNDS` as a **warning** if a successful attach staged zero
bodies. Two placement rules follow from this, both enforced by gates:

- the offset must be **on the view axis** (default `0,20,-60`), not merely
  out of the way of the default scene;
- the offset targets the frame's **bounding-box centre**, not raw S2
  coordinates — adding it to raw coordinates is only meaningful for a scene
  that happens to sit at the origin, and the petri dish does not.

If the screen is empty, read `STAGED_BOUNDS` and compare the centre with
the camera axis before touching anything else.

### Known limitation: static by construction
The dish is **staged once and does not move or vibrate.** This is Phase 0
by design, not a bug:

- bodies are created static, so the solver never moves them;
- bonds are staged with `k = 0, c = 0` — force-free, rendered as lines only;
- `cfg->every == 0` (stage-once; the follow-mode cadence is not wired).

Until follow mode lands, treat the bridge as a live *snapshot* viewer. See
`v15S/REMAINING_WORK.md`.

---

## 5. Building and testing

```bash
# 475 (engine, both suites, headless, all 44 build_* test binaries, capsule)
make clean
make -j$(nproc) engine mpe-tui build_suite build_suite_scalar headless \
     ecosystem/capsule/mpe_capsule.so \
     ecosystem/mfs/mfs_ecosystem.so ecosystem/mfs/plugins/mpe_ftc.so \
     $(grep -oE '^build_[a-z0-9_]+:' makefile | sed 's/build_//; s/://' \
        | grep -v '^suite$' | sed 's/^/build_/' | tr '\n' ' ')

make -C ecosystem/mgb            # bridge .so + its own gates
./test_mpe_suite                 # 43 blocking
./test_mpe_suite_scalar          # 43 blocking, SIMD disabled
python3 ../../tools/test_runner.py --profile quick
```

Gates that pin the behaviour described above:

| Gate | Pins |
|---|---|
| `diag_naming` (canonical suite) | distinct codes per cause; source + call site present; strerror is specific; **an attach hook with side effects is invoked exactly once** |
| `mgb_mpe_test` — spawn tier | each spawn failure mode is named with its own status and errno; strerror never collapses |
| `mgb_mpe_test` — resolver tier | a hit is an existing executable, absolute, canonical, `..`-free; a miss is empty; a bogus override is refused rather than bypassed |

The "hook runs exactly once" gate is not theoretical: an earlier revision of
the error reporting called the hook a *second* time to format its log
message, which would spawn `s2tui` twice.