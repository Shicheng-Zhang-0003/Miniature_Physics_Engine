# MPE Configuration System — v16R1

> 86 tunables in 15 categories, live menu (key `6`, two pages, 13 categories
> reachable), terminal `env`/`export`, round-trip save/load to
> `status/engine.cfg`, corrupt-safe load.

---

## 1. Overview

The configuration system is the single source of truth for all tunable
parameters. `config/mpe_config.h` defines `mpe_config_t`; `config/
mpe_config_schema.c` declares the registry table (`g_registry`, one line per
tunable). Hot-path code reads `g_cfg.<group>.<field>` directly; cold-path
code (menu, terminal, save/load) iterates the registry.

Categories:

| Category | Fields |
|---|---|
| `world` | gravity, drag, floor_friction_s/k, rolling_resistance_coeff, angular_damping_scale |
| `timestep` | solver_iterations, max_substeps, max_linear_speed, max_angular_speed |
| `sleep` | linear_thresh_sq, angular_thresh_sq, timer_duration, wake_linear_thresh_sq, wake_angular_thresh_sq, enable |
| `solver` | penetration_slop, bias_factor, max_separation_bias, restitution_velocity_thresh, static_friction_thresh, warm_start_match_dist_sq, warm_start_damping |
| `depenetration` | correction_factor, max_correction, wake_depth_thresh, rebuild_iterations |
| `broadphase` | cell_size_default/min/max/multiplier, max_cell_span_per_axis |
| `joints` | max_acceleration, default_spring_k/damping, soft_spring_k/damping, revolute_beta/max_bias/motor_gain |
| `boundary` | floor_emergency_slop, safety_net_enabled |
| `spawner` | mass, radius, cube_mass/extent, cyl_mass/radius/half_length, hull_mass/size/height (WP2.6: the three shared hull dials, all five catalog kinds), speed, friction_s/k, overlap_max_attempts/thresh |
| `body_defaults` | sphere/cube/cylinder restitution, friction_s/k |
| `camera` | move_speed, mouse/steer_sensitivity, horizontal_friction, jump_height, ijkl_speed |
| `render` | light_x/y/z, ambient_strength, specular_coeff/exponent |
| `ui` | change_rate_game/debug, long_run_ticks, enter_spawn_delay/interval |
| `composite` | max_parts (per-composite part ceiling, 1..16), hull (WP2.6 hull-consumption master flag, 0/1) |
| `mfs` | robot.composite (WP5: build the FTC robot as one composite body, 0/1) |

Counts per category (registry truth, `grep -o cat_ config/mpe_config_schema.c`):
world 6, timestep 4, sleep 6, solver 7, depenetration 4, broadphase 5,
joints 8, boundary 2, spawner 15, body_defaults 9, camera 6, render 6,
ui 5, composite 2, mfs 1 = **86**.

### GUI menu reachability (honest note)

The key-`6` menu exposes **13 of the 15 categories** across two pages (both
pages' digit keys are routed; `6` is the close toggle):

- Page 1: `1` World, `2` Timestep, `3` Sleep, `4` Solver, `5` Depenetration,
  `7` Broadphase, `8` Joints, `9` Boundary, `0` More.
- Page 2: `1` Spawner, `2` Body Defaults, `3` Camera, `4` Render, `5` UI,
  `7` Save, `8` Reset Defaults, `9`/`0` Back.

`composite` and `mfs` have no menu page. Set them from the TUI shell
(`export composite.hull=1`, `export composite.max_parts=16` — the
snapshot/stream shells keep per-run config copies, but `--shell` binds the
global registry so `export` reaches the live world) or via
`status/engine.cfg` + `config load`. The one exception shape:
`mfs robot.composite`'s key contains a space, so neither terminal's
`export KEY=value` can address it (both tokenize on whitespace) — set it in
`engine.cfg` and `config load`, or from C (`mpe_config_set_int`). In the GUI,
`composite.hull` is nevertheless reachable **without** the 6 menu: the
Spawner Menu → `5: Hull Settings` screen carries a `4: Hull Contacts` toggle
(WP2.6 2.6.6b) that flips `composite.hull` in place.

---

## 2. Critical Contract: Init Before Build

`mpe_config_init()` must run **before any body is built**. Body materials
are stamped from `g_cfg` at construction time and are **never retro-fitted**.
Building a body while `g_cfg` is still zero yields a body with zero friction
and zero restitution that can never be repaired — this was the
`SCENEORDER-2026-09-29` bug (the old suite ran with solver iterations = 0
and was 34/34 green).

Fixed structurally: `mpe_config_ensure_ready()` is called from every
`rigidbody_initialisation_*()` (the choke point) and from
`scene_init_default()`, so caller order no longer matters. Gated by
`mpe_t_body_materials_live`, which forces the unready precondition and fails
4 assertions with the fix removed.

---

## 3. API

```c
void mpe_config_init (void);                       /* call once at startup */
void mpe_config_reset_defaults (void);
bool mpe_config_get_float (const char *key, float *out);
bool mpe_config_get_int (const char *key, int *out);
bool mpe_config_get_bool (const char *key, bool *out);
bool mpe_config_set_float (const char *key, float value);   /* clamps to [min,max] */
bool mpe_config_set_int (const char *key, int value);
bool mpe_config_set_bool (const char *key, bool value);
bool mpe_config_load (const char *path);           /* INI format */
bool mpe_config_save (const char *path);
bool mpe_config_save_guarded (const char *path);   /* refuses to publish F11 torture values */
bool mpe_config_save_backup / load_backup (const char *path);
bool mpe_config_is_ready (void);
void mpe_config_ensure_ready (void);               /* call before building any body */
```

Setters return false on missing key, type mismatch, non-finite float, or a
value that required clamping. Extreme values are clamped to registered
bounds.

---

## 4. Save / Load

- Config saves to `status/engine.cfg` on clean exit (guarded save).
- Config loads on startup and overrides defaults.
- Corrupt or missing config file does not crash the engine.
- Missing keys keep defaults; unknown keys ignored.
- Text INI format, grouped by category.
- F11 torture guardrails: gravity −17…−1, solver_iterations ≥ 96 (proven
  envelope for the 10:1 validation column); material/world extremes stay
  fully random. `mpe_config_save_guarded()` refuses to publish torture
  values as the next boot's defaults.

---

## 5. Debug-Only Parameters

Debug-only parameters are refused in Game Mode. `angular_damping_scale`
(default 1.0 = truth vacuum) is debug-only: 1.0 means no extra rotary
damping; values < 1 add game-only rotary damping with no fluid basis.

---

## 6. Registry Entry

Each tunable is declared exactly once in `mpe_config_schema.c`:

```c
{"world.gravity", "World Gravity", "Gravitational acceleration (m/s^2, negative = down)",
 p_float, cat_world, &g_cfg.world.gravity, -9.81, -50.0, 0.0, false},
/* key, display, help, type, category, storage, def, min, max, debug_only */
```

Adding a new tunable = add one line here.
