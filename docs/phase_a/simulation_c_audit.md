> ⚠️ **STALE DOCUMENT** — This audit was taken before the increment split.
> `simulation.c` has since been reduced from 1123 lines to ~130 lines.
> All 16 functions listed here have been extracted into 9 separate modules.
> This document is retained for historical reference only.

---

# simulation.c Responsibility Map (Phase A audit)

- Total lines: **1123**
- Functions detected: **16** (1060 lines inside functions)

| Function | Lines | Start | End | Globals | Risk |
|---|---:|---:|---:|---:|---:|
| `physics_halt_set` | 6 | 18 | 23 | 0 | LOW |
| `physics_halt_for_ticks` | 7 | 25 | 31 | 0 | LOW |
| `physics_is_halted` | 3 | 33 | 35 | 0 | LOW |
| `on_entry_insert_text` | 13 | 48 | 60 | 0 | LOW |
| `open_numerical_input_dialog` | 46 | 61 | 106 | 2 | MED |
| `editor_reset` | 21 | 108 | 128 | 1 | MED |
| `validation_report_print` | 42 | 130 | 171 | 9 | HIGH |
| `a3_task13_body_is_invalid` | 23 | 191 | 213 | 0 | LOW |
| `long_run_validation_report` | 39 | 215 | 253 | 5 | HIGH |
| `long_run_validation_evaluate` | 59 | 255 | 313 | 4 | HIGH |
| `long_run_validation_tick_update` | 16 | 315 | 330 | 1 | MED |
| `long_run_validation_start` | 39 | 332 | 370 | 3 | MED |
| `a3_depenetration_dispatch` | 22 | 374 | 395 | 0 | LOW |
| `a3_positional_depenetrate_manifold` | 99 | 397 | 495 | 1 | MED |
| `a3_positional_depenetration_pass` | 51 | 497 | 547 | 3 | MED |
| `physics_step_increment` | 574 | 550 | 1123 | 12 | HIGH |

## Lowest-risk extraction candidates (0-2 globals, <200 lines)
- `physics_is_halted` — 3 lines, globals: —
- `physics_halt_set` — 6 lines, globals: —
- `physics_halt_for_ticks` — 7 lines, globals: —
- `on_entry_insert_text` — 13 lines, globals: —
- `a3_depenetration_dispatch` — 22 lines, globals: —
- `a3_task13_body_is_invalid` — 23 lines, globals: —
- `long_run_validation_tick_update` — 16 lines, globals: long_run_validation
- `editor_reset` — 21 lines, globals: main_inputs
- `a3_positional_depenetrate_manifold` — 99 lines, globals: g_cfg
- `open_numerical_input_dialog` — 46 lines, globals: editor_dialog_active, main_inputs

## Largest functions (god-file core)
- `physics_step_increment` — **574 lines**, 12 globals: a3_previous_debug_mode_state, debug_last_, editor_dialog_active, frame_timer, g_cfg, long_run_validation, main_camera_fov, main_inputs …
- `a3_positional_depenetrate_manifold` — **99 lines**, 1 globals: g_cfg
- `long_run_validation_evaluate` — **59 lines**, 4 globals: debug_last_, long_run_validation, obj_per_scene, object_count
- `a3_positional_depenetration_pass` — **51 lines**, 3 globals: g_cfg, obj_per_scene, object_count
- `open_numerical_input_dialog` — **46 lines**, 2 globals: editor_dialog_active, main_inputs
- `validation_report_print` — **42 lines**, 9 globals: broadphase_get, current_joint_count, debug_last_, g_registry, g_registry_count, main_inputs, object_capacity, object_count …
- `long_run_validation_report` — **39 lines**, 5 globals: broadphase_get, g_registry, g_registry_count, long_run_validation, object_count
- `long_run_validation_start` — **39 lines**, 3 globals: g_registry, g_registry_count, long_run_validation

## Global-state inventory (file-wide reference lines)
- `obj_per_scene`: 17 lines
- `object_count`: 28 lines
- `object_capacity`: 2 lines
- `selected_object`: 7 lines
- `current_joint_count`: 1 lines
- `main_inputs`: 124 lines
- `main_camera_fov`: 50 lines
- `editor_dialog_active`: 5 lines
- `g_cfg`: 31 lines
- `g_registry`: 21 lines
- `g_registry_count`: 6 lines
- `long_run_validation`: 69 lines
- `debug_last_`: 20 lines
- `a3_previous_debug_mode_state`: 4 lines
- `frame_timer`: 2 lines
- `main_timer`: 3 lines
- `broadphase_get`: 7 lines
