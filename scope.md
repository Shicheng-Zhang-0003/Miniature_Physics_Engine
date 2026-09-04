# v15R2 — Complete Defect & Debt Audit

> **STATUS UPDATE (post-Milestone-2):**
> This audit was taken before the FTC physics-first pivot. Approximately 30+
> items listed below have since been resolved. Key resolutions:
>
> - **ARCH-002** (simulation.c 44KB god file): RESOLVED — split to ~130 lines
> - **ARCH-009** (monolithic physics_step_increment): RESOLVED — extracted
> - **ARCH-012–017** (dead code): ALL RESOLVED — removed
> - **BUG-001–012**: ALL RESOLVED
> - **PHYS-003** (only spring joints): RESOLVED — revolute joints implemented
> - **REND-010** (render_init every frame): RESOLVED
> - **TERM-010** (MicroVim line length): RESOLVED
> - **QUAL-002** (spelling): RESOLVED
> - **DOC-001–008**: MOSTLY RESOLVED
>
> Still open: PHYS-001 (islanding), PHYS-002 (CCD), PHYS-004 (rolling friction),
> PERF-001 (SIMD), PERF-003 (frustum culling), ARCH-001 (partial), ARCH-003
> (debug_terminal 146KB), ARCH-006 (scene format), SAVE-001–008.


Every issue found across all 88 files, organized by section. Severity scale: **Critical** (breaks correctness), **High** (blocks future work), **Medium** (code quality / maintainability), **Low** (polish / cosmetic).

---

## 1. Bugs (Actual Defects)

| ID | Sub-Section | Issue | Detail | Sev | File |
|---|---|---|---|---|---|
| BUG-001 | MicroVim | `file_exists` always false | `microvim_open()` sets `mv.file_exists = mv_load_file(filename)` then immediately overwrites with `mv.file_exists = false;` on the next line. The `[New]`/`[Modified]` status indicator is always wrong. | **Crit** | `microvim.c` |
| BUG-002 | compile_asan | Script is broken | `CFLAGS="... -fsanitize=address"` — the `...` is literal text, not a placeholder. The script cannot work as written. | **Crit** | `compile_asan` |
| BUG-003 | Version string | v15R1 vs v15R2 mismatch | `#define a3_version_string "v15R1"` in `mpe_engine.h`, but the directory and release cycle are v15R2. Engine prints wrong version on startup. | **High** | `mpe_engine.h` |
| BUG-004 | Release freeze flag | Stale freeze state | `a3_release_freeze` is `0` and note says `"v15R1 development cycle active"`. Should reflect v15R2. | **Med** | `mpe_engine.h` |
| BUG-005 | Terminal uptime | Measures from first call, not engine start | `cmd_uptime` uses a static `a3_term_start_time` initialized on first `uptime` invocation, not at engine boot. Reports wrong uptime. | **Med** | `debug_terminal.c` |
| BUG-006 | Terminal capture | Memory leak on repeated capture | `term_capture_end()` sets `term_capturing = false` but never frees `term_capture_buffer`. Only `term_capture_reset()` frees it. Multiple captures without reset leak. | **Med** | `debug_terminal.c` |
| BUG-007 | Input control | Duplicate condition check | Terminal open condition: `(!config_menu_is_open ()) && (!config_menu_is_open ())` — same check written twice. | **Low** | `input_control.c` |
| BUG-008 | Input control | Duplicate field initialization | `initialise_input` sets `menu_4_pressed`, `menu_5_pressed`, `menu_6_pressed` twice in a row. | **Low** | `input_control.c` |
| BUG-009 | Spawn naming | Variable called `shift_previously_held` but key is Enter | The spawn gun logic uses `shift_previously_held` / `shift_hold_timer` / `shift_spawn_interval_timer` but the actual key is Enter. Leftover from an old iteration. | **Low** | `simulation.c` |
| BUG-010 | Validation scripts | Hardcoded v15R1 paths | `V01.sh`, `V02.sh`, `V04.sh` all reference `SRC="v15R1/src"` but the directory is `v15R2`. Scripts will fail. | **High** | `validation/V0*.sh` |
| BUG-011 | Terminal sudo | Flag persists through recursive execution | `cmd_sudo` sets `term_sudo_active = true`, calls `term_execute`, sets false. But if the executed command is `time <mutating cmd>`, the recursive `term_execute` inherits `sudo_active = true`. | **Low** | `debug_terminal.c` |
| BUG-012 | Terminal fsck | Auto-fix doesn't wake sanitized body | `cmd_fsck -y` calls `rigidbody_sanitize()` on error bodies but doesn't call `rigidbody_wake()` afterward. A sanitized sleeping body stays sleeping with corrected values. | **Low** | `debug_terminal.c` |
| BUG-013 | Release notes | Unchecked validation checkboxes | `release_notes_v15R1.md` has `- [ ]` (unchecked) for all validation items despite `v03_gate_validation.log` showing ALL P0 PASS. | **Low** | `release_notes_v15R1.md` |

---

## 2. Architecture / Structural Debt

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| ARCH-001 | Global state | All scene state is file-scope globals | `obj_per_scene`, `object_count`, `object_capacity`, `main_camera_fov`, `main_inputs`, `selected_object`, `main_timer` are globals. Blocks multithreading, multiple worlds, headless testing, unit tests. | **Crit** | `simulation.c`, `root_gtk.c`, `mpe_engine.h` |
| ARCH-002 | God file | `simulation.c` is 44 KB | Contains physics tick, camera movement, input dispatch, menu handling, validation suite, depenetration, spawn logic — at least 7 distinct responsibilities. | **High** | `simulation.c` |
| ARCH-003 | God file | `debug_terminal.c` is 123 KB | Contains 8 phases of ~70 command implementations, virtual filesystem, alias storage, output capture, MicroVim integration — all in one translation unit. | **High** | `debug_terminal.c` |
| ARCH-004 | God file | `collision_mechanics.c` is 43 KB | Contains all narrowphase, solver prep, solver iteration, contact cache, and floor proxy in one file. | **Med** | `collision_mechanics.c` |
| ARCH-005 | Include header | `mpe_engine.h` includes 25+ headers | Every file that includes `mpe_engine.h` pulls in the entire engine. No forward declarations, no minimal includes. | **Med** | `mpe_engine.h` |
| ARCH-006 | Scene format | Raw binary struct dump | `scene_saving.c` does `fwrite` of 14 fields per body. No joints, no stable IDs, no sleep state, no `nice_value`, no versioning flexibility, no endianness handling. | **High** | `scene_saving.c`, `scene_load.c` |
| ARCH-007 | Sleep system | Staticize/restore hack | Before solver: sleeping bodies get `inverse_mass = 0` and `inverse_inertia_system = zero`. After solver: restored. If anything reads a body between these two passes, it sees corrupted properties. | **High** | `simulation.c` |
| ARCH-008 | Menu system | Magic integer state machine | `spawner_menu_level == 3`, `velocity_menu_level == 21`, `object_menu_level == 85` — scattered across `input_control.c`, `editor.c`, `overlay.c`. No named states, no FSM. | **High** | `input_control.c`, `editor.c`, `overlay.c` |
| ARCH-009 | Physics tick | Monolithic `physics_step_increment` | ~400 lines: camera, input, menus, validation, spawn, physics loop, overlay update — all in one function. | **High** | `simulation.c` |
| ARCH-010 | Config coupling | Registry storage pointers resolved at compile time | `mpe_config_schema.c` uses `&g_cfg.world.gravity` directly in the static array. Any struct layout change silently breaks the registry. | **Med** | `mpe_config_schema.c` |
| ARCH-011 | Floor proxy | Static rigidbody for floor | `collision_static_plane_body_proxy()` returns a pointer to a `static rigidbody`. Not thread-safe. Restitution hardcoded to 1.0. `object_id = 0xFFFFFFFFu` sentinel. | **Med** | `collision_mechanics.c` |
| ARCH-012 | Dead code | `rb_integrate` is unused | The old full-integration function `rb_integrate()` in `rigidbody.c` is superseded by `rb_integrate_velocity()` + `rb_integrate_position()` but still compiled. | **Low** | `rigidbody.c` |
| ARCH-013 | Dead code | `define_forces.c` mostly unused | `force_applicant_universal_gravity`, `force_applicant_vertical_anchor`, `force_to_system_energy_amount`, `force_applicant_friction_rolling` — all implemented, none called. | **Low** | `define_forces.c` |
| ARCH-014 | Dead code | `adjust_joints_after_deletion` is a no-op | Takes `deleted_object_index`, casts it to `(void)`, does nothing. Kept for API compatibility. | **Low** | `spring_joint.c` |
| ARCH-015 | Dead code | `fov_aspr_perspective` in math3D.h | 3×3 perspective function superseded by `math4_perspective_fov`. Never called. | **Low** | `math3D.h` |
| ARCH-016 | Dead file | `frame_timer.c` is empty | Contains only `#include` and a comment "This is only here for building and compiling". | **Low** | `frame_timer.c` |
| ARCH-017 | Dead code | Commented-out camera code in root_gtk.c | Bottom of file has commented-out code about passing camera FOV to GPU. | **Low** | `root_gtk.c` |

---

## 3. Performance Issues

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| PERF-001 | Math | No SIMD | All math in `math3D.h` / `math4_special.h` is scalar. SAT (15 axes), spatial hashing, quaternion integration are all highly vectorizable. Directly causes the ~1136 object performance wall. | **High** | `math3D.h`, `math4_special.h` |
| PERF-002 | Build | `-march=native` disabled by default | `ENABLE_NATIVE ?= 0` in makefile. No auto-vectorization hints. | **Med** | `makefile` |
| PERF-003 | Rendering | No frustum culling | Every object gets a model matrix computed and uploaded every frame regardless of visibility. | **High** | `new_render.c` |
| PERF-004 | Rendering | No dirty-checking on instance packing | The instance buffer packing loop recomputes all model matrices every frame. No check for whether a body actually moved. | **High** | `new_render.c` |
| PERF-005 | Broadphase | Cell size recomputed every tick | `broadphase_update_cell_size()` iterates all objects to compute average radius every physics tick. O(n) per tick. | **Med** | `broadphase.c` |
| PERF-006 | Broadphase | Node pool uses malloc/realloc | `node_pool` in `broadphase.c` uses `malloc`/`realloc` instead of a pre-allocated pool. | **Med** | `broadphase.c` |
| PERF-007 | Contact cache | Linear search for cache matching | `collision_prepare_solver` does a linear scan of `contact_impulse_cache` (up to 16384 entries) for each contact point. | **Med** | `collision_mechanics.c` |
| PERF-008 | MicroVim | O(n²) file loading | `mv_load_file` calls `mv_insert_line` for each line, which shifts all existing lines. Loading a 1000-line file is O(n²). | **Med** | `microvim.c` |
| PERF-009 | MicroVim | O(n) undo per edit | Every `mv_undo_push` deep-copies all lines. 64 undo depth × full document copy per edit. | **Med** | `microvim.c` |
| PERF-010 | Overlay | O(n²) string building | `overlay_append_overflow_text` and `overlay_append_stats_text` use `strlen` + `snprintf` to append — rescans the buffer each time. | **Low** | `overlay.c` |
| PERF-011 | Sanitization | Double sanitize per tick | `rigidbody_sanitize` is called in the sanitize loop AND again after `rb_integrate_position` for every body every tick. | **Med** | `simulation.c` |
| PERF-012 | Raycast | O(n) per click | `selector_ray_tracing` iterates all objects. No spatial acceleration structure. | **Low** | `object_selector.c` |
| PERF-013 | Spawn overlap | O(n) per spawn | `scene_resolve_spawn_overlap` iterates all objects for each new spawn. Spawning n objects is O(n²). | **Low** | `scene_init.c` |
| PERF-014 | Energy calc | Inverse of inverse inertia | `rb_get_kinetic_energy` computes `math3_inverse(rigid_body->inverse_inertia_system)` — the inverse of the inverse is just the inertia tensor. Wasteful. | **Low** | `rigidbody.c` |
| PERF-015 | Depenetration | Conditional broadphase rebuild | If boundary moved any object, `a3_positional_depenetration_pass` rebuilds the entire broadphase. The check is per-position-delta, not per-significance. | **Low** | `simulation.c` |

---

## 4. Code Quality / Maintainability

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| QUAL-001 | Naming | Three naming generations coexist | `a3_` prefix (A3 patch era), `MPE_TASK_XX_` markers (numbered tasks), `MPE_TASK_V15R2_` markers (current). `A3_PATCH_XX_` comments throughout. | **Med** | All files |
| QUAL-002 | Naming | Inconsistent spelling | `initialize_camera` (American) vs `initialise_input` (British). | **Low** | `camera.c`, `input_control.c` |
| QUAL-003 | Naming | `vector4` used for quaternions | `vector4` is a generic 4-component vector. Quaternions should have their own type name. | **Low** | `math3D.h` |
| QUAL-004 | Naming | `math3` / `math4` are generic | 3×3 and 4×4 matrix types have non-descriptive names. | **Low** | `math3D.h`, `math4_special.h` |
| QUAL-005 | Naming | Enum values use lowercase | `object_sphere`, `object_cube` — C convention is usually uppercase for enum constants. | **Low** | `rigidbody.h` |
| QUAL-006 | Error handling | No unified error reporting | Some functions return int, some bool, some -1. Errors go to `stderr` via `fprintf`. The event log exists but is barely used. | **High** | Multiple |
| QUAL-007 | Error handling | `scene_add_object` failure not always checked | Returns -1 on failure. Some callers check, some don't. | **Med** | `object_spawner.c`, `debug_terminal.c` |
| QUAL-008 | Error handling | `add_joint` failure not checked in editor | `editor.c` calls `add_joint()` in the joint-link menu but doesn't check the return value. | **Med** | `editor.c` |
| QUAL-009 | Error handling | `save_scene` result not shown to user | Scene menu calls `save_scene()` but doesn't display success/failure. | **Med** | `simulation.c` |
| QUAL-010 | Error handling | Shader errors only go to stderr | `compile_shader` and `create_shader_program` print to stderr but don't push to event log. | **Low** | `shader_loading.c` |
| QUAL-011 | Memory | No arena/pool allocators | All allocations use raw `malloc`/`realloc`/`free`. No memory tracking. | **Med** | Multiple |
| QUAL-012 | Memory | Render instance buffers never freed | `sphere_instances` and `cube_instances` in `new_render.c` are `malloc`'d in `render_init` but never freed. | **Low** | `new_render.c` |
| QUAL-013 | Memory | Broadphase node pool never freed | `node_pool` in `broadphase.c` is allocated but never freed on shutdown. | **Low** | `broadphase.c` |
| QUAL-014 | Magic numbers | Hardcoded constants scattered | `0.98f` angular damping on boundary hit, `1.01f` wireframe scale, `0.0001f` epsilon thresholds, `0.97f` angular drag multiplier, `8.0f` camera acceleration multiplier. | **Med** | Multiple |
| QUAL-015 | Fixed buffers | snprintf into fixed-size arrays | `overlay.c` uses `char [512]` and `char [1024]`. `config_menu.c` uses `char [2048]`. Could overflow with many objects/params. | **Med** | `overlay.c`, `config_menu.c` |
| QUAL-016 | Fixed arrays | Contact cache limited to 16384 | `max_cached_contacts` is a compile-time constant. No dynamic growth. | **Low** | `mpe_constants.h` |
| QUAL-017 | Fixed arrays | MicroVim file whitelist is hardcoded | `mv_known_files[]` in `debug_terminal.c` lists 32 files with relative paths from `src/`. Breaks if working directory changes. | **Low** | `debug_terminal.c` |
| QUAL-018 | Fixed arrays | Terminal alias table capped at 32 | `term_alias_max` is 32. No dynamic growth. | **Low** | `debug_terminal.c` |
| QUAL-019 | GTK deprecation | Multiple deprecated GTK3 APIs used | `gtk_dialog_run`, `gtk_container_add`, `gtk_box_pack_start`, `gtk_widget_show_all` — all deprecated in GTK4. | **Med** | Multiple |
| QUAL-020 | Config menu | Fixed param array size | `config_menu.c` uses `static const mpe_param *category_params [64]` — breaks if any category exceeds 64 params. | **Low** | `config_menu.c` |

---

## 5. Documentation Issues

| ID | Sub-Section | Issue | Detail | Sev | File |
|---|---|---|---|---|---|
| DOC-001 | how_to_use.md | Config section duplicated 3× | The "Configuration System (Key 6)" section appears three times with identical content. Copy-paste error. | **High** | `how_to_use.md` |
| DOC-002 | readme.md | Version references stale | Multiple references to "v15R1" as the current version. Should reference v15R2 development. | **Med** | `readme.md` |
| DOC-003 | RELEASE_POLICY.md | Header says v15R1 | "This tree is in **v15R1 development**" — should say v15R2. | **Med** | `RELEASE_POLICY.md` |
| DOC-004 | RELEASE_GATES.md | Entirely about v15R1 | All gates reference v15R1. No v15R2 gates defined yet. | **Med** | `RELEASE_GATES.md` |
| DOC-005 | evolution.txt | Doesn't mention v15R2 | "Current Head: v15R1" — needs update. | **Low** | `evolution.txt` |
| DOC-006 | Install instructions | References specific GitHub URL | `git clone https://github.com/shicheng-zhang/physics-engine.git` — may not be correct or may go stale. | **Low** | `linux_install_instructions.md` |
| DOC-007 | Install instructions | Says `./compile` but build uses `make` | Instructions say "Run `./compile`" but the compile script just calls `make`. Inconsistent. | **Low** | `linux_install_instructions.md` |
| DOC-008 | Release notes | Validation checkboxes unchecked | All `- [ ]` in release_notes_v15R1.md despite gates passing. | **Low** | `release_notes_v15R1.md` |

---

## 6. Build System Issues

| ID | Sub-Section | Issue | Detail | Sev | File |
|---|---|---|---|---|---|
| BUILD-001 | Makefile | Uses `find` to discover sources | `SOURCES := $(shell find . -name '*.c')` — slow, fragile, picks up files in unexpected directories. | **Med** | `makefile` |
| BUILD-002 | Makefile | No separate debug/release targets | Only one build configuration. No `make debug` or `make release`. | **Med** | `makefile` |
| BUILD-003 | Makefile | No install target | No `make install`. | **Low** | `makefile` |
| BUILD-004 | compile_asan | Broken (see BUG-002) | Literal `...` in CFLAGS. | **Crit** | `compile_asan` |
| BUILD-005 | compile script | No error checking | `make clean; make -j$(nproc);` — doesn't check if make succeeded. | **Low** | `compile` |
| BUILD-006 | Repo hygiene | status/ files tracked despite .gitignore | `.gitignore` says `status/` but `engine.cfg`, `engine.cfg.bak`, `engine.cfg.backup` are in the dump. | **Med** | `status/` |

---

## 7. Physics Correctness

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| PHYS-001 | Solver | No solver islanding | All manifolds solved in one global loop. Unrelated contacts interfere. Blocks parallel solving. | **High** | `simulation.c` |
| PHYS-002 | CCD | No continuous collision detection | Fast objects (spawner speed up to 500 m/s) can tunnel through thin geometry. | **High** | N/A (missing) |
| PHYS-003 | Constraints | Only spring joints exist | No revolute, fixed, distance, or slider joints. `spring_joint.c` hardcodes spring-specific logic. | **Med** | `spring_joint.c` |
| PHYS-004 | Rolling friction | Not implemented | Only sliding friction. Spheres don't generate rolling resistance torque. | **Med** | `collision_mechanics.c` |
| PHYS-005 | Floor restitution | Proxy hardcoded to 1.0 | `collision_static_plane_body_proxy` sets `restitution = 1.0f`. Actual bounce uses `fminf(body_restitution, 1.0)` which is correct but the proxy value is misleading. | **Low** | `collision_mechanics.c` |
| PHYS-006 | Angular drag | Hardcoded 0.97 multiplier | `angular_damping_factor = powf(g_cfg.world.drag * 0.97f, fixed_physics_dt)` — the 0.97 is not in config. | **Low** | `simulation.c` |
| PHYS-007 | Contact cache | Rotation can invalidate local-space matching | Cache matches contacts by local-space position. If a body rotates significantly between frames, the local-space match can be wrong. | **Low** | `collision_mechanics.c` |
| PHYS-008 | Depenetration | Runs after boundary, before next broadphase | Objects moved by depenetration may create new overlaps not detected until next tick. | **Low** | `simulation.c` |
| PHYS-009 | nice damping | Very aggressive at high values | `nice_value = 19` → factor `0.962` per tick at 60 Hz → velocity halves every ~18 ticks (0.3 seconds). | **Low** | `rigidbody.c` |
| PHYS-010 | Restitution threshold | Negative default is confusing | `restitution_velocity_thresh` defaults to `-1.5`. Correct (approach velocity is negative) but poorly documented. | **Low** | `mpe_config_schema.c` |

---

## 8. Rendering Issues

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| REND-001 | Culling | No frustum culling | All objects rendered regardless of camera view. | **High** | `new_render.c` |
| REND-002 | Instance packing | CPU-bound matrix computation | All model matrices computed on CPU every frame. No compute shader, no dirty flag. | **High** | `new_render.c` |
| REND-003 | Grid | Always rendered | Grid draws even when camera looks away. No visibility check. | **Low** | `grid.c` |
| REND-004 | Grid | Hardcoded color | Grid color `(0.3, 0.3, 0.3)` is hardcoded, not in config. | **Low** | `grid.c` |
| REND-005 | Wireframe | Hardcoded selection color | Yellow `(1.0, 1.0, 0.0)` hardcoded. | **Low** | `wireframe.c` |
| REND-006 | Wireframe | Hardcoded scale factor | `1.01f` scale for wireframe overlay. | **Low** | `wireframe.c` |
| REND-007 | Wireframe | Two separate uniform caches | `a3_wire_cached_program` and `a3_wire_missing_cached_program` — leftover from patch sequence. Should be unified. | **Low** | `wireframe.c` |
| REND-008 | Shaders | No hot-reload | Shader changes require engine restart. MicroVim can edit shaders but can't trigger recompilation. | **Low** | `shader_loading.c` |
| REND-009 | Render init | Magic number status | `render_init_status` uses -1/0/1 instead of an enum. | **Low** | `new_render.c` |
| REND-010 | Render init | Called every frame | `render_scene_current` calls `render_init()` every frame. Early return makes it cheap but it's still a function call. | **Low** | `new_render.c` |

---

## 9. Terminal / MicroVim Issues

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| TERM-001 | Terminal | No tab completion | No command or path completion. | **Low** | `debug_terminal.c` |
| TERM-002 | Terminal | No piping | No `|` support despite having `tee`. Commands can't be chained. | **Low** | `debug_terminal.c` |
| TERM-003 | Terminal | No output redirection | No `>` or `>>` support. Only `tee` writes to files. | **Low** | `debug_terminal.c` |
| TERM-004 | Terminal | `less` is one-shot | Not actually paginated. Prints first 40 lines and stops. | **Low** | `debug_terminal.c` |
| TERM-005 | Terminal | `watch` is one-shot | Not periodic. Prints a disclaimer and runs once. | **Low** | `debug_terminal.c` |
| TERM-006 | Terminal | `tee` writes to arbitrary paths | No path validation. Could overwrite engine files. | **Low** | `debug_terminal.c` |
| TERM-007 | Terminal | `xxd` dumps raw struct bytes | Includes pointers — implementation-defined, exposes addresses. | **Low** | `debug_terminal.c` |
| TERM-008 | Terminal | History push is O(n) | `term_history_push` shifts all 64 entries on each command. | **Low** | `debug_terminal.c` |
| TERM-009 | MicroVim | Search allocates per line | `mv_search_execute` calls `g_ascii_strdown` for every line and every search. No compiled pattern. | **Low** | `microvim.c` |
| TERM-010 | MicroVim | No line length enforcement | Lines can exceed `mv_max_line_len` (4096) through insertions. Buffer overflow risk. | **Med** | `microvim.c` |
| TERM-011 | MicroVim | `:q` error display is awkward | Sets `command_buf` to error text and mode to `mv_command` — the error appears in the command line but isn't a real command. | **Low** | `microvim.c` |

---

## 10. Testing / Validation Gaps

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| TEST-001 | Headless | No headless test mode | All validation (F5–F11) requires GTK window and manual keypresses. Cannot run in CI. | **High** | N/A (missing) |
| TEST-002 | Unit tests | Zero unit tests | No tests for math library, collision functions, solver, or config system. | **High** | N/A (missing) |
| TEST-003 | V03.py | Interactive only | Requires human to type p/f/s for each gate. Cannot be automated. | **Med** | `validation/V03.py` |
| TEST-004 | V04.sh | Just prints instructions | Doesn't actually run the F10 test. Just tells the user what to do. | **Med** | `validation/V04.sh` |
| TEST-005 | V01/V02/V04 | Wrong directory references | All reference `v15R1/src` instead of `v15R2/src`. | **High** | `validation/V0*.sh` |
| TEST-006 | Config torture | Uses weak RNG seed | `srand(time(NULL) ^ 0xDEADBEEF)` — predictable. Fine for testing but worth noting. | **Low** | `scene_init.c` |
| TEST-007 | Config torture | Randomizes debug-only params | F11 randomizes ALL params including debug-only ones that normal users can't touch. Could produce configs that can't be restored via the menu. | **Med** | `scene_init.c` |

---

## 11. Platform / Compatibility

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| PLAT-001 | Wayland | Mouse lock doesn't work | Documented known limitation. Engine forces X11 via `g_setenv("GDK_BACKEND", "x11", TRUE)`. | **Med** | `root_gtk.c` |
| PLAT-002 | X11 forced | Hardcoded backend override | `g_setenv("GDK_BACKEND", "x11", TRUE)` runs unconditionally. Users who want Wayland can't override. | **Med** | `root_gtk.c` |
| PLAT-003 | Windows | Not supported | No Windows build path. GTK3 on Windows is possible but untested. | **Low** | N/A |
| PLAT-004 | macOS | Unsupported | Install docs mention "Intel MacOS users may attempt" — no actual support. | **Low** | N/A |
| PLAT-005 | HiDPI | Only scale factor handling | `on_rendered` multiplies by `gtk_widget_get_scale_factor` but no other HiDPI handling. | **Low** | `root_gtk.c` |
| PLAT-006 | GTK4 migration | Not possible without rewrite | 4+ deprecated GTK3 APIs used. GTK4 removal of `gtk_dialog_run` etc. would require significant rework. | **Med** | Multiple |

---

## 12. Scene / Config Persistence

| ID | Sub-Section | Issue | Detail | Sev | File(s) |
|---|---|---|---|---|---|
| SAVE-001 | Scene | Joints not saved | All spring joints lost on save/load. | **High** | `scene_saving.c` |
| SAVE-002 | Scene | Object IDs reassigned | `scene_load.c` calls `scene_allocate_object_id()` for every loaded body. External references break. | **High** | `scene_load.c` |
| SAVE-003 | Scene | Sleep state not saved | All bodies load awake. | **Med** | `scene_saving.c` |
| SAVE-004 | Scene | `nice_value` not saved | Per-object damping lost on save/load. | **Med** | `scene_saving.c` |
| SAVE-005 | Scene | No integrity check | No checksum, no CRC. Corrupt files cause undefined behavior. | **Med** | `scene_load.c` |
| SAVE-006 | Scene | Endianness-dependent | Raw `fwrite`/`fread` of structs. Big-endian systems would produce corrupt saves. | **Low** | `scene_saving.c` |
| SAVE-007 | Scene | fwrite return values unchecked | `write_float`, `write_int`, `write_vec3`, `write_vec4` don't check return values. | **Low** | `scene_saving.c` |
| SAVE-008 | Scene | Triple sanitization on load | `scene_load.c` calls `rigidbody_set_static`, then `rigidbody_sanitize`, then `rigidbody_update_axes`, then `rigidbody_sanitize` again. | **Low** | `scene_load.c` |
| SAVE-009 | Config | No hot-reload | Config file changes require terminal `config load` or engine restart. No file watching. | **Low** | `mpe_config.c` |
| SAVE-010 | Config | Three config files in repo | `engine.cfg`, `engine.cfg.bak`, `engine.cfg.backup` all present. `.gitignore` says `status/` should be ignored. | **Med** | `status/` |

---

## Summary Counts

| Severity | Count |
|---|---|
| **Critical** | 3 |
| **High** | 22 |
| **Medium** | 33 |
| **Low** | 52 |
| **Total** | **110** |

| Section | Count |
|---|---|
| Bugs | 13 |
| Architecture | 17 |
| Performance | 15 |
| Code Quality | 20 |
| Documentation | 8 |
| Build System | 6 |
| Physics Correctness | 10 |
| Rendering | 10 |
| Terminal / MicroVim | 11 |
| Testing | 7 |
| Platform | 6 |
| Scene / Config Persistence | 10 |

The three **Critical** items (BUG-001 microvim file_exists, BUG-002 compile_asan broken, ARCH-001 global state) should be addressed first. The 22 **High** items form the v15R2 → v15S roadmap.
