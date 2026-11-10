# SANDBOX_PLAN.md — sandbox/editor feature train

> Strategic context: MPE carries the universal robotics & mechatronics
> simulator; MFS is its springboard. MIG WP2 is COMPLETE (`9e9fd43` 2.8
> gates + boundary defect fix, `9f97b68` 2.9 TSan + A2 byte probe —
> see MIG_BUILD_PLAN.md). This train adds the sandbox affordances the
> author asked for on top of the landed engine: freeze the game, toggle
> object situations, and per-object trajectory indicators shown only
> while selected.
>
> House discipline inherited from MIG_BUILD_PLAN.md: each step lands as
> one green commit; the STATUS LINE updates at every landing; an
> engine-touching change commits green or not at all (suite `--all` +
> quick + `make everything` scanned).
>
> ⏸ STATUS LINE (keep updated at every step): **PLANNED 2026-11-09.
> MIG WP2 complete at `9f97b68`; suite 57/55+2, quick 71/71, TSan
> clean, A2 byte probe certified against `8e62e58`.
> S1 LANDED 2026-11-09 (`8077bfa`): P freezes/resumes, O steps one
> tick while frozen (`physics_halt_step_once` — armed tick runs with
> `physics_halted` re-set TRUE for the following tick, so the
> `FROZEN (P resume, O step)` badge never flickers); four-latch key
> discipline (keypress/released/initialize/focus-out); suite 57/0,
> quick 71/71 re-run (physics_halt.c is GUI-linked only, not in the
> suite link list). S2 LANDED 2026-11-09 (`f5d3dde`): the object
> menu's level-5 screen is now `-- Situation --` (static / kinematic /
> no-collide cursor + Enter-toggle, sleeping display-only) through the
> safe mutators with the contact-cache-clear pattern; root menu item
> renamed; status bar shows (Static)/(Kinematic)/(Dynamic,
> no-collide)/(Dynamic). S3 LANDED 2026-11-10: the selected object's
> trajectory trail — cyan `traj_line` polyline (`traj_line_render_colour`
> added; the original purple is now just the default), 256-point ring
> sampled once per SIMULATED tick in `physics_step_increment` (frozen
> worlds never grow it; each O step adds exactly one point), keyed to
> `selected_object_id` so it clears on reselect/deselect, drawn next to
> the selected wireframe ONLY while a body is selected. Files:
> `render/trail.c`+`.h` (new), `traj_line` colour variant,
> `new_render.c` init/cleanup/draw, `simulation.c` sampling. GUI-only;
> suite 57/0 re-run, build 0 under -Werror.
> NEXT: S4 reserve — slow-mo time scale / step-N / object list / trail
> options, after a live review of S1–S3.
> S1 FIX LANDED 2026-11-10 (`8a6ff21`): user report — P locked the
> ENTIRE engine (no camera, no un-freeze). Root cause: the halt gate
> early-returned at the TOP of physics_step_increment, skipping input
> polling, camera AND dispatch — and dispatch is what consumes the
> p_key_pressed latch, so a frozen engine could never resume. The gate
> now sits with THE PHYSICS LOOP and gates only simulation_physics_tick
> (+ the S3 trail sample, which must track simulated ticks): input,
> camera, menus, terminal and overlay run every frame — a frozen world
> is an inspection world. Terminal halt/sleep share the corrected
> semantics. Suite 57/0 re-run. S4a LANDED 2026-11-10: the resize
> panel — a tiny terminal-style GTK window (debug-terminal recipe:
> plain GtkWindow, transient, Esc closes, own CSS provider) toggled
> with T, one slider row per size dimension of the SELECTED body
> (sphere radius / cylinder radius + half-length / cube XYZ half
> extents). Dragging a slider resizes live with the full editor
> radius-edit tail (inertia update, contact-cache clear, wake); cube
> resize maintains the covering bound `radius = |half_extensions|`
> (the spawn convention — stale bound = missed broadphase pairs). Rows
> are built once, shown/hidden per type; sliders sync from the
> selection (id-keyed, `syncing` guard against value-changed
> recursion); opening releases the mouse lock (sliders need the
> pointer). Files: `ui_input/resize_panel.c`+`.h` (new), `T` latch in
> `input_state.h` + the four-latch spots in `input_control.c`,
> dispatch consumption, `resize_panel_sync()` in `overlay_update`.
> GUI-only; suite 57/0 re-run, build 0 under -Werror.**

## Findings this plan builds on (2026-11-09 survey)

- Pause machinery EXISTS: `core/physics_halt.c`
  (`physics_halt_set/_for_ticks/is_halted/tick_update`, `_Atomic`),
  checked at the top of `physics_step_increment`
  (`ui_input/simulation.c:50`) — rendering + overlay keep running while
  physics skips. Terminal `halt`/`sleep` commands exist
  (`ui_input/term_sys.c:306`); NO GUI key is bound.
- Selection EXISTS: right-click ray cast through the crosshair
  (`ui_input/object_selector.c:98`), id-stable via
  `selection_validate`/`scene_find_object_index_by_id`, yellow
  wireframe highlight (`render/wireframe.c:143`,
  `wireframe_render_selected_object` in `render/new_render.c:479`).
- Object menu EXISTS: E on selection opens the per-object menu
  (`ui_input/overlay.c:253` text; routing `ui_input/input_control.c:302`;
  handlers `ui_input/editor.c`). Level 5 is the Immovable
  arrows-toggle/Enter-save screen (`editor.c:246`).
- Situation flags + safe mutators: `static_state`/`kinematic`/
  `no_collide`/`is_sleeping` on `rigidbody`
  (`core/rigidbody.h:57,73,99`); `rigidbody_set_static`
  (`core/rigidbody.c:1282`), `rigidbody_set_kinematic`
  (`core/rigidbody.c:1322`), `rigidbody_wake`
  (`core/rigidbody.c:1247`). `no_collide` has NO mutator — existing
  writers set the field directly (`ecosystem/mfs/modules/ftc/
  gui_robot_registry.c:67`).
- Line-draw facility EXISTS: `render/traj_line.c` (dynamic GL_LINES
  polyline on the utility shader, per-draw colour uniform; the FTC
  predicted-shot line rides it at `render/new_render.c:318`).
- FTC per-tick hooks run INSIDE `physics_world_step`
  (`core/physics_world.c:2046`) so a halt freezes them for free; the
  GUI-side riders (`gamepad_drive_tick`, `ftc_view_tick`,
  `ftc_traj_update`, `ftc_watchdog_tick`) keep running during a halt by
  design — camera and overlay must stay live while frozen.
- Key conflicts: Space = camera up; comma/period = robot intake/shoot;
  digits 5–9 = menu openers; `p`/`o` are free (microvim's same-letter
  vim-keys are modal editor-only — the documented global-free
  convention, `input_control.c:91`).

## S1 — freeze + frame-step keys (P / O)

- [ ] **S1 Freeze/step:** `P` toggles `physics_halt_set(!physics_is_halted())`;
  `O` runs exactly one physics tick while frozen. New
  `physics_halt_step_once()` in `core/physics_halt.c` (arm flag consumed
  by `physics_halt_tick_update`: run the armed tick, re-halt after it —
  `physics_is_halted()` stays TRUE through the stepped tick so the badge
  never flickers). New `p_key_pressed`/`o_key_pressed` in
  `ui_input/input_state.h`, set/cleared in `input_control.c`
  (keypress/released/`initialize_input`/`on_focus_out`), consumed at the
  top of `simulation_input_dispatch.c`. Status bar gains a `FROZEN`
  badge (`ui_input/overlay.c` mode string, both selected and
  no-selection branches) — camera and overlay stay live while frozen.
  GUI-only EXCEPT `core/physics_halt.c` (not in the suite link list —
  the suite binary cannot regress; suite + quick re-run anyway as
  house discipline, `make everything` scanned).

## S2 — per-object situation toggles (object menu 4)

- [ ] **S2 Situation screen:** the object menu's level-5 screen becomes
  `-- Situation --` with four rows: Static (immovable), Kinematic,
  No-collide, Sleeping (display-only). Up/Down moves a cursor, Enter
  toggles the cursor row, E closes (replaces the old
  arrows-toggle-Immovable/Enter-save pattern). Mutations through the
  safe mutators: `rigidbody_set_static`, `rigidbody_set_kinematic`,
  direct `no_collide` write (matching the existing writer convention),
  each followed by `contact_cache_clear` (the MPE_TASK_06 pattern);
  `rigidbody_wake` where a body must re-enter the solver. Root menu
  text: `4: Situation (static/kinematic/no-collide)`. Status bar shows
  the selected body's situation summary. GUI-only; suite untouched.

## S3 — trajectory trail, selected object only

- [ ] **S3 Selected trajectory:** a ring buffer (256 points ≈ 4.3 s)
  of the selected body's position, sampled once per physics tick in
  `physics_step_increment` AFTER the physics tick (skipped while frozen
  — a frozen world must not grow its trail), keyed to
  `selected_object_id` so the trail clears on reselect/deselect. Drawn
  as a GL_LINES polyline through a private `traj_line_mesh`
  (`render/traj_line.c` pattern) in `render_scene_current` next to
  `wireframe_render_selected_object` (`render/new_render.c:479`) —
  drawn ONLY when a body is selected and the trail has ≥ 2 points.
  Distinct colour (cyan) vs the FTC predicted-shot line. GUI-only;
  render files are suite-unlinked.

## S4 — piece together + resize (user request 2026-11-10)

- [x] **S4a resize panel (LANDED):** T toggles a tiny terminal-style
  GTK window with a slider per size dimension of the selected body;
  drag to resize live (inertia + contact cache + wake follow; cube
  covering radius maintained). Landed — see STATUS LINE.
- [ ] **S4b drag-move:** LMB (while mouse-locked) grabs the aimed body
  — crosshair ray selects, then the body holds its distance along the
  camera forward (`grab_dist = dot(pos − cam_pos, fwd)`, min 1.0) and
  follows `cam_pos + fwd · grab_dist` while held; velocities zeroed,
  body woken; release drops it. LMB latches join the four-latch
  discipline (`on_button_press` btn==1 only when `is_mouse_locked`).
- [ ] **S4c weld-on-drop:** releasing a dragged body while its covering
  sphere overlaps another body (nearest, `dist < ra + rb − 0.02`)
  pieces the two into ONE rigid `object_composite` via the public
  `physics_world_add_composite` (MIG API, world-frame part array, cap
  16) — then removes both originals (higher index first) and
  re-selects the new body by id. Per-part friction/restitution carried
  post-hoc (`mig_part` has no material fields; composite parts get
  `body_defaults`). Composite parts of a source composite expand into
  world frame (`pos = body_pos + R_body·local_pos`,
  `ori = q_body ⊗ q_local`). GUI-only; welded composites won't
  scene-persist yet (WP3 scope).
- [ ] **S4 reserve:** candidates after a live review: a slow-motion
  time scale knob, a step-N-ticks variant, an object list overlay,
  trail length/style options, a K-key explicit weld as an alternative
  to drop-weld.
