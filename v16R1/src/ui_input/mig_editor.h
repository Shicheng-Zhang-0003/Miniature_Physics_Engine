#ifndef mig_editor_h
#define mig_editor_h
#include <stdbool.h>
#include "../core/math3d.h"
/* MIG WP4 4.4: GTK editor panel for MIG object specs.
 *
 * Plain GtkWindow, transient for the main window (resize_panel pattern:
 * CSS provider keyed on the widget name, Esc closes, destroy handler
 * nulls the globals, never modal).
 *
 * Zero GUI headers here — the .c file does all GTK work — so new_render.c
 * can declare mig_editor_is_active / mig_editor_com_position as extern
 * without pulling in GTK (the "zero GUI headers in render" rule).
 *
 * The editor edits a copy of a store spec. Save re-parses the edited text
 * through mig_spec_parse_file and stages it back into the spec store
 * (write-on-success: a rejected parse leaves the store unchanged). No
 * editor-only truth exists — the spec text is the single source of truth.
 *
 * Features:
 *  - Spec dropdown (select from the per-session store)
 *  - Part list (add / remove parts)
 *  - Per-part fields: type, dims (per-type subset), pose (position +
 *    w-first quaternion), material (mu_s / mu_k / rest), mass XOR density,
 *    colour
 *  - Total mass + COM readout
 *  - COM marker rendered in the 3D view (utility shader cross at COM)
 *  - Two-axis ground-plane translate gizmo (click+drag on the floor)
 *  - Principal-axis display: named TODO, not v16S scope */
void mig_editor_toggle (void);
void mig_editor_sync (void);
bool mig_editor_is_active (void);
/* Returns true and writes the COM of the currently edited spec into *out
 * when a spec is loaded and valid. False when no spec is loaded or the
 * edited spec is invalid. */
bool mig_editor_com_position (vector3 *out);
/* Returns true when the editor's ground-plane translate gizmo is active
 * (a drag is in progress). Used by input dispatch to suppress camera
 * motion while the gizmo is being dragged. */
bool mig_editor_gizmo_active (void);
#endif /* mig_editor_h */
