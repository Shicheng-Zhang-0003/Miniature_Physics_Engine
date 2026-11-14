/* MIG WP4 4.6c: the miniature CAD WINDOW — a real GL viewport you sit
 * inside and edit with. See ui_input/cad_window.c for the full design
 * (own GtkGLArea context + inline shaders + unit meshes, orbit camera,
 * click-pick/drag-move, the whole verb set driving cad_editor.c's
 * landed model, persistence staged through the shared mig_spec_store).
 *
 * WP4 4.6d lifted the dead-code ruling (the user's connect call): key
 * 2 in the FREE state opens/closes this window (simulation_input_dispatch
 * -> cad_window_toggle, the resize-panel T pattern). The design model
 * (cad_editor.c) and the view math (cad_view_math.c) it drives are
 * already gated; the window itself is GUI-only and suite-unlinked.
 * The spawn-to-world hookup (cad_build -> physics_world_add_composite)
 * is the NEXT brick — this TU is where it will live, and it is the only
 * place allowed to include physics_world.h.
 */
#ifndef cad_window_h
#define cad_window_h

/* Open the CAD window (or close it if already open — a toggle, like
 * the 4.4 panel). The design persists across close/reopen. */
void cad_window_toggle (void);

/* 1 while the window is open. Diagnostic/diag-menu use only; nothing
 * in the shipped input dispatch calls it yet. */
int cad_window_active (void);

#endif /* cad_window_h */
