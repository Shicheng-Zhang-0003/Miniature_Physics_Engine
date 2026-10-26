#ifndef tui_headless_h
#define tui_headless_h
#include "../core/math3d.h"
#include "../core/physics_world.h"

/* Headless shell state (defined in tui/tui_shell.c).
 * Kept separate from the GTK debug terminal so mpe-tui links zero GTK code. */
extern int tui_selected_object;
extern int tui_debug_mode_active;
extern int tui_quit_requested;
extern int tui_paused;
extern float tui_time_scale;
extern float tui_camera_speed;
extern vector3 tui_cam_pos;
extern float tui_cam_yaw, tui_cam_pitch;
void tui_camera_forward (vector3 *out);

/* Run the headless shell (interactive / scriptable engine without a display).
 * Returns process exit code. */
int tui_shell_run (int argc, char **argv);
#endif
