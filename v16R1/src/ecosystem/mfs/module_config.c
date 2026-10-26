/* Module config menu contract — MFS ecosystem side.
 *
 * The host UI (5 key in the engine) dlsyms these symbols on the loaded
 * ecosystem bundle ("mfs-simulator", fallback mpe_ftc.so) and renders a
 * parameter list exactly like the engine config menu (6 key) lists engine
 * parameters. Values are read fresh on every query and edits write
 * directly into live module state, so a "mod load"/"mod unload" cycle
 * changes what the menu offers without any host-side caching.
 *
 * Contract (any module can implement it):
 *   int  mpe_module_config_count (void);
 *   int  mpe_module_config_entry (int index, const char **display,
 *                                 float *value, float *min, float *max);
 *   void mpe_module_config_set (int index, float value);
 *   int  mpe_module_config_default (int index, float *value);   [optional]
 *   void mpe_module_config_reset (void);                        [optional]
 * Numeric float params; display is a human label, min/max clamp host-side
 * AND in the setters, entry returns 0 past the end, set() no-ops unknown
 * indices. The optional pair gives the menu "stock N" hints and a 9-key
 * reset-all; their absence just hides those affordances in older bundles. */
#include "modules/ftc/submodules/intake.h"

typedef struct {
    const char *display;
    float (*get) (void);
    void (*set) (float);
    float min, max, def;
} mfs_mod_param;

static const mfs_mod_param s_params [] = {
    {"Shooter launch elevation (deg)", intake_launch_elev_default, intake_launch_elev_default_set,
     INTAKE_LAUNCH_ELEV_MIN, INTAKE_LAUNCH_ELEV_MAX, 65.0f}, /* shipped default: steeper drops inside the tilted mouth (not the 50 std define) */
    {"Launch speed (m/s)", intake_launch_speed, intake_launch_speed_set,
     3.0f, 14.0f, INTAKE_LAUNCH_SPEED},
    {"Launch backspin (rad/s)", intake_launch_spin, intake_launch_spin_set,
     0.0f, 30.0f, INTAKE_LAUNCH_SPIN},
    {"Launch power floor", intake_launch_power_min, intake_launch_power_min_set,
     0.05f, 0.95f, INTAKE_LAUNCH_POWER_MIN},
};

int mpe_module_config_count (void) {
    return (int) (sizeof (s_params) / sizeof (s_params [0]));
}

int mpe_module_config_entry (int index, const char **display, float *value, float *min, float *max) {
    if ((index < 0) || (index >= (int) (sizeof (s_params) / sizeof (s_params [0])))) { return 0; }
    if (display) { *display = s_params [index].display; }
    if (value) { *value = s_params [index].get (); }
    if (min) { *min = s_params [index].min; }
    if (max) { *max = s_params [index].max; }
    return 1;
}

void mpe_module_config_set (int index, float value) {
    if ((index < 0) || (index >= (int) (sizeof (s_params) / sizeof (s_params [0])))) { return; }
    s_params [index].set (value);
}

int mpe_module_config_default (int index, float *value) {
    if ((index < 0) || (index >= (int) (sizeof (s_params) / sizeof (s_params [0])))) { return 0; }
    if (value) { *value = s_params [index].def; }
    return 1;
}

void mpe_module_config_reset (void) {
    int n = (int) (sizeof (s_params) / sizeof (s_params [0]));
    for (int i = 0; i < n; i++) { s_params [i].set (s_params [i].def); }
}
