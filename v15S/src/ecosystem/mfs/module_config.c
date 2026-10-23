/* Module config menu contract — MFS ecosystem side.
 *
 * The host UI (5 key in the engine) dlsyms these three symbols on the
 * loaded ecosystem bundle ("mfs-simulator", fallback mpe_ftc.so) and
 * renders a parameter list exactly like the engine config menu (6 key)
 * lists engine parameters. Values are read fresh on every query and edits
 * write directly into live module state, so a "mod load"/"mod unload"
 * cycle changes what the menu offers without any host-side caching.
 *
 * Contract (any module can implement it):
 *   int  mpe_module_config_count (void);
 *   int  mpe_module_config_entry (int index, const char **display,
 *                                 float *value, float *min, float *max);
 *   void mpe_module_config_set (int index, float value);
 * Cells: count of editable params, params are numeric (float), display is
 * a human label, min/max clamp host-side AND in the setter, entry returns
 * 0 past the end, set() exits unknown indices. */
#include "modules/ftc/submodules/intake.h"

int mpe_module_config_count (void) {
    return 1; /* launcher angle */
}

int mpe_module_config_entry (int index, const char **display, float *value, float *min, float *max) {
    static const char *displays [] = {"Shooter launch elevation (deg)"};
    static const float mins [] = {INTAKE_LAUNCH_ELEV_MIN};
    static const float maxs [] = {INTAKE_LAUNCH_ELEV_MAX};
    if ((index < 0) || (index >= (int) (sizeof (displays) / sizeof (displays [0])))) { return 0; }
    if (display) { *display = displays [index]; }
    if (value) { *value = intake_launch_elev_default (); }
    if (min) { *min = mins [index]; }
    if (max) { *max = maxs [index]; }
    return 1;
}

void mpe_module_config_set (int index, float value) {
    if (index == 0) { intake_launch_elev_default_set (value); }
}
