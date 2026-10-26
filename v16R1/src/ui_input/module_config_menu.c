/* Module config menu (5 key). With a kernel module loaded, the 5 key opens
 * the module's own parameter menu — the 6 menu, but per module system. The
 * contract is resolved by dlsym on the loaded bundle handle at CALL time
 * (mirrors term_ftc.c/gamepad_drive.c bundle identity: ecosystem bundle
 * first, then the single-module plugin path); an absent contract makes
 * available() false and key 5 inert, so the scene menu's 5 (reset) is
 * untouched.
 *
 * Polished bindings (DESPOT-2026-10-23):
 *   1..n  edit parameter n (dialog prefilled with the live value)
 *   9     reset all parameters to the module's stock values (optional
 *         contract: absent -> logs, does nothing)
 *   0/6   close (6 mirrors the engine config menu toggle, 0 = conventional
 *         back from a list) */
#include "../mpe_engine.h"
#include "../core/mpe_loader.h"
#include "module_config_menu.h"
#include <stdio.h>
#include <string.h>
#define MCM_HANDLE_PRIMARY "mfs-simulator"
#define MCM_HANDLE_MODULE "ecosystem/mfs/plugins/mpe_ftc.so"
static int module_config_menu_level = 0;
static int module_config_menu_selected = -1;
typedef int (*mcm_count_fn) (void);
typedef int (*mcm_entry_fn) (int, const char **, float *, float *, float *);
typedef void (*mcm_set_fn) (int, float);
typedef int (*mcm_default_fn) (int, float *);
typedef void (*mcm_reset_fn) (void);
static void *mcm_sym (const char *sym) {
    void *p = mpe_loader_symbol (MCM_HANDLE_PRIMARY, sym);
    if (p)
        return p;
    return mpe_loader_symbol (MCM_HANDLE_MODULE, sym);
}
bool module_config_menu_is_open (void) {return module_config_menu_level > 0;}
bool module_config_menu_available (void) {
    mcm_count_fn count = (mcm_count_fn) mcm_sym ("mpe_module_config_count");
    return count && (count () > 0);
}
void module_config_menu_open (void) {
    module_config_menu_level = 1;
    module_config_menu_selected = -1;
}
void module_config_menu_close (void) {
    module_config_menu_level = 0;
    module_config_menu_selected = -1;
}
void module_config_menu_key_press (int key_number) {
    if (module_config_menu_level == 0) { return; }
    /* 6 closes (mirror of the engine config menu), 0 closes too. */
    if ((key_number == 6) || (key_number == 0)) {
        module_config_menu_close ();
        return;
    }
    if (key_number == 9) {
        /* Reset-all: optional contract; anything over n is the reset key
         * (params beyond n can never be selected by number). */
        mcm_count_fn count = (mcm_count_fn) mcm_sym ("mpe_module_config_count");
        int n = count ? count () : 0;
        if (key_number <= n) {
            mcm_default_fn def = (mcm_default_fn) mcm_sym ("mpe_module_config_default");
            mcm_set_fn set = (mcm_set_fn) mcm_sym ("mpe_module_config_set");
            if (def && set) {
                float dv = 0.0f;
                if (def (key_number - 1, &dv)) { set (key_number - 1, dv); }
            }
        } else {
            mcm_reset_fn reset = (mcm_reset_fn) mcm_sym ("mpe_module_config_reset");
            if (reset) {
                reset ();
                event_log_push (0, "module params reset to stock");
            } else {
                event_log_push (2, "module exposes no stock reset");
            }
        }
        return;
    }
    if (module_config_menu_level == 1) {
        /* Parameter list: number selects a param to edit. */
        mcm_count_fn count = (mcm_count_fn) mcm_sym ("mpe_module_config_count");
        int n = count ? count () : 0;
        if ((key_number >= 1) && (key_number <= n)) { module_config_menu_selected = key_number - 1; }
    }
}
void module_config_menu_update (GtkWidget *parent_window) {
    /* Same edit flow as config_menu_update: one dialog per selection, then
     * the selection clears so the dialog cannot re-open next frame. */
    if ((module_config_menu_level == 0) || (module_config_menu_selected < 0)) { return; }
    mcm_count_fn count = (mcm_count_fn) mcm_sym ("mpe_module_config_count");
    mcm_entry_fn entry = (mcm_entry_fn) mcm_sym ("mpe_module_config_entry");
    mcm_set_fn set = (mcm_set_fn) mcm_sym ("mpe_module_config_set");
    if (!count || !entry || !set) { module_config_menu_close (); return; }
    int selected = module_config_menu_selected;
    module_config_menu_selected = -1;
    if (selected >= count ()) { return; }
    const char *display = NULL;
    float value = 0.0f, vmin = 0.0f, vmax = 0.0f;
    if (!entry (selected, &display, &value, &vmin, &vmax)) { return; }
    float new_value = open_numerical_input_dialog (parent_window, display, value);
    if (new_value < vmin) { new_value = vmin; }
    if (new_value > vmax) { new_value = vmax; }
    set (selected, new_value);
    char logbuf [160];
    snprintf (logbuf, sizeof (logbuf), "module: %s -> %.4g", display ? display : "param", (double) new_value);
    event_log_push (0, logbuf);
}
void module_config_menu_render (char *buffer, size_t buffer_size) {
    if (module_config_menu_level == 0) {
        buffer [0] = '\0';
        return;
    }
    mcm_count_fn count = (mcm_count_fn) mcm_sym ("mpe_module_config_count");
    mcm_entry_fn entry = (mcm_entry_fn) mcm_sym ("mpe_module_config_entry");
    mcm_default_fn def = (mcm_default_fn) mcm_sym ("mpe_module_config_default");
    mcm_reset_fn reset = (mcm_reset_fn) mcm_sym ("mpe_module_config_reset");
    size_t offset = 0;
    offset += snprintf (buffer, buffer_size, "-- Module Menu (%s) --\n", MCM_HANDLE_PRIMARY);
    if (!count || !entry) {
        snprintf (buffer + offset, buffer_size - offset, "module exposes no mpe_module_config contract\n0/6: Close");
        return;
    }
    int n = count ();
    for (int i = 0; i < n; i++) {
        const char *display = NULL;
        float value = 0.0f, vmin = 0.0f, vmax = 0.0f;
        if (!entry (i, &display, &value, &vmin, &vmax)) { break; }
        float stock = 0.0f;
        int have_stock = def ? def (i, &stock) : 0;
        offset += snprintf (buffer + offset, buffer_size - offset, "%d: %s = %.4g (min %.4g max %.4g%s)",
                            i + 1, display ? display : "?", value, vmin, vmax,
                            have_stock ? ", stock " : "");
        if (have_stock) { offset += snprintf (buffer + offset, buffer_size - offset, " %.4g)", stock); }
        else { offset += snprintf (buffer + offset, buffer_size - offset, ")"); }
        if ((i + 1) < n) { offset += snprintf (buffer + offset, buffer_size - offset, "\n"); }
    }
    snprintf (buffer + offset, buffer_size - offset, "\n1-%d edit | %s | 0/6 close", n,
              (reset && (n < 9)) ? "9 reset all" : "5/6: back");
}
