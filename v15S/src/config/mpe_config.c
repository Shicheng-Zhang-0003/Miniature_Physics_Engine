/* MPE_TASK_28_CONFIG_IMPL_BEGIN */
#include "mpe_config.h"
#include "core/mpe_platform.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#ifndef MPE_OS_WINDOWS
#include <unistd.h>
#endif
#include <math.h>
#include <time.h>
#include <sys/stat.h>
/* ==================================================================
 * MPE Config Store Implementation
 * ================================================================== */
const char *mpe_config_category_name (param_category cat) {
    switch (cat) {
        case cat_world:
        return "world";
        case cat_timestep:
        return "timestep";
        case cat_sleep:
        return "sleep";
        case cat_solver:
        return "solver";
        case cat_depenetration:
        return "depenetration";
        case cat_broadphase:
        return "broadphase";
        case cat_joints:
        return "joints";
        case cat_boundary:
        return "boundary";
        case cat_spawner:
        return "spawner";
        case cat_body_defaults:
        return "body_defaults";
        case cat_camera:
        return "camera";
        case cat_render:
        return "render";
        case cat_ui:
        return "ui";
        default:
        return "unknown";
    }
}
static double param_read_double (const mpe_param *param) {
    if ((!param) || (!param -> storage)) {
        return 0.0;
    }
    switch (param -> type) {
        case p_float:
        return (double) (*(float *) param -> storage);
        case p_int:
        return (double) (*(int *) param -> storage);
        case p_bool:
        return (*(bool *) param -> storage) ? 1.0 : 0.0;
        default:
        return 0.0;
    }
}
static bool param_write_double (const mpe_param *param, double value) {
    if ((!param) || (!param -> storage)) {
        return false;
    }
    bool clamped = false;
    if (value < param -> min) {
        value = param -> min;
        clamped = true;
    }
    if (value > param -> max) {
        value = param -> max;
        clamped = true;
    }
    switch (param -> type) {
        case p_float:
        *(float *) param -> storage = (float) value;
        break;
        case p_int:
        *(int *) param -> storage = (int) value;
        break;
        case p_bool:
        *(bool *) param -> storage = (value != 0.0);
        break;
        default:
        break;
    }
    return clamped;
}
const mpe_param *mpe_config_find (const char *key) {
    if (!key) {
        return NULL;
    }
    for (size_t i = 0; i < g_registry_count; i++) {
        if (strcmp (g_registry [i].key, key) == 0) {
            return &g_registry [i];
        }
    }
    return NULL;
} /* DESPOT-2026-09-29: see mpe_config_ensure_ready(). */
static bool g_config_ready = false;
void mpe_config_init (void) {
    memset (&g_cfg, 0, sizeof (g_cfg));
    g_config_ready = true;
    for (size_t i = 0; i < g_registry_count; i++) {
        param_write_double (&g_registry [i], g_registry [i].def);
    }
    /* Validate registry storage pointers lie inside g_cfg. A struct layout
     * change that silently breaks the registry now fails loudly at startup
     * instead of corrupting unrelated memory. */
    const char *base = (const char *) &g_cfg;
    const char *end = base + sizeof (g_cfg);
    for (size_t i = 0; i < g_registry_count; i++) {
        const char *p = (const char *) g_registry [i].storage;
        if (!p || p < base || p >= end) {
            fprintf (stderr, "[config] registry entry '%s' has invalid storage pointer\n",
                     g_registry [i].key ? g_registry [i].key : "(null)");
        }
    }
} /* DESPOT-2026-09-29: the init-order guard.
 *
 * `g_cfg` is a plain global, so it is ALL ZERO until mpe_config_init() runs.
 * Body materials are stamped from it at CONSTRUCTION time
 * (rigidbody.c: restitution/friction = g_cfg.body_defaults.*), and
 * physics_world_init() does NOT retro-fit them onto bodies that already exist.
 *
 * So any body created before mpe_config_init() is permanently built with zero
 * friction and zero restitution. That is not a subtle tuning error: a body
 * with no friction has no tangential traction and no spin-down torque, so it
 * rolls and spins forever.
 *
 * The game hit exactly this. root_gtk.c built the entire default scene from
 * when_realised() (scene_init_default) while mpe_config_init() sat 24 lines
 * later in app_activate(). The user-visible signature was unmistakable once
 * reported: physics correct INSIDE the F10 validation region and wrong
 * everywhere outside it -- because runtime-spawned content is created after
 * the config exists, and default-scene content was created before it. A
 * spatial boundary around a creation-time difference is the fingerprint of
 * this bug specifically.
 *
 * This function makes the ordering unobservable: idempotent, and callable from
 * anywhere that is about to build a body. */
void mpe_config_ensure_ready (void) {
    if (g_config_ready) {
        return;
    }
    mpe_config_init ();
    g_config_ready = true;
}
bool mpe_config_is_ready (void) {return g_config_ready;}
/* DESPOT-2026-09-29: test hook ONLY.
 *
 * Pretends the process has not initialised its config yet, so a suite test can
 * reproduce the exact precondition that broke the game (a body constructed
 * against a zeroed g_cfg) instead of merely asserting the happy path. Without
 * this the regression test is decorative: mpe_test_begin() initialises the
 * config, so a body built inside a test always looks healthy and the test
 * cannot fail even with the fix removed. Not for production use. */
void mpe_config_force_unready_for_test (void) {
    memset (&g_cfg, 0, sizeof (g_cfg));
    g_config_ready = false;
}
void mpe_config_reset_defaults (void) {mpe_config_init ();}
bool mpe_config_get_float (const char *key, float *out) {
    const mpe_param *param = mpe_config_find (key);
    if ((!param) || (!out) || param -> type != p_float) {
        return false;
    }
    * out = (float) param_read_double (param);
    return true;
}
bool mpe_config_get_int (const char *key, int *out) {
    const mpe_param *param = mpe_config_find (key);
    if ((!param) || (!out) || param -> type != p_int) {
        return false;
    }
    * out = (int) param_read_double (param);
    return true;
}
bool mpe_config_get_bool (const char *key, bool *out) {
    const mpe_param *param = mpe_config_find (key);
    if ((!param) || (!out) || param -> type != p_bool) {
        return false;
    }
    * out = (param_read_double (param) != 0.0);
    return true;
}
bool mpe_config_set_float (const char *key, float value) {
    const mpe_param *param = mpe_config_find (key);
    if (!param || param -> type != p_float || !isfinite (value)) {
        return false;
    }
    bool clamped = param_write_double (param, (double) value);
    return !clamped;
}
bool mpe_config_set_int (const char *key, int value) {
    const mpe_param *param = mpe_config_find (key);
    if (!param || param -> type != p_int) {
        return false;
    }
    bool clamped = param_write_double (param, (double) value);
    return !clamped;
}
bool mpe_config_set_bool (const char *key, bool value) {
    const mpe_param *param = mpe_config_find (key);
    if (!param || param -> type != p_bool) {
        return false;
    }
    param_write_double (param, value ? 1.0 : 0.0);
    return true;
}
size_t mpe_config_count_by_category (param_category cat) {
    size_t count = 0;
    for (size_t i = 0; i < g_registry_count; i++) {
        if (g_registry [i].category == cat) {
            count++;
        }
    }
    return count;
} /* Fill a caller-provided buffer with params of a category.
 * Matches header: size_t get_by_category(cat, out_params, max_params). */
size_t mpe_config_get_by_category (param_category cat, const mpe_param **out_params, size_t max_params) {
    if ((!out_params) || (max_params == 0)) {
        return 0;
    }
    size_t filled = 0;
    for (size_t i = 0; (i < g_registry_count) && (filled < max_params); i++) {
        if (g_registry [i].category == cat) {
            out_params [filled++] = &g_registry [i];
        }
    }
    return filled;
}
static void ensure_parent_dir (const char *path) {
    char copy [512];
    strncpy (copy, path, sizeof (copy) - 1);
    copy [sizeof (copy) - 1] = '\0';
    /* mkdir -p: create every ancestor component, not just the leaf.
     * 0700 for config (no world-readable secrets); errors checked by
     * caller via subsequent fopen failure. Truncation guarded. */
    if (strlen (path) >= sizeof (copy)) {
        return;
    }
    for (char *p = copy + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            (void) mkdir (copy, 0700);
            *p = '/';
        }
    }
    char *last_slash = strrchr (copy, '/');
    if ((last_slash) && (last_slash != copy)) {
        *last_slash = '\0';
        (void) mkdir (copy, 0700);
    }
}
bool mpe_config_save (const char *path) {
    if (!path) {
        return false;
    }
    ensure_parent_dir (path);
    /* R3-03: Atomic write. Write to a temporary file first, then
     * atomically rename over the target. */
    char tmp_path [512];
    int tmp_len = snprintf (tmp_path, sizeof (tmp_path), "%s.tmp", path);
    if (tmp_len < 0 || (size_t) tmp_len >= sizeof (tmp_path)) {
        return false;
    }
    FILE *file = fopen (tmp_path, "w");
    if (!file) {
        return false;
    }
    time_t now = time (NULL);
    struct tm tm_buf;
    struct tm *local_time = localtime_r (&now, &tm_buf);
    char stamp [64];
    if (local_time) {
        strftime (stamp, sizeof (stamp), "%Y-%m-%d %H:%M:%S", local_time);
    } else {
        snprintf (stamp, sizeof (stamp), "unknown-time");
    }
    fprintf (file, "# MPE Engine Configuration\n");
    fprintf (file, "# saved:   %s\n\n", stamp);
    for (int cat = 0; cat <= cat_ui; cat++) {
        bool wrote_header = false;
        for (size_t i = 0; i < g_registry_count; i++) {
            if ((int) g_registry [i].category != cat) {
                continue;
            }
            if (!wrote_header) {
                fprintf (file, "[%s]\n", mpe_config_category_name ((param_category) cat));
                wrote_header = true;
            }
            const char *dot = strchr (g_registry [i].key, '.');
            const char *field = dot ? (dot + 1) : g_registry [i].key;
            if (g_registry [i].type == p_float) {
                /* FIX-AUDIT-DESPOT: %.6f truncated to 6 decimals (gravity
                 * -9.81 survives, but values like 0.0001 print as 0.000100
                 * and high-precision tunables never round-tripped). %.9g
                 * carries 9 significant digits = exact float round-trip. */
                fprintf (file, "%s = %.9g\n", field, param_read_double (&g_registry [i]));
            } else {
                fprintf (file, "%s = %d\n", field, (int) param_read_double (&g_registry [i]));
            }
        }
        if (wrote_header) {
            fprintf (file, "\n");
        }
    }
    /* FIX-AUDIT-DESPOT: fsync the file before rename (like scene_saving:
     * a crash between write and flush must not publish a torn config).
     * Directory sync is best-effort with a warning (bytes are durable).
     * DESPOT-2026-10-04: also fail on a mid-stream fprintf error (ENOSPC
     * mid-write flushes cleanly but the content is short) — ferror catches
     * what fflush cannot. */
    if (ferror (file) || fflush (file) != 0) {
        fclose (file);
        remove (tmp_path);
        return false;
    }
    {
        int fd = fileno (file);
        if (fd < 0 || fsync (fd) != 0) {
            fclose (file);
            remove (tmp_path);
            return false;
        }
    }
    if (fclose (file) != 0) {
        remove (tmp_path);
        return false;
    }
    /* R3-03: Atomic rename over the target */
    if (rename (tmp_path, path) != 0) {
        remove (tmp_path);
        return false;
    }
    {
        char parent [512];
        strncpy (parent, path, sizeof (parent) - 1);
        parent [sizeof (parent) - 1] = '\0';
        char *slash = strrchr (parent, '/');
        if (!slash) {
            memcpy (parent, ".", 2);
        } else if (slash == parent) {
            slash [1] = '\0';
        } else {
            *slash = '\0';
        }
        int dir_fd = open (parent, O_RDONLY);
        if (dir_fd >= 0) {
            if (fsync (dir_fd) != 0) {
                fprintf (stderr, "[config] warning: parent-directory sync failed for '%s'\n", path);
            }
            if (close (dir_fd) != 0) {
                fprintf (stderr, "[config] warning: parent-directory close failed for '%s'\n", path);
            }
        }
    }
    return true;
}
static char *term_trim (char *str) {
    if (!str) {
        return str;
    }
    while ((*str == ' ') || (*str == '\t')) {
        str++;
    }
    size_t len = strlen (str);
    if (len == 0) {
        return str;
    }
    char *end = str + len - 1;
    while ((end > str) && ((*end == ' ') || (*end == '\t') || (*end == '\n') || (*end == '\r'))) {
        *end = '\0';
        end--;
    }
    return str;
} /* DESPOT-2026-10-04: torture-live probe without an include cycle.
 * long_run_validation.c includes mpe_config.h; including its header back
 * here would cycle. The three ints are plain globals — declare, don't
 * include. If the validation TU is ever not linked (unit harnesses that
 * stub the world), the linker would fail; so the probe is defensive:
 * torture is reported only when the linked flags say so, and a missing
 * symbol falls back to "not live" via weak linkage where supported. */
#if defined(__GNUC__) || defined(__clang__)
extern int long_run_validation_active __attribute__ ((weak));
extern int long_run_validation_restore_config __attribute__ ((weak));
extern int long_run_validation_is_torture __attribute__ ((weak));
#else
extern int long_run_validation_active;
extern int long_run_validation_restore_config;
extern int long_run_validation_is_torture;
#endif
static int mpe_config_torture_live_probe (void) {
#if defined(__GNUC__) || defined(__clang__)
    /* A harness that links config without the validation TU gets NULL
     * weak symbols: torture trivially not live, never a link error. */
    if (!&long_run_validation_active || !&long_run_validation_is_torture || !&long_run_validation_restore_config) {
        return 0;
    }
#endif
    return (long_run_validation_active && long_run_validation_is_torture) || (long_run_validation_is_torture != 0) ||
           (long_run_validation_restore_config != 0);
}
static unsigned long s_torture_save_blocked = 0;
unsigned long mpe_config_torture_save_blocked_total (void) {return s_torture_save_blocked;}
bool mpe_config_save_guarded (const char *path) {
    if (mpe_config_torture_live_probe ()) {
        s_torture_save_blocked++;
        fprintf (stderr,
                 "[config] REFUSED save to '%s': F11 torture values are live in g_cfg "
                 "(blocked #%lu). Run the validation to completion or restart; "
                 "the clean config is untouched.\n",
                 path ? path : "(null)", s_torture_save_blocked);
        return false;
    }
    return mpe_config_save (path);
}
bool mpe_config_load (const char *path) {
    if (!path) {
        return false;
    }
    FILE *file = fopen (path, "r");
    if (!file) {
        return false;
    }
    char line [512];
    char section [64] = "";
    while (fgets (line, sizeof (line), file)) {
        /* Truncation detection: line without newline was split; consume
         * the rest so a split key never parses as two keys. */
        if (!strchr (line, '\n') && !feof (file)) {
            int ch;
            while ((ch = fgetc (file)) != '\n' && ch != EOF) {
            }
            continue;
        }
        char *cursor = term_trim (line);
        if ((*cursor == '\0') || (*cursor == '#')) {
            continue;
        }
        if (*cursor == '[') {
            char *close = strchr (cursor, ']');
            if (close) {
                *close = '\0';
                strncpy (section, cursor + 1, sizeof (section) - 1);
                section [sizeof (section) - 1] = '\0';
            }
            continue;
        }
        char *equals = strchr (cursor, '=');
        if (!equals) {
            continue;
        }
        * equals = '\0';
        char *key_part = term_trim (cursor);
        char *value_part = term_trim (equals + 1);
        char full_key [128];
        int key_len;
        if (section [0] != '\0') {
            key_len = snprintf (full_key, sizeof (full_key), "%s.%s", section, key_part);
        } else {
            key_len = snprintf (full_key, sizeof (full_key), "%s", key_part);
        }
        /* FIX-AUDIT-DESPOT: unchecked snprintf truncation could forge a
         * shorter-but-valid key ("abc...x" -> "abc") and mis-assign an
         * unrelated tunable. Drop overlong keys instead of guessing. */
        if (key_len < 0 || (size_t) key_len >= sizeof (full_key)) {
            fprintf (stderr, "[config] warning: overlong key dropped (section='%s')\n", section);
            continue;
        }
        const mpe_param *param = mpe_config_find (full_key);
        if (!param) {
            continue;
        }
        char *endptr = NULL;
        double parsed = strtod (value_part, &endptr);
        if ((endptr == value_part) || (!isfinite (parsed))) {
            continue;
        }
        /* DESPOT-2026-10-04: strtod accepts "12abc" as 12. A suffixed
         * value is a corrupt line, not a number — drop it loudly instead
         * of silently adopting the prefix. */
        while ((*endptr == ' ') || (*endptr == '\t')) {
            endptr++;
        }
        if (*endptr != '\0') {
            fprintf (stderr, "[config] warning: trailing garbage dropped (key='%s', value='%s')\n", full_key,
                     value_part);
            continue;
        }
        /* FIX-AUDIT-DESPOT: clamping was silent (a hostile/hand-edited file
         * could halve gravity with no trace). Log every clamped load. */
        if (param_write_double (param, parsed)) {
            fprintf (stderr, "[config] '%s' clamped to [%g, %g] (file had %g)\n", full_key, param -> min, param -> max,
                     parsed);
        }
    }
    fclose (file);
    return true;
} /* MPE_TASK_28_CONFIG_IMPL_END */
/* MPE_TASK_39_FIX_BACKUP_HELPERS_BEGIN */
bool mpe_config_save_backup (const char *path) {return mpe_config_save (path);}
bool mpe_config_load_backup (const char *path) {return mpe_config_load (path);}
/* MPE_TASK_39_FIX_BACKUP_HELPERS_END */
