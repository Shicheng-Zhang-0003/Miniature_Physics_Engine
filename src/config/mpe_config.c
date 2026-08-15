/* MPE_TASK_28_CONFIG_IMPL_BEGIN */
#include "mpe_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>

/* ==================================================================
 * MPE Config Store Implementation
 * ================================================================== */

const char *mpe_config_category_name(param_category cat) {
    switch (cat) {
        case CAT_WORLD:         return "world";
        case CAT_TIMESTEP:      return "timestep";
        case CAT_SLEEP:         return "sleep";
        case CAT_SOLVER:        return "solver";
        case CAT_DEPENETRATION: return "depenetration";
        case CAT_BROADPHASE:    return "broadphase";
        case CAT_JOINTS:        return "joints";
        case CAT_BOUNDARY:      return "boundary";
        case CAT_SPAWNER:       return "spawner";
        case CAT_BODY_DEFAULTS: return "body_defaults";
        case CAT_CAMERA:        return "camera";
        case CAT_RENDER:        return "render";
        case CAT_UI:            return "ui";
        default:                return "unknown";
    }
}

static double param_read_double(const mpe_param *param) {
    if ((!param) || (!param->storage)) {return 0.0;}
    switch (param->type) {
        case P_FLOAT: return (double)(*(float *)param->storage);
        case P_INT:   return (double)(*(int *)param->storage);
        case P_BOOL:  return (*(bool *)param->storage) ? 1.0 : 0.0;
        default:      return 0.0;
    }
}

static bool param_write_double(const mpe_param *param, double value) {
    if ((!param) || (!param->storage)) {return false;}
    bool clamped = false;
    if (value < param->min) {value = param->min; clamped = true;}
    if (value > param->max) {value = param->max; clamped = true;}
    switch (param->type) {
        case P_FLOAT: *(float *)param->storage = (float)value; break;
        case P_INT:   *(int *)param->storage = (int)value; break;
        case P_BOOL:  *(bool *)param->storage = (value != 0.0); break;
        default: break;
    }
    return clamped;
}

const mpe_param *mpe_config_find(const char *key) {
    if (!key) {return NULL;}
    for (size_t i = 0; i < g_registry_count; i++) {
        if (strcmp(g_registry[i].key, key) == 0) {return &g_registry[i];}
    }
    return NULL;
}

void mpe_config_init(void) {
    memset(&g_cfg, 0, sizeof(g_cfg));
    for (size_t i = 0; i < g_registry_count; i++) {
        param_write_double(&g_registry[i], g_registry[i].def);
    }
}

void mpe_config_reset_defaults(void) {
    mpe_config_init();
}

bool mpe_config_get_float(const char *key, float *out) {
    const mpe_param *param = mpe_config_find(key);
    if ((!param) || (!out)) {return false;}
    *out = (float)param_read_double(param);
    return true;
}

bool mpe_config_get_int(const char *key, int *out) {
    const mpe_param *param = mpe_config_find(key);
    if ((!param) || (!out)) {return false;}
    *out = (int)param_read_double(param);
    return true;
}

bool mpe_config_get_bool(const char *key, bool *out) {
    const mpe_param *param = mpe_config_find(key);
    if ((!param) || (!out)) {return false;}
    *out = (param_read_double(param) != 0.0);
    return true;
}

bool mpe_config_set_float(const char *key, float value) {
    const mpe_param *param = mpe_config_find(key);
    if (!param) {return false;}
    bool clamped = param_write_double(param, (double)value);
    return !clamped;
}

bool mpe_config_set_int(const char *key, int value) {
    const mpe_param *param = mpe_config_find(key);
    if (!param) {return false;}
    bool clamped = param_write_double(param, (double)value);
    return !clamped;
}

bool mpe_config_set_bool(const char *key, bool value) {
    const mpe_param *param = mpe_config_find(key);
    if (!param) {return false;}
    param_write_double(param, value ? 1.0 : 0.0);
    return true;
}

size_t mpe_config_count_by_category(param_category cat) {
    size_t count = 0;
    for (size_t i = 0; i < g_registry_count; i++) {
        if (g_registry[i].category == cat) {count++;}
    }
    return count;
}

/* Fill a caller-provided buffer with params of a category.
 * Matches header: size_t get_by_category(cat, out_params, max_params). */
size_t mpe_config_get_by_category(param_category cat, const mpe_param **out_params, size_t max_params) {
    if ((!out_params) || (max_params == 0)) {return 0;}
    size_t filled = 0;
    for (size_t i = 0; (i < g_registry_count) && (filled < max_params); i++) {
        if (g_registry[i].category == cat) {
            out_params[filled++] = &g_registry[i];
        }
    }
    return filled;
}

static void ensure_parent_dir(const char *path) {
    char copy[512];
    strncpy(copy, path, sizeof(copy) - 1);
    copy[sizeof(copy) - 1] = '\0';
    char *last_slash = strrchr(copy, '/');
    if ((last_slash) && (last_slash != copy)) {
        *last_slash = '\0';
        mkdir(copy, 0755);
    }
}

bool mpe_config_save(const char *path) {
    if (!path) {return false;}
    ensure_parent_dir(path);
    FILE *file = fopen(path, "w");
    if (!file) {return false;}
    time_t now = time(NULL);
    struct tm *local_time = localtime(&now);
    char stamp[64];
    strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", local_time);
    fprintf(file, "# MPE Engine Configuration\n");
    fprintf(file, "# saved:   %s\n\n", stamp);
    for (int cat = 0; cat <= CAT_UI; cat++) {
        bool wrote_header = false;
        for (size_t i = 0; i < g_registry_count; i++) {
            if ((int)g_registry[i].category != cat) {continue;}
            if (!wrote_header) {
                fprintf(file, "[%s]\n", mpe_config_category_name((param_category)cat));
                wrote_header = true;
            }
            const char *dot = strchr(g_registry[i].key, '.');
            const char *field = dot ? (dot + 1) : g_registry[i].key;
            if (g_registry[i].type == P_FLOAT) {
                fprintf(file, "%s = %.6f\n", field, param_read_double(&g_registry[i]));
            } else {
                fprintf(file, "%s = %d\n", field, (int)param_read_double(&g_registry[i]));
            }
        }
        if (wrote_header) {fprintf(file, "\n");}
    }
    fclose(file);
    return true;
}

static char *term_trim(char *str) {
    while ((*str == ' ') || (*str == '\t')) {str++;}
    char *end = str + strlen(str) - 1;
    while ((end > str) && ((*end == ' ') || (*end == '\t') || (*end == '\n') || (*end == '\r'))) {
        *end = '\0';
        end--;
    }
    return str;
}

bool mpe_config_load(const char *path) {
    if (!path) {return false;}
    FILE *file = fopen(path, "r");
    if (!file) {return false;}
    char line[512];
    char section[64] = "";
    while (fgets(line, sizeof(line), file)) {
        char *cursor = term_trim(line);
        if ((*cursor == '\0') || (*cursor == '#')) {continue;}
        if (*cursor == '[') {
            char *close = strchr(cursor, ']');
            if (close) {
                *close = '\0';
                strncpy(section, cursor + 1, sizeof(section) - 1);
                section[sizeof(section) - 1] = '\0';
            }
            continue;
        }
        char *equals = strchr(cursor, '=');
        if (!equals) {continue;}
        *equals = '\0';
        char *key_part = term_trim(cursor);
        char *value_part = term_trim(equals + 1);
        char full_key[128];
        if (section[0] != '\0') {
            snprintf(full_key, sizeof(full_key), "%s.%s", section, key_part);
        } else {
            snprintf(full_key, sizeof(full_key), "%s", key_part);
        }
        const mpe_param *param = mpe_config_find(full_key);
        if (!param) {continue;}
        char *endptr = NULL;
        double parsed = strtod(value_part, &endptr);
        if ((endptr == value_part) || (!isfinite(parsed))) {continue;}
        param_write_double(param, parsed);
    }
    fclose(file);
    return true;
}
/* MPE_TASK_28_CONFIG_IMPL_END */

/* MPE_TASK_39_FIX_BACKUP_HELPERS_BEGIN */
bool mpe_config_save_backup(const char *path) {
    return mpe_config_save(path);
}

bool mpe_config_load_backup(const char *path) {
    return mpe_config_load(path);
}
/* MPE_TASK_39_FIX_BACKUP_HELPERS_END */
