#include "mpe_loader.h"
#include "mpe_registry.h"
#include <dlfcn.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>

#define MPE_MAX_HANDLES 32
static struct { void *h; char path[PATH_MAX]; const mpe_module_desc_t *desc; int attachments; } s_h[MPE_MAX_HANDLES];
static int s_n = 0;

static int plugin_path_is_confined(const char *path, char resolved[PATH_MAX]) {
    const char *prefix_a = "plugins/";
    const char *prefix_b = "./plugins/";
    if ((!path) || (strncmp(path, prefix_a, strlen(prefix_a)) != 0 &&
                    strncmp(path, prefix_b, strlen(prefix_b)) != 0)) return 0;
    const char *base = path + ((path[0] == '.') ? strlen(prefix_b) : strlen(prefix_a));
    if (!*base || strchr(base, '/') || strstr(base, "..") || strlen(base) < 4 ||
        strcmp(base + strlen(base) - 3, ".so") != 0) return 0;
    char root[PATH_MAX];
    if (!realpath("plugins", root) || !realpath(path, resolved)) return 0;
    size_t root_len = strlen(root);
    return strncmp(resolved, root, root_len) == 0 && resolved[root_len] == '/';
}

int mpe_loader_load(const char *path, char *errbuf, int errlen) {
    if (!path || !*path) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "empty path");
        return -1;
    }
    char resolved[PATH_MAX];
    if (!plugin_path_is_confined(path, resolved)) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "path must resolve inside plugins/<name>.so");
        return -1;
    }
    for (int i = 0; i < s_n; i++)
        if (strcmp(s_h[i].path, resolved) == 0) return 0; /* already loaded */
    if (s_n >= MPE_MAX_HANDLES) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "handle table full");
        return -1;
    }
    dlerror();
    void *h = dlopen(resolved, RTLD_NOW | RTLD_LOCAL);
    if (!h) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "%s", dlerror());
        return -1;
    }
    const mpe_module_desc_t *desc =
        (const mpe_module_desc_t *)dlsym(h, "mpe_module_desc");
    const char *sym_err = dlerror();
    if (sym_err || !desc) {
        if (errbuf && errlen > 0)
            snprintf(errbuf, (size_t)errlen, "missing mpe_module_desc: %s", sym_err ? sym_err : "null");
        dlclose(h);
        return -1;
    }
    if (desc->abi != MPE_MODULE_ABI || !desc->name) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "ABI/name mismatch");
        dlclose(h);
        return -1;
    }
    if (mpe_register_module(desc) < 0) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "registry full/dup");
        dlclose(h);
        return -1;
    }
    /* If the module provides a pair handler via optional exported
     * symbol mpe_pair_* it self-registers in its constructor; nothing
     * more to do here (keeps loader generic across kinds). */
    if (strlen(resolved) >= sizeof(s_h[s_n].path)) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "path too long");
        dlclose(h);
        return -1;
    }
    s_h[s_n].h = h;
    snprintf(s_h[s_n].path, sizeof(s_h[s_n].path), "%s", resolved);
    s_h[s_n].desc = desc;
    s_h[s_n].attachments = 0;
    s_n++;
    return 0;
}

int mpe_loader_unload(const char *path_or_name) {
    if (!path_or_name) return -1;
    char resolved[PATH_MAX];
    const char *identity = path_or_name;
    if (strchr(path_or_name, '/')) {
        if (!plugin_path_is_confined(path_or_name, resolved)) return -1;
        identity = resolved;
    }
    for (int i = 0; i < s_n; i++) {
        const char *n = s_h[i].desc ? s_h[i].desc->name : "";
        if (strcmp(s_h[i].path, identity) == 0 || strcmp(n, identity) == 0) {
            if (s_h[i].attachments > 0) return -1;
            /* Unregister before dlclose so no future dispatch jumps into
             * unmapped code. Worlds holding the module stay valid: tick
             * hooks are looked up per-tick via registry, pair handlers for
             * this .so must self-unregister or they are dropped here by
             * module name where possible. Detach from primary to avoid
             * dangling tick pointers. */
            char modname[128];
            snprintf(modname, sizeof(modname), "%s", n ? n : "");
            if (modname[0]) {
                mpe_unregister_module(modname);
                mpe_unregister_broadphase(modname);
                mpe_unregister_solver(modname);
            }
            dlclose(s_h[i].h);
            for (int j = i; j + 1 < s_n; j++) s_h[j] = s_h[j + 1];
            s_n--;
            return 0;
        }
    }
    return -1;
}

int mpe_loader_count(void) { return s_n; }
const char *mpe_loader_path_at(int i) {
    if (i < 0 || i >= s_n) return 0;
    return s_h[i].path;
}

void mpe_loader_retain_module(const void *desc) {
    for (int i = 0; i < s_n; i++) if (s_h[i].desc == desc) { s_h[i].attachments++; return; }
}

void mpe_loader_release_module(const void *desc) {
    for (int i = 0; i < s_n; i++) if (s_h[i].desc == desc) {
        if (s_h[i].attachments > 0) s_h[i].attachments--;
        return;
    }
}
