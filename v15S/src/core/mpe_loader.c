#include "mpe_loader.h"
#include "mpe_registry.h"
#include <dlfcn.h>
#include <string.h>
#include <stdio.h>

#define MPE_MAX_HANDLES 32
static struct { void *h; char path[512]; const mpe_module_desc_t *desc; } s_h[MPE_MAX_HANDLES];
static int s_n = 0;

int mpe_loader_load(const char *path, char *errbuf, int errlen) {
    if (!path || !*path) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "empty path");
        return -1;
    }
    for (int i = 0; i < s_n; i++)
        if (strcmp(s_h[i].path, path) == 0) return 0; /* already loaded */
    if (s_n >= MPE_MAX_HANDLES) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "handle table full");
        return -1;
    }
    dlerror();
    void *h = dlopen(path, RTLD_NOW | RTLD_LOCAL);
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
    s_h[s_n].h = h;
    snprintf(s_h[s_n].path, sizeof(s_h[s_n].path), "%s", path);
    s_h[s_n].desc = desc;
    s_n++;
    return 0;
}

int mpe_loader_unload(const char *path_or_name) {
    if (!path_or_name) return -1;
    for (int i = 0; i < s_n; i++) {
        const char *n = s_h[i].desc ? s_h[i].desc->name : "";
        if (strcmp(s_h[i].path, path_or_name) == 0 || strcmp(n, path_or_name) == 0) {
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
