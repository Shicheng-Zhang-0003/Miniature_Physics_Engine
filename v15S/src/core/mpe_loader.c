#define _GNU_SOURCE /* dladdr */
#include "mpe_loader.h"
#include "mpe_registry.h"
#include "physics_world.h"
#include "../ecosystem/mpe_ecosystem.h"
#include <dlfcn.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>
#include <stdlib.h>

#define MPE_MAX_HANDLES 32
static struct {
    void *h;
    char path[PATH_MAX];
    const mpe_module_desc_t *desc;      /* module .so (NULL for ecosystems) */
    const mpe_ecosystem_desc_t *eco;    /* ecosystem .so (NULL for modules) */
    int is_ecosystem;
    int attachments;
} s_h[MPE_MAX_HANDLES];
static int s_n = 0;

/* Jail: plugins/<name>.so for modules, ecosystem/mfs/<name>.so for
 * ecosystem bundles (both CWD-relative, normally v15S/src). Same
 * traversal-proofing in both roots. */
static int plugin_path_is_confined(const char *path, char resolved[PATH_MAX]) {
    const char *prefix_a = "plugins/";
    const char *prefix_b = "./plugins/";
    const char *prefix_c = "ecosystem/mfs/";
    const char *prefix_d = "./ecosystem/mfs/";
    const char *dir = NULL;
    const char *base = NULL;
    if (path && strncmp(path, prefix_a, strlen(prefix_a)) == 0) {
        dir = "plugins";
        base = path + strlen(prefix_a);
    } else if (path && strncmp(path, prefix_b, strlen(prefix_b)) == 0) {
        dir = "plugins";
        base = path + strlen(prefix_b);
    } else if (path && strncmp(path, prefix_c, strlen(prefix_c)) == 0) {
        dir = "ecosystem/mfs";
        base = path + strlen(prefix_c);
    } else if (path && strncmp(path, prefix_d, strlen(prefix_d)) == 0) {
        dir = "ecosystem/mfs";
        base = path + strlen(prefix_d);
    } else {
        return 0;
    }
    if (!*base || strchr(base, '/') || strstr(base, "..") || strlen(base) < 4 ||
        strcmp(base + strlen(base) - 3, ".so") != 0) return 0;
    char root[PATH_MAX];
    if (!realpath(dir, root) || !realpath(path, resolved)) return 0;
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
    /* Length validated BEFORE any registration (a late failure used to
     * dlclose while the module stayed registered -> dangling dispatch). */
    if (strlen(resolved) >= PATH_MAX - 64) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "path too long");
        return -1;
    }
    for (int i = 0; i < s_n; i++)
        if (strcmp(s_h[i].path, resolved) == 0) return 0; /* already loaded */
    if (s_n >= MPE_MAX_HANDLES) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "handle table full");
        return -1;
    }
    /* Snapshot registry counts so constructor registrations can be rolled
     * back if anything below fails. The lock is NOT held across dlopen:
     * plugin constructors register (locking internally), so holding it
     * would deadlock. Load/unload are tick-boundary admin operations;
     * concurrent loader use from step threads is misuse (documented in
     * mpe_loader.h). */
    int snap_pairs = mpe_registry_pair_count();
    int snap_broad = mpe_registry_broadphase_count();
    int snap_solvers = mpe_registry_solver_count();
    dlerror();
    void *h = dlopen(resolved, RTLD_NOW | RTLD_LOCAL);
    if (!h) {
        if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "%s", dlerror());
        return -1;
    }
    /* Precedence: ecosystem bundles (mpe_ecosystem_desc) win over plain
     * modules. A bundle links its inner modules' objects for code, so it
     * exports their mpe_module_desc too — taking it would register a
     * fragment instead of the bundle. */
    dlerror();
    const mpe_ecosystem_desc_t *eco_first =
        (const mpe_ecosystem_desc_t *)dlsym(h, "mpe_ecosystem_desc");
    const char *sym_err = dlerror();
    if (!sym_err && eco_first) {
        if (eco_first->abi != MPE_ECOSYSTEM_ABI || !eco_first->name) {
            if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "ecosystem ABI/name mismatch");
            dlclose(h);
            return -1;
        }
        if (mpe_ecosystem_register(eco_first) < 0) {
            if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "ecosystem registry full/dup");
            dlclose(h);
            return -1;
        }
        s_h[s_n].h = h;
        snprintf(s_h[s_n].path, sizeof(s_h[s_n].path), "%s", resolved);
        s_h[s_n].desc = NULL;
        s_h[s_n].eco = eco_first;
        s_h[s_n].is_ecosystem = 1;
        s_h[s_n].attachments = 0;
        s_n++;
        return 0;
    }
    dlerror();
    const mpe_module_desc_t *desc =
        (const mpe_module_desc_t *)dlsym(h, "mpe_module_desc");
    sym_err = dlerror();
    if (!sym_err && desc) {
        if (desc->abi != MPE_MODULE_ABI || !desc->name) {
            if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "ABI/name mismatch");
            dlclose(h);
            mpe_registry_truncate_pairs(snap_pairs);
            mpe_registry_truncate_broadphase(snap_broad);
            mpe_registry_truncate_solvers(snap_solvers);
            return -1;
        }
        if (mpe_register_module(desc) < 0) {
            if (errbuf && errlen > 0) snprintf(errbuf, (size_t)errlen, "registry full/dup");
            dlclose(h);
            mpe_registry_truncate_pairs(snap_pairs);
            mpe_registry_truncate_broadphase(snap_broad);
            mpe_registry_truncate_solvers(snap_solvers);
            return -1;
        }
        /* Pair handlers self-register in the plugin constructor; stages
         * register explicitly. Covered by the snapshot above on failure. */
        s_h[s_n].h = h;
        snprintf(s_h[s_n].path, sizeof(s_h[s_n].path), "%s", resolved);
        s_h[s_n].desc = desc;
        s_h[s_n].eco = NULL;
        s_h[s_n].is_ecosystem = 0;
        s_h[s_n].attachments = 0;
        s_n++;
        mpe_registry_set_origin(desc->name, resolved);
        return 0;
    }
    if (errbuf && errlen > 0)
        snprintf(errbuf, (size_t)errlen, "missing mpe_module_desc: %s", sym_err ? sym_err : "null");
    dlclose(h); /* destructor self-unregisters well-behaved plugins */
    mpe_registry_truncate_pairs(snap_pairs);
    mpe_registry_truncate_broadphase(snap_broad);
    mpe_registry_truncate_solvers(snap_solvers);
    return -1;
}

/* Path equality: realpath when possible, basename fallback otherwise. */
static int same_path(const char *a, const char *b) {
    if (!a || !b) return 0;
    if (strcmp(a, b) == 0) return 1;
    char ca[PATH_MAX], cb[PATH_MAX];
    if (realpath(a, ca) && realpath(b, cb) && strcmp(ca, cb) == 0) return 1;
    const char *ba = strrchr(a, '/');
    const char *bb = strrchr(b, '/');
    ba = ba ? ba + 1 : a;
    bb = bb ? bb + 1 : b;
    return strcmp(ba, bb) == 0;
}

static int handle_by_path(const char *path) {
    for (int i = 0; i < s_n; i++) {
        if (same_path(s_h[i].path, path)) return i;
    }
    return -1;
}

/* Which loaded handle owns this desc? Registry-slot origin first (exact
 * for registry copies handed out by find), else the desc's own image.
 * -1 = static/host code that never unloads. */
static int handle_for_desc(const mpe_module_desc_t *d) {
    if (!d) return -1;
    const char *origin = mpe_registry_module_origin(d);
    if (origin) {
        char obuf[256];
        snprintf(obuf, sizeof(obuf), "%s", origin);
        return handle_by_path(obuf);
    }
    Dl_info info;
    if (dladdr((const void *)d, &info) && info.dli_fname) {
        return handle_by_path(info.dli_fname);
    }
    return -1;
}

/* True when fn's code lives inside the plugin at path `resolved`.
 * Well-behaved plugins self-unregister in their destructor; this is the
 * backstop for the rest. dli_fname may be relative — resolve it when
 * possible, else fall back to basename comparison. */
static int fn_in_plugin(mpe_collide_fn fn, const char *resolved) {
    Dl_info info;
    if (!fn || !resolved || dladdr((const void *)fn, &info) == 0 || !info.dli_fname) return 0;
    if (strcmp(info.dli_fname, resolved) == 0) return 1;
    char canon[PATH_MAX];
    if (realpath(info.dli_fname, canon) && strcmp(canon, resolved) == 0) return 1;
    const char *a = strrchr(info.dli_fname, '/');
    const char *b = strrchr(resolved, '/');
    a = a ? a + 1 : info.dli_fname;
    b = b ? b + 1 : resolved;
    return strcmp(a, b) == 0;
}

/* Does any live world reference this handle's code? Tick attachments by
 * owning handle (origin-aware: a static same-named desc does NOT pin the
 * .so), stage hooks by code address, plus the retain counter. */
static int desc_busy_in_world(physics_world *w, int hi) {
    for (int k = 0; k < w->tick_module_count; k++) {
        if (w->tick_modules[k] && handle_for_desc(w->tick_modules[k]) == hi) {
            return 1;
        }
    }
    if (w->broadphase_if && w->broadphase_if->generate &&
        fn_in_plugin((mpe_collide_fn)(void *)w->broadphase_if->generate, s_h[hi].path)) {
        return 1;
    }
    if (w->solver_if) {
        const void *hooks[4] = { (const void *)w->solver_if->resolve,
                                 (const void *)w->solver_if->poisson,
                                 (const void *)w->solver_if->rolling,
                                 (const void *)w->solver_if->split };
        for (int k = 0; k < 4; k++) {
            if (hooks[k] && fn_in_plugin((mpe_collide_fn)hooks[k], s_h[hi].path)) {
                return 1;
            }
        }
    }
    return 0;
}

static int handle_busy(int hi) {
    if (s_h[hi].attachments > 0) return 1;
    physics_world *ws[MPE_MAX_LIVE_WORLDS];
    int n = physics_world_live_list(ws, MPE_MAX_LIVE_WORLDS);
    for (int i = 0; i < n; i++) {
        if (desc_busy_in_world(ws[i], hi)) return 1;
    }
    return 0;
}

/* Detach only tick modules owned by this handle (origin-aware: static
 * same-named attachments are left alone). Hooks run pre-dlclose. */
static void detach_handle_modules(int hi) {
    physics_world *ws[MPE_MAX_LIVE_WORLDS];
    int n = physics_world_live_list(ws, MPE_MAX_LIVE_WORLDS);
    for (int i = 0; i < n; i++) {
        physics_world *w = ws[i];
        for (int k = 0; k < w->tick_module_count;) {
            const mpe_module_desc_t *d = w->tick_modules[k];
            if (d && d->name && handle_for_desc(d) == hi) {
                char nm[128];
                snprintf(nm, sizeof(nm), "%s", d->name);
                physics_world_detach_module(w, nm);
            } else {
                k++;
            }
        }
        /* Stage slots owned by this handle revert to builtin. */
        if (w->broadphase_if && w->broadphase_if->generate &&
            fn_in_plugin((mpe_collide_fn)(void *)w->broadphase_if->generate, s_h[hi].path)) {
            w->broadphase_if = NULL;
            w->broadphase_state = NULL;
        }
        if (w->solver_if) {
            const void *hooks[4] = { (const void *)w->solver_if->resolve,
                                     (const void *)w->solver_if->poisson,
                                     (const void *)w->solver_if->rolling,
                                     (const void *)w->solver_if->split };
            for (int k = 0; k < 4; k++) {
                if (hooks[k] && fn_in_plugin((mpe_collide_fn)hooks[k], s_h[hi].path)) {
                    w->solver_if = NULL;
                    w->solver_state = NULL;
                    break;
                }
            }
        }
    }
}

/* Purge pair handlers whose code lives in this .so (backstop for plugins
 * without a destructor; capsule-style destructors run first via dlclose
 * ordering — purge runs BEFORE dlclose so dladdr still resolves). */
static void purge_plugin_pairs(int hi) {
    for (int round = 0; round < 8; round++) {
        int done = 1;
        /* Snapshot fns under lock, match outside it (dladdr may load). */
        mpe_collide_fn fns[MPE_MAX_PAIR_HANDLERS];
        int nfns = 0;
        for (int i = 0; i < MPE_MAX_PAIR_HANDLERS && nfns < MPE_MAX_PAIR_HANDLERS; i++) {
            if (mpe_registry_pair_fn_at(i, &fns[nfns]) == 0) nfns++;
        }
        for (int i = 0; i < nfns; i++) {
            if (fn_in_plugin(fns[i], s_h[hi].path)) {
                mpe_unregister_pair_handler(fns[i]);
                done = 0;
            }
        }
        if (done) break;
    }
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
        const char *n = s_h[i].is_ecosystem
                            ? (s_h[i].eco && s_h[i].eco->name ? s_h[i].eco->name : "")
                            : (s_h[i].desc ? s_h[i].desc->name : "");
        if (strcmp(s_h[i].path, identity) == 0 || strcmp(n, identity) == 0) {
            /* Ecosystem bundles detach everywhere (no busy concept: the
             * detach pass itself quiesces every world pre-dlclose). */
            if (s_h[i].is_ecosystem) {
                char enm[128];
                snprintf(enm, sizeof(enm), "%s", n ? n : "");
                if (enm[0]) {
                    mpe_ecosystem_detach_everywhere(enm);
                    mpe_ecosystem_unregister(enm);
                }
                dlclose(s_h[i].h);
                for (int j = i; j + 1 < s_n; j++) s_h[j] = s_h[j + 1];
                s_n--;
                return 0;
            }
            /* -2 = busy (referenced by a live world); detach/stage-reset
             * first, then retry. -1 = unknown handle. */
            if (handle_busy(i)) return -2;
            char modname[128];
            snprintf(modname, sizeof(modname), "%s", n ? n : "");
            /* Targeted teardown, all pre-dlclose: detach only this
             * handle's tick modules (static same-named ones stay),
             * unregister same-named stages (world slots reset to builtin
             * by the registry purge), remove only this handle's module
             * slot by origin, purge leftover pair handlers by address. */
            detach_handle_modules(i);
            if (modname[0]) {
                mpe_unregister_broadphase(modname);
                mpe_unregister_solver(modname);
                mpe_unregister_module_origin(modname, s_h[i].path);
            }
            purge_plugin_pairs(i);
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

const char *mpe_loader_name_at(int i) {
    if (i < 0 || i >= s_n) return 0;
    if (s_h[i].is_ecosystem) {
        return (s_h[i].eco && s_h[i].eco->name) ? s_h[i].eco->name : 0;
    }
    if (!s_h[i].desc || !s_h[i].desc->name) return 0;
    return s_h[i].desc->name;
}

/* Resolve a symbol from a loaded handle (by path, module name, or
 * ecosystem name) without taking a new reference. Powers terminal
 * commands (ftc/eco) that drive plugin APIs the engine never links.
 * Returns NULL when unknown (caller reports, never crashes). */
void *mpe_loader_symbol(const char *path_or_name, const char *sym) {
    if (!path_or_name || !sym || !*sym) return NULL;
    char resolved[PATH_MAX];
    const char *identity = path_or_name;
    int by_path = (strchr(path_or_name, '/') != NULL);
    if (by_path) {
        if (!plugin_path_is_confined(path_or_name, resolved)) return NULL;
        identity = resolved;
    }
    for (int i = 0; i < s_n; i++) {
        const char *n = s_h[i].is_ecosystem
                            ? ((s_h[i].eco && s_h[i].eco->name) ? s_h[i].eco->name : "")
                            : ((s_h[i].desc && s_h[i].desc->name) ? s_h[i].desc->name : "");
        if (strcmp(s_h[i].path, identity) != 0 && strcmp(n, identity) != 0) continue;
        /* Resolve against the stored open handle (never loads). */
        dlerror();
        void *p = dlsym(s_h[i].h, sym);
        if (dlerror() != NULL) return NULL;
        return p;
    }
    return NULL;
}

/* Retain/release resolve the OWNING handle (origin-aware): attach may
 * store a registry copy or a static original, and a static same-named
 * desc must never pin the .so. Pointer comparison alone could never
 * match (registry copy vs .so original), so the old guard never fired;
 * name comparison alone over-matches statics. handle_for_desc does both. */
void mpe_loader_retain_module(const void *desc) {
    int hi = handle_for_desc((const mpe_module_desc_t *)desc);
    if (hi >= 0) s_h[hi].attachments++;
}

void mpe_loader_release_module(const void *desc) {
    int hi = handle_for_desc((const mpe_module_desc_t *)desc);
    if (hi >= 0 && s_h[hi].attachments > 0) s_h[hi].attachments--;
}
