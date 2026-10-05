#ifndef _GNU_SOURCE
#define _GNU_SOURCE /* dladdr */
#endif
#include "mpe_loader.h"
#include "mpe_platform.h"
#include "mpe_registry.h"
#include "physics_world.h"
#include "../ecosystem/mpe_ecosystem.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/stat.h>
#define MPE_MAX_HANDLES 32
static struct {
    void *h;
    char path [PATH_MAX];
    const mpe_module_desc_t *desc; /* module .so (NULL for ecosystems) */
    const mpe_ecosystem_desc_t *eco; /* ecosystem .so (NULL for modules) */
    int is_ecosystem;
    int attachments; /* guarded by s_loader_lock (retain/release from any thread) */
    long long f_mtime; /* on-disk identity at load: re-load of a changed file */
    long long f_size; /* must NOT silently keep stale code (despot trap) */
} s_h [MPE_MAX_HANDLES];
static int s_n = 0;
/* Jail: plugins/<name>.so|.dll for modules, ecosystem/mfs/<name>.so|.dll
 * for ecosystem bundles (both CWD-relative, normally v15S/src). Same
 * traversal-proofing in both roots. Windows accepts '/' and '\\', both
 * extensions, case-insensitively; Linux keeps exact '.so' behaviour. */
static int mpe_has_plugin_ext (const char *base) {
    size_t n = strlen (base);
#ifdef MPE_OS_WINDOWS
    /* accept .so (MSYS2) and .dll (native) case-insensitively */
    if (n >= 4 && _stricmp (base + n - 4, ".dll") == 0)
        return 1;
    if (n >= 3 && _stricmp (base + n - 3, ".so") == 0)
        return 1;
    return 0;
#else
    if (n < 3)
        return 0;
    return strcmp (base + n - 3, ".so") == 0;
#endif
}
/* On-disk identity for stale-.so detection (despot trap: `mod load` on an
 * already-loaded path used to return 0/“loaded” while running the OLD
 * in-memory image — e.g. articulated rollers after the analytic rebuild.
 * Changed files now fail with -3 instead of silently lying). */
static int file_identity (const char *path, long long *mt, long long *sz) {
    struct stat st;
    if (stat (path, &st) != 0)
        return -1;
    /* Nanosecond mtime: second granularity misses same-second rebuilds. */
#if defined(__APPLE__)
    long long nsec = (long long) st.st_mtimespec.tv_nsec;
#else
    long long nsec = (long long) st.st_mtim.tv_nsec;
#endif
    if (mt)
        * mt = (long long) st.st_mtime * 1000000000LL + nsec;
    if (sz)
        * sz = (long long) st.st_size;
    return 0;
}
/* Loader-table lock: serialises every s_h[]/s_n read and mutation plus the
 * attachments accounting (retain/release run on step-adjacent threads while
 * unload runs at the tick boundary). Recursive: detach paths call back into
 * retain/release (physics_world_detach_module) while the unload path holds
 * it. Lock ORDER is always loader -> registry (registry calls into the
 * loader only after dropping s_reg_lock; see mpe_registry.c unregister
 * paths), so the two locks can never deadlock against each other. The lock
 * is NOT held across dlopen: plugin constructors register (locking
 * internally), so holding it would serialize constructor work for no
 * benefit; load re-validates the table after dlopen instead. */
static pthread_mutex_t s_loader_lock;
static pthread_once_t s_loader_lock_once = PTHREAD_ONCE_INIT;
static void s_loader_lock_init (void) {
    pthread_mutexattr_t at;
    pthread_mutexattr_init (&at);
    pthread_mutexattr_settype (&at, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init (&s_loader_lock, &at);
    pthread_mutexattr_destroy (&at);
}
static inline void loader_lock (void) {
    pthread_once (&s_loader_lock_once, s_loader_lock_init);
    pthread_mutex_lock (&s_loader_lock);
}
static inline void loader_unlock (void) {
    pthread_mutex_unlock (&s_loader_lock);
}
/* Jail: plugins/<name>.so for modules, ecosystem/mfs/<name>.so for
 * ecosystem bundles (both CWD-relative, normally v15S/src). Same
 * traversal-proofing in both roots. */
static int plugin_path_is_confined (const char *path, char resolved [PATH_MAX]) {
#ifdef MPE_OS_WINDOWS
    /* Normalise backslashes to slashes for prefix matching. */
    char norm [PATH_MAX * 2];
    size_t pi = 0;
    if (!path)
        return 0;
    for (size_t i = 0; path [i] && pi + 1 < sizeof (norm); i++) {
        norm [pi++] = (path [i] == '\\') ? '/' : path [i];
    }
    norm [pi] = '\0';
    path = norm;
#endif
    const char *prefix_a = "plugins/";
    const char *prefix_b = "./plugins/";
    const char *prefix_c = "ecosystem/mfs/";
    const char *prefix_d = "./ecosystem/mfs/";
    const char *prefix_e = ".\\plugins\\";
    const char *dir = NULL;
    const char *base = NULL;
    if (path && strncmp (path, prefix_a, strlen (prefix_a)) == 0) {
        dir = "plugins";
        base = path + strlen (prefix_a);
    } else if (path && strncmp (path, prefix_b, strlen (prefix_b)) == 0) {
        dir = "plugins";
        base = path + strlen (prefix_b);
    } else if (path && strncmp (path, prefix_c, strlen (prefix_c)) == 0) {
        dir = "ecosystem/mfs";
        base = path + strlen (prefix_c);
    } else if (path && strncmp (path, prefix_d, strlen (prefix_d)) == 0) {
        dir = "ecosystem/mfs";
        base = path + strlen (prefix_d);
    } else {
        return 0;
    }
    (void) prefix_e;
    if (!*base || strchr (base, '/') ||
#ifdef MPE_OS_WINDOWS
        strchr (base, '\\') ||
#endif
        strstr (base, "..") || !mpe_has_plugin_ext (base))
        return 0;
    char root [PATH_MAX];
    if (!realpath (dir, root) || !realpath (path, resolved))
        return 0;
    size_t root_len = strlen (root);
    return strncmp (resolved, root, root_len) == 0 && resolved [root_len] == '/';
}
int mpe_loader_load (const char *path, char *errbuf, int errlen) {
    if (!path || !*path) {
        if (errbuf && errlen > 0)
            snprintf (errbuf, (size_t) errlen, "empty path");
        return -1;
    }
    char resolved [PATH_MAX];
    if (!plugin_path_is_confined (path, resolved)) {
        /* Distinguish "file missing / wrong working directory" (the common
         * MFS stumble: engine must run from v15S/src) from a genuine jail
         * violation so users are told how to fix it. Extension is
         * platform-aware (MPE_PLUGIN_EXT: .so, or .dll on Windows). */
        if (errbuf && errlen > 0) {
            FILE *probe = fopen (path, "rb");
            if (probe) {
                fclose (probe);
                snprintf (
                    errbuf, (size_t) errlen,
                    "path escapes module jail (run from v15S/src; use plugins/<name>%s or ecosystem/mfs/<name>%s)",
                    MPE_PLUGIN_EXT, MPE_PLUGIN_EXT);
            } else {
                snprintf (errbuf, (size_t) errlen,
                          "no such file '%s' (run from v15S/src; modules live at plugins/<name>%s, bundles at "
                          "ecosystem/mfs/<name>%s)",
                          path, MPE_PLUGIN_EXT, MPE_PLUGIN_EXT);
            }
        }
        return -1;
    }
    /* Length validated BEFORE any registration (a late failure used to
     * dlclose while the module stayed registered -> dangling dispatch). */
    /* DESPOT-2026-09-29: this allowed PATH_MAX-64 (4031) bytes, but the
     * registry stores a module's origin in a 256-byte buffer and compares it
     * with strcmp at unload. A longer path truncated silently, so the
     * origin-scoped unregister never matched and the module slot survived
     * the dlclose still live. Bound the load by what the registry can store. */
    if (strlen (resolved) >= MPE_MODULE_ORIGIN_MAX - 1) {
        if (errbuf && errlen > 0)
            snprintf (errbuf, (size_t) errlen, "path too long");
        return -1;
    }
    /* Table pre-check under lock; the lock is dropped across dlopen below
     * (constructors run there) and every table predicate is re-checked
     * after it, so a racing load can never double-insert. */
    loader_lock ();
    for (int i = 0; i < s_n; i++)
        if (strcmp (s_h [i].path, resolved) == 0) {
        /* Already loaded: serve the in-memory image ONLY if the file
             * is unchanged. A rebuild on disk + silent old-code execution
             * is the stale-.so despot trap — refuse with -3 and a fix. */
        long long mt_now = 0, sz_now = 0;
        if (file_identity (resolved, &mt_now, &sz_now) != 0) {
            if (errbuf && errlen > 0)
                snprintf (errbuf, (size_t) errlen,
                              "already loaded but file vanished '%s' (restart engine to clear)", path);
            loader_unlock ();
            return -1;
        }
        if (mt_now != s_h [i].f_mtime || sz_now != s_h [i].f_size) {
            if (errbuf && errlen > 0)
                snprintf (errbuf, (size_t) errlen,
                              "already loaded but file changed on disk (stale code running): unload '%s', then load "
                              "again — or restart the engine",
                              path);
            loader_unlock ();
            return -3;
        }
        loader_unlock ();
        return 0;
    } /* already loaded */
    if (s_n >= MPE_MAX_HANDLES) {
        if (errbuf && errlen > 0)
            snprintf (errbuf, (size_t) errlen, "handle table full");
        loader_unlock ();
        return -1;
    }
    loader_unlock ();
    /* Snapshot registry counts so constructor registrations can be rolled
     * back if anything below fails. The registry lock is NOT held across
     * dlopen: plugin constructors register (locking internally), so
     * holding it would deadlock. Load/unload are tick-boundary admin
     * operations; concurrent loader use from step threads is misuse
     * (documented in mpe_loader.h). */
    int snap_pairs = mpe_registry_pair_count ();
    int snap_broad = mpe_registry_broadphase_count ();
    int snap_solvers = mpe_registry_solver_count ();
    /* DESPOT-2026-09-29: module table was never snapshotted, so nothing could
     * roll it back. A constructor that registers a module and then fails the
     * desc checks used to leave a live descriptor pointing into a dlclose'd
     * image. */
    /* DESPOT-2026-10-03: this MUST be the index high-water, not the live count.
     * mpe_module_count() counts live entries while
     * mpe_registry_truncate_modules(keep) truncates by index, so with a
     * tombstone hole (reg A, reg B, unreg B, reg C -> live 2, high water 3)
     * rolling back to the live count tombstoned C, a pre-existing LIVE
     * registration, silently and with no diagnostic. Silent loss of a
     * registered module on any failed load. */
    int snap_modules = mpe_module_slot_count ();
    dlerror ();
    void *h = dlopen (resolved, RTLD_NOW | RTLD_LOCAL);
    if (!h) {
        if (errbuf && errlen > 0)
            snprintf (errbuf, (size_t) errlen, "%s", dlerror ());
        return -1;
    }
    /* MSVC has no constructor/destructor: call explicit init if exported.
     * Idempotent on GCC (flag-guarded), so harmless on Linux/MinGW. */
    {
        dlerror ();
        void (*p_init) (void) = (void (*) (void)) dlsym (h, "mpe_capsule_init");
        if (dlerror () == NULL && p_init)
            p_init ();
    }
    loader_lock ();
    /* Re-validate under lock: a concurrent load may have won the race. */
    for (int i = 0; i < s_n; i++) {
        if (strcmp (s_h [i].path, resolved) == 0) {
            loader_unlock ();
            dlclose (h);
            mpe_registry_truncate_pairs (snap_pairs);
            mpe_registry_truncate_modules (snap_modules);
            mpe_registry_truncate_broadphase (snap_broad);
            mpe_registry_truncate_solvers (snap_solvers);
            return 0;
        }
    }
    if (s_n >= MPE_MAX_HANDLES) {
        loader_unlock ();
        if (errbuf && errlen > 0)
            snprintf (errbuf, (size_t) errlen, "handle table full");
        dlclose (h);
        mpe_registry_truncate_pairs (snap_pairs);
        mpe_registry_truncate_modules (snap_modules);
        mpe_registry_truncate_broadphase (snap_broad);
        mpe_registry_truncate_solvers (snap_solvers);
        return -1;
    }
    /* Precedence: ecosystem bundles (mpe_ecosystem_desc) win over plain
     * modules. A bundle links its inner modules' objects for code, so it
     * exports their mpe_module_desc too — taking it would register a
     * fragment instead of the bundle. */
    dlerror ();
    const mpe_ecosystem_desc_t *eco_first = (const mpe_ecosystem_desc_t *) dlsym (h, "mpe_ecosystem_desc");
    const char *sym_err = dlerror ();
    if (!sym_err && eco_first) {
        if (eco_first -> abi != MPE_ECOSYSTEM_ABI || !eco_first -> name) {
            if (errbuf && errlen > 0)
                snprintf (errbuf, (size_t) errlen, "ecosystem ABI/name mismatch");
            loader_unlock ();
            dlclose (h);
            mpe_registry_truncate_pairs (snap_pairs);
            mpe_registry_truncate_modules (snap_modules);
            mpe_registry_truncate_broadphase (snap_broad);
            mpe_registry_truncate_solvers (snap_solvers);
            return -1;
        }
        if (mpe_ecosystem_register (eco_first) < 0) {
            if (errbuf && errlen > 0)
                snprintf (errbuf, (size_t) errlen, "ecosystem registry full/dup");
            loader_unlock ();
            dlclose (h);
            mpe_registry_truncate_pairs (snap_pairs);
            mpe_registry_truncate_modules (snap_modules);
            mpe_registry_truncate_broadphase (snap_broad);
            mpe_registry_truncate_solvers (snap_solvers);
            return -1;
        }
        s_h [s_n].h = h;
        snprintf (s_h [s_n].path, sizeof (s_h [s_n].path), "%s", resolved);
        s_h [s_n].desc = NULL;
        s_h [s_n].eco = eco_first;
        s_h [s_n].is_ecosystem = 1;
        s_h [s_n].attachments = 0;
        file_identity (resolved, &s_h [s_n].f_mtime, &s_h [s_n].f_size);
        s_n++;
        loader_unlock ();
        return 0;
    }
    dlerror ();
    const mpe_module_desc_t *desc = (const mpe_module_desc_t *) dlsym (h, "mpe_module_desc");
    sym_err = dlerror ();
    if (!sym_err && desc) {
        if (desc -> abi != MPE_MODULE_ABI || !desc -> name) {
            if (errbuf && errlen > 0)
                snprintf (errbuf, (size_t) errlen, "ABI/name mismatch");
            loader_unlock ();
            dlclose (h);
            mpe_registry_truncate_pairs (snap_pairs);
            mpe_registry_truncate_modules (snap_modules);
            mpe_registry_truncate_broadphase (snap_broad);
            mpe_registry_truncate_solvers (snap_solvers);
            return -1;
        }
        if (mpe_register_module (desc) < 0) {
            if (errbuf && errlen > 0)
                snprintf (errbuf, (size_t) errlen, "registry full/dup");
            loader_unlock ();
            dlclose (h);
            mpe_registry_truncate_pairs (snap_pairs);
            mpe_registry_truncate_modules (snap_modules);
            mpe_registry_truncate_broadphase (snap_broad);
            mpe_registry_truncate_solvers (snap_solvers);
            return -1;
        }
        /* Pair handlers self-register in the plugin constructor; stages
         * register explicitly. Covered by the snapshot above on failure. */
        s_h [s_n].h = h;
        snprintf (s_h [s_n].path, sizeof (s_h [s_n].path), "%s", resolved);
        s_h [s_n].desc = desc;
        s_h [s_n].eco = NULL;
        s_h [s_n].is_ecosystem = 0;
        s_h [s_n].attachments = 0;
        file_identity (resolved, &s_h [s_n].f_mtime, &s_h [s_n].f_size);
        s_n++;
        loader_unlock ();
        mpe_registry_set_origin (desc -> name, resolved);
        return 0;
    }
    if (errbuf && errlen > 0)
        snprintf (errbuf, (size_t) errlen, "missing mpe_module_desc: %s", sym_err ? sym_err : "null");
    loader_unlock ();
    dlclose (h); /* destructor self-unregisters well-behaved plugins */
    mpe_registry_truncate_pairs (snap_pairs);
    mpe_registry_truncate_modules (snap_modules);
    mpe_registry_truncate_broadphase (snap_broad);
    mpe_registry_truncate_solvers (snap_solvers);
    return -1;
}
/* Path equality: exact match or realpath-canonicalised match ONLY.
 * DESPOT-2026-10-01: the old basename fallback made different dirs with the
 * same filename compare equal, so unload/detach/handle_for_desc could tear
 * down the wrong handle and dlclose a still-referenced image. Exact paths. */
static int same_path (const char *a, const char *b) {
    if (!a || !b)
        return 0;
#ifdef MPE_OS_WINDOWS
    if (_stricmp (a, b) == 0)
        return 1;
#else
    if (strcmp (a, b) == 0)
        return 1;
#endif
    char ca [PATH_MAX], cb [PATH_MAX];
#ifdef MPE_OS_WINDOWS
    if (realpath (a, ca) && realpath (b, cb) && _stricmp (ca, cb) == 0)
        return 1;
    return 0;
#else
    if (realpath (a, ca) && realpath (b, cb) && strcmp (ca, cb) == 0)
        return 1;
    return 0;
#endif
}
static int handle_by_path_locked (const char *path) {
    for (int i = 0; i < s_n; i++) {
        if (same_path (s_h [i].path, path))
            return i;
    }
    return -1;
}
/* NOTE: every *_locked helper below requires s_loader_lock held. Public
 * entry points take it; unload-time sweeps already hold it throughout. */
/* Which loaded handle owns this desc? Registry-slot origin first (exact
 * for registry copies handed out by find), else the desc's own image.
 * -1 = static/host code that never unloads. */
static int handle_for_desc_locked (const mpe_module_desc_t *d) {
    if (!d)
        return -1;
    const char *origin = mpe_registry_module_origin (d);
    if (origin) {
        char obuf [MPE_MODULE_ORIGIN_MAX];
        snprintf (obuf, sizeof (obuf), "%s", origin);
        return handle_by_path_locked (obuf);
    }
    Dl_info info;
    if (dladdr ((const void *) d, &info) && info.dli_fname) {
        return handle_by_path_locked (info.dli_fname);
    }
    return -1;
}
/* Validated stage_detach read: the hook is an append-only tail field, so a
 * .so built against the older header has no such pointer and the word past
 * its struct is unowned image data. Read exactly one function pointer
 * (benign: same mapped page), require non-NULL, then require dladdr to
 * resolve it INSIDE the owning plugin image before invoking — stale
 * garbage can never redirect control. `owner_path` is the handle path;
 * NULL falls back to the desc's own image. */
static void call_stage_detach_validated (const mpe_module_desc_t *desc, const char *owner_path, physics_world *w) {
    if (!desc || !w)
        return;
    void (*hook) (physics_world *) = NULL;
    memcpy (&hook, &desc -> stage_detach, sizeof (hook));
    if (!hook)
        return; /* old .so, or module with no foreign stage state */
    Dl_info hi;
    if (dladdr ((const void *) hook, &hi) == 0 || !hi.dli_fname)
        return;
    const char *dpath = owner_path;
    Dl_info di;
    char dbuf [PATH_MAX];
    if (!dpath) {
        if (dladdr ((const void *) desc, &di) && di.dli_fname) {
            snprintf (dbuf, sizeof (dbuf), "%s", di.dli_fname);
            dpath = dbuf;
        } else {
            return;
        }
    }
    if (!same_path (hi.dli_fname, dpath))
        return; /* foreign/garbage: refuse */
    hook (w);
}
/* Code-address variant (locked): stage slots keep the iface, not the desc. */
static void call_stage_detach_for_fn_locked (const void *fn, physics_world *w) {
    if (!fn || !w)
        return;
    Dl_info info;
    if (dladdr (fn, &info) == 0 || !info.dli_fname)
        return;
    int hi = handle_by_path_locked (info.dli_fname);
    if (hi < 0 || s_h [hi].is_ecosystem || !s_h [hi].desc)
        return;
    call_stage_detach_validated (s_h [hi].desc, s_h [hi].path, w);
}
void mpe_loader_call_stage_detach (const void *desc, struct physics_world *world) {
    if (!desc || !world)
        return;
    /* Invoked under lock: teardown paths guarantee the .so is still mapped
     * exactly while the loader table is stable; dropping the lock before
     * the call would admit a racing dlclose under the hook. The lock is
     * recursive and the order stays loader -> registry, so module code
     * that re-enters the loader/registry cannot deadlock. */
    loader_lock ();
    const mpe_module_desc_t *d = (const mpe_module_desc_t *) desc;
    int hi = handle_for_desc_locked (d);
    call_stage_detach_validated (d, (hi >= 0) ? s_h [hi].path : NULL, world);
    loader_unlock ();
}
void mpe_loader_call_stage_detach_for_fn (const void *fn, struct physics_world *world) {
    if (!fn || !world)
        return;
    loader_lock ();
    call_stage_detach_for_fn_locked (fn, world);
    loader_unlock ();
}
/* True when fn's code lives inside the plugin at path `resolved`.
 * Well-behaved plugins self-unregister in their destructor; this is the
 * backstop for the rest. dli_fname may be relative — resolve it when
 * possible, else fall back to basename comparison. */
static int fn_in_plugin (mpe_collide_fn fn, const char *resolved) {
    Dl_info info;
    if (!fn || !resolved || dladdr ((const void *) fn, &info) == 0 || !info.dli_fname)
        return 0;
#ifdef MPE_OS_WINDOWS
    if (_stricmp (info.dli_fname, resolved) == 0)
        return 1;
    char canon [PATH_MAX];
    if (realpath (info.dli_fname, canon) && _stricmp (canon, resolved) == 0)
        return 1;
    const char *a = strrchr (info.dli_fname, '/');
    const char *asa = strrchr (info.dli_fname, '\\');
    const char *b = strrchr (resolved, '/');
    const char *bsb = strrchr (resolved, '\\');
    if (asa && (!a || asa > a))
        a = asa;
    if (bsb && (!b || bsb > b))
        b = bsb;
    a = a ? a + 1 : info.dli_fname;
    b = b ? b + 1 : resolved;
    return _stricmp (a, b) == 0;
#else
    if (strcmp (info.dli_fname, resolved) == 0)
        return 1;
    char canon [PATH_MAX];
    if (realpath (info.dli_fname, canon) && strcmp (canon, resolved) == 0)
        return 1;
    const char *a = strrchr (info.dli_fname, '/');
    const char *b = strrchr (resolved, '/');
    a = a ? a + 1 : info.dli_fname;
    b = b ? b + 1 : resolved;
    return strcmp (a, b) == 0;
#endif
}
/* Does any live world reference this handle's code? Tick attachments by
 * owning handle (origin-aware: a static same-named desc does NOT pin the
 * .so), stage hooks by code address, plus the retain counter.
 * Requires s_loader_lock. */
static int desc_busy_in_world (physics_world *w, int hi) {
    for (int k = 0; k < w -> tick_module_count; k++) {
        if (w -> tick_modules [k] && handle_for_desc_locked (w -> tick_modules [k]) == hi) {
            return 1;
        }
    }
    if (w -> broadphase_if && w -> broadphase_if -> generate &&
        fn_in_plugin ((mpe_collide_fn) (void *) w -> broadphase_if -> generate, s_h [hi].path)) {
        return 1;
    }
    if (w -> solver_if) {
        const void *hooks [4] = {(const void *) w -> solver_if -> resolve, (const void *) w -> solver_if -> poisson,
                                (const void *) w -> solver_if -> rolling, (const void *) w -> solver_if -> split};
        for (int k = 0; k < 4; k++) {
            if (hooks [k] && fn_in_plugin ((mpe_collide_fn) hooks [k], s_h [hi].path)) {
                return 1;
            }
        }
    }
    return 0;
}
static int handle_busy (int hi) {
    if (s_h [hi].attachments > 0)
        return 1; /* locked caller (unload) */
    physics_world *ws [MPE_MAX_LIVE_WORLDS];
    int n = physics_world_live_list (ws, MPE_MAX_LIVE_WORLDS);
    for (int i = 0; i < n; i++) {
        if (desc_busy_in_world (ws [i], hi))
            return 1;
    }
    return 0;
}
/* Detach only tick modules owned by this handle (origin-aware: static
 * same-named attachments are left alone). Hooks run pre-dlclose.
 * Requires s_loader_lock. */
static void detach_handle_modules (int hi) {
    physics_world *ws [MPE_MAX_LIVE_WORLDS];
    int n = physics_world_live_list (ws, MPE_MAX_LIVE_WORLDS);
    const mpe_module_desc_t *owner = s_h [hi].is_ecosystem ? NULL : s_h [hi].desc;
    for (int i = 0; i < n; i++) {
        physics_world *w = ws [i];
        for (int k = 0; k < w -> tick_module_count;) {
            const mpe_module_desc_t *d = w -> tick_modules [k];
            if (d && d -> name && handle_for_desc_locked (d) == hi) {
                char nm [128];
                snprintf (nm, sizeof (nm), "%s", d -> name);
                physics_world_detach_module (w, nm);
            } else {
                k++;
            }
        }
        /* Stage slots owned by this handle revert to builtin. Foreign
         * stage state is owned by the module (physics_world.h:133-137:
         * never freed by the world), so run its stage_detach hook BEFORE
         * NULLing the pointers — otherwise the state leaks. */
        if (w -> broadphase_if && w -> broadphase_if -> generate &&
            fn_in_plugin ((mpe_collide_fn) (void *) w -> broadphase_if -> generate, s_h [hi].path)) {
            if (owner)
                call_stage_detach_validated (owner, s_h [hi].path, w);
            w -> broadphase_if = NULL;
            w -> broadphase_state = NULL;
        }
        if (w -> solver_if) {
            const void *hooks [4] = {(const void *) w -> solver_if -> resolve, (const void *) w -> solver_if -> poisson,
                                    (const void *) w -> solver_if -> rolling, (const void *) w -> solver_if -> split};
            for (int k = 0; k < 4; k++) {
                if (hooks [k] && fn_in_plugin ((mpe_collide_fn) hooks [k], s_h [hi].path)) {
                    if (owner)
                        call_stage_detach_validated (owner, s_h [hi].path, w);
                    w -> solver_if = NULL;
                    w -> solver_state = NULL;
                    break;
                }
            }
        }
    }
}
/* Purge pair handlers whose code lives in this .so (backstop for plugins
 * without a destructor; capsule-style destructors run first via dlclose
 * ordering — purge runs BEFORE dlclose so dladdr still resolves). */
static void purge_plugin_pairs (int hi) {
    for (int round = 0; round < 8; round++) {
        int done = 1;
        /* Snapshot fns under lock, match outside it (dladdr may load). */
        mpe_collide_fn fns [MPE_MAX_PAIR_HANDLERS];
        int nfns = 0;
        for (int i = 0; i < MPE_MAX_PAIR_HANDLERS && nfns < MPE_MAX_PAIR_HANDLERS; i++) {
            if (mpe_registry_pair_fn_at (i, &fns [nfns]) == 0)
                nfns++;
        }
        for (int i = 0; i < nfns; i++) {
            if (fn_in_plugin (fns [i], s_h [hi].path)) {
                mpe_unregister_pair_handler (fns [i]);
                done = 0;
            }
        }
        if (done)
            break;
    }
}
int mpe_loader_unload (const char *path_or_name) {
    if (!path_or_name)
        return -1;
    char resolved [PATH_MAX];
    const char *identity = path_or_name;
#ifdef MPE_OS_WINDOWS
    if (strchr (path_or_name, '/') || strchr (path_or_name, '\\')) {
#else
        if (strchr (path_or_name, '/')) {
#endif
            if (!plugin_path_is_confined (path_or_name, resolved))
                return -1;
            identity = resolved;
        }
        loader_lock ();
        for (int i = 0; i < s_n; i++) {
            const char *n = s_h [i].is_ecosystem ? (s_h [i].eco && s_h [i].eco -> name ? s_h [i].eco -> name : "")
                : (s_h [i].desc ? s_h [i].desc -> name : "");
            if (strcmp (s_h [i].path, identity) == 0 || strcmp (n, identity) == 0) {
                /* Ecosystem bundles are NOT exempt from the busy check.
             *
             * DESPOT-2026-09-29 CRITICAL. The comment here used to claim
             * "no busy concept: the detach pass itself quiesces every world
             * pre-dlclose". That is false. mpe_ecosystem_detach_everywhere
             * only clears the ECOSYSTEM registry's own attachment table
             * (mpe_ecosystem.c s_attached[]); it does not touch
             * world->tick_modules[]. A bundle can legitimately export
             * mpe_module_desc, and term_ftc.c attaches exactly that:
             * `ftc_sym("mpe_module_desc")` then physics_world_attach_module.
             * The world then holds a pointer INTO the bundle image, and this
             * branch dlclose'd it with attachments > 0 and returned 0. The
             * next physics_world_step read the descriptor out of unmapped
             * memory and called through it.
             *
             * handle_busy() and detach_handle_modules() both resolve a
             * descriptor to its owning handle via dladdr, so they already work
             * for ecosystem handles -- they were simply never called here. */
                if (s_h [i].attachments > 0) {
                    loader_unlock ();
                    return -2;
                }
                {
                    physics_world *ws [MPE_MAX_LIVE_WORLDS];
                    int nw = physics_world_live_list (ws, MPE_MAX_LIVE_WORLDS);
                    for (int wi2 = 0; wi2 < nw; wi2++) {
                        if (desc_busy_in_world (ws [wi2], i)) {
                            loader_unlock ();
                            return -2;
                        }
                    }
                }
                if (s_h [i].is_ecosystem) {
                    char enm [128];
                    snprintf (enm, sizeof (enm), "%s", n ? n : "");
                    if (strlen (enm) == sizeof (enm) - 1 && strlen (n ? n : "") >= sizeof (enm)) {
                        /* The name does not fit the buffer. Truncating it would
                     * make detach_everywhere/unregister match nothing, leaving
                     * entries aliasing a dead slot. Refuse rather than
                     * half-tear-down. */
                        loader_unlock ();
                        return -3;
                    }
                    if (enm [0]) {
                        mpe_ecosystem_detach_everywhere (enm);
                        mpe_ecosystem_unregister (enm);
                    }
                    /* Same pre-dlclose teardown the module branch uses: detach any
                 * tick modules and stage slots owned by this image, and drop
                 * pair handlers whose code lives here. */
                    detach_handle_modules (i);
                    purge_plugin_pairs (i);
                    dlerror ();
                    void (*p_fini) (void) = (void (*) (void)) dlsym (s_h [i].h, "mpe_capsule_fini");
                    if (dlerror () == NULL && p_fini)
                        p_fini ();
                    dlclose (s_h [i].h);
                    for (int j = i; j + 1 < s_n; j++)
                        s_h [j] = s_h [j + 1];
                    s_n--;
                    loader_unlock ();
                    return 0;
                }
                /* -2 = busy (referenced by a live world); detach/stage-reset
             * first, then retry. -1 = unknown handle. */
                if (handle_busy (i)) {
                    loader_unlock ();
                    return -2;
                }
                char modname [128];
                snprintf (modname, sizeof (modname), "%s", n ? n : "");
                /* Targeted teardown, all pre-dlclose: detach only this
             * handle's tick modules (static same-named ones stay),
             * unregister same-named stages (world slots reset to builtin
             * by the registry purge), remove only this handle's module
             * slot by origin, purge leftover pair handlers by address. */
                detach_handle_modules (i);
                if (modname [0]) {
                    mpe_unregister_broadphase (modname);
                    mpe_unregister_solver (modname);
                    mpe_unregister_module_origin (modname, s_h [i].path);
                }
                purge_plugin_pairs (i);
                dlerror ();
                {
                    void (*p_fini) (void) = (void (*) (void)) dlsym (s_h [i].h, "mpe_capsule_fini");
                    if (dlerror () == NULL && p_fini)
                        p_fini ();
                }
                dlclose (s_h [i].h);
                for (int j = i; j + 1 < s_n; j++)
                    s_h [j] = s_h [j + 1];
                s_n--;
                loader_unlock ();
                return 0;
            }
        }
        loader_unlock ();
        return -1;
    }
    int mpe_loader_count (void) {
        loader_lock ();
        int n = s_n;
        loader_unlock ();
        return n;
    }
    const char *mpe_loader_path_at (int i) {
        /* DESPOT-2026-10-01: interior pointer dangled across load/unload/dlclose.
     * Return a rotating snapshot copy (4 slots, PATH_MAX each) taken under
     * lock. Valid until 4 further calls or next unload — callers needing
     * longevity must copy. No heap, no NULL-deref, no use-after-dlclose. */
        static char snaps [4][PATH_MAX];
        static int next = 0;
        loader_lock ();
        const char *src = (i >= 0 && i < s_n) ? s_h [i].path : NULL;
        const char *out = NULL;
        if (src) {
            snprintf (snaps [next], sizeof (snaps [next]), "%s", src);
            out = snaps [next];
            next = (next + 1) & 3;
        }
        loader_unlock ();
        return out;
    }
    const char *mpe_loader_name_at (int i) {
        /* Same snapshot discipline as path_at (see above). */
        static char snaps [4][128];
        static int next = 0;
        loader_lock ();
        const char *src = NULL;
        const char *out = NULL;
        if (i >= 0 && i < s_n) {
            if (s_h [i].is_ecosystem) {
                src = (s_h [i].eco && s_h [i].eco -> name) ? s_h [i].eco -> name : NULL;
            } else if (s_h [i].desc && s_h [i].desc -> name) {
                src = s_h [i].desc -> name;
            }
        }
        if (src) {
            snprintf (snaps [next], sizeof (snaps [next]), "%s", src);
            out = snaps [next];
            next = (next + 1) & 3;
        }
        loader_unlock ();
        return out;
    }
    /* Resolve a symbol from a loaded handle (by path, module name, or
 * ecosystem name) without taking a new reference. Powers terminal
 * commands (ftc/eco) that drive plugin APIs the engine never links.
 * Returns NULL when unknown (caller reports, never crashes). */
    void *mpe_loader_symbol (const char *path_or_name, const char *sym) {
        if (!path_or_name || !sym || !*sym)
            return NULL;
        char resolved [PATH_MAX];
        const char *identity = path_or_name;
#ifdef MPE_OS_WINDOWS
        int by_path = (strchr (path_or_name, '/') != NULL || strchr (path_or_name, '\\') != NULL);
#else
        int by_path = (strchr (path_or_name, '/') != NULL);
#endif
        if (by_path) {
            if (!plugin_path_is_confined (path_or_name, resolved))
                return NULL;
            identity = resolved;
        }
        loader_lock ();
        void *out = NULL;
        for (int i = 0; i < s_n; i++) {
            const char *n = s_h [i].is_ecosystem ? ((s_h [i].eco && s_h [i].eco -> name) ? s_h [i].eco -> name : "")
                : ((s_h [i].desc && s_h [i].desc -> name) ? s_h [i].desc -> name : "");
            if (strcmp (s_h [i].path, identity) != 0 && strcmp (n, identity) != 0)
                continue;
            /* Resolve against the stored open handle (never loads). dlsym
         * takes no loader/registry lock; the table is stable under ours. */
            dlerror ();
            void *p = dlsym (s_h [i].h, sym);
            if (dlerror () != NULL) {
                out = NULL;
                break;
            }
            out = p;
            break;
        }
        loader_unlock ();
        return out;
    }
    /* Retain/release resolve the OWNING handle (origin-aware): attach may
 * store a registry copy or a static original, and a static same-named
 * desc must never pin the .so. Pointer comparison alone could never
 * match (registry copy vs .so original), so the old guard never fired;
 * name comparison alone over-matches statics. handle_for_desc_locked does
 * both. The attachments counter is guarded by s_loader_lock (equivalent
 * to an atomic fetch_add for this table's purposes: every increment and
 * the busy-check in unload are mutually exclusive). */
    void mpe_loader_retain_module (const void *desc) {
        if (!desc)
            return;
        loader_lock ();
        int hi = handle_for_desc_locked ((const mpe_module_desc_t *) desc);
        if (hi >= 0)
            s_h [hi].attachments++;
        loader_unlock ();
    }
    void mpe_loader_release_module (const void *desc) {
        if (!desc)
            return;
        loader_lock ();
        int hi = handle_for_desc_locked ((const mpe_module_desc_t *) desc);
        if (hi >= 0 && s_h [hi].attachments > 0)
            s_h [hi].attachments--;
        loader_unlock ();
    }
