#define _GNU_SOURCE /* dladdr + Dl_info */
/* s2_bridge.c — MPI "s2-bridge" module: a live s2tui per world.
 *
 * Lifecycle: attach resolves the s2tui binary (cfg > $MGB_S2TUI_BIN >
 * sibling default), spawns + handshakes, configures the scene through the
 * S2 shell, stages the first frame (static bodies + zero-force joints).
 * pre_step refreshes every cfg.every ticks (snapshot -> move staged
 * bodies; count change -> restage). detach kills + reaps the child.
 * Child death or parse failure freezes the last good frame, counts the
 * error, and prints loudly on stderr — physics never notices (display-only
 * module, deterministic=false: S2 doubles + libm + wall-clock pipes stay
 * outside the determinism envelope by design).
 */
#include "s2_bridge.h"
#include "mgb.h"
#include "core/mpe_module.h"
#include "core/physics_world.h"
#include "core/mpe_diag.h"
#include "core/rigidbody.h"
#include <dlfcn.h>
#include <errno.h>
#include <limits.h>
#ifndef PATH_MAX
#define PATH_MAX 4096
#endif
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int mgb_sink_mpe_stage (physics_world *world, const mgb_frame_t *frame,
                        double mm_per_a, float ox, float oy, float oz,
                        int *index_map, int map_cap);

typedef struct {
    mgb_session_t *s;
    s2_bridge_cfg_t cfg;
    int *index_map;
    int map_cap;
    int base_count;         /* world body_count before staging */
    unsigned long tick;
    int frames, errors;
    int atoms;
    int dead;               /* child gone: frozen, loud once */
} bridge_state_t;

void s2_bridge_default_cfg (s2_bridge_cfg_t *cfg) {
    if (!cfg) return;
    memset (cfg, 0, sizeof *cfg);
    cfg->s2tui_path[0] = '\0';  /* resolved at attach */
    snprintf (cfg->scene, sizeof cfg->scene, "petri");
    cfg->mm_per_a = MGB_MM_PER_A;
    cfg->every = 0;
    /* DESPOT-2026-10-09: the stage offset used to be (0,120,0), chosen to
     * lift the dish clear of the default scene. It also put the entire dish
     * 100 units ABOVE the engine's default camera, which sits at (0,20,50)
     * looking down -Z with pitch 0 through a 45-degree vertical FOV. Every
     * staged atom was 53..76 degrees off-axis against a 22.5-degree
     * half-FOV, so the renderer's own frustum test culled all 776 of them:
     * attach reported success and the screen stayed empty. Placement has to
     * be derived from where the camera actually looks, not from where it is
     * convenient to put the data.
     *
     * (0, 20, -60) centres the dish on the default view axis, 110 units in
     * front of the camera. The bodies are static and the bonds are k=0, so
     * this placement is inert to the simulation and clear of the default
     * cubes (which sit at z >= 0). Override with MGB_OX/MGB_OY/MGB_OZ. */
    cfg->ox = 0.0f;
    cfg->oy = 20.0f;
    cfg->oz = -60.0f;
    {
        const char *e;
        if ((e = getenv ("MGB_OX")) != NULL && *e) cfg->ox = (float) atof (e);
        if ((e = getenv ("MGB_OY")) != NULL && *e) cfg->oy = (float) atof (e);
        if ((e = getenv ("MGB_OZ")) != NULL && *e) cfg->oz = (float) atof (e);
    }
}

/* Executable, not merely readable. A readable-but-not-executable s2tui
 * passed this check and then failed inside execl, which surfaced as the
 * same NULL as a timeout. errno is kept so the report can say WHICH. */
static int file_ok (const char *p, int *why_errno) {
    if (why_errno) *why_errno = 0;
    if (access (p, X_OK) != 0) {
        if (why_errno) *why_errno = errno;
        return 0;
    }
    return 1;
}
/* Compact trail of every location tried and why each was rejected, so a
 * miss reports the SEARCH, not just the outcome. */
static char g_search[512];
static void note_try (const char *what, const char *why) {
    size_t used = strlen (g_search);
    if (used + strlen (what) + strlen (why) + 8 >= sizeof g_search) return;
    snprintf (g_search + used, sizeof g_search - used, "%s%s=%s;", used ? " " : "", what, why);
}
static const char *errno_word (int e) {
    switch (e) {
        case ENOENT: return "ENOENT";
        case EACCES: return "EACCES";
        case ENOEXEC: return "ENOEXEC";
        case ELOOP: return "ELOOP";
        case ENOTDIR: return "ENOTDIR";
        case ENAMETOOLONG: return "ENAMETOOLONG";
        default: return "other";
    }
}

static void s2_self_anchor (void) {
}
static int dladdr_failed = 0;

/* Walk up from `from_dir` looking for the sibling tree, validating EVERY
 * candidate with access(X_OK). Returns 1 on a hit.
 *
 * DESPOT-2026-10-09: this replaced a hardcoded "strip 7 path components"
 * scheme. That arithmetic silently assumed one exact install depth, so any
 * symlinked or renamed checkout produced a path containing ".." segments
 * that the kernel then had to resolve through a directory that may not
 * exist -- and the failure surfaced much later as a spawn error against a
 * path nobody could find. A bounded upward search that only ever returns a
 * VERIFIED executable is correct for any nesting depth, any symlink, and
 * any checkout name, and it can never hand back a non-existent path. */
#define S2_REL_MAX_UP 14
static int find_s2tui_upward (const char *from_dir, char *out, size_t out_cap) {
    const char *rel = "4179-Magi/S2/biological/v9R4/s2tui";
    char dir [PATH_MAX];
    if (!from_dir || !*from_dir) { return 0; }
    snprintf (dir, sizeof (dir), "%s", from_dir);
    for (int up = 0; up <= S2_REL_MAX_UP; up++) {
        char cand [PATH_MAX];
        int n = snprintf (cand, sizeof (cand), "%s/%s", dir, rel);
        if (n > 0 && (size_t) n < sizeof (cand)) {
            int why = 0;
            if (file_ok (cand, &why)) {
                /* Normalise on success so later messages are unambiguous. */
                char real [PATH_MAX];
                const char *use = realpath (cand, real) ? real : cand;
                if (strlen (use) < out_cap) {
                    snprintf (out, out_cap, "%s", use);
                    return 1;
                }
                note_try (up == 0 ? "beside-so" : "beside-so-up", "PATH_TOO_LONG");
            }
        }
        size_t L = strlen (dir);
        while (L > 1 && dir [L - 1] != '/') L--;
        if (L > 1) { dir [L - 1] = '\0'; }
        if (strlen (dir) <= 1) { break; }
    }
    return 0;
}

static void resolve_path (s2_bridge_cfg_t *cfg) {
    if (cfg->s2tui_path[0]) return;
    g_search[0] = '\0';

    /* 1. Explicit override always wins, and is validated: an unset or
     *    wrong MGB_S2TUI_BIN must fail HERE with a name, not three frames
     *    later inside execl. */
    const char *env = getenv ("MGB_S2TUI_BIN");
    if (env && *env) {
        int why = 0;
        if (file_ok (env, &why)) {
            /* Normalise the override as well. It used to be stored verbatim,
             * so a relative MGB_S2TUI_BIN produced a relative path here while
             * every other branch produced an absolute one -- and a relative
             * path silently depends on the CWD of whatever reads it next. */
            char real [PATH_MAX]; /* realpath() demands PATH_MAX */
            const char *use = realpath (env, real) ? real : env;
            if (strlen (use) < sizeof cfg->s2tui_path) {
                snprintf (cfg->s2tui_path, sizeof cfg->s2tui_path, "%s", use);
                return;
            }
            MPE_DIAG_ERROR ("s2bridge", "E_BADENV", "attach rejected: MGB_S2TUI_BIN resolves to a path too long to store (%zu >= %zu)",
                            strlen (use), sizeof cfg->s2tui_path);
            note_try ("env", "PATH_TOO_LONG");
            cfg->s2tui_path[0] = '\0';
            return;
        }
        /* An EXPLICIT override that does not resolve is fatal, and we stop
         * here. Falling back to the search would silently run a DIFFERENT
         * binary than the one the operator explicitly asked for — the worst
         * possible outcome for an override, which exists precisely to pin
         * the executable. Name the typo instead. */
        MPE_DIAG_ERROR ("s2bridge", "E_BADENV",
                        "attach rejected: MGB_S2TUI_BIN='%s' is not an executable file (%s). It is set, so no fallback "
                        "search was attempted — fix the variable or unset it",
                        env, errno_word (why));
        note_try ("env", errno_word (why));
        cfg->s2tui_path[0] = '\0';
        return;
    }
    note_try ("env", "unset");

    /* 2. Next to THIS image. Derived from dladdr on a static anchor (local
     *    binding, so a same-named GLOBAL in another plugin cannot interpose
     *    and steer this arithmetic) — but only the DIRECTORY is used now, so
     *    no component count is baked in. */
    Dl_info info;
    memset (&info, 0, sizeof info);
    dladdr_failed = dladdr ((const void *) s2_self_anchor, &info) == 0;
    if (dladdr ((const void *) s2_self_anchor, &info) && info.dli_fname &&
        strstr (info.dli_fname, "mgb_bridge")) {
        char self [PATH_MAX];
        snprintf (self, sizeof (self), "%s", info.dli_fname);
        char dir [PATH_MAX];
        const char *slash = strrchr (self, '/');
        if (slash) {
            size_t dl = (size_t) (slash - self);
            if (dl == 0) dl = 1; /* "/mgb_bridge.so" */
            memcpy (dir, self, dl);
            dir [dl] = '\0';
        } else {
            snprintf (dir, sizeof (dir), "%s", ".");
            note_try ("beside-so", "dli_fname has no directory (relative load)");
        }
        if (find_s2tui_upward (dir, cfg->s2tui_path, sizeof cfg->s2tui_path)) { return; }
    } else
        note_try ("beside-so", dladdr_failed ? "DLADDR_FAILED" : "IMAGE_MISMATCH");

    /* 3. Last resort: the same upward search from the working directory. */
    {
        char cwd [PATH_MAX];
        if (getcwd (cwd, sizeof (cwd)) && find_s2tui_upward (cwd, cfg->s2tui_path, sizeof cfg->s2tui_path)) { return; }
    }

    /* A miss stays a miss. This used to install an unvalidated guess, which
     * handed spawn a path that never existed. */
    cfg->s2tui_path[0] = '\0';
}

/* Testable wrapper: runs the whole search and reports what it decided.
 * Exported (not static) so the suite gates the REAL resolver instead of a
 * re-implementation of it -- a duplicate of this logic in the test is
 * exactly how the original defect survived. */
int s2_bridge_resolve_s2tui (char *out, size_t out_cap) {
    if (!out || out_cap == 0) { return 0; }
    out [0] = '\0';
    s2_bridge_cfg_t probe;
    s2_bridge_default_cfg (&probe);
    resolve_path (&probe);
    if (!probe.s2tui_path [0]) { out [0] = '\0'; return 0; }
    snprintf (out, out_cap, "%s", probe.s2tui_path);
    return 1;
}
static int bridge_stage (mpe_world_t *world, bridge_state_t *st) {
    char *snap = mgb_snapshot (st->s, 60);
    if (!snap) return -1;
    mgb_frame_t fr;
    memset (&fr, 0, sizeof fr);
    int rc = mgb_frame_parse (snap, &fr);
    free (snap);
    if (rc != 0) return -1;
    if (fr.natoms > st->map_cap) {
        mgb_frame_free (&fr);
        return -1;
    }
    int n = mgb_sink_mpe_stage (world, &fr, st->cfg.mm_per_a, st->cfg.ox,
                                st->cfg.oy, st->cfg.oz, st->index_map,
                                st->map_cap);
    st->atoms = fr.natoms;
    mgb_frame_free (&fr);
    return n < 0 ? -1 : 0;
}

/* Attach failure codes. The engine reports the code verbatim
 * (physics_world.c MPE_ATTACH_E_HOOK) and pairs it with these, so an operator
 * sees "s2bridge/E_NO_S2TUI" instead of one vague sentence. Values are
 * stable: they are a published contract for anyone scripting attach. */
#define S2B_E_NULL (-1)       /* engine called us wrong */
#define S2B_E_ALLOC (-2)      /* out of memory */
#define S2B_E_NO_S2TUI (-3)   /* resolved path is empty or not executable */
#define S2B_E_SPAWN (-4)      /* fork/exec of s2tui failed */
#define S2B_E_SCENE (-5)      /* scene name unknown */
#define S2B_E_CONFIG (-6)     /* s2tui refused our dd configure command */
#define S2B_E_MAP (-7)        /* mapper allocation failed */
#define S2B_E_STAGE (-8)      /* staging bodies into the world failed */
#define S2B_E_NOTHING (-9)    /* no atoms parsed from the live stream */

static int bridge_attach (mpe_world_t *world, void **mod_state) {
    if (!world || !mod_state) {
        MPE_DIAG_ERROR ("s2bridge", "E_NULL", "attach rejected: engine passed world=%p mod_state=%p", (const void *) world,
                        (void *) mod_state);
        return S2B_E_NULL;
    }
    bridge_state_t *st = calloc (1, sizeof *st);
    if (!st) {
        MPE_DIAG_ERROR ("s2bridge", "E_ALLOC", "attach rejected: out of memory allocating %zu bytes", sizeof *st);
        return S2B_E_ALLOC;
    }
    s2_bridge_default_cfg (&st->cfg);
    /* Per-world config could override here (mpe_world_cfg); Phase 0 takes
     * defaults + env. Documented, not silently global. */
    resolve_path (&st->cfg);
    /* Name the path we are ABOUT to use, and say so when it is empty: an
     * empty path previously became an unhelpful spawn failure three lines
     * down, with the real cause (nothing was found to run) never stated. */
    if (!st->cfg.s2tui_path [0]) {
        MPE_DIAG_ERROR ("s2bridge", "E_NO_S2TUI",
                        "attach rejected: no executable s2tui found. Searched: %s%s%s Set MGB_S2TUI_BIN=/abs/path/to/s2tui",
                        g_search[0] ? g_search : "(nothing recorded) ", g_search[0] ? "" : "",
                        (getenv ("MGB_S2TUI_BIN") && *getenv ("MGB_S2TUI_BIN")) ? ""
                                                                               : "(MGB_S2TUI_BIN is unset) ");
        free (st);
        return S2B_E_NO_S2TUI;
    }
    MPE_DIAG_INFO ("s2bridge", "E_RESOLVED", "resolved s2tui='%s' scene='%s'", st->cfg.s2tui_path, st->cfg.scene);
    st->s = mgb_spawn (st->cfg.s2tui_path, 30);
    if (!st->s) {
        int why_err = 0;
        mgb_spawn_status_t why = mgb_spawn_reason (&why_err);
        /* The EXACT cause, not a two-way guess. errno is included whenever
         * the OS knows it, which distinguishes "no such file" from "no exec
         * bit" from "exec rejected the format" from "it ran but stayed
         * silent" — previously all four arrived as one NULL. */
        /* Only the handshake case waited; every other cause failed before
         * the child could even start, so claiming a 30s wait there would
         * send the operator looking for a hang that never happened. */
        MPE_DIAG_ERROR ("s2bridge", "E_SPAWN", "attach rejected: spawn failed for '%s' — %s%s%s%s%s",
                        st->cfg.s2tui_path, mgb_spawn_strerror (why), why_err ? ", errno=" : "",
                        why_err ? errno_word (why_err) : "", why_err ? "" : "",
                        why == mgb_spawn_e_handshake ? " (waited 30s for the stderr banner)" : "");
        free (st);
        return S2B_E_SPAWN;
    }
    if (strcmp (st->cfg.scene, "petri") != 0) {
        MPE_DIAG_ERROR ("s2bridge", "E_SCENE", "attach rejected: unknown scene '%s' (only 'petri' is implemented)",
                        st->cfg.scene);
        mgb_close (st->s);
        free (st);
        return S2B_E_SCENE;
    }
    if (mgb_cmd (st->s, "dd if=petri of=world", 60) != 0) {
        MPE_DIAG_ERROR ("s2bridge", "E_CONFIG", "attach rejected: s2tui refused the configure command 'dd if=petri of=world'");
        mgb_close (st->s);
        free (st);
        return S2B_E_CONFIG;
    }
    st->map_cap = MGB_MAX_ATOMS;
    st->index_map = malloc (sizeof (int) * (size_t)st->map_cap);
    if (!st->index_map) {
        MPE_DIAG_ERROR ("s2bridge", "E_MAP", "attach rejected: cannot allocate the %d-entry atom index map", st->map_cap);
        mgb_close (st->s);
        free (st);
        return S2B_E_MAP;
    }
    st->base_count = world->body_count;
    if (bridge_stage (world, st) != 0) {
        MPE_DIAG_ERROR ("s2bridge", "E_STAGE",
                        "attach rejected: staging failed after %d atom(s) parsed; world had %d body/bodies before stage",
                        st->atoms, st->base_count);
        mgb_close (st->s);
        free (st->index_map);
        free (st);
        return S2B_E_STAGE;
    }
    if (st->atoms <= 0) {
        MPE_DIAG_ERROR ("s2bridge", "E_NOTHING",
                        "attach rejected: the live stream parsed but yielded ZERO atoms — s2tui ran and accepted "
                        "commands, but wrote no usable frame");
        mgb_close (st->s);
        free (st->index_map);
        free (st);
        return S2B_E_NOTHING;
    }
    st->frames = 1;
    *mod_state = st;
    /* Report WHERE the bodies actually landed. A successful attach that
     * renders nothing is indistinguishable from a good one unless the bounds
     * are stated: the renderer frustum-culls silently, so an off-screen or
     * sub-pixel dish is a green log and an empty screen. */
    {
        double lo [3] = {1e30, 1e30, 1e30}, hi [3] = {-1e30, -1e30, -1e30};
        double r_min = 1e30, r_max = -1e30;
        int counted = 0;
        for (int i = st->base_count; i < world->body_count; i++) {
            const rigidbody *b = &world->bodies [i];
            if (!b) continue;
            double p [3] = {b->position.x, b->position.y, b->position.z};
            for (int k = 0; k < 3; k++) {
                if (p [k] < lo [k]) lo [k] = p [k];
                if (p [k] > hi [k]) hi [k] = p [k];
            }
            if (b->radius > 0.0) {
                if (b->radius < r_min) r_min = b->radius;
                if (b->radius > r_max) r_max = b->radius;
            }
            counted++;
        }
        if (counted > 0)
            MPE_DIAG_INFO ("s2bridge", "STAGED_BOUNDS",
                           "staged %d body/bodies in slots %d..%d | centre (%.2f, %.2f, %.2f) | extent %.2f x %.2f x %.2f "
                           "| radius %.3f..%.3f. The default camera is at (0,20,50) looking down -Z with a 45 deg vertical "
                           "FOV: anything more than 22.5 deg off that axis is frustum-culled and will NOT appear.",
                           counted, st->base_count, world->body_count - 1, (lo [0] + hi [0]) * 0.5, (lo [1] + hi [1]) * 0.5,
                           (lo [2] + hi [2]) * 0.5, hi [0] - lo [0], hi [1] - lo [1], hi [2] - lo [2], r_min, r_max);
        else
            MPE_DIAG_WARN ("s2bridge", "STAGED_BOUNDS",
                           "attach reported success but staged ZERO bodies — nothing can possibly be displayed");
    }
    MPE_DIAG_INFO ("s2bridge", "OK", "live: %d atoms staged from '%s' (world body_count %d -> %d)", st->atoms,
                   st->cfg.s2tui_path, st->base_count, world->body_count);
    return 0;
}

static void bridge_detach (mpe_world_t *world, void *mod_state) {
    (void)world;
    bridge_state_t *st = mod_state;
    if (!st) return;
    mgb_close (st->s);
    free (st->index_map);
    free (st);
}

static void bridge_pre_step (mpe_world_t *world, float dt, void *mod_state) {
    (void)dt;
    bridge_state_t *st = mod_state;
    if (!world || !st || st->dead) return;
    st->tick++;
    if (st->cfg.every <= 0) return;
    if (st->tick % (unsigned long)st->cfg.every != 0) return;
    if (mgb_dead (st->s) || bridge_stage (world, st) != 0) {
        st->errors++;
        if (!st->dead) {
            st->dead = 1;
            fprintf (stderr,
                     "[s2bridge] link lost: freezing last good frame "
                     "(%d atoms, %d frames staged, errors=%d)\n",
                     st->atoms, st->frames, st->errors);
        }
        return;
    }
    st->frames++;
}

int s2_bridge_stats (mpe_world_t *world, int *frames, int *errors,
                     int *atoms) {
    (void)world;
    /* Phase 0: stats via direct state are test-internal; the debug
     * terminal + TUI section query lands with the graphical phase.
     * Returning -1 keeps callers honest until then. */
    if (frames) *frames = 0;
    if (errors) *errors = 0;
    if (atoms) *atoms = 0;
    return -1;
}

const mpe_module_desc_t mpe_module_desc = {
    1, "s2-bridge", "0.1", "generic", false,
    bridge_attach, bridge_detach, bridge_pre_step, NULL, NULL
};
