#define _POSIX_C_SOURCE 200809L /* setenv prototype */
#include <errno.h>
#include <sys/stat.h>
/* mgb_mpe_test.c — headless gates for the 475 copy.
 * Tier A (no s2tui): golden frame -> stage -> static bodies + zero-force
 * joints + quarantine (statics don't move under stepping).
 * Tier B (live): petri through s2tui (MGB_S2TUI_BIN or sibling default;
 * skips gracefully when absent). Exit 0 green, 1 red, counts at the end.
 */
#include "mgb.h"
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "config/mpe_config.h"
#include "physics/spring_joint.h"
#include "modules/s2/s2_bridge.h"
extern const mpe_module_desc_t mpe_module_desc;
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>

/* Render-path stubs for spring_joint.o (world path never calls GL).
 * Same set as tests/spring_test.c and tui/tui_main.c. */
#include "ui_input/camera.h"
camera main_camera_fov;
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;
int scene_find_object_index_by_id (uint32_t id) {
    (void)id;
    return -1;
}
rigidbody *scene_resolve_object_by_id (uint32_t id) {
    (void)id;
    return NULL;
}

int mgb_sink_mpe_stage (physics_world *world, const mgb_frame_t *frame,
                        double mm_per_a, float ox, float oy, float oz,
                        int *index_map, int map_cap);

static int npass = 0, nfail = 0;
#define CHECK(cond, name) do { \
    if (cond) { npass++; } \
    else { nfail++; printf ("  FAIL %s\n", name); } \
} while (0)

static void tier_a (const char *dir) {
    printf ("== tier A: golden stage (no s2tui) ==\n");
    char path[1024];
    snprintf (path, sizeof path, "%s/tests/golden_water.s2", dir);
    mgb_frame_t fr;
    memset (&fr, 0, sizeof fr);
    CHECK (mgb_frame_parse (path, &fr) == 0, "golden parses");
    if (fr.natoms != 3) {
        printf ("  FAIL golden counts\n");
        nfail++;
        mgb_frame_free (&fr);
        return;
    }
    mpe_config_init ();
    physics_world world;
    physics_world_init (&world);
    int map[16];
    int n = mgb_sink_mpe_stage (&world, &fr, 1.0, 0.0f, 120.0f, 0.0f, map,
                                16);
    CHECK (n == 3, "3 bodies staged");
    CHECK (world.body_count == 3, "world holds 3");
    int all_static = 1, finite = 1;
    for (int i = 0; i < 3; i++) {
        rigidbody *rb = &world.bodies[i];
        if (!rb->static_state) all_static = 0;
        if (!isfinite (rb->position.x + rb->position.y + rb->position.z))
            finite = 0;
    }
    CHECK (all_static, "all staged bodies static (solver-skipped)");
    CHECK (finite, "positions finite");
    /* DESPOT-2026-10-09: the offset is now applied to the frame's bounding
     * box CENTRE, not added to the raw S2 coordinates, so that "offset"
     * means the same thing for every scene. These two atoms therefore only
     * assert the RELATIVE geometry — the O..H separation in mm, and the
     * 0.9572 A -> 0.9572 unit scale — which is scene-independent. Absolute
     * placement is asserted where it belongs: the STAGED_BOUNDS diagnostic,
     * and the tier that checks the box centre lands on the offset target. */
    {
        double d = world.bodies[1].position.x - world.bodies[0].position.x;
        double dy = world.bodies[1].position.y - world.bodies[0].position.y;
        double dz = world.bodies[1].position.z - world.bodies[0].position.z;
        double sep = sqrt (d * d + dy * dy + dz * dz);
        /* O-H in the fixture is 0.9572 A along +x at mm_per_a = 1. */
        CHECK (fabs (sep - 0.9572) < 1e-3, "O-H separation preserved (0.9572 A -> 0.9572 units)");
        CHECK (fabs (dy) < 1e-6 && fabs (dz) < 1e-6, "O-H still lies along +x");
    }
    /* The box centre must land exactly on the requested offset target. */
    {
        double lo [3] = {1e30, 1e30, 1e30}, hi [3] = {-1e30, -1e30, -1e30};
        for (int i = 0; i < world.body_count; i++) {
            double q [3] = {world.bodies[i].position.x, world.bodies[i].position.y, world.bodies[i].position.z};
            for (int k = 0; k < 3; k++) {
                if (q [k] < lo [k]) lo [k] = q [k];
                if (q [k] > hi [k]) hi [k] = q [k];
            }
        }
        CHECK (fabs ((lo [0] + hi [0]) * 0.5 - 0.0) < 1e-3 &&
                   fabs ((lo [1] + hi [1]) * 0.5 - 120.0) < 1e-3 &&
                   fabs ((lo [2] + hi [2]) * 0.5 - 0.0) < 1e-3,
               "box centre lands on the requested stage offset (0,120,0)");
    }
    /* 2 zero-force joints */
    int nj = 0, zeroforce = 1;
    for (int i = 0; i < mpe_max_joints; i++) {
        if (world.spring_joints[i].is_active) {
            nj++;
            if (world.spring_joints[i].spring_constant != 0.0f ||
                world.spring_joints[i].damping_coefficient != 0.0f)
                zeroforce = 0;
        }
    }
    CHECK (nj == 2, "2 bond joints staged");
    CHECK (zeroforce, "joints zero-force (rendered, not physical)");
    /* Quarantine: 60 ticks must not move statics. Compare before/after
     * instead of against hardcoded coordinates -- the assertion is about
     * immobility, and pinning absolutes made it silently depend on the
     * staging offset (it broke when the offset began targeting the frame's
     * box centre rather than the raw scene origin). */
    {
        vector3 before [2];
        for (int i = 0; i < 2; i++) before [i] = world.bodies[i].position;
        for (int tick = 0; tick < 60; tick++) physics_world_step (&world, 1.0f / 60.0f);
        CHECK (vector3_length (vector3_subtraction (world.bodies[0].position, before [0])) < 1e-6 &&
                   vector3_length (vector3_subtraction (world.bodies[1].position, before [1])) < 1e-6,
               "statics unmoved after 60 ticks");
    }
    physics_world_cleanup (&world);
    mgb_frame_free (&fr);
}

static void tier_b (void) {
    printf ("== tier B: live petri (needs s2tui) ==\n");
    const char *env = getenv ("MGB_S2TUI_BIN");
    char def[1024];
    const char *path = env && *env ? env : NULL;
    if (!path) {
        snprintf (def, sizeof def,
                  "../../../../../4179-Magi/S2/biological/v9R4/s2tui");
        FILE *f = fopen (def, "r");
        if (!f) {
            printf ("  SKIP live tier (no s2tui; set MGB_S2TUI_BIN)\n");
            return;
        }
        fclose (f);
        path = def;
    }
    mgb_session_t *s = mgb_spawn (path, 30);
    CHECK (s != NULL, "spawn+handshake");
    if (!s) return;
    CHECK (mgb_cmd (s, "dd if=petri of=world", 60) == 0, "configure petri");
    char *snap = mgb_snapshot (s, 60);
    CHECK (snap != NULL, "snapshot");
    mgb_frame_t fr;
    memset (&fr, 0, sizeof fr);
    int ok = snap && mgb_frame_parse (snap, &fr) == 0;
    CHECK (ok, "petri parses");
    if (ok) {
        char d[128];
        snprintf (d, sizeof d, "(atoms=%d bonds=%d)", fr.natoms, fr.nbonds);
        CHECK (fr.natoms == 776 && fr.nbonds == 533, d);
        mpe_config_init ();
        physics_world world;
        physics_world_init (&world);
        int *map = malloc (sizeof (int) * (size_t)fr.natoms);
        int n = map ? mgb_sink_mpe_stage (&world, &fr, 1.0, 0.0f, 120.0f,
                                          0.0f, map, fr.natoms) : -1;
        snprintf (d, sizeof d, "(staged=%d)", n);
        CHECK (n == 776, d);
        int nj = 0;
        for (int i = 0; i < mpe_max_joints; i++)
            if (world.spring_joints[i].is_active) nj++;
        snprintf (d, sizeof d, "(joints=%d)", nj);
        CHECK (nj == 533, d);
        free (map);
        physics_world_cleanup (&world);
        mgb_frame_free (&fr);
    }
    free (snap);
    mgb_close (s);
    /* MPI path: bogus path must refuse cleanly (no world mutation). */
    setenv ("MGB_S2TUI_BIN", "/tmp/mgb_t_no_such_binary", 1);
    {
        mpe_config_init ();
        physics_world w2;
        physics_world_init (&w2);
        int base = w2.body_count;
        CHECK (physics_world_attach_module (&w2, &mpe_module_desc) != 0,
               "attach refuses bogus s2tui");
        CHECK (w2.body_count == base, "refused attach stages nothing");
        physics_world_cleanup (&w2);
    }
    /* MPI path live: attach stages the dish through the module. */
    setenv ("MGB_S2TUI_BIN", path, 1);
    {
        mpe_config_init ();
        physics_world w3;
        physics_world_init (&w3);
        int rc = physics_world_attach_module (&w3, &mpe_module_desc);
        CHECK (rc == 0, "attach stages live dish");
        if (rc == 0) {
            char d[128];
            snprintf (d, sizeof d, "(bodies=%d)", w3.body_count);
            CHECK (w3.body_count == 776, d);
        }
        physics_world_cleanup (&w3);
    }
}

/* Every spawn failure must NAME itself. All of these used to return NULL
 * identically, which is how a missing file, a missing exec bit and a silent
 * child became one indistinguishable "spawn failed" — and the operator had
 * to guess between them. */
static void tier_spawn_naming (void) {
    int e = 0;
    mgb_spawn_reason (&e);
    CHECK (mgb_spawn ("", 1) == NULL, "empty path spawn is refused");
    CHECK (mgb_spawn_reason (&e) == mgb_spawn_e_no_path, "empty path is named no_path");

    CHECK (mgb_spawn ("/tmp/mgb_absent_target.so", 1) == NULL, "missing file spawn is refused");
    CHECK (mgb_spawn_reason (&e) == mgb_spawn_e_notfound, "missing file is named notfound");
    CHECK (e == ENOENT, "missing file reports ENOENT");

    /* Readable but NOT executable: the case a readability check misses. */
    {
        FILE *f = fopen ("/tmp/mgb_notexec_probe", "w");
        if (f) {
            fputs ("not a program\n", f);
            fclose (f);
            if (chmod ("/tmp/mgb_notexec_probe", 0644) == 0) {
                CHECK (mgb_spawn ("/tmp/mgb_notexec_probe", 1) == NULL, "non-executable file is refused");
                CHECK (mgb_spawn_reason (&e) == mgb_spawn_e_noexecbit, "non-executable file is named noexecbit");
                CHECK (e == EACCES, "non-executable file reports EACCES");
            }
            remove ("/tmp/mgb_notexec_probe");
        }
    }
    /* Executable bit set, but the contents are not a program: exec() itself
     * must fail, and that ENOEXEC must arrive instead of a bogus timeout. */
    {
        FILE *f = fopen ("/tmp/mgb_noexec_probe", "w");
        if (f) {
            fputs ("still not a program\n", f);
            fclose (f);
            if (chmod ("/tmp/mgb_noexec_probe", 0755) == 0) {
                CHECK (mgb_spawn ("/tmp/mgb_noexec_probe", 2) == NULL, "bad exec format is refused");
                CHECK (mgb_spawn_reason (&e) == mgb_spawn_e_noexec, "bad exec format is named noexec");
                CHECK (e == ENOEXEC, "bad exec format reports ENOEXEC");
            }
            remove ("/tmp/mgb_noexec_probe");
        }
    }
    /* strerror must never collapse: every distinct cause has its own text. */
    CHECK (strcmp (mgb_spawn_strerror (mgb_spawn_e_notfound), mgb_spawn_strerror (mgb_spawn_e_noexecbit)) != 0,
           "notfound and noexecbit read differently");
    CHECK (strcmp (mgb_spawn_strerror (mgb_spawn_e_noexec), mgb_spawn_strerror (mgb_spawn_e_handshake)) != 0,
           "noexec and handshake read differently");
}
/* Resolver contract. The old scheme stripped a FIXED number of path
 * components, assuming one exact install depth. Reached through a symlinked
 * working directory it emitted
 *   <root>/4179-MPE/../4179-Magi/.../s2tui
 * which resolves ONLY while that symlink exists. When it did not, the kernel
 * returned ENOENT and the operator was shown "no such file" for a path that
 * looked entirely plausible -- the plausible-looking path was the bug.
 *
 * These gates call the REAL resolver (not a copy) and pin the two properties
 * that make that failure impossible:
 *   1. a hit is always an existing, executable file;
 *   2. a hit never contains an unresolved ".." segment.
 * A miss must be a MISS (empty), never a plausible-looking non-existent path. */
static void tier_resolver (const char *dir) {
    /* Clear any override a previous tier left behind. This gate is about the
     * SEARCH, and an inherited MGB_S2TUI_BIN short-circuits it entirely --
     * these checks must not pass merely because some other tier set the
     * variable. */
    unsetenv ("MGB_S2TUI_BIN");
    char got [1024];
    int hit = s2_bridge_resolve_s2tui (got, sizeof (got));
    if (!hit) {
        /* Legitimate only when the sibling tree genuinely is not reachable
         * from this working directory. Either way the contract is the same:
         * nothing plausible-looking may be handed back. */
        CHECK (got[0] == '\0', "a resolver miss returns an EMPTY path, never a guess");
        return;
    }
    CHECK (access (got, X_OK) == 0, "a resolver hit is an existing executable file");
    CHECK (strstr (got, "..") == NULL, "a resolver hit contains no unresolved '..' segment");
    /* Absolute and canonical: this is what every later message quotes. */
    CHECK (got[0] == '/', "a resolver hit is absolute");
    {
        /* realpath() REQUIRES a PATH_MAX output buffer; 2048 aborted the
         * process under _FORTIFY_SOURCE. */
        char real [PATH_MAX];
        CHECK (realpath (got, real) != NULL, "a resolver hit is canonicalisable");
        if (realpath (got, real)) CHECK (strcmp (got, real) == 0, "a resolver hit is already normalised");
    }
    /* An EXPLICIT override that does not resolve must be a MISS, never a
     * silent fallback to some other binary: an override exists to pin the
     * executable, and quietly running a different one is worse than failing. */
    setenv ("MGB_S2TUI_BIN", "/tmp/mgb_absent_override_s2tui", 1);
    {
        char with_env [1024];
        int h2 = s2_bridge_resolve_s2tui (with_env, sizeof (with_env));
        CHECK (h2 == 0, "an unusable MGB_S2TUI_BIN is refused, not bypassed");
        CHECK (with_env[0] == '\0', "a refused override yields an empty path, never a substitute binary");
    }
    if (hit) {
        setenv ("MGB_S2TUI_BIN", got, 1);
        char ok_env [1024];
        CHECK (s2_bridge_resolve_s2tui (ok_env, sizeof (ok_env)) == 1 && strcmp (ok_env, got) == 0,
               "a valid MGB_S2TUI_BIN is honoured and comes back normalised");
    }
    unsetenv ("MGB_S2TUI_BIN");
    (void) dir;
}
int main (int argc, char **argv) {
    const char *dir = argc > 1 ? argv[1] : ".";
    tier_a (dir);
    tier_b ();
    tier_spawn_naming ();
    tier_resolver (dir);
    printf ("mgb-mpe: PASS %d FAIL %d\n", npass, nfail);
    return nfail ? 1 : 0;
}
