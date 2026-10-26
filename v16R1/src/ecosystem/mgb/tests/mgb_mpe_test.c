#define _POSIX_C_SOURCE 200809L /* setenv prototype */
#include <errno.h>
#include <sys/stat.h>
/* mgb_mpe_test.c — headless gates for the 475 copy.
 * Tier A (no s2tui): golden frame -> stage -> kinematic bodies + zero-force
 * joints + quarantine (staged bodies do not move under stepping).
 * Tier B (live): petri through an explicitly selected MGB_S2TUI_BIN;
 * skips gracefully when absent. Exit 0 green, 1 red, counts at the end.
 */
#include "mgb.h"
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "config/mpe_config.h"
#include "physics/constraint.h"
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
int mgb_sink_mpe_move (physics_world *world, const mgb_frame_t *frame,
                       double mm_per_a, float ox, float oy, float oz,
                       int *index_map, int map_cap);

static int npass = 0, nfail = 0;
static char g_scratch_dir[PATH_MAX];
static char g_absent_s2tui[PATH_MAX];
static char g_absent_module[PATH_MAX];
static char g_nonexec_probe[PATH_MAX];
static char g_badexec_probe[PATH_MAX];
static char g_fake_s2tui[PATH_MAX];
static char g_closed_stdin_probe[PATH_MAX];
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
    int all_kinematic = 1, finite = 1;
    for (int i = 0; i < 3; i++) {
        rigidbody *rb = &world.bodies[i];
        if (!rb->kinematic || rb->static_state ||
            rigidbody_effective_inv_mass (rb) != 0.0f) all_kinematic = 0;
        if (!isfinite (rb->position.x + rb->position.y + rb->position.z))
            finite = 0;
    }
    CHECK (all_kinematic, "all staged bodies kinematic (immovable to solver)");
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
    /* Refresh must preserve body identity and move the staged pose in place. */
    {
        int first_body = map[0];
        vector3 before = world.bodies[first_body].position;
        fr.atoms[0].z += 2.0;
        CHECK (mgb_sink_mpe_move (&world, &fr, 1.0, 0.0f, 120.0f, 0.0f,
                                  map, 16) == 0,
               "refresh repositions the existing staged bodies");
        CHECK (world.body_count == 3 && map[0] == first_body,
               "refresh preserves body count and stable atom mapping");
        CHECK (vector3_length (vector3_subtraction (
                   world.bodies[first_body].position, before)) > 1e-3f &&
                   world.bodies[first_body].kinematic,
               "refresh moves the same body and keeps it kinematic");
    }
    /* Quarantine: 60 ticks must not move kinematic stage bodies. Compare before/after
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
               "kinematic stage bodies unmoved after 60 ticks");
    }
    physics_world_cleanup (&world);
    /* Attaching the display bridge to a populated scene must retain its
     * existing spring and constraint pools. */
    {
        physics_world scene;
        physics_world_init (&scene);
        int a = physics_world_add_sphere (&scene, 0.25f, 1.0f, (vector3) {0.0f, 2.0f, 0.0f});
        int b = physics_world_add_sphere (&scene, 0.25f, 1.0f, (vector3) {1.0f, 2.0f, 0.0f});
        int spring = (a >= 0 && b >= 0) ? add_joint (&scene, a, b, 1.0f, 10.0f, 0.5f) : -1;
        int fixed = (spring >= 0) ? constraint_add_fixed (&scene,
            scene.bodies[a].object_id, scene.bodies[b].object_id,
            vector3_zero (), vector3_zero ()) : -1;
        int staged_map[16];
        int staged = fixed >= 0 ? mgb_sink_mpe_stage (&scene, &fr, 1.0,
            0.0f, 120.0f, 0.0f, staged_map, 16) : -1;
        CHECK (staged == 3, "bridge stages into an already-populated scene");
        CHECK (spring_joint_count (&scene) == 3,
               "staging preserves existing spring joints");
        CHECK (constraint_get_count (&scene) == 1 &&
                   constraint_pool_at (&scene, fixed) != NULL &&
                   constraint_pool_at (&scene, fixed)->is_active,
               "staging preserves existing constraints");
        physics_world_cleanup (&scene);
    }
    mgb_frame_free (&fr);
}

static void tier_b (void) {
    printf ("== tier B: live petri (needs s2tui) ==\n");
    const char *env = getenv ("MGB_S2TUI_BIN");
    const char *path = env && *env ? env : NULL;
    if (!path) {
        /* A test must not discover and launch an unrelated executable from
         * outside this checkout. The live tier is opt-in by explicit path. */
        printf ("  SKIP live tier (set MGB_S2TUI_BIN explicitly)\n");
        return;
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
    setenv ("MGB_S2TUI_BIN", g_absent_s2tui, 1);
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

    CHECK (mgb_spawn (g_absent_module, 1) == NULL, "missing file spawn is refused");
    CHECK (mgb_spawn_reason (&e) == mgb_spawn_e_notfound, "missing file is named notfound");
    CHECK (e == ENOENT, "missing file reports ENOENT");

    /* Readable but NOT executable: the case a readability check misses. */
    {
        FILE *f = fopen (g_nonexec_probe, "w");
        CHECK (f != NULL, "non-executable fixture is created in project scratch");
        if (f) {
            fputs ("not a program\n", f);
            fclose (f);
            if (chmod (g_nonexec_probe, 0644) == 0) {
                CHECK (mgb_spawn (g_nonexec_probe, 1) == NULL, "non-executable file is refused");
                CHECK (mgb_spawn_reason (&e) == mgb_spawn_e_noexecbit, "non-executable file is named noexecbit");
                CHECK (e == EACCES, "non-executable file reports EACCES");
            } else CHECK (0, "non-executable fixture mode is set");
            remove (g_nonexec_probe);
        }
    }
    /* Executable bit set, but the contents are not a program: exec() itself
     * must fail, and that ENOEXEC must arrive instead of a bogus timeout. */
    {
        FILE *f = fopen (g_badexec_probe, "w");
        CHECK (f != NULL, "bad-exec fixture is created in project scratch");
        if (f) {
            fputs ("still not a program\n", f);
            fclose (f);
            if (chmod (g_badexec_probe, 0755) == 0) {
                CHECK (mgb_spawn (g_badexec_probe, 2) == NULL, "bad exec format is refused");
                CHECK (mgb_spawn_reason (&e) == mgb_spawn_e_noexec, "bad exec format is named noexec");
                CHECK (e == ENOEXEC, "bad exec format reports ENOEXEC");
            } else CHECK (0, "bad-exec fixture mode is set");
            remove (g_badexec_probe);
        }
    }
    /* A valid child may close its command pipe after a successful exchange.
     * The next write must return an error instead of delivering SIGPIPE to
     * the embedding process. The child closes stdin before sending its next
     * prompt, then stays alive until mgb_close reaps it. */
    {
        mgb_session_t *closed = mgb_spawn (g_closed_stdin_probe, 2);
        CHECK (closed != NULL, "closed-stdin fixture handshakes");
        if (closed) {
            CHECK (mgb_cmd (closed, "close-stdin", 2) == 0,
                   "first command completes before child closes stdin");
            CHECK (mgb_cmd (closed, "must-fail", 1) != 0,
                   "closed child pipe returns an error without SIGPIPE");
            mgb_close (closed);
        }
    }
    /* strerror must never collapse: every distinct cause has its own text. */
    CHECK (strcmp (mgb_spawn_strerror (mgb_spawn_e_notfound), mgb_spawn_strerror (mgb_spawn_e_noexecbit)) != 0,
           "notfound and noexecbit read differently");
    CHECK (strcmp (mgb_spawn_strerror (mgb_spawn_e_noexec), mgb_spawn_strerror (mgb_spawn_e_handshake)) != 0,
           "noexec and handshake read differently");
    /* DESPOT-2026-10-20: the appended wrong-program banner reason must not
     * collide with the plain handshake failure it used to fall through to. */
    CHECK (mgb_spawn_e_banner != mgb_spawn_e_handshake &&
           strcmp (mgb_spawn_strerror (mgb_spawn_e_banner), mgb_spawn_strerror (mgb_spawn_e_handshake)) != 0,
           "banner-reason reads differently from handshake");
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
    setenv ("MGB_S2TUI_BIN", g_absent_s2tui, 1);
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
    char root[PATH_MAX], fake_dir[PATH_MAX];
    if (!realpath (dir, root)) {
        perror ("realpath MGB test root");
        return 1;
    }
    int n = snprintf (g_scratch_dir, sizeof g_scratch_dir, "%s/build/mgb_mpe_test.%ld", root, (long)getpid ());
    if (n < 0 || (size_t)n >= sizeof g_scratch_dir || mkdir (g_scratch_dir, 0700) != 0) {
        perror ("create project-local MGB test scratch");
        return 1;
    }
    n = snprintf (g_absent_s2tui, sizeof g_absent_s2tui, "%s/absent_s2tui", g_scratch_dir);
    if (n < 0 || (size_t)n >= sizeof g_absent_s2tui) return 1;
    n = snprintf (g_absent_module, sizeof g_absent_module, "%s/absent_module.so", g_scratch_dir);
    if (n < 0 || (size_t)n >= sizeof g_absent_module) return 1;
    n = snprintf (g_nonexec_probe, sizeof g_nonexec_probe, "%s/not_executable", g_scratch_dir);
    if (n < 0 || (size_t)n >= sizeof g_nonexec_probe) return 1;
    n = snprintf (g_badexec_probe, sizeof g_badexec_probe, "%s/bad_executable", g_scratch_dir);
    if (n < 0 || (size_t)n >= sizeof g_badexec_probe) return 1;
    n = snprintf (fake_dir, sizeof fake_dir, "%s/4179-Magi", g_scratch_dir);
    if (n < 0 || (size_t)n >= sizeof fake_dir || mkdir (fake_dir, 0700) != 0) {
        perror ("create project-local resolver fixture directory");
        return 1;
    }
    n = snprintf (g_fake_s2tui, sizeof g_fake_s2tui, "%s/s2tui", fake_dir);
    if (n < 0 || (size_t)n >= sizeof g_fake_s2tui) return 1;
    FILE *fixture = fopen (g_fake_s2tui, "w");
    if (!fixture) {
        perror ("create project-local resolver fixture");
        return 1;
    }
    fputs ("#!/bin/sh\nexit 0\n", fixture);
    fclose (fixture);
    if (chmod (g_fake_s2tui, 0700) != 0) {
        perror ("make project-local resolver fixture executable");
        return 1;
    }
    n = snprintf (g_closed_stdin_probe, sizeof g_closed_stdin_probe,
                  "%s/closed_stdin_s2tui", g_scratch_dir);
    if (n < 0 || (size_t)n >= sizeof g_closed_stdin_probe) return 1;
    FILE *closed_fixture = fopen (g_closed_stdin_probe, "w");
    if (!closed_fixture) {
        perror ("create project-local closed-stdin fixture");
        return 1;
    }
    fputs ("#!/bin/sh\nprintf 's2tui test\\ns2> ' >&2\n"
           "IFS= read -r command || exit 1\n"
           "exec 0<&-\nprintf 's2> ' >&2\nwhile :; do :; done\n",
           closed_fixture);
    fclose (closed_fixture);
    if (chmod (g_closed_stdin_probe, 0700) != 0) {
        perror ("make closed-stdin fixture executable");
        return 1;
    }
    if (setenv ("TMPDIR", g_scratch_dir, 1) != 0) {
        perror ("set project-local MGB TMPDIR");
        return 1;
    }
    tier_a (dir);
    tier_b ();
    tier_spawn_naming ();
    if (chdir (g_scratch_dir) != 0) {
        perror ("enter project-local MGB test scratch");
        nfail++;
    }
    tier_resolver (dir);
    if (chdir (root) != 0) {
        perror ("leave project-local MGB test scratch");
        nfail++;
    }
    remove (g_fake_s2tui);
    remove (g_closed_stdin_probe);
    rmdir (fake_dir);
    rmdir (g_scratch_dir);
    printf ("mgb-mpe: PASS %d FAIL %d\n", npass, nfail);
    return nfail ? 1 : 0;
}
