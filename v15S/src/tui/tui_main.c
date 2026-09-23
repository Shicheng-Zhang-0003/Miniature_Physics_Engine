/* MPE-TUI — terminal-only real-time debugger and debug-output suite.
 *
 *   mpe-tui                              live ncurses inspector (needs a TTY)
 *   mpe-tui --snapshot [N] [--scene S]   one full state dump (pipeable)
 *   mpe-tui --stream N [--every K] [--scene S]  dumps every K ticks
 *
 * Without a TTY on stdout and without an explicit mode, --snapshot is
 * assumed so the binary works as the scriptable debugging output suite.
 * Exit code is 0 on finite state, 1 when any body goes non-finite.
 */
#include <math.h>
#include <ncurses.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "../ui_input/camera.h"
#include "core/physics_world.h"
#include "core/mpe_registry.h"
#include "physics/constraint.h"
#include "physics/spring_joint.h"
#include "config/mpe_config.h"
#include "tui_debugger.h"

/* Stubs for legacy-TU symbols referenced by spring_joint.o (render path
 * only; the TUI never calls GL rendering). Same set as the spring test. */
camera main_camera_fov;
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;
int scene_find_object_index_by_id(uint32_t id) {
    (void) id;
    return -1;
}
rigidbody *scene_resolve_object_by_id(uint32_t id) {
    (void) id;
    return NULL;
}

static void print_help(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\nModes (default: live TUI when stdout is a TTY, else --snapshot):\n");
    printf("  --snapshot [N]       run N ticks (default 600) then dump full state\n");
    printf("  --stream N           dump state every --every ticks while running N ticks\n");
    printf("  --every K            stream interval (default 60)\n");
    printf("  --scene NAME         demo|tower|pendulum|springlab|f10|stress|ccd (default demo)\n");
    printf("  --broadphase NAME    hash (default) or registered foreign backend\n");
    printf("  --solver NAME        seq-impulse (default) or registered foreign backend\n");
    printf("  --ticks N            ticks to run in live mode before auto-exit (0 = run forever)\n");
    printf("  --help               this text\n");
    printf("\nLive keys: 1..5 screens, Tab cycle, j/k select, Space pause, s step,\n");
    printf("  +/- speed, / filter, c clear filter, q quit.\n");
    printf("\nSnapshot sections: [engine] [body i] [springs] [constraints] [pairs]\n");
    printf("[islands] [stats] [result] — fixed format, diffable.\n");
}

/* ------------------------------------------------------------------ */
/* Demo scenes (deterministic; exercise every body + joint type)        */
/* ------------------------------------------------------------------ */

static void scene_floor(physics_world *world) {
    physics_world_add_cube(world, (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);
}

static void scene_tower_only(physics_world *world) {
    scene_floor(world);
    const float h = 0.4f;
    for (int i = 0; i < 6; i++) {
        physics_world_add_cube(world, (vector3){0.0f, h + (float) i * 2.0f * h, 0.0f},
                               (vector3){h, h, h}, 1.0f);
    }
}

static void scene_pendulum_only(physics_world *world) {
    scene_floor(world);
    constraint_pool_init(world);
    int pivot = physics_world_add_cube(world, (vector3){0.0f, 10.0f, 0.0f}, (vector3){0.2f, 0.2f, 0.2f}, 1.0f);
    rigidbody_set_static(&world->bodies[pivot], true);
    int bob = physics_world_add_sphere(world, 0.3f, 2.0f, (vector3){1.0f, 8.0f, 0.0f});
    constraint_add_revolute(world, world->bodies[pivot].object_id, world->bodies[bob].object_id,
                            (vector3){0.0f, 0.0f, 0.0f}, (vector3){-1.0f, 2.0f, 0.0f},
                            (vector3){0.0f, 0.0f, 1.0f});
}

/* F10 long-run validation scene replica (exact geometry/props from
 * scene_spawn_long_run_validation; spawn-overlap resolution NOT applied so
 * the opening transient is, if anything, harsher than in-engine). */
static void scene_f10_only(physics_world *world) {
    constraint_pool_init(world);
    joint_init_pool(world);
    /* Coulomb floor (top y=0, mu matched to the scene). Without it the pile
     * rests on the frictionless boundary clamp and disperses instead of
     * settling (measured 2026-09-23: 0/27 asleep, KE=30, spread 235 m at
     * 60 s). With it: 27/27 asleep, KE=0, run-max 0.0. */
    {
        int f = physics_world_add_cube(world, (vector3){0.0f, -0.5f, 0.0f},
                                       (vector3){30.0f, 0.5f, 30.0f}, 0.0f);
        if (f >= 0) {
            world->bodies[f].friction_static = 0.8f;
            world->bodies[f].friction_kinetic = 0.7f;
            world->bodies[f].restitution = 0.0f;
        }
    }
    for (int i = 0; i < 10; i++) {
        int idx = physics_world_add_cube(world, (vector3){20.0f, 0.5f + (float) i * 0.99f, 0.0f},
                                         (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
        if (idx >= 0) {
            world->bodies[idx].restitution = 0.0f;
            world->bodies[idx].friction_static = 0.8f;
            world->bodies[idx].friction_kinetic = 0.7f;
        }
    }
    for (int gx = 0; gx < 3; gx++) {
        for (int gz = 0; gz < 3; gz++) {
            int idx = physics_world_add_cube(
                world, (vector3){-20.0f + ((float) gx - 1.0f) * 1.1f, 0.5f, ((float) gz - 1.0f) * 1.1f},
                (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
            if (idx >= 0) {
                world->bodies[idx].restitution = 0.0f;
                world->bodies[idx].friction_static = 0.8f;
                world->bodies[idx].friction_kinetic = 0.7f;
            }
        }
    }
    for (int gx = 0; gx < 2; gx++) {
        for (int gz = 0; gz < 2; gz++) {
            int idx = physics_world_add_cube(
                world, (vector3){-20.0f + ((float) gx - 0.5f) * 1.1f, 1.49f, ((float) gz - 0.5f) * 1.1f},
                (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
            if (idx >= 0) {
                world->bodies[idx].restitution = 0.0f;
                world->bodies[idx].friction_static = 0.8f;
                world->bodies[idx].friction_kinetic = 0.7f;
            }
        }
    }
    int top = physics_world_add_cube(world, (vector3){-20.0f, 2.48f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 1.0f);
    if (top >= 0) {
        world->bodies[top].restitution = 0.0f;
        world->bodies[top].friction_static = 0.8f;
        world->bodies[top].friction_kinetic = 0.7f;
    }
    for (int i = 0; i < 3; i++) {
        int idx = physics_world_add_sphere(world, 0.35f, 1.0f, (vector3){-30.0f + (float) i * 3.0f, 0.35f, 8.0f});
        if (idx >= 0) {
            world->bodies[idx].restitution = 0.0f;
            world->bodies[idx].friction_static = 0.8f;
            world->bodies[idx].friction_kinetic = 0.7f;
        }
    }
}

/* Spawn-stress scene: 300 mixed bodies in a grid + one of every joint
 * type + a kinematic conveyor + a fast CCD ball. Deterministic. */
static void scene_stress_only(physics_world *world) {
    constraint_pool_init(world);
    joint_init_pool(world);
    for (int i = 0; i < 100; i++) {
        float x = (float) (i % 10) * 1.2f - 5.0f;
        float z = (float) ((i / 10) % 10) * 1.2f - 5.0f;
        float y = 2.0f + (float) (i / 100);
        physics_world_add_sphere(world, 0.35f, 1.0f, (vector3){x, y, z});
        physics_world_add_cube(world, (vector3){x + 0.5f, y + 2.0f, z}, (vector3){0.35f, 0.35f, 0.35f},
                               1.0f);
        physics_world_add_cylinder(world, 0.3f, 0.4f, 1.0f, (vector3){x, y + 4.0f, z + 0.5f});
    }
    int anchor = physics_world_add_cube(world, (vector3){0.0f, 50.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f},
                                        0.0f);
    int smass = physics_world_add_sphere(world, 0.2f, 1.0f, (vector3){2.5f, 50.0f, 0.0f});
    add_joint_by_ids(world, world->bodies[anchor].object_id, world->bodies[smass].object_id, 2.0f, 20.0f,
                     0.0f);
    int pivot = physics_world_add_cube(world, (vector3){8.0f, 10.0f, 0.0f}, (vector3){0.2f, 0.2f, 0.2f},
                                       1.0f);
    rigidbody_set_static(&world->bodies[pivot], true);
    int bob = physics_world_add_sphere(world, 0.3f, 2.0f, (vector3){9.0f, 8.0f, 0.0f});
    constraint_add_revolute(world, world->bodies[pivot].object_id, world->bodies[bob].object_id,
                            (vector3){0.0f, 0.0f, 0.0f}, (vector3){-1.0f, 2.0f, 0.0f},
                            (vector3){0.0f, 0.0f, 1.0f});
    int plat = physics_world_add_cube(world, (vector3){0.0f, 0.25f, -8.0f}, (vector3){1.0f, 0.25f, 1.0f},
                                      1.0f);
    rigidbody_set_kinematic(&world->bodies[plat], true);
    world->bodies[plat].velocity = (vector3){1.5f, 0.0f, 0.0f};
    physics_world_add_cube(world, (vector3){0.0f, 0.75f, -8.0f}, (vector3){0.25f, 0.25f, 0.25f}, 1.0f);
    int fast = physics_world_add_sphere(world, 0.5f, 1.0f, (vector3){-30.0f, 5.0f, 8.0f});
    world->bodies[fast].velocity = (vector3){144.0f, 0.0f, 0.0f};
    world->bodies[fast].restitution = 0.0f;
    rigidbody_wake(&world->bodies[fast]);
}

/* CCD battery: thin static wall + three restitution-0 balls at
 * 60/144/300 m/s in separate z lanes. */
static void scene_ccd_only(physics_world *world) {
    physics_world_add_cube(world, (vector3){0.0f, 5.0f, 0.0f}, (vector3){0.05f, 5.0f, 8.0f}, 0.0f);
    float speeds[3] = {60.0f, 144.0f, 300.0f};
    for (int k = 0; k < 3; k++) {
        int s = physics_world_add_sphere(world, 0.5f, 1.0f, (vector3){-5.7f, 5.0f, -5.0f + 5.0f * k});
        world->bodies[s].velocity = (vector3){speeds[k], 0.0f, 0.0f};
        world->bodies[s].restitution = 0.0f;
        rigidbody_wake(&world->bodies[s]);
    }
}

static void scene_springlab_only(physics_world *world) {
    /* Per-scene config: zero-g vacuum WITHOUT touching the global g_cfg,
     * so other scenes/runs in this process are unaffected. */
    mpe_config_t *sc = mpe_world_cfg_mut(world);
    sc->world.gravity = 0.0f;
    sc->world.drag = 1.0f;
    sc->world.angular_damping_scale = 1.0f;
    constraint_pool_init(world);
    joint_init_pool(world);
    int anchor = physics_world_add_cube(world, (vector3){0.0f, 50.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 0.0f);
    int mass = physics_world_add_sphere(world, 0.2f, 1.0f, (vector3){2.5f, 50.0f, 0.0f});
    add_joint_by_ids(world, world->bodies[anchor].object_id, world->bodies[mass].object_id, 2.0f, 20.0f, 0.0f);
}

static void scene_demo(physics_world *world) {
    scene_floor(world);
    constraint_pool_init(world);
    joint_init_pool(world);

    /* 3-cube tower (stacking truth). */
    const float h = 0.4f;
    for (int i = 0; i < 3; i++) {
        physics_world_add_cube(world, (vector3){-3.0f, h + (float) i * 2.0f * h, 0.0f},
                               (vector3){h, h, h}, 1.0f);
    }
    /* Pure-rolling sphere (rolling-resistance truth). */
    int roller = physics_world_add_sphere(world, 0.5f, 1.0f, (vector3){3.0f, 0.5f, 0.0f});
    world->bodies[roller].velocity = (vector3){2.0f, 0.0f, 0.0f};
    world->bodies[roller].angular_velocity = (vector3){0.0f, 0.0f, -4.0f};
    rigidbody_wake(&world->bodies[roller]);
    /* Cylinder wheel. */
    int wheel = physics_world_add_cylinder(world, 0.3f, 0.15f, 2.0f, (vector3){5.0f, 0.6f, 2.0f});
    world->bodies[wheel].velocity = (vector3){1.0f, 0.0f, 0.0f};
    rigidbody_wake(&world->bodies[wheel]);

    /* Spring lab, high above contacts. */
    int anchor = physics_world_add_cube(world, (vector3){0.0f, 50.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 0.0f);
    int smass = physics_world_add_sphere(world, 0.2f, 1.0f, (vector3){2.5f, 50.0f, 0.0f});
    add_joint_by_ids(world, world->bodies[anchor].object_id, world->bodies[smass].object_id, 2.0f, 20.0f, 0.0f);

    /* Revolute pendulum. */
    int pivot = physics_world_add_cube(world, (vector3){8.0f, 10.0f, 0.0f}, (vector3){0.2f, 0.2f, 0.2f}, 1.0f);
    rigidbody_set_static(&world->bodies[pivot], true);
    int bob = physics_world_add_sphere(world, 0.3f, 2.0f, (vector3){9.0f, 8.0f, 0.0f});
    int rev = constraint_add_revolute(world, world->bodies[pivot].object_id, world->bodies[bob].object_id,
                                      (vector3){0.0f, 0.0f, 0.0f}, (vector3){-1.0f, 2.0f, 0.0f},
                                      (vector3){0.0f, 0.0f, 1.0f});
    (void) rev;

    /* Prismatic slider (free vertical slide, limited). */
    int pa = physics_world_add_cube(world, (vector3){-8.0f, 20.0f, 0.0f}, (vector3){0.3f, 0.3f, 0.3f}, 1.0f);
    int pb = physics_world_add_cube(world, (vector3){-8.0f, 21.0f, 0.0f}, (vector3){0.3f, 0.3f, 0.3f}, 1.0f);
    int pri = constraint_add_prismatic(world, world->bodies[pa].object_id, world->bodies[pb].object_id,
                                       (vector3){0.0f, 0.0f, 0.0f}, (vector3){0.0f, -1.0f, 0.0f},
                                       (vector3){0.0f, 1.0f, 0.0f});
    if (pri >= 0) {
        constraint_set_prismatic_limits(world, pri, true, -0.5f, 0.5f);
    }

    /* Rope (slack) + distance rod + fixed weld, all free-falling groups. */
    int r1 = physics_world_add_sphere(world, 0.25f, 1.0f, (vector3){2.0f, 30.0f, 0.0f});
    int r2 = physics_world_add_sphere(world, 0.25f, 1.0f, (vector3){3.2f, 30.0f, 0.0f});
    constraint_add_rope(world, world->bodies[r1].object_id, world->bodies[r2].object_id,
                        (vector3){0.0f, 0.0f, 0.0f}, (vector3){0.0f, 0.0f, 0.0f}, 2.0f);
    int d1 = physics_world_add_sphere(world, 0.25f, 1.0f, (vector3){-2.0f, 35.0f, 0.0f});
    int d2 = physics_world_add_sphere(world, 0.25f, 1.0f, (vector3){-1.0f, 35.0f, 0.0f});
    constraint_add_distance(world, world->bodies[d1].object_id, world->bodies[d2].object_id,
                            (vector3){0.0f, 0.0f, 0.0f}, (vector3){0.0f, 0.0f, 0.0f}, 1.0f);
    int f1 = physics_world_add_cube(world, (vector3){2.0f, 40.0f, 0.0f}, (vector3){0.25f, 0.25f, 0.25f}, 1.0f);
    int f2 = physics_world_add_cube(world, (vector3){2.0f, 40.5f, 0.0f}, (vector3){0.25f, 0.25f, 0.25f}, 1.0f);
    constraint_add_fixed(world, world->bodies[f1].object_id, world->bodies[f2].object_id,
                         (vector3){0.0f, 0.25f, 0.0f}, (vector3){0.0f, -0.25f, 0.0f});

    /* Kinematic conveyor + rider (prescribed velocity truth). */
    int plat = physics_world_add_cube(world, (vector3){0.0f, 0.25f, -5.0f}, (vector3){1.0f, 0.25f, 1.0f}, 1.0f);
    rigidbody_set_kinematic(&world->bodies[plat], true);
    world->bodies[plat].velocity = (vector3){1.5f, 0.0f, 0.0f};
    physics_world_add_cube(world, (vector3){0.0f, 0.75f, -5.0f}, (vector3){0.25f, 0.25f, 0.25f}, 1.0f);
}

static int build_scene(physics_world *world, const char *name) {
    if (!name || strcmp(name, "demo") == 0) {
        scene_demo(world);
        return 0;
    }
    if (strcmp(name, "tower") == 0) {
        scene_tower_only(world);
        return 0;
    }
    if (strcmp(name, "pendulum") == 0) {
        scene_pendulum_only(world);
        return 0;
    }
    if (strcmp(name, "springlab") == 0) {
        scene_springlab_only(world);
        return 0;
    }
    if (strcmp(name, "f10") == 0) {
        scene_f10_only(world);
        return 0;
    }
    if (strcmp(name, "stress") == 0) {
        scene_stress_only(world);
        return 0;
    }
    if (strcmp(name, "ccd") == 0) {
        scene_ccd_only(world);
        return 0;
    }
    fprintf(stderr, "mpe-tui: unknown scene '%s' (demo|tower|pendulum|springlab|f10|stress|ccd)\n", name);
    return -1;
}

/* ------------------------------------------------------------------ */

static double now_seconds(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

int main(int argc, char *argv[]) {
    const float dt = 1.0f / 60.0f;
    const char *scene = "demo";
    const char *want_broadphase = NULL;
    const char *want_solver = NULL;
    long ticks = 600;
    long every = 60;
    long live_ticks = 0;
    bool want_snapshot = false;
    bool want_stream = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_help(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "--snapshot") == 0) {
            want_snapshot = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                ticks = atol(argv[++i]);
            }
        } else if (strcmp(argv[i], "--stream") == 0) {
            want_stream = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                ticks = atol(argv[++i]);
            }
        } else if (strcmp(argv[i], "--every") == 0 && i + 1 < argc) {
            every = atol(argv[++i]);
        } else if (strcmp(argv[i], "--scene") == 0 && i + 1 < argc) {
            scene = argv[++i];
        } else if (strcmp(argv[i], "--broadphase") == 0 && i + 1 < argc) {
            want_broadphase = argv[++i];
        } else if (strcmp(argv[i], "--solver") == 0 && i + 1 < argc) {
            want_solver = argv[++i];
        } else if (strcmp(argv[i], "--ticks") == 0 && i + 1 < argc) {
            live_ticks = atol(argv[++i]);
        } else {
            fprintf(stderr, "mpe-tui: unknown option '%s' (see --help)\n", argv[i]);
            return 2;
        }
    }
    if (ticks < 0) {
        ticks = 0;
    }
    if (every <= 0) {
        every = 1;
    }
    /* Non-TTY stdout without an explicit live request becomes the suite. */
    bool tty = isatty(STDOUT_FILENO) != 0;
    if (!want_snapshot && !want_stream && !tty) {
        want_snapshot = true;
    }

    mpe_config_init();

    if (want_snapshot || want_stream) {
        physics_world world;
        physics_world_init(&world);
        /* Per-run config copy: scenes may tune physics (springlab vacuum)
         * without leaking into the global registry or other runs. */
        mpe_config_t tui_scene_cfg;
        tui_scene_cfg = g_cfg;
        physics_world_set_config(&world, &tui_scene_cfg);
        mpe_register_builtins();
        if (want_broadphase) {
            const mpe_broadphase_if_t *b = mpe_find_broadphase(want_broadphase);
            if (!b) {
                fprintf(stderr, "mpe-tui: unknown broadphase '%s'\n", want_broadphase);
                physics_world_cleanup(&world);
                return 2;
            }
            physics_world_set_broadphase(&world, b);
        }
        if (want_solver) {
            const mpe_solver_if_t *s = mpe_find_solver(want_solver);
            if (!s) {
                fprintf(stderr, "mpe-tui: unknown solver '%s'\n", want_solver);
                physics_world_cleanup(&world);
                return 2;
            }
            physics_world_set_solver(&world, s);
        }
        if (build_scene(&world, scene) != 0) {
            physics_world_cleanup(&world);
            return 2;
        }
        int rc = 0;
        if (want_stream) {
            if (tui_dump_snapshot(stdout, &world, 0, dt) != 0) {
                rc = 1;
            }
            for (long t = 1; t <= ticks; t++) {
                physics_world_step(&world, dt);
                if (t % every == 0 || t == ticks) {
                    printf("\n");
                    if (tui_dump_snapshot(stdout, &world, (unsigned long) t, dt) != 0) {
                        rc = 1;
                    }
                }
            }
        } else {
            for (long t = 0; t < ticks; t++) {
                physics_world_step(&world, dt);
            }
            rc = tui_dump_snapshot(stdout, &world, (unsigned long) ticks, dt);
        }
        physics_world_cleanup(&world);
        return rc;
    }

    /* ---------------- live ncurses inspector ---------------- */
    if (!isatty(STDIN_FILENO)) {
        fprintf(stderr, "mpe-tui: live mode needs a terminal; use --snapshot/--stream without one.\n");
        return 2;
    }
    physics_world world;
    physics_world_init(&world);
    mpe_config_t tui_live_cfg;
    tui_live_cfg = g_cfg;
    physics_world_set_config(&world, &tui_live_cfg);
    if (build_scene(&world, scene) != 0) {
        physics_world_cleanup(&world);
        return 2;
    }
    tui_debugger_t dbg;
    tui_debugger_init(&dbg, &world);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(50);
    if (has_colors()) {
        start_color();
    }

    double acc = 0.0;
    double last = now_seconds();
    unsigned long tick = 0;
    bool quit = false;
    while (!quit) {
        int ch = getch();
        if (ch == 'q' || ch == 'Q') {
            quit = true;
        } else if (ch != ERR) {
            tui_debugger_handle_input(&dbg, ch);
        }
        double now = now_seconds();
        double frame = now - last;
        last = now;
        if (frame > 0.5) {
            frame = 0.5;
        }
        if (!dbg.paused) {
            acc += frame * (double) dbg.time_scale;
            int sub = 0;
            while (acc >= dt && sub < 5) {
                physics_world_step(&world, dt);
                tick++;
                dbg.frame_count++;
                acc -= dt;
                sub++;
                if (live_ticks > 0 && tick >= (unsigned long) live_ticks) {
                    quit = true;
                    break;
                }
            }
            if (sub == 5) {
                acc = 0.0; /* anti-spiral: drop excess wall time */
            }
        } else if (live_ticks > 0 && tick >= (unsigned long) live_ticks) {
            quit = true;
        }
        tui_debugger_render(&dbg);
    }

    tui_debugger_cleanup(&dbg);
    endwin();
    int final_bodies = world.body_count;
    physics_world_cleanup(&world);
    printf("[tui] ticks=%lu bodies=%d\n", tick, final_bodies);
    return 0;
}
