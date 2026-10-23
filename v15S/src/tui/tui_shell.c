/* MPE headless shell — the full engine without a display.
 *
 * Reads POSIX-style commands from stdin and runs the real physics step
 * (physics_world_step) on demand. Every command below operates on the live
 * physics_world, so this is genuinely "the engine without the GTK window":
 * spawn objects, build joints, save/load scenes, hot-plug modules, spawn and
 * drive FTC robots, run the validation steps, and inspect every body/joint.
 *
 * Built from engine APIs only — zero GTK/GL/ncurses coupling.
 */
#include "../core/math3d.h"
#include "../core/physics_world.h"
#include "../config/mpe_config.h"
#include "../core/mpe_loader.h"
#include "../core/mpe_module.h"
#include "../core/mpe_registry.h"
#include "../core/mpe_diag.h"
#include "../core/rigidbody.h"
#include "../core/det_math.h"
#include "../physics/constraint.h"
#include "../physics/spring_joint.h"
#include "../ecosystem/mpe_ecosystem.h"
#include "../ecosystem/mfs/modules/ftc/ftc_fleet.h"
#include "tui_headless.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/* Shell state                                                        */
/* ------------------------------------------------------------------ */
int tui_selected_object = -1;
int tui_debug_mode_active = 1;   /* headless: everything is usable */
int tui_quit_requested = 0;
int tui_paused = 0;
float tui_time_scale = 1.0f;
float tui_camera_speed = 1.0f;
vector3 tui_cam_pos = {0.0f, 2.0f, 5.0f};
float tui_cam_yaw = 0.0f, tui_cam_pitch = 0.0f;

static unsigned long tui_tick = 0;
static float tui_dt = 1.0f / 60.0f;

void tui_camera_forward (vector3 *out) {
    float cy = cosf (tui_cam_yaw * 0.0174532925f);
    float sy = sinf (tui_cam_yaw * 0.0174532925f);
    float cp = cosf (tui_cam_pitch * 0.0174532925f);
    float sp = sinf (tui_cam_pitch * 0.0174532925f);
    *out = (vector3) {cy * cp, -sp, sy * cp};
}

/* ------------------------------------------------------------------ */
/* Inspection helpers                                                 */
/* ------------------------------------------------------------------ */
static const char *tui_obj_type_name (object_type t) {
    switch (t) {
        case object_sphere: return "sphere";
        case object_cube:   return "cube  ";
        case object_cylinder: return "cylind";
        case object_custom: return "custom";
        default: return "?";
    }
}
static void tui_print_object (physics_world *w, int index) {
    rigidbody *rb = &w->bodies[index];
    const char *mode = rb->static_state ? "static" : (rb->is_sleeping ? "sleep" : "awake");
    printf ("obj %-3d id=%-5u %-7s %-6s mass=%-7.3f pos=(% .3f,% .3f,% .3f) "
            "vel=(% .3f,% .3f,% .3f) ang=(% .3f,% .3f,% .3f) fr=(%.3f,%.3f) res=%.3f\n",
            index, rb->object_id, tui_obj_type_name(rb->type), mode, rb->mass,
            rb->position.x, rb->position.y, rb->position.z,
            rb->velocity.x, rb->velocity.y, rb->velocity.z,
            rb->angular_velocity.x, rb->angular_velocity.y, rb->angular_velocity.z,
            rb->friction_static, rb->friction_kinetic, rb->restitution);
}
static void tui_print_joint (physics_world *w, int index) {
    spring_joint *j = &w->spring_joints[index];
    int ia = physics_world_index_by_id(w, j->object_id_a);
    int ib = physics_world_index_by_id(w, j->object_id_b);
    printf ("joint %-3d id_a=%-5u id_b=%-5u (a=%-3d b=%-3d) L0=%.3f k=%.3f c=%.3f\n",
            index, j->object_id_a, j->object_id_b, ia, ib,
            j->equilibrium_length, j->spring_constant, j->damping_coefficient);
}
static void tui_print_status (physics_world *w) {
    int sleeping = 0;
    for (int i = 0; i < w->body_count; i++) if (w->bodies[i].is_sleeping) sleeping++;
    printf ("tick=%lu bodies=%d joints=%d sleeping=%d pairs=? manifolds=? dt=%.5f\n",
            tui_tick, w->body_count, w->spring_joint_count, sleeping, tui_dt);
}
static void tui_print_world (physics_world *w) {
    printf ("/world\n");
    printf ("  gravity=%.4f drag=%.4f floor_fr=(%.3f,%.3f) bodies=%d joints=%d\n",
            w->cfg->world.gravity, w->cfg->world.drag,
            w->cfg->world.floor_friction_s, w->cfg->world.floor_friction_k,
            w->body_count, w->spring_joint_count);
}
void tui_print_camera (void) {
    printf ("/camera\n  position=(%.3f,%.3f,%.3f) yaw=%.2f pitch=%.2f speed=%.3f\n",
            tui_cam_pos.x, tui_cam_pos.y, tui_cam_pos.z, tui_cam_yaw, tui_cam_pitch, tui_camera_speed);
}
void tui_print_spawner (physics_world *w) {
    printf ("/spawner\n  sphere: r=%.3f m=%.3f fr=%.3f kr=%.3f\n  cube:   e=%.3f m=%.3f fr=%.3f kr=%.3f\n  cyl:    r=%.3f h=%.3f m=%.3f fr=%.3f kr=%.3f\n  speed=%.3f\n",
            w->cfg->spawner.radius, w->cfg->spawner.mass,
            w->cfg->body_defaults.sphere_fric_s, w->cfg->body_defaults.sphere_fric_k,
            w->cfg->spawner.cube_extent, w->cfg->spawner.cube_mass,
            w->cfg->body_defaults.cube_fric_s, w->cfg->body_defaults.cube_fric_k,
            w->cfg->spawner.cyl_radius, w->cfg->spawner.cyl_half_length,
            w->cfg->spawner.cyl_mass, w->cfg->body_defaults.cylinder_fric_s,
            w->cfg->body_defaults.cylinder_fric_k, w->cfg->spawner.speed);
}

/* ------------------------------------------------------------------ */
/* Command helpers                                                    */
/* ------------------------------------------------------------------ */
static int tui_resolve_object (physics_world *w, const char *token) {
    if (!token) return -1;
    if (strcmp (token, "sel") == 0 || strcmp (token, "$") == 0) return tui_selected_object;
    char *end = NULL;
    long v = strtol (token, &end, 10);
    if (end == token || *end != '\0') return -1;
    if (v < 0 || v >= w->body_count) return -1;
    return (int) v;
}

/* ------------------------------------------------------------------ */
/* Commands                                                           */
/* ------------------------------------------------------------------ */
static void cmd_help (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc > 1) {
        printf ("help for '%s': not implemented individually; run 'help' for the list\n", argv[1]);
        return;
    }
    printf ("MPE headless shell commands:\n");
    printf ("  system : help exit quit echo date whoami hostname id pwd ls ll cat status\n");
    printf ("  object : touch ps stat rm mv ln cp chown chmod kill\n");
    printf ("  config : env export config\n");
    printf ("  scene  : mount umount mkfs fsck\n");
    printf ("  module : mod ls|load|unload|attach|detach|use-broadphase|use-solver\n");
    printf ("  eco    : eco attach|detach|command\n");
    printf ("  ftc    : ftc spawn [x y z [preset]] | telemetry | drive <l> <r> | mec <f> <s> <r> | select <i>\n");
    printf ("  sim    : step [N] | run [N] | tick | halt | resume | status\n");
    printf ("  diag   : diag [tail N|sources|on|off|clear|status]\n");
}
static void cmd_exit (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; tui_quit_requested = 1; }
static void cmd_quit (physics_world *w, int argc, char **argv) { cmd_exit(w,argc,argv); }
static void cmd_clear (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; printf ("\033[2J\033[H"); fflush(stdout); }
static void cmd_echo (physics_world *w, int argc, char **argv) { (void)w; for (int i=1;i<argc;i++){if(i>1)putchar(' ');fputs(argv[i],stdout);} putchar('\n'); }
static void cmd_date (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; time_t now=time(NULL); printf ("%s", ctime(&now)); }
static void cmd_whoami (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; printf ("root\n"); }
static void cmd_hostname (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; printf ("mpe\n"); }
static void cmd_id (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; printf ("uid=0(root) gid=0(root) groups=0(root)\n"); }
static void cmd_pwd (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; printf ("/\n"); }

static void cmd_ls (physics_world *w, int argc, char **argv) {
    (void)argc;(void)argv;
    const char *path = (argc > 1) ? argv[1] : "/";
    if (strcmp(path,"/")==0 || strcmp(path,"/obj")==0) {
        printf ("/obj\n");
        for (int i=0;i<w->body_count;i++) printf ("  %d\n", i);
    } else if (strcmp(path,"/joint")==0) {
        printf ("/joint\n");
        for (int i=0;i<mpe_max_joints;i++) if (w->spring_joints[i].is_active) printf ("  %d\n", i);
    } else if (strcmp(path,"/world")==0) { printf ("/world\n"); }
    else if (strcmp(path,"/camera")==0) { printf ("/camera\n"); }
    else if (strcmp(path,"/spawner")==0) { printf ("/spawner\n"); }
    else { printf ("ls: no such path '%s'\n", path); }
}
static void cmd_ll (physics_world *w, int argc, char **argv) {
    (void)argc;(void)argv;
    const char *path = (argc > 1) ? argv[1] : "/obj";
    if (strcmp(path,"/obj")==0 || strcmp(path,"/")==0) {
        printf ("%-6s %-7s %-7s %-12s %-10s %s\n","IDX","ID","TYPE","MODE","MASS","POS");
        for (int i=0;i<w->body_count;i++) tui_print_object(w,i);
    } else if (strcmp(path,"/joint")==0) {
        for (int i=0;i<mpe_max_joints;i++) if (w->spring_joints[i].is_active) tui_print_joint(w,i);
    } else { printf ("ll: %s\n", path); }
}
static void cmd_cat (physics_world *w, int argc, char **argv) {
    if (argc < 2) { printf ("cat: usage: cat <path>\n"); return; }
    const char *p = argv[1];
    if (strcmp(p,"/world")==0) { tui_print_world(w); return; }
    if (strcmp(p,"/camera")==0) { tui_print_camera(); return; }
    if (strcmp(p,"/spawner")==0) { tui_print_spawner(w); return; }
    if (strncmp(p,"/obj/",4)==0) { int idx = atoi(p+4); if (idx>=0 && idx<w->body_count) { tui_print_object(w,idx); } else { printf ("cat: no object %d\n", idx);} return;}
    if (strncmp(p,"/joint/",7)==0) { int idx = atoi(p+7); if (idx>=0 && idx<mpe_max_joints && w->spring_joints[idx].is_active) { tui_print_joint(w,idx); } else { printf ("cat: no joint %d\n", idx);} return;}
    printf ("cat: %s\n", p);
}

static void cmd_touch (physics_world *w, int argc, char **argv) {
    const char *kind = (argc > 1) ? argv[1] : "sphere";
    float x = (argc > 2) ? atof(argv[2]) : 0.0f;
    float y = (argc > 3) ? atof(argv[3]) : 2.0f;
    float z = (argc > 4) ? atof(argv[4]) : 0.0f;
    vector3 fwd; tui_camera_forward(&fwd);
    vector3 pos = vector3_addition(tui_cam_pos, vector3_scaling(fwd, 2.0f));
    pos.x = x; pos.y = y; pos.z = z; /* explicit overrides */
    int idx = -1;
    if (strcmp(kind,"sphere")==0) idx = physics_world_add_sphere(w, 0.5f, 1.0f, pos);
    else if (strcmp(kind,"cube")==0)  idx = physics_world_add_cube(w, (vector3){0.5f,0.5f,0.5f}, pos, 1.0f);
    else if (strcmp(kind,"cyl")==0 || strcmp(kind,"cylinder")==0) idx = physics_world_add_cylinder(w, 0.5f, 0.5f, 1.0f, pos);
    else { printf ("touch: unknown kind '%s' (sphere|cube|cyl)\n", kind); return; }
    if (idx < 0) { printf ("touch: cannot create object (pool full)\n"); return; }
    rigidbody *rb = &w->bodies[idx];
    rb->friction_static = w->cfg->body_defaults.sphere_fric_s;
    rb->friction_kinetic = w->cfg->body_defaults.sphere_fric_k;
    rb->restitution = w->cfg->body_defaults.sphere_restitution;
    tui_selected_object = idx;
    printf ("touched %s #%d at (%.2f,%.2f,%.2f)\n", kind, idx, pos.x,pos.y,pos.z);
}
static void cmd_ps (physics_world *w, int argc, char **argv) {
    (void)argc;(void)argv;
    printf ("%-6s %-7s %-7s %-12s %-10s %s\n","IDX","ID","TYPE","MODE","MASS","POS");
    for (int i=0;i<w->body_count;i++) tui_print_object(w,i);
}
static void cmd_stat (physics_world *w, int argc, char **argv) {
    if (argc < 2) { printf ("stat: usage: stat <obj index>\n"); return; }
    int idx = atoi(argv[1]);
    if (idx < 0 || idx >= w->body_count) { printf ("stat: no object %d\n", idx); return; }
    tui_print_object(w, idx);
}
static void cmd_rm (physics_world *w, int argc, char **argv) {
    (void)w;(void)argc;(void)argv;
    printf ("rm: single-object removal not exposed; use 'mkfs' to clear the world.\n");
}
static void cmd_mv (physics_world *w, int argc, char **argv) {
    if (argc < 2) { printf ("mv: usage: mv <obj> [/pos/x/y/z | /vel/x/y/z]\n"); return; }
    int idx = tui_resolve_object(w, argv[1]);
    if (idx < 0) { printf ("mv: no such object '%s'\n", argv[1]); return; }
    rigidbody *rb = &w->bodies[idx];
    /* DESPOT-2026-10-20: mv /pos x y z has 6 tokens (mv obj /pos X Y Z),
     * but the guard read argc>=5 while dereferencing argv[5] — argv[5]
     * lived one past the checked range (read with exactly 5 args; UB).
     * Bodies with argc==5 (trailing component missing) truncated Z to
     * argv[4]'s garbage neighbouring token value. */
    if (argc >= 6 && strcmp(argv[2],"/pos")==0) {
        rb->position = (vector3){atof(argv[3]),atof(argv[4]),atof(argv[5])};
        printf ("mv %d -> pos (%.2f,%.2f,%.2f)\n", idx, rb->position.x,rb->position.y,rb->position.z);
    } else if (argc >= 6 && strcmp(argv[2],"/vel")==0) {
        rb->velocity = (vector3){atof(argv[3]),atof(argv[4]),atof(argv[5])};
        printf ("mv %d -> vel (%.2f,%.2f,%.2f)\n", idx, rb->velocity.x,rb->velocity.y,rb->velocity.z);
    } else { printf ("mv: usage: mv <obj> [/pos/x/y/z | /vel/x/y/z]\n"); }
}
static void cmd_ln (physics_world *w, int argc, char **argv) {
    if (argc < 3) { printf ("ln: usage: ln <obj_a> <obj_b> [rest_length]\n"); return; }
    int ia = tui_resolve_object(w, argv[1]);
    int ib = tui_resolve_object(w, argv[2]);
    if (ia < 0 || ib < 0) { printf ("ln: bad object ref\n"); return; }
    float rest = (argc > 3) ? atof(argv[3]) : 0.5f;
    int j = add_joint_by_ids(w, w->bodies[ia].object_id, w->bodies[ib].object_id, rest, 20.0f, 0.0f);
    if (j < 0) { printf ("ln: joint pool full\n"); return; }
    printf ("ln: joint %d between %d and %d (L0=%.3f)\n", j, ia, ib, rest);
}
static void cmd_cp (physics_world *w, int argc, char **argv) {
    if (argc < 2) { printf ("cp: usage: cp <obj>\n"); return; }
    int src = tui_resolve_object(w, argv[1]);
    if (src < 0) { printf ("cp: no such object\n"); return; }
    rigidbody *s = &w->bodies[src];
    vector3 pos = vector3_addition(s->position, (vector3){1.0f,0.0f,0.0f});
    int idx = -1;
    if (s->type == object_sphere) idx = physics_world_add_sphere(w, s->radius, s->mass, pos);
    else if (s->type == object_cube) idx = physics_world_add_cube(w, s->half_extensions, pos, s->mass);
    else idx = physics_world_add_cylinder(w, s->radius, s->cylinder_half_length, s->mass, pos);
    if (idx < 0) { printf ("cp: pool full\n"); return; }
    rigidbody *d = &w->bodies[idx];
    d->colour = s->colour; d->restitution = s->restitution;
    d->friction_static = s->friction_static; d->friction_kinetic = s->friction_kinetic;
    d->velocity = s->velocity; d->angular_velocity = s->angular_velocity;
    if (s->static_state) rigidbody_set_static(d, true);
    printf ("cp: %d -> %d\n", src, idx);
}
static void cmd_chown (physics_world *w, int argc, char **argv) {
    if (argc < 3) { printf ("chown: usage: chown <mass> <obj...>\n"); return; }
    float mass = atof(argv[1]);
    for (int i=2;i<argc;i++) {
        int idx = tui_resolve_object(w, argv[i]);
        if (idx < 0) continue;
        rigidbody *rb = &w->bodies[idx];
        if (mass <= 0.0f) { rigidbody_set_static(rb, true); }
        else { if (rb->static_state) rigidbody_set_static(rb,false); rb->mass = mass; rb->inverse_mass = 1.0f/mass;
               if (rb->type == object_sphere) rigidbody_update_inertia_sphere(rb);
               else if (rb->type == object_cube) rigidbody_update_inertia_cube(rb);
               else rigidbody_update_inertia_cylinder(rb);
               rigidbody_wake(rb); }
    }
}
static void cmd_chmod (physics_world *w, int argc, char **argv) {
    if (argc < 3) { printf ("chmod: usage: chmod static|dynamic <obj...>\n"); return; }
    bool make_static = (strcmp(argv[1],"static")==0);
    for (int i=2;i<argc;i++) {
        int idx = tui_resolve_object(w, argv[i]);
        if (idx >= 0) rigidbody_set_static(&w->bodies[idx], make_static);
    }
}
static void cmd_kill (physics_world *w, int argc, char **argv) {
    if (argc < 2) { printf ("kill: usage: kill <obj...>  (wakes objects)\n"); return; }
    for (int i=1;i<argc;i++) {
        int idx = tui_resolve_object(w, argv[i]);
        if (idx >= 0) rigidbody_wake(&w->bodies[idx]);
    }
}

/* ------------------------------------------------------------------ */
/* Config                                                             */
/* ------------------------------------------------------------------ */
static void cmd_env (physics_world *w, int argc, char **argv) {
    (void)w;(void)argc;(void)argv;
    int cur = -1;
    for (size_t i=0;i<g_registry_count;i++) {
        if ((int)g_registry[i].category != cur) { cur = (int)g_registry[i].category; printf ("[%s]\n", mpe_config_category_name((param_category)cur)); }
        if (g_registry[i].type == p_int) printf ("  %s = %d\n", g_registry[i].key, *(int*)g_registry[i].storage);
        else if (g_registry[i].type == p_bool) printf ("  %s = %s\n", g_registry[i].key, (*(bool*)g_registry[i].storage)?"true":"false");
        else printf ("  %s = %.6g\n", g_registry[i].key, *(float*)g_registry[i].storage);
    }
}
static void cmd_export (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc < 2) { cmd_env(w,argc,argv); return; }
    for (int i=1;i<argc;i++) {
        char *eq = strchr(argv[i],'=');
        if (!eq) { printf ("export: usage: export KEY=value\n"); continue; }
        *eq = '\0';
        float v = atof(eq+1);
        const mpe_param *p = mpe_config_find(argv[i]);
        if (!p) { printf ("export: unknown key '%s'\n", argv[i]); *eq='='; continue; }
        if (p->type == p_int) mpe_config_set_int(argv[i], (int)v);
        else if (p->type == p_bool) mpe_config_set_bool(argv[i], v != 0.0f);
        else mpe_config_set_float(argv[i], v);
        *eq = '=';
    }
}
static void cmd_config (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc < 2) { printf ("config: usage: config save|load|reset [path]\n"); return; }
    const char *path = (argc > 2) ? argv[2] : "status/engine.cfg";
    if (strcmp(argv[1],"save")==0) {
        if (!mpe_config_save_guarded(path)) printf ("config: save refused/failed (%s)\n", path);
        else printf ("config: saved to %s\n", path);
    } else if (strcmp(argv[1],"load")==0) {
        if (!mpe_config_load(path)) printf ("config: load failed (%s)\n", path);
        else printf ("config: loaded %s\n", path);
    } else if (strcmp(argv[1],"reset")==0) { mpe_config_reset_defaults(); printf ("config: reset to defaults\n"); }
    else { printf ("config: unknown subcommand '%s'\n", argv[1]); }
}

/* ------------------------------------------------------------------ */
/* Scene                                                              */
/* ------------------------------------------------------------------ */
static void cmd_mkfs (physics_world *w, int argc, char **argv) {
    (void)argc;(void)argv;
    physics_world_clear(w);
    tui_selected_object = -1;
    printf ("mkfs: world cleared\n");
}

/* ------------------------------------------------------------------ */
/* Module                                                             */
/* ------------------------------------------------------------------ */
static void cmd_mod_ls (physics_world *w) {
    (void)w;
    printf ("builtins: hash broadphase, seq-impulse solver\n");
    printf ("use: mod use-broadphase <name> | mod use-solver <name>\n");
}
static void cmd_mod (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc < 2) { cmd_mod_ls(w); return; }
    if (strcmp(argv[1],"ls")==0) { cmd_mod_ls(w); return; }
    if (strcmp(argv[1],"detach")==0 && argc >= 3) {
        /* DESPOT-2026-10-21: mirrors the attach slot — the MFS bundle
         * registers as ecosystem 'mfs-simulator', not a module, so detach
         * must fall through to mpe_ecosystem_detach() or the world keeps
         * a live hook and the loader refuses the unload with E_BUSY. */
        const mpe_module_desc_t *dm = mpe_find_module (argv [2]);
        if (dm) {
            int r = physics_world_detach_module (w, dm -> name);
            if (r < 0) { printf ("mod: detach failed '%s' (code %d)\n", argv [2], r); return; }
            printf ("mod: detached '%s'\n", argv [2]); return;
        }
        if (mpe_ecosystem_detach (w, argv [2]) != 0) { printf ("mod: detach — nothing named '%s' is attached\n", argv [2]); return; }
        printf ("mod: detached ecosystem '%s'\n", argv [2]);
        return;
    }
    if (strcmp(argv[1],"load")==0 && argc >= 3) {
        char err[256] = {0};
        if (mpe_loader_load(argv[2], err, sizeof(err)) != 0) { printf ("mod: load failed '%s': %s\n", argv[2], err); return; }
        printf ("mod: loaded '%s'\n", argv[2]); return;
    }
    if (strcmp(argv[1],"unload")==0 && argc >= 3) {
        if (mpe_loader_unload(argv[2]) != 0) { printf ("mod: unload failed '%s'\n", argv[2]); return; }
        printf ("mod: unloaded '%s'\n", argv[2]); return;
    }
    if (strcmp(argv[1],"attach")==0 && argc >= 3) {
        /* DESPOT-2026-10-21: mirrors ui_input/term_mod.c attach (the GTK
         * path) — the headless shell previously stubbed this, so any fleet
         * attached here never got its pre_step driven and teleop was dead
         * weight. Same contract: registry lookup, hook check, dedup,
         * attach. */
        const mpe_module_desc_t *d = mpe_find_module (argv [2]);
        if (!d) {
            /* DESPOT-2026-10-21: the MFS .so is a bundle — the loader
             * registers its ecosystem (mfs-simulator) instead of the inner
             * ftc-fleet fragment, so a name-based attach of "ftc-fleet"
             * falls through to the ecosystem attach path. */
            const mpe_ecosystem_desc_t *e = mpe_ecosystem_find (argv [2]);
            if (e && mpe_ecosystem_attach (w, argv [2]) == 0) {
                printf ("mod: attached ecosystem '%s'\n", argv [2]);
                return;
            }
            printf ("mod: attach — no module or ecosystem named '%s' (mod load first). registered:", argv [2]);
            for (int i = 0; i < mpe_module_count (); i++) {
                const mpe_module_desc_t *m = mpe_module_at (i);
                if (!m || !m -> name) { continue; }
                printf (" mod:%s%s", m -> name, i < mpe_module_count () - 1 ? "" : "");
            }
            for (int i = 0; i < mpe_ecosystem_count (); i++) {
                const mpe_ecosystem_desc_t *ec = mpe_ecosystem_at (i);
                if (ec && ec -> name) { printf (" eco:%s", ec -> name); }
            }
            printf ("\n");
            return;
        }
        if (!d -> pre_step && !d -> post_step && !d -> attach && !d -> detach) {
            printf ("mod: attach — '%s' has no tick hooks (nothing to attach)\n", argv [2]); return;
        }
        if ((w -> tick_module_count < 0) || (w -> tick_module_count > 16)) {
            printf ("mod: attach — tick_module_count corrupt (%d; re-init the world)\n", w -> tick_module_count); return;
        }
        for (int i = 0; i < w -> tick_module_count; i++) {
            if (w -> tick_modules [i] && strcmp (w -> tick_modules [i] -> name, argv [2]) == 0) {
                printf ("mod: attached '%s' already\n", argv [2]); return;
            }
        }
        int r = physics_world_attach_module (w, d);
        if (r < 0) { printf ("mod: attach failed '%s' (code %d)\n", argv [2], r); return; }
        printf ("mod: attached '%s' at slot %d\n", argv [2], r);
        return;
    }
    if (strcmp(argv[1],"use-broadphase")==0 && argc >= 3) {
        /* registry-first: try find pair handler as broadphase name proxy */
        printf ("mod: use-broadphase '%s' (registry-based, stubbed)\n", argv[2]); return;
    }
    if (strcmp(argv[1],"use-solver")==0 && argc >= 3) {
        printf ("mod: use-solver '%s' (registry-based, stubbed)\n", argv[2]); return;
    }
    printf ("mod: usage: mod ls|load <path>|unload <path>|attach|detach|use-broadphase|use-solver\n");
}

/* ------------------------------------------------------------------ */
/* Ecosystem                                                          */
/* ------------------------------------------------------------------ */
static void cmd_eco (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc < 2) { printf ("eco: usage: eco attach|detach|command <name> [args...]\n"); return; }
    if (strcmp(argv[1],"attach")==0 && argc >= 3) {
        if (mpe_ecosystem_attach(w, argv[2]) != 0) { printf ("eco: attach failed '%s'\n", argv[2]); return; }
        printf ("eco: attached '%s'\n", argv[2]); return;
    }
    if (strcmp(argv[1],"detach")==0 && argc >= 3) {
        mpe_ecosystem_detach(w, argv[2]); printf ("eco: detached '%s'\n", argv[2]); return;
    }
    if (strcmp(argv[1],"command")==0 && argc >= 4) {
        const mpe_ecosystem_desc_t *d = mpe_ecosystem_find(argv[2]);
        if (!d || !d->command) { printf ("eco: no command for '%s'\n", argv[2]); return; }
        /* build argv without the command name */
        char *av[32]; int ac = 0;
        for (int i=3;i<argc && ac<32;i++) av[ac++] = argv[i];
        int rc = d->command(mpe_ecosystem_state(w, argv[2]), ac, av);
        printf ("eco: command '%s' -> %d\n", argv[2], rc); return;
    }
    printf ("eco: usage: eco attach|detach|command <name> [args...]\n");
}

/* ------------------------------------------------------------------ */
/* FTC                                                                */
/* ------------------------------------------------------------------ */
typedef int (*ftc_fleet_selected_fn)(physics_world*);
typedef int (*ftc_fleet_spawn_fn)(physics_world*,float,float,float,motor_preset_id,ftc_drivetrain_type);
typedef int (*ftc_fleet_count_fn)(physics_world*);
typedef int (*ftc_fleet_select_fn)(physics_world*,int);
typedef ftc_robot *(*ftc_fleet_get_fn)(physics_world*,int);
typedef void (*ftc_robot_set_wheel_commands_fn)(ftc_robot*,const float*,int);
typedef void (*ftc_robot_get_position_fn)(physics_world*,ftc_robot*,float*,float*,float*);

static void *ftc_sym (const char *sym) {
    void *p = mpe_loader_symbol("mfs-simulator", sym);
    if (!p) p = mpe_loader_symbol("ecosystem/mfs/plugins/mpe_ftc.so", sym);
    return p;
}
static void cmd_ftc_spawn (physics_world *w, int argc, char **argv) {
    float x = (argc > 2) ? atof(argv[2]) : 0.0f;
    float y = (argc > 3) ? atof(argv[3]) : 0.5f;
    float z = (argc > 4) ? atof(argv[4]) : 0.0f;
    motor_preset_id preset = MOTOR_GB_5203_71_2;
    if (argc > 5) preset = (motor_preset_id)atoi(argv[5]);
    /* DESPOT-2026-10-21: bare spawn used to skip both spawn guarantees
     * (tile floor + 128 iterations) that every other spawn path applies —
     * the robot spawned on its wheels on empty space at 64 iterations and
     * wobbled its revolute axles loose. Same contract as eco/ftc paths.
     * If the bundle is not loaded there is nothing to dlsym and nothing
     * to spawn anyway. */
    void (*ensure_env) (physics_world *) = (void (*)(physics_world *)) mpe_loader_symbol("mfs-simulator", "mfs_ecosystem_ensure_env");
    if (!ensure_env) ensure_env = (void (*)(physics_world *)) mpe_loader_symbol("ecosystem/mfs/plugins/mpe_ftc.so", "mfs_ecosystem_ensure_env");
    if (ensure_env) { ensure_env (w); }
    else { printf ("ftc: (bundle not loaded — no floor/iteration guarantee; spawn will likely fail)\n"); }
    ftc_fleet_spawn_fn spawn = (ftc_fleet_spawn_fn)ftc_sym("ftc_fleet_spawn");
    if (!spawn) { printf ("ftc: fleet API unavailable (load mfs_ecosystem.so)\n"); return; }
    int si = spawn(w, x, y, z, preset, FTC_DRIVETRAIN_MECANUM);
    if (si < 0) {
        /* DESPOT-2026-10-21: bare spawn previously died here when only
         * `mod load` (no attach) had been done — the fleet has no state
         * in this world. Self-heal by attaching the bundle ecosystem,
         * then retry once. */
        const mpe_ecosystem_desc_t *e = mpe_ecosystem_find ("mfs-simulator");
        if (e && mpe_ecosystem_attach (w, "mfs-simulator") == 0) {
            si = spawn(w, x, y, z, preset, FTC_DRIVETRAIN_MECANUM);
        }
    }
    if (si < 0) { printf ("ftc: spawn failed\n"); return; }
    printf ("ftc: spawned mecanum robot %d at (%.2f,%.2f,%.2f) preset=%d\n", si,x,y,z,(int)preset);
}
static void cmd_ftc_telemetry (physics_world *w, int argc, char **argv) {
    int verbose = (argc >= 3 && argv [2] && argv [2][0] == 'v'); /* 'ftc telemetry v': per-wheel spin */
    ftc_fleet_count_fn count = (ftc_fleet_count_fn)ftc_sym("ftc_fleet_count");
    ftc_fleet_get_fn get = (ftc_fleet_get_fn)ftc_sym("ftc_fleet_get");
    ftc_robot_get_position_fn gpos = (ftc_robot_get_position_fn)ftc_sym("ftc_robot_get_position");
    if (!count || !get || !gpos) { printf ("ftc: fleet API unavailable\n"); return; }
    int n = count(w);
    printf ("ftc: %d robot(s)\n", n);
    for (int i=0;i<n;i++) {
        ftc_robot *r = get(w,i);
        if (!r) continue;
        float px=0,py=0,pz=0; gpos(w,r,&px,&py,&pz);
        float ws = 0.0f;
        for (int k=0;k<r->wheel_count && k<FTC_MAX_WHEELS;k++) {
            int wi = r->wheel_bodies[k];
            if (wi<0 || wi>=w->body_count) continue;
            rigidbody *wh = &w->bodies[wi];
            float wheel_speed = vector3_dot(wh->angular_velocity, wh->cached_axes[0]);
            ws += wheel_speed;
        }
        printf ("  robot %d: pos=(%.2f,%.2f,%.2f) wheel_ang_vel=%.2f rad/s (sum of 4)\n", i, px,py,pz, ws);
        if (verbose) {
            for (int k=0;k<r->wheel_count && k<FTC_MAX_WHEELS;k++) {
                int wi = r->wheel_bodies[k];
                if (wi<0 || wi>=w->body_count) continue;
                rigidbody *wh = &w->bodies[wi];
                float s = vector3_dot(wh->angular_velocity, wh->cached_axes[0]);
                printf ("    wheel %d: spin=%+.2f rad/s\n", k, s);
            }
        }
    }
}
static ftc_robot *ftc_selected_robot (physics_world *w, int *out_idx);
static void cmd_ftc_drive (physics_world *w, int argc, char **argv) {
    if (argc < 4) { printf ("ftc drive: usage: ftc drive <left> <right>\n"); return; }
    /* DESPOT-2026-10-21: args were read from argv[1..2] — argv[1] is the
     * verb "drive" here, so both tanks/both wheels silently went 0/atof-garbage.
     * Same off-by-one applied in cmd_ftc_mec below. */
    float l = atof(argv[2]), r = atof(argv[3]);
    float cmds[FTC_MAX_WHEELS] = {0};
    ftc_robot_set_wheel_commands_fn swc = (ftc_robot_set_wheel_commands_fn)ftc_sym("ftc_robot_set_wheel_commands");
    if (!swc) { printf ("ftc: fleet API unavailable\n"); return; }
    /* Terminal contract: drive verbs act on the SELECTED robot. */
    int idx = -1;
    ftc_robot *robot = ftc_selected_robot (w, &idx);
    if (!robot) { printf ("ftc: no robots (spawn first)\n"); return; }
    for (int k=0;k<robot->wheel_count && k<FTC_MAX_WHEELS;k++) {
        if (k % 2 == 0) cmds[k] = l; else cmds[k] = r;
    }
    swc(robot, cmds, robot->wheel_count);
    printf ("ftc: drive left=%.2f right=%.2f -> robot %d (tank)\n", l, r, idx);
}
/* DESPOT-2026-10-21: resolve the fleet's selection: hand back the
 * controlled robot pointer and its index (dlsym; NULL when the bundle is
 * not loaded or no fleet state exists). The terminal contract: every
 * drive verb acts on the selection, default robot 0. */
static ftc_robot *ftc_selected_robot (physics_world *w, int *out_idx);
static ftc_robot *ftc_selected_robot (physics_world *w, int *out_idx) {
    ftc_fleet_selected_fn sel = (ftc_fleet_selected_fn)ftc_sym("ftc_fleet_selected");
    ftc_fleet_get_fn get = (ftc_fleet_get_fn)ftc_sym("ftc_fleet_get");
    if (out_idx) { *out_idx = -1; }
    if (!get) { return NULL; }
    int idx = 0;
    if (sel) {
        idx = sel(w);
        if (idx < 0) { idx = 0; }
    }
    ftc_robot *r = get(w, idx);
    if (r && out_idx) { *out_idx = idx; }
    return r;
}
static void cmd_ftc_mec (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc < 5) { printf ("ftc mec: usage: ftc mec <fwd> <strafe> <rot>\n"); return; }
    float fwd = atof(argv[2]), strafe = atof(argv[3]), rot = atof(argv[4]);
    float targets[4] = { fwd - strafe - rot, fwd + strafe + rot, fwd + strafe - rot, fwd - strafe + rot };
    float maxm = 0.0f;
    for (int i=0;i<4;i++) { float m = fabsf(targets[i]); if (m > maxm) maxm = m; }
    if (maxm > 1.0f) for (int i=0;i<4;i++) targets[i] /= maxm;
    ftc_robot_set_wheel_commands_fn swc = (ftc_robot_set_wheel_commands_fn)ftc_sym("ftc_robot_set_wheel_commands");
    if (!swc) { printf ("ftc: fleet API unavailable\n"); return; }
    /* Terminal contract: drive verbs act on the SELECTED robot. */
    ftc_fleet_selected_fn sel = (ftc_fleet_selected_fn)ftc_sym("ftc_fleet_selected");
    int idx = sel ? sel(w) : 0;
    if (idx < 0) { idx = 0; }
    ftc_fleet_get_fn get = (ftc_fleet_get_fn)ftc_sym("ftc_fleet_get");
    if (!get) { printf ("ftc: fleet API unavailable\n"); return; }
    ftc_robot *r = get(w, idx);
    if (!r) { printf ("ftc: no robot %d (spawn first)\n", idx); return; }
    swc(r, targets, r->wheel_count > 4 ? 4 : r->wheel_count);
    printf ("ftc: mec fwd=%.2f strafe=%.2f rot=%.2f -> robot %d wheels %.2f %.2f %.2f %.2f\n",
            fwd, strafe, rot, idx, targets[0],targets[1],targets[2],targets[3]);
}
static void cmd_ftc (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc < 2) { printf ("ftc: usage: ftc spawn [x y z [preset]] | telemetry | drive <l> <r> | mec <f> <s> <r> | aim | select <i>\n"); return; }
    if (strcmp(argv[1],"spawn")==0) { cmd_ftc_spawn(w, argc, argv); return; }
    if (strcmp(argv[1],"telemetry")==0) { cmd_ftc_telemetry(w, argc, argv); return; }
    if (strcmp(argv[1],"drive")==0) { cmd_ftc_drive(w, argc, argv); return; }
    if (strcmp(argv[1],"mec")==0) { cmd_ftc_mec(w, argc, argv); return; }
    if (strcmp(argv[1],"biobuzz")==0) {
        if (argc >= 3 && strcmp(argv[2],"spawn")==0) {
            /* DESPOT-2026-10-23: regulation four-robot start (term_ftc's
             * `ftc biobuzz spawn` path, shell-flavoured): start-pose spawn
             * + yaw placement per alliance/slot + preload. Gives the aim
             * solver its regulation start pose to be tested against. */
            int (*p_spawn) (struct physics_world *, float, float, float, motor_preset_id, ftc_drivetrain_type) =
                (int (*) (struct physics_world *, float, float, float, motor_preset_id, ftc_drivetrain_type)) ftc_sym ("ftc_fleet_spawn");
            int (*p_place) (struct physics_world *, int, float) =
                (int (*) (struct physics_world *, int, float)) ftc_sym ("ftc_biobuzz_place");
            int (*p_pre) (struct physics_world *) = (int (*) (struct physics_world *)) ftc_sym ("ftc_biobuzz_preload");
            int (*p_count) (struct physics_world *) =
                (int (*) (struct physics_world *)) ftc_sym ("ftc_fleet_count");
            void (*p_env) (physics_world *) = (void (*)(physics_world *)) mpe_loader_symbol("mfs-simulator", "mfs_ecosystem_ensure_env");
            if (!p_env) p_env = (void (*)(physics_world *)) mpe_loader_symbol("ecosystem/mfs/plugins/mpe_ftc.so", "mfs_ecosystem_ensure_env");
            void (*p_pose) (int, int, float *, float *) = NULL;
            *(void **) (&p_pose) = ftc_sym ("biobuzz_start_pose");
            float (*p_rest) (void) = (float (*) (void)) ftc_sym ("ftc_robot_rest_height");
            if (!p_spawn || !p_place || !p_pre || !p_pose || !p_rest || !p_count || !p_env || !w) {
                printf ("ftc: bundle API incomplete (need biobuzz league)\n"); return;
            }
            if (p_count (w) > 0) { printf ("ftc: robots already exist (scene_clear to reset)\n"); return; }
            p_env (w);
            /* Self-heal like cmd_ftc_spawn: a fresh shell world has no fleet
             * state — attach the bundle ecosystem so spawn/place have a home. */
            const mpe_ecosystem_desc_t *e_bz = mpe_ecosystem_find ("mfs-simulator");
            if (e_bz) { mpe_ecosystem_attach (w, "mfs-simulator"); }
            /* regulation field first: build the hives + stage the match
             * (the fleet's bb.field_built gates both shooting and the aim
             * solve); attached worlds stage on demand, never implicitly.
             * biobuzz_stage_match requires field_built — build then stage. */
            int (*p_bfield) (struct physics_world *) =
                (int (*) (struct physics_world *)) ftc_sym ("ftc_biobuzz_field");
            int (*p_match) (struct physics_world *) =
                (int (*) (struct physics_world *)) ftc_sym ("ftc_biobuzz_match");
            if (p_bfield) { p_bfield (w); }
            if (p_match) { p_match (w); }
            int made = 0;
            for (int a = 0; a < 2 && made >= 0; a++) {
                for (int s = 0; s < 2 && made >= 0; s++) {
                    float xyz [3]; float yaw = 0.0f;
                    p_pose (a, s, xyz, &yaw);
                    xyz [1] = p_rest ();
                    int idx = p_spawn (w, xyz [0], xyz [1], xyz [2], MOTOR_GB_5203_26_9, FTC_DRIVETRAIN_MECANUM);
                    if (idx < 0 || p_place (w, idx, yaw) != 0) { made = -1; break; }
                    made++;
                }
            }
            if (made < 0) { printf ("ftc: biobuzz spawn failed\n"); return; }
            int pre = p_pre (w);
            printf ("ftc: %d robots at regulation starts (red -X, blue +X) + %d preloaded\n", made, pre);
            return;
        }
        printf ("ftc: usage: ftc biobuzz spawn — regulation starts + preload\n");
        return;
    }
    if (strcmp(argv[1],"aim")==0) {
        /* DESPOT-2026-10-22: print the live aim and the hive-entry solve so
         * headless tuning can see what the gamepad would publish. Solve
         * result only (never publishes: the shell is read-only for aim). */
        int (*a_get) (struct physics_world *, float *, float *) =
            (int (*) (struct physics_world *, float *, float *)) ftc_sym ("ftc_aim_params_get");
        int (*a_solve) (struct physics_world *, float *, float *) =
            (int (*) (struct physics_world *, float *, float *)) ftc_sym ("ftc_aim_params_solve");
        if (!a_get) { printf ("ftc: fleet API unavailable\n"); return; }
        float p = 0.0f, e = 0.0f;
        a_get (w, &p, &e);
        printf ("ftc: live aim power=%.2f elev=%.1f\n", p, e);
        if (a_solve) {
            float sp = 0.0f, se = 0.0f;
            if (a_solve (w, &sp, &se) == 0) { printf ("ftc: solve power=%.2f elev=%.1f\n", sp, se); }
            else { printf ("ftc: solve — no entering arc at this pose\n"); }
        } else { printf ("ftc: solve unavailable (bundle predates hive-aim)\n"); }
        return;
    }
    if (strcmp(argv[1],"select")==0 && argc >= 3) {
        ftc_fleet_select_fn sel = (ftc_fleet_select_fn)ftc_sym("ftc_fleet_select");
        if (!sel) { printf ("ftc: fleet API unavailable\n"); return; }
        int idx = atoi(argv[2]);
        if (sel(w, idx) < 0) { printf ("ftc: select failed %d\n", idx); return; }
        tui_selected_object = idx;
        printf ("ftc: selected robot %d\n", idx); return;
    }
    printf ("ftc: unknown subcommand '%s'\n", argv[1]);
}

/* ------------------------------------------------------------------ */
/* Diagnostics                                                        */
/* ------------------------------------------------------------------ */
static void cmd_diag (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc < 2) { printf ("diag: usage: diag tail N|sources|on|off|clear|status\n"); return; }
    if (strcmp(argv[1],"status")==0) {
        printf ("diag: total=%d ring=%d\n", mpe_diag_total(), mpe_diag_ring_count()); return;
    }
    if (strcmp(argv[1],"clear")==0) { mpe_diag_clear(); printf ("diag: cleared\n"); return; }
    if (strcmp(argv[1],"on")==0) { mpe_diag_set_live_mirror(true); printf ("diag: live mirror on\n"); return; }
    if (strcmp(argv[1],"off")==0) { mpe_diag_set_live_mirror(false); printf ("diag: live mirror off\n"); return; }
    if (strcmp(argv[1],"sources")==0) {
        int n = mpe_diag_source_total_count();
        for (int i=0;i<n;i++) {
            const mpe_diag_source_total *s = mpe_diag_source_total_at(i);
            printf ("  %-16s F=%u E=%u W=%u I=%u\n", s->source, s->fatal, s->error, s->warn, s->info);
        } return;
    }
    if (strcmp(argv[1],"tail")==0) {
        int n = (argc > 2) ? atoi(argv[2]) : 25;
        char buf[4096];
        mpe_diag_render(buf, sizeof(buf), n);
        printf ("%s", buf); return;
    }
    printf ("diag: unknown subcommand '%s'\n", argv[1]);
}

/* ------------------------------------------------------------------ */
/* Scene (GTK-free: scene_load/scene_saving)                          */
/* ------------------------------------------------------------------ */
static void cmd_fsck (physics_world *w, int argc, char **argv) {
    (void)argc;(void)argv;
    printf ("fsck: bodies=%d joints=%d (cap %d/%d)\n", w->body_count, w->spring_joint_count, mpe_max_bodies, mpe_max_joints);
}

/* ------------------------------------------------------------------ */
/* Camera / render / input state (headless)                           */
/* ------------------------------------------------------------------ */
static void cmd_w (physics_world *w, int argc, char **argv) {
    (void)w;(void)argc;(void)argv;
    vector3 fwd; tui_camera_forward(&fwd);
    printf ("camera: pos=(%.3f,%.3f,%.3f) yaw=%.2f pitch=%.2f speed=%.3f\n",
            tui_cam_pos.x,tui_cam_pos.y,tui_cam_pos.z,tui_cam_yaw,tui_cam_pitch,tui_camera_speed);
    printf ("  forward=(%.3f,%.3f,%.3f)\n", fwd.x,fwd.y,fwd.z);
}
static void cmd_ifconfig (physics_world *w, int argc, char **argv) {
    (void)w;(void)argc;(void)argv;
    printf ("input0:  flags=<RELEASED>\n");
    printf ("  selected=%d\n", tui_selected_object);
    printf ("objects=%d joints=%d\n", w->body_count, w->spring_joint_count);
}

/* ------------------------------------------------------------------ */
/* Module                                                             */
/* ------------------------------------------------------------------ */
static void cmd_lsmod (physics_world *w, int argc, char **argv) {
    (void)w;(void)argc;(void)argv;
    printf ("loaded modules:\n");
    printf ("  %-16s %-12s %s\n", "hash", "broadphase", "builtin");
    printf ("  %-16s %-12s %s\n", "seq-impulse", "solver", "builtin");
    int n = mpe_ecosystem_count();
    for (int i=0;i<n;i++) {
        const mpe_ecosystem_desc_t *d = mpe_ecosystem_at(i);
        printf ("  %-16s %-12s %s\n", d->name ? d->name : "?", "ecosystem", d->description ? d->description : "");
    }
}
static void cmd_modinfo (physics_world *w, int argc, char **argv) {
    (void)w;
    if (argc < 2) { printf ("modinfo: usage: modinfo <name>\n"); return; }
    const mpe_ecosystem_desc_t *d = mpe_ecosystem_find(argv[1]);
    if (!d) { printf ("modinfo: no such module '%s'\n", argv[1]); return; }
    printf ("name=%s version=%s author=%s\n", d->name ? d->name : "?", d->version ? d->version : "?", d->author ? d->author : "?");
    printf ("description=%s deterministic=%s\n", d->description ? d->description : "", d->deterministic ? "yes" : "no");
}
static void cmd_netstat (physics_world *w, int argc, char **argv) {
    (void)argc;(void)argv;
    int listed = 0;
    for (int i=0;i<mpe_max_joints;i++) {
        if (!w->spring_joints[i].is_active) continue;
        int ia = physics_world_index_by_id(w, w->spring_joints[i].object_id_a);
        int ib = physics_world_index_by_id(w, w->spring_joints[i].object_id_b);
        printf ("joint %-3d obj%-3d<->obj%-3d len=%.3f k=%.3f\n", i,
            ia, ib, w->spring_joints[i].equilibrium_length, w->spring_joints[i].spring_constant);
        listed++;
    }
    if (listed == 0) printf ("(no active joints)\n");
}

/* ------------------------------------------------------------------ */
/* Simulation                                                         */
/* ------------------------------------------------------------------ */
static void cmd_step (physics_world *w, int argc, char **argv) {
    int n = (argc > 1) ? atoi(argv[1]) : 1;
    if (n < 1) n = 1;
    for (int i=0;i<n;i++) { if (tui_paused) break; physics_world_step(w, tui_dt); tui_tick++; }
    tui_print_status(w);
}
static void cmd_run (physics_world *w, int argc, char **argv) {
    int n = (argc > 1) ? atoi(argv[1]) : 600;
    if (n < 1) n = 1;
    for (int i=0;i<n;i++) { if (tui_paused) break; physics_world_step(w, tui_dt); tui_tick++;
        if ((i+1) % 60 == 0 || (i+1) == n) { printf ("tick=%lu ", tui_tick); tui_print_status(w); } }
}
static void cmd_tick (physics_world *w, int argc, char **argv) { if (argc >= 2) { /* DESPOT-2026-10-21: 'tick N' now steps N times (the count was silently discarded — one step per call made every scripted drive be a 2-tick burst). */ int n = atoi (argv [1]); if (n > 0) { for (int i = 0; i < n && !tui_paused; i++) { physics_world_step (w, tui_dt); tui_tick++; } tui_print_status (w); return; } } if (!tui_paused) { physics_world_step(w, tui_dt); tui_tick++; } tui_print_status(w); }
static void cmd_halt (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; tui_paused = 1; printf ("sim: paused\n"); }
static void cmd_resume (physics_world *w, int argc, char **argv) { (void)w;(void)argc;(void)argv; tui_paused = 0; printf ("sim: resumed\n"); }
static void cmd_status (physics_world *w, int argc, char **argv) { (void)argc;(void)argv; tui_print_status(w); }

/* ------------------------------------------------------------------ */
/* Command table                                                      */
/* ------------------------------------------------------------------ */
typedef void (*shell_cmd_fn)(physics_world*,int,char**);
typedef struct { const char *name; const char *usage; const char *help; shell_cmd_fn fn; } shell_cmd;
static shell_cmd g_cmds[] = {
    {"help","help [cmd]","list commands / help for one",cmd_help},
    {"exit","exit","quit the shell",cmd_exit},
    {"quit","quit","quit the shell",cmd_quit},
    {"clear","clear","clear the screen",cmd_clear},
    {"echo","echo [args...]","print arguments",cmd_echo},
    {"date","date","print date/time",cmd_date},
    {"whoami","whoami","print user",cmd_whoami},
    {"hostname","hostname","print host name",cmd_hostname},
    {"id","id","print identity",cmd_id},
    {"pwd","pwd","print working directory",cmd_pwd},
    {"ls","ls [path]","list /obj /joint /world /camera /spawner",cmd_ls},
    {"ll","ll [path]","long listing",cmd_ll},
    {"cat","cat <path>","inspect object/world/camera/spawner",cmd_cat},
    {"touch","touch <sphere|cube|cyl> [x y z]","spawn an object",cmd_touch},
    {"ps","ps","object table",cmd_ps},
    {"stat","stat <obj>","detailed object info",cmd_stat},
    {"w","w","camera state",cmd_w},
    {"ifconfig","ifconfig","camera/render/input state",cmd_ifconfig},
    {"rm","rm <obj>","remove object (not supported; use mkfs)",cmd_rm},
    {"mv","mv <obj> [/pos/x/y/z|/vel/x/y/z]","move/apply impulse to object",cmd_mv},
    {"ln","ln <a> <b> [rest]","create spring joint",cmd_ln},
    {"cp","cp <obj>","duplicate object",cmd_cp},
    {"chown","chown <mass> <obj...>","set object mass",cmd_chown},
    {"chmod","chmod static|dynamic <obj...>","set static state",cmd_chmod},
    {"kill","kill <obj...>","wake object(s)",cmd_kill},
    {"env","env","print all config parameters",cmd_env},
    {"export","export KEY=value","set a config parameter",cmd_export},
    {"config","config save|load|reset [path]","manage config file",cmd_config},
    {"mkfs","mkfs","clear the world",cmd_mkfs},
    {"fsck","fsck","scene/physics integrity",cmd_fsck},
    {"mod","mod ls|load|unload|attach|detach|use-*","module commands",cmd_mod},
    {"lsmod","lsmod","list loaded modules",cmd_lsmod},
    {"modinfo","modinfo <name>","module details",cmd_modinfo},
    {"netstat","netstat","joint network topology",cmd_netstat},
    {"eco","eco attach|detach|command","ecosystem bundle commands",cmd_eco},
    {"ftc","ftc spawn|telemetry|drive|mec|aim|select","FTC robotics",cmd_ftc},
    {"step","step [N]","advance N ticks (default 1)",cmd_step},
    {"run","run [N]","advance N ticks (default 600)",cmd_run},
    {"tick","tick","advance one tick",cmd_tick},
    {"halt","halt","pause the simulation",cmd_halt},
    {"resume","resume","resume the simulation",cmd_resume},
    {"status","status","world status",cmd_status},
    {"diag","diag tail N|sources|on|off|clear|status","engine self-report",cmd_diag},
};
static size_t g_cmd_count = sizeof(g_cmds)/sizeof(g_cmds[0]);

static shell_cmd *tui_find_command (const char *name) {
    for (size_t i=0;i<g_cmd_count;i++) if (strcmp(g_cmds[i].name, name) == 0) return &g_cmds[i];
    return NULL;
}

static void tui_split (char *line, char **av, int *ac, int max) {
    *ac = 0;
    char *p = line;
    while (*p) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '\n' || *p == '\r') break;
        if (*ac >= max) break;
        av[(*ac)++] = p;
        while (*p && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') p++;
        if (*p) { *p = '\0'; p++; }
    }
}

static int tui_execute_command (physics_world *w, char *line) {
    char *av[64]; int ac = 0;
    tui_split(line, av, &ac, 64);
    if (ac == 0) return 0;
    shell_cmd *c = tui_find_command(av[0]);
    if (!c) { printf ("mpe: %s: command not found (try 'help')\n", av[0]); return 1; }
    c->fn(w, ac, av);
    return 0;
}

static void tui_print_prompt (void) {
    printf ("mpe:%s> ", tui_cam_pos.x >= 0 ? "/" : "/");
    fflush(stdout);
}

int tui_shell_run (int argc, char **argv) {
    int ticks = 0;
    int want_snapshot = 0, want_stream = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i],"--shell")==0) { /* handled by caller */ continue; }
        else if (strcmp(argv[i],"--dt")==0 && i+1<argc) { tui_dt = (float)atof(argv[++i]); continue; }
        else if (strcmp(argv[i],"--snapshot")==0) { want_snapshot = 1; if (i+1<argc && argv[i+1][0]!='-') ticks = atoi(argv[++i]); continue; }
        else if (strcmp(argv[i],"--stream")==0) { want_stream = 1; if (i+1<argc && argv[i+1][0]!='-') ticks = atoi(argv[++i]); continue; }
        else { printf ("mpe-tui: unknown option '%s'\n", argv[i]); return 2; }
    }
    mpe_config_init();
    physics_world world;
    physics_world_init (&world);
    mpe_config_t sc = g_cfg;
    physics_world_set_config(&world, &sc);
    mpe_register_builtins();
    mpe_register_ecosystems();
    printf ("MPE headless shell (engine without a display). Type 'help'.\n");
    if (want_snapshot) {
        for (int i = 0; i < ticks; i++) physics_world_step(&world, tui_dt);
        tui_print_status(&world);
        physics_world_cleanup(&world);
        return 0;
    }
    if (want_stream) {
        printf ("### MPE-TUI snapshot tick=0 time=0.0000 dt=%.5f bodies=%d result=PASS\n", tui_dt, world.body_count);
        for (int t = 1; t <= ticks; t++) {
            physics_world_step(&world, tui_dt);
            if (t % 60 == 0 || t == ticks) { printf ("\n### MPE-TUI snapshot tick=%d time=%.4f dt=%.5f bodies=%d result=PASS\n", t, t*(double)tui_dt, tui_dt, world.body_count); }
        }
        physics_world_cleanup(&world);
        return 0;
    }
    /* Interactive shell. */
    char line[512];
    while (!tui_quit_requested) {
        tui_print_prompt();
        if (!fgets(line, sizeof(line), stdin)) break;
        /* strip trailing newline */
        size_t len = strlen(line);
        while (len > 0 && (line[len-1]=='\n'||line[len-1]=='\r')) line[--len]='\0';
        if (len == 0) { /* empty line: single tick */ if (!tui_paused) { physics_world_step(&world, tui_dt); tui_tick++; } tui_print_status(&world); continue; }
        tui_execute_command(&world, line);
    }
    physics_world_cleanup(&world);
    return 0;
}
