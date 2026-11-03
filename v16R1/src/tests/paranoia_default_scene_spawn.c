/* PARANOIA TEST: default scene spawn clearance.
 *
 * Live report: "when I try driving the robot the robot AND the robot view
 * twitches and shivers and refuses to cooperate with me". The GUI default
 * sandbox used to place its immovable filler sphere (radius 2 m) at
 * (0, 2, 0). That sphere touches the floor at the origin and therefore
 * encloses the FTC robot spawned by `ftc spawn` at rest height: the solver
 * squeezes the chassis between the sphere and the floor, the chassis rolls
 * and jitters, and every follow view inherits the shake.
 *
 * This gate exercises the REAL `scene_init_default()` path (not the shared
 * placement constants), spawns a real FTC robot at the origin, settles it,
 * and drives forward. It fails on the old arrangement in three independent
 * ways: the filler surface intrudes into the spawn envelope, the settled
 * chassis sits far below rest height, and the driving roll/pitch/height
 * spread is the squeeze transient rather than a drive.
 *
 * Gate name: default_scene_spawn_clearance
 */
#ifdef mpe_paranoia_default_scene_spawn
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "config/mpe_config.h"
#include "physics/constraint.h"
#include "physics/spring_joint.h"
#include "modules/ftc/submodules/robot.h"
#include "modules/ftc/submodules/drivetrain.h"
#include "scene/scene_init.h"
#include "ui_input/input_state.h"

/* UI stubs for scene_init.c (same contract as paranoia_scene_persistence.c).
 * Joint-pool helpers come from the linked physics/spring_joint.c. */
int selected_object = -1;
uint32_t selected_object_id = 0;
input_status main_inputs = {0};
void clear_selection (void) {}

static int failures = 0;
static void gate (int condition, const char *what) {
    if (!condition) {
        printf ("[FAIL] default_scene_spawn_clearance: %s\n", what);
        failures++;
    }
}

int main (void) {
    mpe_config_init ();
    physics_world *world = physics_world_get_primary ();
    physics_world_init (world);
    constraint_pool_init (world);
    scene_init_default ();

    gate (world -> static_plane_enabled, "default scene lost its friction floor");

    /* 1. The actual sandbox filler must stay clear of the spawn point. A
     * robot start cube is 0.4572 m per side; 0.5 m of surface clearance is
     * deliberately conservative and still catches the old sphere (which
     * reached the origin exactly: clearance 0). */
    int filler = -1;
    for (int i = 0; i < world -> body_count; i++) {
        rigidbody *b = &world -> bodies [i];
        if (b -> type == object_sphere && b -> mass == 0.0f && b -> radius >= 1.0f) {
            if (filler >= 0) {
                gate (0, "default scene has more than one large static filler sphere");
            }
            filler = i;
        }
    }
    gate (filler >= 0, "default scene has no large static filler sphere");
    if (filler >= 0) {
        rigidbody *b = &world -> bodies [filler];
        float centre_distance = sqrtf (b -> position.x * b -> position.x + b -> position.y * b -> position.y +
                                       b -> position.z * b -> position.z);
        float clearance = centre_distance - b -> radius;
        printf ("[INFO] default_scene_spawn_clearance filler=(%.3f,%.3f,%.3f) r=%.3f clearance=%.3f\n",
                b -> position.x, b -> position.y, b -> position.z, b -> radius, clearance);
        gate (clearance >= 0.5f, "filler sphere intrudes into the robot spawn envelope");
    }

    /* 2. Spawn the same robot `ftc spawn` creates, at the same origin. */
    ftc_robot robot;
    if (ftc_robot_create (world, &robot, 0.0f, ftc_robot_rest_height (), 0.0f, MOTOR_GB_5203_26_9) != 0) {
        printf ("[FAIL] default_scene_spawn_clearance: ftc_robot_create failed\n");
        physics_world_cleanup (world);
        return 1;
    }
    if ((robot.chassis_body < 0) || (robot.chassis_body >= world -> body_count)) {
        printf ("[FAIL] default_scene_spawn_clearance: chassis body index invalid\n");
        physics_world_cleanup (world);
        return 1;
    }
    rigidbody *chassis = &world -> bodies [robot.chassis_body];
    const float dt = 1.0f / 60.0f;
    const float rest_height = ftc_robot_rest_height ();

    for (int t = 0; t < 120; t++) {
        drivetrain_update (world, &robot, dt);
        physics_world_step (world, dt);
    }
    float settled_y = chassis -> position.y;
    printf ("[INFO] default_scene_spawn_clearance settled chassis y=%.5f rest=%.5f\n", settled_y, rest_height);
    gate (fabsf (settled_y - rest_height) < 0.02f, "chassis settled below rest height (squeezed by scene body)");

    /* 3. Drive forward and measure the chassis, exactly like the live view
     * follows it. With clear spawn space the chassis tracks the floor at
     * rest height with negligible roll/pitch; the squeeze produced a
     * 0.46 rad roll excursion and a 0.16 m vertical range. */
    float min_roll = 1e9f, max_roll = -1e9f;
    float min_pitch = 1e9f, max_pitch = -1e9f;
    float min_y = 1e9f, max_y = -1e9f;
    float max_step = 0.0f;
    float prev_x = chassis -> position.x;
    float prev_z = chassis -> position.z;
    int nonfinite = 0;
    for (int t = 0; t < 600; t++) {
        drivetrain_mecanum (&robot, 0.5f, 0.0f, 0.0f);
        drivetrain_update (world, &robot, dt);
        physics_world_step (world, dt);
        if (!isfinite (chassis -> position.x) || !isfinite (chassis -> position.y) ||
            !isfinite (chassis -> position.z) || !isfinite (chassis -> orientation.w)) {
            nonfinite++;
            continue;
        }
        vector3 forward = chassis -> cached_axes [2];
        vector3 left = chassis -> cached_axes [0];
        float roll = atan2f (left.y, left.x);
        float pitch = asinf (fmaxf (-1.0f, fminf (1.0f, -forward.y)));
        if (roll < min_roll) { min_roll = roll; }
        if (roll > max_roll) { max_roll = roll; }
        if (pitch < min_pitch) { min_pitch = pitch; }
        if (pitch > max_pitch) { max_pitch = pitch; }
        if (chassis -> position.y < min_y) { min_y = chassis -> position.y; }
        if (chassis -> position.y > max_y) { max_y = chassis -> position.y; }
        float dx = chassis -> position.x - prev_x;
        float dz = chassis -> position.z - prev_z;
        prev_x = chassis -> position.x;
        prev_z = chassis -> position.z;
        float step = sqrtf (dx * dx + dz * dz);
        if (step > max_step) { max_step = step; }
    }
    float travel = sqrtf ((chassis -> position.x) * (chassis -> position.x) +
                          (chassis -> position.z) * (chassis -> position.z));
    printf ("[INFO] default_scene_spawn_clearance drive travel=%.4f roll=[%.4f,%.4f] pitch=[%.4f,%.4f] "
            "y=[%.5f,%.5f] max_step=%.4f nonfinite=%d\n",
            travel, min_roll, max_roll, min_pitch, max_pitch, min_y, max_y, max_step, nonfinite);
    gate (nonfinite == 0, "chassis state went non-finite while driving");
    gate (travel > 1.0f, "robot refused to drive away from the spawn (travel too small)");
    gate ((max_roll - min_roll) < 0.05f, "chassis roll shook while driving (spawn squeeze)");
    gate ((max_pitch - min_pitch) < 0.05f, "chassis pitch shook while driving (spawn squeeze)");
    gate ((min_y > (rest_height - 0.02f)) && (max_y < (rest_height + 0.03f)),
          "chassis height bounced outside the resting band while driving");
    gate (max_step < 0.03f, "chassis position jittered per tick while driving");

    physics_world_cleanup (world);
    if (failures == 0) {
        printf ("[PASS] default_scene_spawn_clearance: filler clear, chassis settles at rest, drive is stable\n");
    }
    return failures ? 1 : 0;
}
#endif /* mpe_paranoia_default_scene_spawn */
