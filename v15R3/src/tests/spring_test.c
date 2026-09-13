/* Spring truth: undamped mass-spring period must match 2*pi*sqrt(m/k), and
 * total mechanical energy must not drift (symplectic bounded oscillation). */
#ifdef MPE_SPRING_TEST
#include <stdio.h>
#include <math.h>
#include "../ui_input/camera.h"
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "physics/spring_joint.h"
#include "config/mpe_config.h"
#include "config/mpe_constants.h"

/* Stubs for legacy-TU symbols in spring_joint.o (world path never calls). */
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

int main(void) {
    mpe_config_init();
    g_cfg.world.drag = 1.0f;
    g_cfg.world.angular_damping_scale = 1.0f;
    g_cfg.world.gravity = 0.0f; /* space lab: pure 1-D oscillation */
    physics_world world;
    physics_world_init(&world);
    constraint_pool_init(&world);
    joint_init_pool(&world);

    /* Static anchor + unit mass, k=20, c=0, L0=2, amplitude 0.5 along x.
     * T = 2*pi*sqrt(1/20) = 1.40496 s. High in the air: no contacts. */
    const float k = 20.0f;
    int anchor = physics_world_add_cube(&world, (vector3){0.0f, 50.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 0.0f);
    int mass = physics_world_add_sphere(&world, 0.2f, 1.0f, (vector3){2.5f, 50.0f, 0.0f});
    uint32_t ida = world.bodies[anchor].object_id;
    uint32_t idm = world.bodies[mass].object_id;
    if (add_joint_by_ids(&world, ida, idm, 2.0f, k, 0.0f) < 0) {
        printf("[FAIL] joint creation\n");
        physics_world_cleanup(&world);
        return 1;
    }

    const float dt = 1.0f / 60.0f;
    float prev_x = world.bodies[mass].position.x - 2.0f; /* extension */
    int crossings = 0;
    int first_cross = -1, last_cross = -1;
    float E0 = 0.5f * k * 0.25f;
    float Emax_dev = 0.0f;
    for (int t = 0; t < 600; t++) {
        physics_world_step(&world, dt);
        rigidbody *mb = &world.bodies[mass];
        if (!isfinite(mb->position.x)) {
            printf("[FAIL] NaN\n");
            physics_world_cleanup(&world);
            return 1;
        }
        float x = mb->position.x - 2.0f;
        if ((prev_x <= 0.0f && x > 0.0f) || (prev_x >= 0.0f && x < 0.0f)) {
            crossings++;
            if (first_cross < 0) {
                first_cross = t;
            }
            last_cross = t;
        }
        prev_x = x;
        float E = 0.5f * k * x * x + 0.5f * 1.0f * vector3_length_squared(mb->velocity);
        float dev = fabsf(E - E0) / E0;
        if (dev > Emax_dev) {
            Emax_dev = dev;
        }
    }
    int fail = 0;
    /* Half-periods between first and last crossing. */
    float measured_T = 0.0f;
    if (crossings >= 4) {
        measured_T = 2.0f * (float)(last_cross - first_cross) * dt / (float)(crossings - 1);
    }
    float analytic_T = 2.0f * 3.14159265f * sqrtf(1.0f / k);
    printf("[info] period: measured=%.4f analytic=%.4f crossings=%d\n", measured_T, analytic_T, crossings);
    if (fabsf(measured_T - analytic_T) / analytic_T > 0.05f) {
        printf("[FAIL] spring period off\n");
        fail = 1;
    } else {
        printf("[PASS] spring period matches 2*pi*sqrt(m/k)\n");
    }
    printf("[info] max energy deviation: %.3f\n", Emax_dev);
    if (Emax_dev > 0.10f) {
        printf("[FAIL] spring energy drifts\n");
        fail = 1;
    } else {
        printf("[PASS] spring energy bounded (symplectic)\n");
    }
    physics_world_cleanup(&world);
    return fail;
}
#endif /* MPE_SPRING_TEST */
