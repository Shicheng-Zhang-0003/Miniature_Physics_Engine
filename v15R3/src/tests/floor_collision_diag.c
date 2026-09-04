
#ifdef MPE_FLOOR_DIAG
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "config/mpe_config.h"

static const float DT = 1.0f / 60.0f;

int main(void) {
    mpe_config_init();
    printf("\n=== FLOOR COLLISION DIAGNOSTIC ===\n");

    physics_world world;
    physics_world_init(&world);

    /* Static floor, top surface at y=0 */
    int floor_idx = physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f},
        (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    float r = 0.05f;
    int cyl_idx = physics_world_add_cylinder(&world, r, 0.02f, 0.5f,
                                             (vector3){0.0f, 1.0f, 0.0f});

    printf("floor_idx=%d cyl_idx=%d\n", floor_idx, cyl_idx);
    printf("cylinder initial: pos=(%.4f,%.4f,%.4f) vel=(%.4f,%.4f,%.4f)\n",
           world.bodies[cyl_idx].position.x, world.bodies[cyl_idx].position.y,
           world.bodies[cyl_idx].position.z,
           world.bodies[cyl_idx].velocity.x, world.bodies[cyl_idx].velocity.y,
           world.bodies[cyl_idx].velocity.z);
    printf("cylinder: radius=%.4f half_length=%.4f mass=%.4f\n",
           world.bodies[cyl_idx].radius, world.bodies[cyl_idx].cylinder_half_length,
           world.bodies[cyl_idx].mass);
    printf("floor: pos=(%.4f,%.4f,%.4f) half_ext=(%.4f,%.4f,%.4f) static=%d\n",
           world.bodies[floor_idx].position.x, world.bodies[floor_idx].position.y,
           world.bodies[floor_idx].position.z,
           world.bodies[floor_idx].half_extensions.x,
           world.bodies[floor_idx].half_extensions.y,
           world.bodies[floor_idx].half_extensions.z,
           world.bodies[floor_idx].static_state);
    printf("floor top y=%.4f (pos.y + half_ext.y)\n",
           world.bodies[floor_idx].position.y + world.bodies[floor_idx].half_extensions.y);
    printf("\n");

    for (int i = 0; i < 60; i++) { /* MFS_139_EXTEND: run longer to reach floor */
        physics_world_step(&world, DT);
        printf("step=%2d y=%.6f vy=%.6f\n",
               i + 1, world.bodies[cyl_idx].position.y,
               world.bodies[cyl_idx].velocity.y);
    }

    printf("\n=== DIAG COMPLETE ===\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif
