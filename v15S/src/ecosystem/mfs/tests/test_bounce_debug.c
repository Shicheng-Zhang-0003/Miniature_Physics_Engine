#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "core/rigidbody.h"
#include "config/mpe_config.h"
#define DT (1.0f / 60.0f)
int main () {
    mpe_config_init ();
    g_cfg.timestep.solver_iterations = 128;
    g_cfg.sleep.enable = 0;
    physics_world w;
    physics_world_init (&w);
    int f = physics_world_add_cube (&w, (vector3) {0.0f, -0.5f, 0.0f}, (vector3) {10.0f, 0.5f, 10.0f}, 0.0f);
    w.bodies [f].friction_static = 1.0f;
    w.bodies [f].friction_kinetic = 0.8f;
    w.bodies [f].restitution = 0.6f;
    int s = physics_world_add_sphere (&w, 0.5f, 1.0f, (vector3) {0, 5.0f, 0});
    if (s < 0) {
        printf ("Failed to create sphere\n");
        return 1;
    }
    w.bodies [s].restitution = 0.6f;
    w.bodies [s].velocity = (vector3) {0, 0, 0};
    w.bodies [s].restitution = 0.6f;
    printf ("body_count=%d, sphere_idx=%d\n", w.body_count, s);
    rigidbody *sph = &w.bodies [s];
    printf ("Sphere: mass=%.2f, pos=(%.2f,%.2f,%.2f), inv_mass=%.2f, sleeping=%d\n", sph -> mass, sph -> position.x,
            sph -> position.y, sph -> position.z, sph -> inverse_mass, sph -> is_sleeping);
    const float dt = 1.0f / 60.0f;
    for (int k = 0; k < 300; k++) {
        physics_world_step (&w, DT);
        rigidbody *sph = &w.bodies [s];
        if (k % 50 == 0) {
            printf ("Tick %d: pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)\n", k, sph -> position.x, sph -> position.y,
                    sph -> position.z, sph -> velocity.x, sph -> velocity.y, sph -> velocity.z);
        }
    }
    rigidbody *sph2 = &w.bodies [s];
    float max_y = sph2 -> position.y;
    printf ("Final pos=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f,%.3f)\n", w.bodies [s].position.x, w.bodies [s].position.y,
            w.bodies [s].position.z, w.bodies [s].velocity.x, w.bodies [s].velocity.y, w.bodies [s].velocity.z);
    physics_world_cleanup (&w);
    return 0;
}
