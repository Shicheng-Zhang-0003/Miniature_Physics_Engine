/* EXAMPLE RUN ONLY */
//DO NOT EXECUTE, NO LINKING LIBRARY HEADER FILES


#include "physics.h"
#include <stdio.h>
#include <time.h>
#define NUM_PARTICLES 2
#define DT 0.01f
#define WIDTH 800.0f
#define HEIGHT 600.0f
Particle particles [NUM_PARTICLES];
int main () {
    //Initialise Particles and locale settings
    particles [0].position = (Vector2) {100, 100};
    particles [0].velocity = (Vector2) {50, 0};
    particles [0].mass = 1.0f; particles [0].radius = 10.0f;
    for (int frame = 0; frame < 100; frame++) {  //100 Frame Run Time for this simulation
        for (int i = 0; i < NUM_PARTICLES; i++) {
            app_gravity_base (&particles [i], 98.1f);
            app_friction_force_kinetic (&particles [i], 0.1f);
            // Input Other forces and update system
            ParticleUpdate (&particles [i], DT, WIDTH, HEIGHT);
        }
        // Collision Logic
        for (int i = 0; i < NUM_PARTICLES; i++) {
            for (int j = i + 1; j < NUM_PARTICLES; j++) {resolve_particle_collision (&particles [i], &particles [j]);}
        }
        // Debuging Output
        printf("Frame %d: Pos (%.1f, %.1f)\n", frame, particles [0].position.x, particles [0].position.y);
    } return 0;
}
