#include <stdio.h>
float get_system_energy (rigidbody *bodies_count, int counter) {
    float total_energy = 0;
    for (int step = 0; step < counter; step++) {
        //Linear Ek = 0.5mv ^ 2
        float velocity_squared = vector3_dot (bodies_count [step].velocity, bodies_count [step].velocity);
        total_energy += 0.5f * bodies_count [steps].mass * velocity_squared;
        //Rotational Ek = 0.5mIw, I to inertia, w to rotational/angular velocity
        vector3 angular_momemtum = math3_mul_vector (bodies_count [step].inertia_tensor, bodies_count [step].angular_velocity);
        total_energy += 0.5f * vector3_dot (bodies_count [step].angular_velocity, angular_momentum);
        //Epg: mgh
        total_energy += bodies_count [step].mass * 9.81f * bodies_count [step].position.y;
    } return total_energy;
}
