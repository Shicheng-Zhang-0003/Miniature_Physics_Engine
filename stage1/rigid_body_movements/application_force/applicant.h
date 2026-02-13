#include <stdio.h>
//Apply forces to body in question (Total application, all sides and motion at once, no rotational disparity)
void apply_forces (rigidbody *b, vector3 force) {
    b->force_accumilation = vector3_addition (b->force_accumilation, force);
} //Apply force at a specific point on a object (torque generating)
void apply_force_point_only (rigidbody *b, vector3 force, vector3 point) {
    apply_forces (b, force);
    //Torque = r * F, where r = linear position vector, F = force with vectoring
    vector3 r = vector3_subtraction (point, b->position);
    b->torque_accumilation = vector3_add (b->torque_accumilation, vector3_crossover (r, force)); //3D compilation of torqued motion
}
