/* MPE_FTC_062 */
#ifndef revolute_joint_h
#define revolute_joint_h
#include "constraint.h"
/* Iterative positional/axis solve (call once per tick). */
void revolute_solve(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
/* Motor: adds drive torque to the torque accumulator (call once per tick). */
void revolute_apply_motor(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
#endif
