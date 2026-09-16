/* MPE_FTC_062 */
#ifndef revolute_joint_h
#define revolute_joint_h
#include "constraint.h"
/* Iterative positional/axis solve (call once per tick). */
void revolute_solve(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
/* Positional axis drift correction (call once per tick, AFTER solver loop). */
void revolute_correct_axis_drift(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
/* Motor: adds drive torque to the torque accumulator (call once per tick). */
void revolute_apply_motor(revolute_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
/* Fixed weld: lock relative position and all relative rotation. */
void fixed_solve(fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
/* Fixed angular positional correction (call once per tick, AFTER loop).
 * TRUTH: velocity-only angular lock lets welds flex rotationally over
 * seconds; this Baumgarte corrects accumulated orientation error. */
void fixed_correct_angular_drift(fixed_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
/* Distance: ROD (equality: pushes and pulls, free rotation). A rope
 * (inequality, pulls only) needs early-out when dist<rest; tracked. */
void distance_solve(distance_params *p, rigidbody *body_a, rigidbody *body_b, float dt);
#endif
