/* MPE_FTC_074: Drivetrain systems */
#ifndef drivetrain_h
#define drivetrain_h
#include "robot.h"
#include "../core/physics_world.h"

/* Tank drive: independent left/right power */
void drivetrain_tank(ftc_robot *robot, float left_power, float right_power);

/* Mecanum drive: forward/strafe/rotate (stub for Phase 3) */
void drivetrain_mecanum(ftc_robot *robot, float forward, float strafe, float rotate);

/* One drivetrain update tick: sets motor commands, then updates motors. */
void drivetrain_update(physics_world *world, ftc_robot *robot, float dt);

#endif /* drivetrain_h */
