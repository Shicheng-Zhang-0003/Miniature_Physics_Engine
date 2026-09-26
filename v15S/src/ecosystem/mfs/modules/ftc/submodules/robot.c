/* MPE_FTC_073: FTC robot object implementation */
/* MPE_FTC_094_CLEANUP: wheel_traction removed — real cylinder friction */
#include "robot.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"
#include <math.h>
#include <string.h>

/* Robot dimensions (metres, approximate FTC 18" x 18" chassis) */
#define CHASSIS_HALF_X 0.225f
#define CHASSIS_HALF_Y 0.075f
#define CHASSIS_HALF_Z 0.225f
#define CHASSIS_MASS 8.0f /* ~18 lb robot */
#define WHEEL_RADIUS 0.05f /* 100mm wheels */
#define WHEEL_MASS 0.2f
#define WHEEL_HALF_WIDTH 0.02f /* 40mm wide wheels */
#define WHEEL_OFFSET_X 0.24f /* slightly outside chassis */
#define WHEEL_OFFSET_Z 0.20f
#define WHEEL_Y_OFFSET (-CHASSIS_HALF_Y - WHEEL_RADIUS - 0.005f) /* MFS_PORT_V15S: 5mm ground clearance. The old +0.01f tucked wheel tops 10mm INSIDE the chassis box, so the contact solver fought the joint-held pose every tick (sinking, chatter, pitch-unload). Joints hold anchors, not volumes — interpenetrating rest poses are solver poison. */
#define WHEEL_PRELOAD 0.002f /* MFS_PORT_V15S: joint anchors sit 2mm BELOW exact touch so P2P preloads wheels into persistent floor contact. Inside slop (10mm): no positional fight, but the manifold never grazes out to a hover-skid (which starves odometry). Real suspensions run droop/preload the same way. */

/* ---------------------------------------------------------------------
 * MECANUM ROLLER GEOMETRY (real dimensions, not a force model)
 *
 * A mecanum wheel is NOT a cylinder. It is a hub carrying a ring of small
 * rollers, each on a free bearing, whose axes lie in the wheel's plane at
 * 45 deg to the forward direction. That geometry - not any special force
 * law - is what makes the chassis omnidirectional: each roller can only
 * push along its own roll direction, and the four wheels' differing 45 deg
 * handedness makes the sideways components add instead of cancel.
 *
 * So the rollers are modelled as what they physically are: real rigid
 * bodies on real free revolute bearings, with real contacts against the
 * floor. The lateral thrust is then an emergent consequence of rigid-body
 * dynamics plus Coulomb friction, bounded by the friction cone for free.
 * Nothing projects, scales or invents a lateral force.
 *
 * Dimensions are those of a 100 mm mecanum wheel:
 *   outer radius  R = 0.050 m  (envelope: R_r + r)
 *   roller radius r = 0.006 m  (12 mm rollers)
 *   roller pitch radius R_r = R - r = 0.044 m
 *   hub plate radius 0.030 m  (inside the roller ring, so the hub never
 *                              touches the floor - which is physically
 *                              correct: on a real wheel the rollers stand
 *                              proud of the plate)
 * The hub keeps a 0.030 m collision radius rather than the 0.050 m outer
 * envelope precisely so it cannot reach the ground; obstacle contact is
 * then within ~1 mm of the real rim, which is the honest approximation.
 * --------------------------------------------------------------------- */
#define MECANUM_ROLLER_RADIUS 0.006f
#define MECANUM_ROLLER_PITCH_RADIUS (WHEEL_RADIUS - MECANUM_ROLLER_RADIUS)
#define MECANUM_ROLLER_HALF_LEN 0.008f   /* 16 mm long rollers */
#ifndef MECANUM_ROLLERS_PER_ROW
#define MECANUM_ROLLERS_PER_ROW 8        /* around the rim */
#endif
#ifndef MECANUM_ROLLER_ROWS
/* ONE row, not two. Measured on the corrected geometry: 8x1 gives strafe
 * 0.224 m and forward 3.05 m, while 8x2 gives 0.021 m and 1.55 m. Doubling the
 * body count makes the revolute stack worse conditioned, so the rollers stop
 * developing their rail force and the drive loses traction as well - worse on
 * every axis, at twice the cost. A single row already resolves the continuum
 * the wheel actually needs, and the deficit is a conditioning limit, not a
 * geometry one. */
#define MECANUM_ROLLER_ROWS 1
#endif
#define MECANUM_ROLLER_ROW_X 0.010f      /* +-10 mm from wheel centre plane */
#define MECANUM_HUB_PLATE_RADIUS 0.030f /* recessed plate: 8 mm clear of rollers */

/* DESPOT-DECISION: the aggregate RAIL contact is PARKED; real rollers are
 * active again (see below for why the rail plates stalled).
 * Model history, measured in this engine:
 * - Real rollers + FROZEN friction frame: strafe ~0.0005 m (no rail force).
 * - Wheel-level rail ellipse in chassis frame: strafe ~0.005 m, forward
 *   peel-out (plate-aggregate mus cannot carry drive: single-roller 0.10
 *   across caps forward at 0.14*N while stall delivers 74 N-equiv).
 * - Real rollers were condemned for "conditioning" (0.002-0.014 m) while
 *   TWO independent bugs strangled them: the revolute P2P effective-mass
 *   sign flip (indefinite K at roller lever arms) and the governor
 *   breakaway starvation (0.385 N.m cap vs 0.97 N.m cone). Both are fixed
 *   now, so the verdict is re-tested rather than trusted.
 * Physics note: a free roller ROLLS across its axle (static grip holds,
 * no slip) and only slides along it — directionality comes from the free
 * BEARING (a kinematic constraint), not from friction coefficients. So
 * rollers are ISOTROPIC rubber (0.9/0.7); anisotropic Coulomb caps would
 * model sliding anisotropy the rollers never exhibit (they roll instead).
 * (MECANUM_USE_RAIL_CONTACT is deliberately UNDEFINED: the aggregate
 * rail model is mathematically insufficient; the real-roller model
 * with free-bearing constraints is the active path.) */
#undef MECANUM_USE_RAIL_CONTACT

/* DESPOT-FIX: lock the storage bound to the builder. If someone raises
 * ROWS/ROW again, this fails at compile time instead of silently overflowing
 * roller_bodies/roller_joints at runtime. */
_Static_assert(MFS_ROLLERS_PER_WHEEL >= MECANUM_ROLLERS_PER_ROW * MECANUM_ROLLER_ROWS,
               "MFS_ROLLERS_PER_WHEEL too small for roller builder");

/* Mecanum contact-patch friction anisotropy.
 *
 * A free roller is a RAIL, not an ellipse. Its bearing is free, so it rolls
 * without resistance in the direction perpendicular to its own axle: the
 * contact point can move that way and no friction is needed, so the
 * perpendicular coefficient is ZERO. Along its axle the roller cannot roll,
 * so it must slide, and that sliding is resisted by ordinary kinetic
 * friction.
 *
 * That is the whole mecanum mechanism, and it is a statement about the
 * DIRECTION of the contact force, not just its size: the contact can only
 * push along the roller axle, limited by mu_slide * N. Four wheels with
 * mirrored axles therefore produce four 45-degree force diagonals that
 * combine into forward, lateral and yaw all at once.
 *
 * Setting the perpendicular coefficient high instead (the intuitive "grip the
 * rolling direction" reading) does NOT work, and it is worth recording why:
 * Coulomb friction always opposes the contact's slip velocity, so raising
 * grip on a diagonal only shrinks the disc along that diagonal and makes the
 * wheel slip MORE, it never rotates the force onto the diagonal. Measured:
 * forward and yaw came out healthy but strafe was 0.002 m, i.e. nothing.
 * The redirection has to come from a real constraint, and the roller's free
 * bearing is that constraint.
 *
 * mu_slide is a real material number: a small steel roller skidding on its own
 * axle across FTC tile. It is deliberately not 0 (then the wheel would have no
 * lateral authority and could not be steered) and not the isotropic 0.9 (then
 * the rollers would grip sideways and it would be an ordinary wheel). */
#ifndef MECANUM_MU_ROLLER_RAIL
#define MECANUM_MU_ROLLER_RAIL 0.9f
#endif
#ifndef MECANUM_MU_ROLLER_ACROSS
/* Small but NOT zero. A real roller is a deforming steel cylinder, not an
 * ideal rail: it has edge rounding, contact-patch shear and slip, so it does
 * transmit a little force across its own axle. Exactly zero is also the
 * knife-edge of the cone, and it measured chaotically - the same build gave
 * +68.7 deg and then -97.3 deg on the isolated wheel. */
#define MECANUM_MU_ROLLER_ACROSS 0.10f
#endif
/* DESPOT-FIX (effective plate aggregate): MECANUM_MU_ROLLER_ACROSS above is
 * the SINGLE-roller across-grip (used by the parked real-roller builder).
 * The active RAIL path puts the ellipse on the hub PLATE, whose contact
 * patch spans many rollers plus rubber deformation: its effective
 * across-grip is rubber-lateral (~0.45), NOT the single-roller 0.10.
 * Using 0.10 at plate level caps forward grip at 0.14*N (ellipse minor
 * projected 45 deg) while stall delivers 74 N-equiv per wheel: permanent
 * burnout (traction cut at 11 N can never re-grip inside a 3 N cone;
 * measured odometry drift 99%). With 0.45 the forward/lateral semi-axes
 * are ~0.57*N — above the cut level so traction regulates instead of
 * sliding forever — while the rail (0.9) still dominates directionally,
 * which is what makes strafe possible at all (symmetric lock without it).
 * Two macros because they are two different physical quantities; the
 * single-roller value is untouched for the parked builder. */
#ifndef MECANUM_MU_PLATE_ACROSS
#define MECANUM_MU_PLATE_ACROSS 0.45f
#endif
#ifndef MECANUM_RAIL_SIGN
#define MECANUM_RAIL_SIGN 1
#endif
/* Vertical component of the roller axis (0 = purely horizontal). Exposed as
 * an override so the roller orientation can be swept empirically rather than
 * asserted. */
/* FIX-AUDIT-DESPOT: the duplicate copy of this block that lived below
 * MECANUM_ROLLER_ANGLE is removed; this is the single definition. */
#ifndef MFS_ROLLER_AXIS_Y
#define MFS_ROLLER_AXIS_Y 0.0f
#endif

#define MECANUM_ROLLER_ANGLE 0.7853981634f /* 45 deg, the defining dimension */

/* Roller mass is DERIVED from its own geometry and a stated material, not
 * picked as a round number. A 4 g "roller" was an untraceable magic value.
 * Steel (7850 kg/m^3) is the standard material for real mecanum rollers:
 *   V = pi * r^2 * L = pi * 0.006^2 * 0.016 = 1.8096e-6 m^3
 *   m = 7850 * 1.8096e-6 = 0.01421 kg  (14.2 g)
 * This also matters numerically: a 4 g roller against an 8 kg chassis is a
 * 2000:1 mass ratio, which the iterative solver cannot converge; 14 g is both
 * more accurate and better conditioned. */
#define MECANUM_ROLLER_DENSITY 7850.0f            /* steel, kg/m^3 */
#define MECANUM_ROLLER_LENGTH  (2.0f * MECANUM_ROLLER_HALF_LEN)
#define MECANUM_ROLLER_VOLUME  (M_PI * MECANUM_ROLLER_RADIUS * MECANUM_ROLLER_RADIUS * MECANUM_ROLLER_LENGTH)
#define MECANUM_ROLLER_MASS    (MECANUM_ROLLER_DENSITY * MECANUM_ROLLER_VOLUME)

/* Orient a body so its local +X (the cylinder axis) points along `axis`.
 * Built from the minimal rotation taking +X to the target, so a cylinder is
 * created with its axis already correct instead of being rotated afterwards.
 * quaternion layout here is (w, x, y, z) and identity is w=1 (see math3d.h). */
static vector4 rb_orient_from_axis(vector3 axis) {
    const vector3 from = {1.0f, 0.0f, 0.0f};
    float len = vector3_length(axis);
    if (!(len > 1.0e-6f)) {
        return vector4_identity();
    }
    vector3 a = vector3_scaling(axis, 1.0f / len);
    float d = vector3_dot(from, a);
    if (d > 0.999999f) {
        return vector4_identity();
    }
    if (d < -0.999999f) {
        /* Antiparallel: 180 deg about any axis perpendicular to +X. */
        return vector4_from_axis_with_angle((vector3){0.0f, 1.0f, 0.0f}, 3.14159265f);
    }
    /* Shortest-arc quaternion for rotating +X onto a: (cos(t/2), n*sin(t/2))
     * with n = (+X x a) and cos(t) = d gives the half-angle form
     * (1+d, c) which is already unit up to a scale of sqrt(2(1+d)). */
    vector3 c = vector3_cross(from, a);
    return vector4_normalisation((vector4){1.0f + d, c.x, c.y, c.z});
}

/* MPE_FTC_095: chassis-centre height where the wheels just touch floor y=0 */
float ftc_robot_rest_height(void) {
    return WHEEL_RADIUS - WHEEL_Y_OFFSET;
}

int ftc_world_setup_field(physics_world *world, float mus, float muk) {
    /* DESPOT-FIX: was `return 1` on bad args — the only `1`-on-error in the
     * tree (everyone else uses 0 ok / -1 fail, and callers test `!= 0`).
     * 1 still failed the check, but mixed conventions hide failures from
     * `rc < 0` callers. Now -1 like the rest. */
    if (!world) {return -1;}
    if (mus < 0.0f || muk < 0.0f) {return -1;}
    if (!isfinite(mus) || !isfinite(muk)) {return -1;}
    mpe_config_t *cfg = mpe_world_cfg_mut(world);
    if (cfg) {
        cfg->world.floor_friction_s = mus;
        cfg->world.floor_friction_k = muk;
    }
    world->static_plane_enabled = true;
    world->static_plane_body.friction_static = mus;
    world->static_plane_body.friction_kinetic = muk;
    return 0;
}

int ftc_robot_create_with_drive(physics_world *world, ftc_robot *robot, float x, float y, float z,
                                motor_preset_id preset, ftc_drivetrain_type drivetrain_type) {
/* MFS_161_NULL_FIX: null-check FIRST, before any dereference */
/* DESPOT-FIX: was `return 1` on all fail paths (only 1-on-error in tree).
 * Callers test `!= 0` so behaviour is unchanged, but -1 matches every
 * other MFS fail return and is caught by `rc < 0` checks too. */
if ((!world) || (!robot)) {
return -1;
}
    memset(robot, 0, sizeof(ftc_robot));
/* memset zeroes odom_x/z/theta and wheel_radians — no separate init needed */
    /* Traction scales start open (1.0); memset leaves 0.0 = fully cut. */
    for (int i = 0; i < FTC_MAX_WHEELS; i++) {
        robot->wheel_traction_scale[i] = 1.0f;
    }
    robot->motor_preset = preset;
    robot->drivetrain_type = drivetrain_type;
    robot->axle_axis_x = 1.0f; /* axles point along X (left-right) */
    robot->axle_axis_y = 0.0f;
    robot->axle_axis_z = 0.0f;
    battery_init(&robot->battery);

    /* FLEET PARTIAL-SPAWN UNWIND: bodies form an append-only pool with no
     * removal API, and ftc_robot_create_with_drive can fail part way through
     * (e.g. the joint table is full at wheel 3). Previously every one of
     * those `return 1` paths abandoned the chassis, every wheel already
     * created and every joint already added, all of them still live in the
     * world and still colliding: ftc_fleet_spawn then returned -1 without
     * the robot ever entering the fleet, so a transient failure (one full
     * joint table) permanently polluted the world with an invisible,
     * un-derivable robot. Record the pool watermark and unwind to it.
     * This is sound because the only body-adding calls below are appends
     * and nothing between them can add a body on our behalf. */
    const int body_watermark = world->body_count;
    /* FIX-AUDIT-DESPOT (CRITICAL unwind bug): joints_made used to interleave
     * wheel+roller joint counts but the fail path removed
     * wheel_joints[0..joints_made) — wrong slots (roller joints never
     * removed, uninitialised wheel slots treated as joints). Every
     * successfully created joint index is recorded here in creation order;
     * the fail path removes exactly these. DESPOT-FIX: 80 covers the true
     * maximum 8 wheels + 8*8=64 roller joints = 72 with margin (a 4-wheel
     * mecanum build makes 4+32=36); creation bails out rather than
     * overflow. The old 64 assumed 16 rollers/wheel and overflowed an
     * 8-wheel build. */
    int created_joints[80];
    int ncreated = 0;
    /* FIX-AUDIT-DESPOT: memset(robot,0) above leaves joint slots at 0, which
     * the old `>= 0` fail-path check mistook for live joint 0. Poison all
     * joint slots to -1 before creation so "not created" is unambiguous. */
    memset(robot->wheel_joints, 0xFF, sizeof(robot->wheel_joints));
    memset(robot->roller_joints, 0xFF, sizeof(robot->roller_joints));

    /* Chassis: a box at the given position */
    robot->chassis_body = physics_world_add_cube(
        world, (vector3){x, y, z}, (vector3){CHASSIS_HALF_X, CHASSIS_HALF_Y, CHASSIS_HALF_Z}, CHASSIS_MASS);
    if (robot->chassis_body < 0) {
        goto fail;
    }

    uint32_t chassis_id = world->bodies[robot->chassis_body].object_id;

    /* 4 wheels at corners */
    float wheel_positions[4][3] = {
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z}, /* front-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z - WHEEL_OFFSET_Z}, /* front-right */
        {x - WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z}, /* back-left */
        {x + WHEEL_OFFSET_X, y + WHEEL_Y_OFFSET, z + WHEEL_OFFSET_Z}, /* back-right */
    };
    robot->wheel_count = 4;

    for (int i = 0; i < robot->wheel_count; i++) {
        /* The HUB is the rigid plate the rollers bolt to. Its collision
         * radius is the plate radius, NOT the 0.050 m outer envelope: on a
         * real mecanum wheel the rollers stand proud of the plate, so the
         * plate never reaches the ground. Giving the hub the envelope radius
         * (as this used to) made the plate itself the contact body, which is
         * exactly the "wheel is a cylinder" simplification that has no
         * lateral force at all. Tank wheels keep the full radius - there the
         * tyre IS the contact body. */
        const bool wheel_is_mecanum_w =
            (drivetrain_type == FTC_DRIVETRAIN_MECANUM);
        /* Both drivetrains contact the ground at the full running radius:
         * for tank that is the tyre, for mecanum the outer surface of the
         * rollers (pitch radius + roller radius = WHEEL_RADIUS numerically, but the
         * sum records the provenance: change the roller and the radius
         * follows); tank contacts on the tyre, i.e. WHEEL_RADIUS.
         * DESPOT-FIX: under MECANUM_USE_RAIL_CONTACT no roller bodies are
         * built, so the hub plate itself must reach the floor: plate radius
         * is WHEEL_RADIUS there (a 44 mm plate would hover 6 mm and the
         * robot would hang off its joints). Rail model → full-radius plate;
         * real-roller model → recessed plate inside the roller ring. */
#if defined(MECANUM_USE_RAIL_CONTACT)
        const float hub_radius = WHEEL_RADIUS;
#else
        /* DESPOT-FIX: plate recessed INSIDE the roller ring (30 mm). The
         * pitch-radius plate (44 mm) sat exactly on the roller-center
         * circle, half-burying every roller so it ground the plate with
         * 0.9 rubber friction on top of its pin joint (measured: chassis
         * jacked +14 mm, 0.42 rad yaw in straight drive, judder). With a
         * 30 mm plate the clearance is 44-30-6 = 8 mm: the only hub-ground
         * path is through rollers on their bearings, as the geometry
         * comment above always specified. */
        const float hub_radius = wheel_is_mecanum_w ? MECANUM_HUB_PLATE_RADIUS
                                                    : WHEEL_RADIUS;
#endif
        /* Running-surface radius, DERIVED per drivetrain (FIX-AUDIT-DESPOT):
         * mecanum contacts the ground on the roller envelope
         * (pitch radius + roller radius == WHEEL_RADIUS numerically, but the
         * sum records the provenance: change the roller and the radius
         * follows); tank contacts on the tyre, i.e. WHEEL_RADIUS. */
        robot->wheel_effective_radius[i] =
            wheel_is_mecanum_w ? (MECANUM_ROLLER_PITCH_RADIUS + MECANUM_ROLLER_RADIUS)
                               : WHEEL_RADIUS;
        robot->wheel_bodies[i] =
            physics_world_add_cylinder(world, hub_radius, WHEEL_HALF_WIDTH, WHEEL_MASS,
                                     (vector3){wheel_positions[i][0], wheel_positions[i][1], wheel_positions[i][2]});
        if (robot->wheel_bodies[i] < 0) {
            goto fail;
        }
        /* Grippy rubber on tile (was engine defaults ~0.3/0.2: glassy).
         * Contact mu = min(wheel, floor); the drivetrain budgets against
         * these same wheel materials (see drivetrain_update). */
        world->bodies[robot->wheel_bodies[i]].friction_static = 0.9f;
        world->bodies[robot->wheel_bodies[i]].friction_kinetic = 0.7f;
        world->bodies[robot->wheel_bodies[i]].restitution = 0.0f;

        uint32_t wheel_id = world->bodies[robot->wheel_bodies[i]].object_id;

        /* Revolute joint: chassis (body_a) to wheel (body_b), axle along X.
         * Anchor sits WHEEL_PRELOAD below exact touch (see above). */
        vector3 anchor_on_chassis = {wheel_positions[i][0] - x, WHEEL_Y_OFFSET - WHEEL_PRELOAD,
                                     wheel_positions[i][2] - z};
        vector3 anchor_on_wheel = {0.0f, 0.0f, 0.0f}; /* wheel centre */
        vector3 axle_axis = {robot->axle_axis_x, robot->axle_axis_y, robot->axle_axis_z};

        robot->wheel_joints[i] =
            constraint_add_revolute(world, chassis_id, wheel_id, anchor_on_chassis, anchor_on_wheel,
                                    axle_axis);
        if (robot->wheel_joints[i] < 0) {
            goto fail;
        }
        if (ncreated < (int)(sizeof(created_joints) / sizeof(created_joints[0]))) {
            created_joints[ncreated++] = robot->wheel_joints[i];
        } else {
            goto fail; /* joint untrackable: unwind rather than leak */
        }

        /* MFS_PORT_V15S: roller geometry is robot-local state now (the
         * parked rigidbody is_mecanum/roller_angle_rad fields are gone
         * from the core). Standard layout: FL +45°, FR -45°, BL -45°,
         * BR +45°. Tank robots get 0/false (plain cylinders). */
        float roller_angle = 0.0f;
        bool is_mecanum = false;
        if (robot->drivetrain_type == FTC_DRIVETRAIN_MECANUM) {
            is_mecanum = true;
            /* M10 GENERALIZATION: was an if/else-if chain hard-coded to
             * i == 0..3, so any 5th+ wheel silently got 0 rad (a plain
             * cylinder inside a mecanum chassis). The X pattern is
             * "mirrored across left/right AND front/back":
             *   side = +1 (left wheels 0,2), -1 (right wheels 1,3)
             *   front_back = +1 (front 0,1), -1 (back 2,3)
             * which reproduces the original +45, -45, -45, +45 for i=0..3
             * and gives every additional wheel a defined, matching roller angle. */
            float side = (i % 2 == 0) ? 1.0f : -1.0f;
            float front_back = (i < 2) ? 1.0f : -1.0f;
            roller_angle = 0.785398f * side * front_back;
        }
        robot->wheel_roller_angle[i] = roller_angle;
        robot->wheel_is_mecanum[i] = is_mecanum;

        /* Set up motor for this wheel */
        motor_preset_apply(&robot->wheel_motors[i], preset);

        /* --- Mecanum contact: wheel-level anisotropic Coulomb friction ---
         * The 64-real-roller model (16 free-bearing roller bodies per wheel)
         * was built first and measured. It does not work in this engine, and
         * the reason is structural rather than geometric: a driven roller
         * must satisfy a no-slip constraint that couples ground contact ->
         * roller spin -> revolute bearing -> hub -> revolute bearing ->
         * chassis. That is a 5-link constraint chain per roller, 64 of them,
         * resolved by a Gauss-Seidel solver in 128 iterations. The lateral
         * component simply never converges: an axis sweep (roller axis from
         * horizontal through vertical, 6 positions) gave strafe of
         * 0.002-0.014 m against a >0.3 m requirement, while forward stayed
         * healthy at 0.6-0.7 m. Geometry is not the variable; solver
         * conditioning is.
         *
         * So the rollers are modelled by their AGGREGATE effect on the
         * contact patch, which is what a continuum wheel model does and what
         * the engine's anisotropic Coulomb cone already exists to express.
         *
         * Why this is physics and not another injected force:
         *   - It is a CONTACT constraint on the real wheel body against the
         *     real floor, carrying the real normal force. It is not a
         *     force added to the chassis, so it cannot push the robot
         *     sideways without the wheels being in contact.
         *   - It is clamped to a Coulomb ELLIPSE whose semi-axes are real
         *     friction coefficients. The old hack applied
         *     sin(45)*sum(torque)/radius straight to the chassis, which had
         *     no normal force and no cone, and is why strafe ran 4.3x over
         *     the ceiling. Here the lateral force is bounded by
         *     mu_lateral * N by construction.
         *   - mu_along/mu_across are the roller's real friction anisotropy:
         *     a free roller cannot slide freely, but it also cannot transmit
         *     force along its own axle without slipping, which is precisely
         *     what limits the wheel's lateral grip. The ratio is the roller
         *     geometry's doing, not a tuned gain.
         *
         * The wheel therefore contacts the ground at the roller's OUTER
         * surface (pitch radius + roller radius = WHEEL_RADIUS), which is
         * the same running radius a tyre would have, so the hub reaching the
         * floor is the honest envelope rather than a shortcut. */
        robot->roller_count[i] = 0;
#if defined(MECANUM_USE_RAIL_CONTACT)
        if (is_mecanum) {
            /* Wheel-level RAIL contact: the free rollers are collapsed into the
             * aggregate they physically are - a contact patch that transmits
             * force ALONG each roller's axle and nothing across it. The
             * engine's anisotropic Coulomb cone expresses exactly that: an
             * ellipse whose across-axis semi-axis is 0 is a line. */
            const float sgn0 = (roller_angle >= 0.0f) ? 1.0f : -1.0f;
            const float sgn = sgn0 * (float)MECANUM_RAIL_SIGN;
            /* DESPOT-FIX: the old code used a hardcoded sgn for X only,
             * leaving Z always positive. The rail axis IS the roller axle
             * direction: X = sin(θ), Z = cos(θ). Angle is measured from +Z. */
            const vector3 rail_axis = {
                sinf(roller_angle), 0.0f, cosf(roller_angle)
            };
            /* The rail belongs to the WHEEL MOUNT, not to the spinning hub, so
             * the axis is given in the chassis frame. A hub-local axis would
             * sweep around with the wheel and leave the contact with no fixed
             * direction at all. */
            rigidbody_set_friction_anisotropic_in_frame(
                &world->bodies[robot->wheel_bodies[i]], chassis_id,
                rail_axis, MECANUM_MU_ROLLER_RAIL, MECANUM_MU_PLATE_ACROSS);
        }
#else
        if (is_mecanum) {
            /* Real free rollers on real free bearings.
             *
             * Each roller is a genuine body, a genuine cylinder, joined to the
             * hub by a genuine revolute constraint whose axis IS the roller's
             * axle. Nothing about the lateral force is written down anywhere:
             * the roller spins because the hub drags it, the contact friction
             * acting across the roller axle is what the roller cannot
             * transmit, and the redirection falls out of the two constraints.
             *
             * Geometry: the axle is along X, so the rim lies in the Y-Z plane
             * and roller k sits at rim angle phi. The roller's own axle is
             * HORIZONTAL (in the ground plane) at 45 deg to the wheel's
             * rolling direction - which is the defining dimension of a
             * mecanum wheel and is also what makes the roller perpendicular
             * to the radial direction at the contact, so it is geometrically
             * free to spin there. */
            const float sgn = (roller_angle >= 0.0f) ? 1.0f : -1.0f;
/* DESPOT-FIX: the old code used a hardcoded sgn for X only,
             * leaving Z always positive. The roller axis must rotate with
             * the actual roller_angle in the XZ plane. Angle is measured
             * from +Z (forward): X = sin(θ), Z = cos(θ). */
            const vector3 roller_axis = {
                sinf(roller_angle), 0.0f, cosf(roller_angle)
            };
            const int n_rollers = MECANUM_ROLLERS_PER_ROW * MECANUM_ROLLER_ROWS;
            int roller_count = 0;
            const vector3 wc = {wheel_positions[i][0], wheel_positions[i][1], wheel_positions[i][2]};
            const uint32_t wheel_id_w = world->bodies[robot->wheel_bodies[i]].object_id;

            if (n_rollers > MFS_ROLLERS_PER_WHEEL) {
                goto fail;
            }
            for (int k = 0; k < n_rollers; k++) {
                const int row = k / MECANUM_ROLLERS_PER_ROW;
                const int rim = k % MECANUM_ROLLERS_PER_ROW;
                /* Stagger the two rows along the axle, as real wheels are. */
                const float row_x = (MECANUM_ROLLER_ROWS == 1)
                                         ? 0.0f
                                         : ((row & 1) ? MECANUM_ROLLER_ROW_X : -MECANUM_ROLLER_ROW_X);
                /* DESPOT-FIX: phi = 180° (π rad) puts roller 0 at the BOTTOM (0,-R,0).
                 * phi = 0 is top (+Y), π/2 is front (+Z), π is bottom (-Y),
                 * -π/2 is back (-Z). The old code used -90° which placed
                 * rollers at the BACK, not the bottom — they never touched
                 * the ground. */
                const float phi = (float)(3.14159265358979f +
                                          2.0 * 3.14159265358979 * (double)rim /
                                          (double)MECANUM_ROLLERS_PER_ROW);
                const vector3 offset = {row_x,
                                        MECANUM_ROLLER_PITCH_RADIUS * cosf(phi),
                                        MECANUM_ROLLER_PITCH_RADIUS * sinf(phi)};
                const vector3 rp = vector3_addition(wc, offset);
                const int rb = physics_world_add_cylinder(
                    world, MECANUM_ROLLER_RADIUS, MECANUM_ROLLER_HALF_LEN,
                    MECANUM_ROLLER_MASS, rp);
                if (rb < 0) {
                    goto fail;
                }
                /* Same rubber-on-tile material as the hub, so the contact mu
                 * the engine reduces over is the one the drivetrain budgets. */
                world->bodies[rb].friction_static = 0.9f;
                world->bodies[rb].friction_kinetic = 0.7f;
                world->bodies[rb].restitution = 0.0f;
                /* The roller is what decouples hub spin from contact-force
                 * direction, so the anisotropy belongs on the ROLLER, not on
                 * the hub. A contact modelled on the hub cannot do this job:
                 * a rail force there also torques the hub about its own axle
                 * and stalls it (measured: hub_w = 0 with mu_across = 0), so
                 * the wheel either spins or steers, never both. On the roller,
                 * a rail along its own axle lets it spin freely about that axle
                 * - the force has no moment about the roller's own spin axis -
                 * while the contact still transmits only along that axle. That
                 * is the definition of a free roller, and it is what makes the
                 * four mirrored rails combine into omnidirectional motion.
                 *
                 * The roller's local +X IS its axle (set by rb_orient_from_axis
                 * below), and spinning about X leaves that axis invariant, so
                 * the self frame is stable here - no reference body needed. */
                /* DESPOT-FIX: rollers stay ISOTROPIC (0.9/0.7 set above).
                 * The roller-level anisotropic cap (0.9 along / 0.10 across
                 * its axle) limited across-force to 0.10*N, but a rolling
                 * roller transmits across-force through STATIC grip (it
                 * rolls, not slides) — the 0.10 cap strangled the very
                 * rolling accommodation the free bearing exists to provide.
                 * Directionality lives in the bearing constraint, not in
                 * friction coefficients. Call kept (compiled out) so the
                 * experiment can be re-run by flipping the 0 below. */
#if 0 /* roller anisotropy: parked, see above */
                rigidbody_set_friction_anisotropic(&world->bodies[rb], (vector3){1.0f, 0.0f, 0.0f},
                                                   MECANUM_MU_ROLLER_RAIL, MECANUM_MU_ROLLER_ACROSS);
#endif
                /* physics_world_add_cylinder always builds the axis along
                 * local +X, so the roller's own axis has to be turned to the
                 * rail direction or its contact patch is a disc, not a
                 * cylinder lying across the rail. */
                world->bodies[rb].orientation = rb_orient_from_axis(roller_axis);
                robot->roller_bodies[i][k] = rb;

                const uint32_t roller_id = world->bodies[rb].object_id;
                const int rj = constraint_add_revolute(world, wheel_id_w, roller_id,
                                                       offset, (vector3){0.0f, 0.0f, 0.0f},
                                                       roller_axis);
                if (rj < 0) {
                    goto fail;
                }
                robot->roller_joints[i][k] = rj;
                if (ncreated < (int)(sizeof(created_joints) / sizeof(created_joints[0]))) {
                    created_joints[ncreated++] = rj;
                } else {
                    goto fail; /* joint untrackable: unwind rather than leak */
                }
                roller_count++;
            }
            robot->roller_count[i] = roller_count;
        }
#endif
    }

    return 0;

fail:
    /* Release exactly the joints this call created (creation order), then
     * rewind the append-only body pool. constraint_remove deactivates by
     * index and is order-independent here; the pool rewind needs the
     * revision bump so any id->index cache built from the abandoned bodies
     * is discarded.
     * FIX-AUDIT-DESPOT: was `wheel_joints[0..joints_made)` where joints_made
     * interleaved wheel+roller counts — roller joints leaked and stale
     * wheel slots (0 from memset) removed joint 0. */
    for (int i = 0; i < ncreated; i++) {
        constraint_remove(world, created_joints[i]);
    }
    for (int i = 0; i < FTC_MAX_WHEELS; i++) {
        robot->wheel_joints[i] = -1;
    }
    memset(robot->roller_joints, 0xFF, sizeof(robot->roller_joints));
    if (world->body_count > body_watermark) {
        world->body_count = body_watermark;
        physics_world_bump_revision(world);
    }
    robot->chassis_body = -1;
    robot->wheel_count = 0;
    return -1;
}


int ftc_robot_create(physics_world *world, ftc_robot *robot, float x, float y, float z, motor_preset_id preset) {
    return ftc_robot_create_with_drive(world, robot, x, y, z, preset, FTC_DRIVETRAIN_MECANUM);
}

void ftc_robot_update(physics_world *world, ftc_robot *robot, float dt) {
    if ((!world) || (!robot) || (dt <= 0.0f)) {
        return;
    }

    /* A commanded robot is awake by definition. Wake the chassis when any
     * wheel is commanded: motor vibration and driver intent keep a real
     * robot active, and the velocity integrator drains forces for sleeping
     * bodies, so a sleeping chassis would swallow traction forever after
     * an idle settle. Wheels are woken below per-wheel. */
    int any_command = 0;
    for (int i = 0; i < robot->wheel_count; i++) {
        if (fabsf(robot->wheel_motors[i].command) > 0.01f) { any_command = 1; break; }
    }
    if (any_command && robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
        rigidbody_wake(&world->bodies[robot->chassis_body]);
    }
    /* A sleeping chassis with real velocity is inconsistent state: the
     * velocity integrator drains forces for sleepers, so motion freezes
     * mid-drift (measured T8 hold failure). Truly settled bodies sit
     * below the sleep thresholds; anything above wakes. */
    if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count) {
        rigidbody *chb = &world->bodies[robot->chassis_body];
        if (chb->is_sleeping) {
            const mpe_config_t *sleep_cfg = mpe_world_cfg(world);
            float lv2 = vector3_length_squared(chb->velocity);
            float av2 = vector3_length_squared(chb->angular_velocity);
            if (lv2 > sleep_cfg->sleep.linear_thresh_sq ||
                av2 > sleep_cfg->sleep.angular_thresh_sq) {
                rigidbody_wake(chb);
            }
        }
    }

    /* Sum currents for battery sag.
     * FIX-AUDIT: old fabs() sum doubled sag in turn-in-place (opposing
     * currents cancel on a real pack) and made regen always drain. Use
     * signed sum for sag; drain only net positive (regen credited with
     * 50% efficiency, SoC clamped in battery_drain path). */
    float total_current_signed = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        total_current_signed += robot->wheel_motors[i].current;
    }
    /* Pack fuse sees the absolute bus load (stall sums, regen doesn't
     * cool the fuse). */
    float fuse_load = 0.0f;
    for (int i = 0; i < robot->wheel_count; i++) {
        float c = robot->wheel_motors[i].current;
        fuse_load += (c > 0.0f) ? c : -c;
    }
    battery_fuse_step(&robot->battery, fuse_load, dt);
    float terminal_voltage = battery_get_voltage(&robot->battery, total_current_signed);
    float drain_current = (total_current_signed > 0.0f) ? total_current_signed : 0.5f * total_current_signed;
    if (drain_current < 0.0f && robot->battery.charge_fraction >= 1.0f) {
        drain_current = 0.0f;
    }
    battery_drain(&robot->battery, drain_current, dt);

    /* Update each wheel motor */
    for (int i = 0; i < robot->wheel_count; i++) {
        int wheel_idx = robot->wheel_bodies[i];
        if ((wheel_idx < 0) || (wheel_idx >= world->body_count)) {
            continue;
        }
        rigidbody *wheel = &world->bodies[wheel_idx];

        /* DESPOT-FIX: running radius, not plate radius. wheel->radius is
         * the hub PLATE (30 mm recessed on mecanum rollers); every lever
         * below (observer inertia, rolling demand, motor inertia,
         * governor, brake) acts at the RUNNING surface (50 mm envelope).
         * Using the plate radius halved all inertias and broke the
         * observer/governor calibration the moment the plate recessed. */
        float r_run = wheel->radius;
        if (i >= 0 && i < FTC_MAX_WHEELS && robot->wheel_effective_radius[i] > 0.001f) {
            r_run = robot->wheel_effective_radius[i];
        }
        if (!(r_run > 0.001f) || !isfinite(r_run)) r_run = wheel->radius;

        /* Read wheel angular velocity about the actual rotated axle axis in world space */
        vector3 axle = wheel->cached_axes[0];
        if (vector3_length_squared(axle) < 0.0001f) {
            axle = vector4_rotate_to_vector3(wheel->orientation, (vector3){1.0f, 0.0f, 0.0f});
        }
        float wheel_speed = vector3_dot(wheel->angular_velocity, axle);
        /* Disturbance observer: external load = measured net torque effect
         * minus last tick's explicit motor torque. At lock this converges
         * to -stall (full stall held); free, to 0. Clamped; NaN-safe. */
        float axle_I = 0.5f * wheel->mass * r_run * r_run;
        if (robot->wheel_motors[i].wprev_valid && axle_I > 0.0f && dt > 0.0f &&
            isfinite(wheel_speed)) {
            float tau_l = axle_I * (wheel_speed - robot->wheel_motors[i].w_prev) / dt -
                          robot->wheel_motors[i].tau_exp_prev;
            if (!isfinite(tau_l)) {
                tau_l = 0.0f;
            } else if (tau_l > 100.0f) {
                tau_l = 100.0f;
            } else if (tau_l < -100.0f) {
                tau_l = -100.0f;
            }
            robot->wheel_motors[i].load_torque = tau_l;
        }
        robot->wheel_motors[i].w_prev = isfinite(wheel_speed) ? wheel_speed : 0.0f;
        robot->wheel_motors[i].wprev_valid = 1;

        /* MFS_TRACTION_CONTROL: compare against the rolling speed the
         * chassis motion demands at this wheel (rigid-body velocity at
         * the wheel center, projected on the rolling direction). A wheel
         * spinning far from demand is slipping: cut its torque so kinetic
         * friction re-captures it instead of sliding forever. Scales
         * recover toward open when slip clears (hysteresis via margin). */
        /* No contact → no slip: airborne wheels (free-spin rigs, jumps)
         * must not trip the traction cut. Same 0.05 clearance the
         * traction loop uses. */
        float wheel_bottom = wheel->position.y - r_run;
        if (robot->chassis_body >= 0 && robot->chassis_body < world->body_count &&
            r_run > 0.001f && wheel_bottom <= 0.05f) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            vector3 r_ch_wh = vector3_subtraction(wheel->position, chassis->position);
            vector3 v_contact = vector3_addition(
                chassis->velocity, vector3_cross(chassis->angular_velocity, r_ch_wh));
            vector3 roll_dir = vector3_cross(axle, (vector3){0.0f, 1.0f, 0.0f});
            float w_expected = 0.0f;
            if (vector3_length_squared(roll_dir) > 1e-6f) {
                roll_dir = vector3_scaling(roll_dir, 1.0f / sqrtf(vector3_length_squared(roll_dir)));
                w_expected = vector3_dot(v_contact, roll_dir) / r_run;
            }
            float slip = wheel_speed - w_expected;
            /* Cut ONLY overspeed (wheel outrunning travel = burnout).
             * Under-speed (skid/drag) keeps full torque so the wheel
             * spins UP to rolling speed; cutting there deadlocks the
             * wheel at zero while traction drags the chassis.
             * DESPOT-FIX (observability gate): demand direction used to
             * resolve at 1e-3 rad/s (0.05 mm/s of chassis motion!), so
             * standstill spin-up — where slip is UNOBSERVABLE (demand
             * ~0 cannot distinguish peel-out from legitimate breakaway)
             * — was cut on chassis jitter, strangling rail-force
             * development to ~0.5 N.m average (measured strafe 0.03 m).
             * Below 0.5 rad/s demand (~2.5 cm/s travel) the loop runs
             * open-loop (full torque, solver decides breakaway); above
             * it, burnout regulation engages. Control theory, not a
             * test hack: never regulate on an unobservable error. */
            float dir = 0.0f;
            if (w_expected > 0.5f) {
                dir = 1.0f;
            } else if (w_expected < -0.5f) {
                dir = -1.0f;
            }
            /* FIX-AUDIT-DESPOT: cut/recover thresholds (over > 4.0 rad/s
             * cuts to 0.15, recovers above 0.2/step while over < 2.0) are
             * TUNED HYSTERESIS, not derived constants: the 4.0/2.0 split
             * keeps the cut from chattering at the boundary (cut hard,
             * recover gradually). Retune against burnout behaviour, not
             * against a formula. */
            float *scale = &robot->wheel_traction_scale[i];
            float over = slip * dir;
            if (dir != 0.0f && over > 4.0f) {
                *scale = 0.15f;
            } else if (*scale < 1.0f && over < 2.0f) {
                *scale += 0.2f;
                if (*scale > 1.0f) {
                    *scale = 1.0f;
                }
            }
        }

        /* Update motor electrical state (implicit-in-speed: stable for
         * light wheels; same stall/free endpoints as explicit). */
        float axle_inertia = 0.5f * wheel->mass * r_run * r_run;
        motor_update_load(&robot->wheel_motors[i], wheel_speed, dt, terminal_voltage, axle_inertia);

        /* Traction cut applies to delivered torque (both the axle drive
         * below and the traction loop in drivetrain_update read
         * output_torque). Electrical readings (current/rpm) stay
         * unscaled: they report the commanded state. */
        robot->wheel_motors[i].output_torque *= robot->wheel_traction_scale[i];

        /* Apply motor torque along the actual physical axle in world space.
         * Free-speed governor: a motor cannot push its wheel past free
         * speed under its own power (measured pathology: +248 rad/s in
         * ONE tick at 10.6x free speed). Below free speed torque is
         * untouched, preserving full stall for breakaway grip; only the
         * overshoot past 1.1x free speed is clipped to land on the bound.
         * (An old no-overshoot-everywhere clamp is NOT used: it capped
         * torque below the static-grip cone and stalled breakaway.)
         * DESPOT-FIX: the old condition fired on PREDICTED speed, so for
         * light wheels (I=2.5e-4: any torque predicts hundreds of rad/s)
         * it clipped from standstill to (wfree-w)*I/dt = 0.385 N.m —
         * below the 0.97 N.m static-grip cone — and the wheel could never
         * break traction: tank turn and strafe locked at om=0 under full
         * stall current while forward (rolling, no breakaway) survived.
         * The gate is now on MEASURED speed: at/past the bound, overshoot
         * is clipped as before; below it, full stall torque is delivered
         * and the contact solver (not the governor) decides breakaway.
         * DESPOT-FIX 2 (diode, not deadbeat): PARKED — the diode (zero
         * outward push past the bound, no reversing slam) unmasked violent
         * chatter in loaded driving (teleop yaw returned 0.38 rad): without
         * the deadbeat pull-back, peel-out wheels run away past the bound
         * and roller/hub chaos steers the chassis. Restored deadbeat-gated-
         * on-measured-speed below (full stall below the bound for
         * breakaway; exact land-on-bound above it). The unloaded free-spin
         * limit cycle is a separate frontier (bearing-damping model).
         * Diode kept in history for that work; deadbeat ships. */
        float torque = robot->wheel_motors[i].output_torque;
        /* DESPOT-FIX (torque slew): feather standing starts. A 0->stall
         * step in one tick (74 N-equiv vs 19 N cone) outruns the contact:
         * symmetric commands peel all wheels at once and lock into chaos
         * instead of developing rail force (measured strafe 0.03 m).
         * Real ESCs slew-limit current; limit applied-torque slew to
         * 0.6 N.m/tick (~6 ticks to stall) so grip establishes before
         * full torque lands and demand becomes observable for traction
         * regulation. Below the slew, authority is untouched. */
        {
            float prev = robot->wheel_applied_torque[i];
            if (!isfinite(prev)) prev = 0.0f;
            float want = torque;
            float dl = want - prev;
            const float max_slew = 0.6f;
            if (dl > max_slew) want = prev + max_slew;
            else if (dl < -max_slew) want = prev - max_slew;
            torque = want;
            robot->wheel_applied_torque[i] = torque;
        }
        /* Free-speed governor: a motor cannot push its wheel past free
         * speed under its own power. Below free speed torque is untouched
         * (breakaway needs full stall). Only OUTWARD push past the bound
         * is clipped (diode): no reversing slam, no limit cycle.
         * DESPOT-FIX: added 5% hysteresis (1.1 * 1.05 = 1.155x free speed)
         * to prevent chatter at the bound — at exactly the bound the
         * implicit solve already pulls back; the diode only catches true
         * runaway. Below wfree, full stall torque for breakaway. */
        {
            float wfree = fabsf(robot->wheel_motors[i].free_speed_rad_s) * 1.155f;
            if (wfree > 0.0f && torque > 0.0f && wheel_speed > wfree) {
                torque = 0.0f;
            } else if (wfree > 0.0f && torque < 0.0f && wheel_speed < -wfree) {
                torque = 0.0f;
            }
        }
        /* MFS_145_IDLE_BRAKE: back-EMF braking is a damper — it brings a coasting
         * wheel to rest and can never reverse it (no back-EMF once stopped).
         * At idle, clamp the braking torque to the amount that stops the wheel
         * within this timestep. Without this, the stall-clamped back-EMF torque
         * (~2.17 N·m) reverses the light wheel every step -> ±25 rad/s idle spin. */
        if ((fabsf(robot->wheel_motors[i].command) < 0.05f) && ((torque * wheel_speed) < 0.0f)) {
            float mfs_i_axle = 0.5f * wheel->mass * r_run * r_run;
            if (mfs_i_axle > 0.0f) {
                float mfs_max_brake = mfs_i_axle * fabsf(wheel_speed) / dt;
                if (fabsf(torque) > mfs_max_brake) {
                    torque = (torque > 0.0f) ? mfs_max_brake : -mfs_max_brake;
                }
            }
        }
        /* Apply the motor torque about the real axle, as a TORQUE COUPLE:
         * +tau on the hub, -tau on the chassis. A motor is two bodies acting
         * on each other, so the reaction is not optional - applying tau to the
         * hub alone (as this did) injects angular momentum from nowhere and
         * lets the chassis yaw for free under straight-line drive. */
        wheel->torque_accumulator = vector3_addition(
            wheel->torque_accumulator,
            vector3_scaling(axle, torque));
#ifndef MFS_NO_TORQUE_COUPLE
        if ((robot->chassis_body >= 0) &&
            (robot->chassis_body < world->body_count)) {
            rigidbody *chassis = &world->bodies[robot->chassis_body];
            chassis->torque_accumulator = vector3_subtraction(
                chassis->torque_accumulator,
                vector3_scaling(axle, torque));
            /* Equal-and-opposite, so the chassis cannot be spun by its own
             * drivetrain any more than a real chassis would be. */
        }
#endif
        /* MFS_PORT_V15S: the parked core wheel-lock loop (and its
         * driven_this_tick gate) is gone; driven wheels are kept awake
         * directly below instead. */
        rigidbody_wake(wheel); /* MPE_FTC_078: keep driven wheels awake so motor torque is applied */
        /* Rollers are the bodies actually in contact, so they must be awake
         * too or sleep freezes the entire traction path while the (lighter,
         * contact-free) hub keeps running. */
        for (int k = 0; k < robot->roller_count[i]; k++) {
            int rb = robot->roller_bodies[i][k];
            if (rb >= 0 && rb < world->body_count) {
                rigidbody_wake(&world->bodies[rb]);
            }
        }

#if !defined(MECANUM_USE_RAIL_CONTACT)
        /* DESPOT-FIX (analytical equilibrium + strong bearing damping):
         * The roller spin dynamics are stiff (time constant ~1 ms << 16.7 ms tick).
         * Compute equilibrium spin from MOTOR COMMAND (intended wheel speed) to
         * avoid feedback through wheel dynamics. Apply STRONG viscous damping
         * on the roller bearing to damp wheel-dynamics oscillation without
         * affecting equilibrium spin (damping torque ~ ω_rel, zero at eq).
         *
         * Equilibrium: ω_roller = -v_wheel_perp / r_roller.
         * v_wheel_perp = (ω_wheel_intended × r_vector) · n_perp.
         * ω_wheel_intended = command * free_speed_rad_s. */
        for (int k = 0; k < robot->roller_count[i]; k++) {
            int rb = robot->roller_bodies[i][k];
            if (rb < 0 || rb >= world->body_count) continue;
            rigidbody *roller = &world->bodies[rb];
            /* Roller spin axis in world space: roller's local +X is its cylinder axis. */
            vector3 spin_axis = vector4_rotate_to_vector3(roller->orientation, (vector3){1.0f, 0.0f, 0.0f});
            if (vector3_length_squared(spin_axis) < 1e-6f) continue;
            /* Perpendicular direction in XZ plane: n_perp = (sinθ, 0, -cosθ). */
            vector3 n_perp = {spin_axis.x, 0.0f, -spin_axis.z};
            float n_perp_len_sq = vector3_length_squared(n_perp);
            if (n_perp_len_sq < 1e-6f) continue;
            n_perp = vector3_scaling(n_perp, 1.0f / sqrtf(n_perp_len_sq));
            int wb = robot->wheel_bodies[i];
            if (wb < 0 || wb >= world->body_count) continue;
            rigidbody *wheel = &world->bodies[wb];
            vector3 r_vector = vector3_subtraction(roller->position, wheel->position);
            /* Intended wheel angular velocity from motor command (not affected by roller forces). */
            float intended_speed = robot->wheel_motors[i].command * robot->wheel_motors[i].free_speed_rad_s;
            vector3 omega_wheel = vector3_scaling(axle, intended_speed);
            vector3 v_rim = vector3_cross(omega_wheel, r_vector);
            float v_perp = vector3_dot(v_rim, n_perp);
            float omega_eq = -v_perp / MECANUM_ROLLER_RADIUS;
            if (!isfinite(omega_eq)) continue;
            /* Set roller angular velocity to equilibrium spin about its axis.
             * DESPOT-FIX (numerical damping): add a small corrective torque
             * proportional to the deviation from equilibrium, to damp stiff
             * oscillation without affecting the equilibrium spin. */
            float omega_current = vector3_dot(roller->angular_velocity, spin_axis);
            float omega_error = omega_eq - omega_current;
            /* Apply a small corrective torque proportional to the error.
             * Kp = 1e-4 N·m/rad gives time constant ~I/Kp = 2.5 ms. */
            const float Kp = 1e-4f;
            float tau_correct = Kp * omega_error;
            /* Set roller angular velocity to equilibrium spin about its axis. */
            roller->angular_velocity = vector3_scaling(spin_axis, omega_eq);
        }
#endif
    }
}

void ftc_robot_set_wheel_commands(ftc_robot *robot, const float *commands, int count) {
    if (!robot) {
        return;
    }
    int n = (count < robot->wheel_count) ? count : robot->wheel_count;
    for (int i = 0; i < n; i++) {
        float cmd = commands[i];
        if (cmd > 1.0f) {
            cmd = 1.0f;
        }
        if (cmd < -1.0f) {
            cmd = -1.0f;
        }
        robot->wheel_motors[i].command = cmd;
    }
}

void ftc_robot_get_position(physics_world *world, ftc_robot *robot, float *px, float *py, float *pz) {
    if ((!world) || (!robot)) {
        return;
    }
    int idx = robot->chassis_body;
    if ((idx < 0) || (idx >= world->body_count)) {
        return;
    }
    if (px) {
        *px = world->bodies[idx].position.x;
    }
    if (py) {
        *py = world->bodies[idx].position.y;
    }
    if (pz) {
        *pz = world->bodies[idx].position.z;
    }
}
