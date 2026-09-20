/* FTC fleet tick module ("ftc-fleet"): the import surface of the MPE
 * kernel module ecosystem.
 *
 * A host that can dlopen this translation unit (directly or via
 * mpe_loader_load, same call the `mod load` terminal command makes)
 * gets a hot-pluggable robot fleet:
 *
 *   const mpe_module_desc_t *d = <from registry or dlsym>;
 *   physics_world_attach_module(world, d);   // allocate per-world fleet
 *   ftc_fleet_spawn(world, ...);             // add robots (any time)
 *   physics_world_step(world, dt);           // pre_step drives the fleet
 *   physics_world_detach_module(world, "ftc-fleet");
 *
 * pre_step runs drivetrain_update() per fleet member at exactly the
 * tick position the manual flow uses (motor torques + traction forces
 * land in the accumulators right before velocity integration), so the
 * module path is behaviorally identical to calling drivetrain_update()
 * by hand. Commands are still set by the host each tick
 * (drivetrain_tank/mecanum on ftc_fleet_get(world, i)).
 *
 * Determinism: marked NON-deterministic — the odometry heading frame
 * integrates with libm cosf/sinf (drivetrain.c). Force/torque paths
 * are IEEE-exact; only the pose readout varies across libms. Same
 * machine + same libm is bit-stable (proven by ftc_hotload_test).
 */
#include "ftc_fleet.h"
#include "core/mpe_module.h"

static int ftc_fleet_attach(mpe_world_t *world, void **mod_state) {
    if (!world || !mod_state) return -1;
    void *f = ftc_fleet_create();
    if (!f) return -1;
    *mod_state = f;
    return 0;
}

static void ftc_fleet_detach(mpe_world_t *world, void *mod_state) {
    (void)world;
    ftc_fleet_destroy(mod_state);
}

static void ftc_fleet_pre_step(mpe_world_t *world, float dt, void *mod_state) {
    ftc_fleet_step_all(world, mod_state, dt);
}

const mpe_module_desc_t mpe_module_desc = {
    .abi = MPE_MODULE_ABI,
    .name = "ftc-fleet",
    .version = "1.0",
    .kind = "generic",
    .deterministic = false, /* odometry cosf/sinf; see above */
    .attach = ftc_fleet_attach,
    .detach = ftc_fleet_detach,
    .pre_step = ftc_fleet_pre_step,
    .post_step = 0,
};
