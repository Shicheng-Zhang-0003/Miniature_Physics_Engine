/* MFS Module 1 Test
 * Verifies module loads, creates field/robot, and runs simulation
 */
#ifdef MFS_MODULE_1_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "core/mpe_module.h"
#include "core/det_math.h"
#include "core/mpe_registry.h"
#include "config/mpe_config.h"
#include "mfs_module_1.h"

int main(void) {
    printf("[MFS Module 1 Test] Starting...\n");
    
    mpe_config_init();
    
    /* Create physics world */
    physics_world world;
    physics_world_init(&world);
    
    /* Load module descriptor */
    const mpe_module_desc_t *desc = &mfs_module_1_desc;
    if (!desc) {
        printf("[FAIL] Module descriptor not found\n");
        return 1;
    }
    printf("[OK] Module descriptor: %s v%s (ABI %u)\n", 
           desc->name, desc->version, desc->abi);
    
    /* Attach module */
    void *mod_state = NULL;
    if (desc->attach(&world, &mod_state) != 0) {
        printf("[FAIL] Module attach failed\n");
        return 1;
    }
    printf("[OK] Module attached\n");
    
    mfs_module_1_state *state = (mfs_module_1_state *)mod_state;

    /* Verify field created. PHYSICS-FIX: object_id 0 is a valid first id,
     * so ==0 conflated "unset" (calloc zero) with a live floor. Verify by
     * lookup instead: the id must resolve to a body. */
    if (!physics_world_body_by_id(&world, state->field_floor_id)) {
        printf("[FAIL] Field floor not created (id=%u)\n", state->field_floor_id);
        return 1;
    }
    printf("[OK] Field created (floor id=%u)\n", state->field_floor_id);
    
    /* Verify robot created */
    if (!state->robot_created) {
        printf("[FAIL] Robot not created\n");
        return 1;
    }
    printf("[OK] Robot created (wheel_count=%d)\n", state->robot.wheel_count);
    
    /* Verify balls spawned */
    if (state->ball_count == 0) {
        printf("[FAIL] No balls spawned\n");
        return 1;
    }
    printf("[OK] %d balls spawned\n", state->ball_count);
    
    /* Run simulation for 100 ticks */
    const float dt = 1.0f / 60.0f;
    for (int t = 0; t < 100; t++) {
        if (desc->pre_step) {
            desc->pre_step(&world, dt, mod_state);
        }
        
        physics_world_step(&world, dt);
        
        if (desc->post_step) {
            desc->post_step(&world, dt, mod_state);
        }
        
        /* Check for NaN (null-safe: chassis may be gone after detach paths) */
        rigidbody *chassis_check = mfs_get_chassis(state);
        if (!chassis_check || !isfinite(chassis_check->position.x)) {
            printf("[FAIL] NaN/missing chassis at tick %d\n", t);
            return 1;
        }
        
        /* Test control inputs at tick 30 */
        if (t == 30) {
            mfs_module_1_set_drive_commands(state, 1.0f, 0.0f, 0.0f);
            mfs_module_1_set_intake(state, true);
        }
        
        /* Test shooter at tick 50 */
        if (t == 50) {
            mfs_module_1_set_shooter(state, true, false);
        }
        
        /* Fire at tick 80 */
        if (t == 80) {
            mfs_module_1_set_shooter(state, true, true);
        }
    }
    
    /* Verify robot moved */
    rigidbody *chassis = mfs_get_chassis(state);
    if (!chassis) {
        printf("[FAIL] Chassis body not found\n");
        return 1;
    }
    
    float dist = sqrtf(chassis->position.x * chassis->position.x + 
                       chassis->position.z * chassis->position.z);
    if (dist < 0.1f) {
        printf("[FAIL] Robot didn't move (dist=%.3f)\n", dist);
        return 1;
    }
    printf("[OK] Robot moved %.3f meters\n", dist);
    
    /* Verify shooter spun up */
    if (state->shooter_rpm < 1000.0f) {
        printf("[FAIL] Shooter didn't spin up (rpm=%.1f)\n", state->shooter_rpm);
        return 1;
    }
    printf("[OK] Shooter RPM: %.1f\n", state->shooter_rpm);
    
    /* Verify balls still exist */
    int valid_balls = 0;
    for (int i = 0; i < state->ball_count; i++) {
        if (physics_world_body_by_id(&world, state->ball_body_ids[i])) {
            valid_balls++;
        }
    }
    if (valid_balls == 0) {
        printf("[FAIL] All balls lost\n");
        return 1;
    }
    printf("[OK] %d/%d balls remain in simulation\n", valid_balls, state->ball_count);
    
    /* Detach module */
    if (desc->detach) {
        desc->detach(&world, mod_state);
    }
    printf("[OK] Module detached\n");
    
    physics_world_cleanup(&world);
    
    printf("\n[PASS] MFS Module 1 test complete\n");
    return 0;
}
#endif