/* MPE TUI dump — plain-text full-state snapshot of the physics engine.
 *
 * No ncurses calls here: output goes to any FILE (stdout, pipe, file), so
 * `mpe-tui --snapshot/--stream` BECOMES the terminal debug-output suite:
 * scriptable, diffable, greppable. Formatting is fixed-precision and
 * deterministic for a given simulation state.
 *
 * Sections: [engine] truth params + totals, [body i] full characteristics
 * and mathematics, [springs], [constraints], [pairs] relative positions,
 * [islands], [stats] solver/broadphase diagnostics, [result].
 */
#include "tui_debugger.h"
#include "../config/mpe_config.h"
#include "../config/mpe_constants.h"
#include "../physics/broadphase.h"
#include "../physics/islands.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *dump_type(object_type t) {
    switch (t) {
        case object_sphere:
            return "sphere";
        case object_cube:
            return "cube";
        case object_cylinder:
            return "cylinder";
        case object_custom:
            return "custom";
        default:
            return "unknown";
    }
}

static const char *dump_state(const rigidbody *rb) {
    if (!rb) {
        return "invalid";
    }
    if (rb->static_state) {
        return "static";
    }
    if (rb->kinematic) {
        return "kinematic";
    }
    if (rb->is_sleeping) {
        return "sleeping";
    }
    return "awake";
}

static const char *dump_ctype(constraint_type t) {
    switch (t) {
        case constraint_revolute:
            return "revolute";
        case constraint_fixed:
            return "fixed";
        case constraint_prismatic:
            return "prismatic";
        case constraint_distance:
            return "distance";
        case constraint_rope:
            return "rope";
        default:
            return "spring";
    }
}

static int dump_index_by_id(physics_world *world, uint32_t id) {
    if (!world || id == 0) {
        return -1;
    }
    /* Reuse the world id->index cache (O(1)) instead of linear scan. */
    int hit = physics_world_index_by_id(world, id);
    if (hit >= 0) {
        return hit;
    }
    for (int i = 0; i < world->body_count; i++) {
        if (world->bodies[i].object_id == id) {
            return i;
        }
    }
    return -1;
}

static vector3 dump_anchor_world(const rigidbody *rb, vector3 local) {
    return vector3_addition(rb->position, vector4_rotate_to_vector3(rb->orientation, local));
}

int tui_dump_snapshot(FILE *out, physics_world *world, unsigned long tick, float dt) {
    if (!out || !world || !world->bodies) {
        return 1;
    }
    int bad = 0;
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        if (!isfinite(rb->position.x) || !isfinite(rb->position.y) || !isfinite(rb->position.z) ||
            !isfinite(rb->velocity.x) || !isfinite(rb->velocity.y) || !isfinite(rb->velocity.z) ||
            !isfinite(rb->angular_velocity.x) || !isfinite(rb->angular_velocity.y) ||
            !isfinite(rb->angular_velocity.z)) {
            bad = 1;
        }
    }

    char b0[160], b1[160], b2[320];
    fprintf(out, "### MPE-TUI snapshot tick=%lu time=%.4f dt=%.5f bodies=%d result=%s\n", tick, tick * (double) dt,
            dt, world->body_count, bad ? "FAIL(non-finite)" : "PASS");

    /* ---- engine truth parameters + totals ---- */
    double ke = 0.0;
    vector3 mom = {0.0f, 0.0f, 0.0f};
    int awake = 0, sleeping = 0, statics = 0, kinematics = 0;
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        ke += (double) rb_get_kinetic_energy(rb);
        mom = vector3_addition(mom, vector3_scaling(rb->velocity, rb->mass));
        if (rb->static_state) {
            statics++;
        } else if (rb->kinematic) {
            kinematics++;
        } else if (rb->is_sleeping) {
            sleeping++;
        } else {
            awake++;
        }
    }
    /* Per-world truth params (a scene-local config may differ from g_cfg). */
    const mpe_config_t *C = mpe_world_cfg(world);
    int ncustom = 0;
    for (int ci = 0; ci < world->body_count; ci++) {
        if (world->bodies[ci].type == object_custom) {
            ncustom++;
        }
    }
    fprintf(out, "[engine] gravity=%+.4f drag=%.5f angScale=%.4f iters=%d substeps=%d\n",
            C->world.gravity, C->world.drag, C->world.angular_damping_scale, C->timestep.solver_iterations,
            C->timestep.max_substeps);
    fprintf(out, "[engine] slop=%.4f beta=%.3f maxBias=%.2f restThresh=%+.2f warmMatchSq=%.5f\n",
            C->solver.penetration_slop, C->solver.bias_factor, C->solver.max_separation_bias,
            C->solver.restitution_velocity_thresh, C->solver.warm_start_match_dist_sq);
    fprintf(out, "[engine] sleepEn=%d linThSq=%.5f angThSq=%.6f timer=%.2f wakeLinSq=%.4f wakeAngSq=%.5f\n",
            C->sleep.enable, C->sleep.linear_thresh_sq, C->sleep.angular_thresh_sq, C->sleep.timer_duration,
            C->sleep.wake_linear_thresh_sq, C->sleep.wake_angular_thresh_sq);
    fprintf(out, "[engine] depenFactor=%.2f depenMax=%.3f wakeDepth=%.3f rebuildIters=%d rollMu=%.4f\n",
            C->depenetration.correction_factor, C->depenetration.max_correction,
            C->depenetration.wake_depth_thresh, C->depenetration.rebuild_iterations,
            C->world.rolling_resistance_coeff);
    fprintf(out, "[engine] jointBeta=%.2f jointMaxBias=%.2f motorGain=%.2f maxAccel=%.1f floorSlop=%.3f\n",
            C->joints.revolute_beta, C->joints.revolute_max_bias, C->joints.revolute_motor_gain,
            C->joints.max_acceleration, C->boundary.floor_emergency_slop);
    fprintf(out, "[world] awake=%d sleeping=%d static=%d kinematic=%d custom=%d tickMods=%d bodies=%d/%d KE=%.6f P=(%+.4f,%+.4f,%+.4f)|P|=%.5f\n",
            awake, sleeping, statics, kinematics, ncustom, world->tick_module_count, world->body_count,
            world->body_capacity, ke, mom.x, mom.y, mom.z, vector3_length(mom));

    /* ---- bodies: characteristics + mathematics ---- */
    for (int i = 0; i < world->body_count; i++) {
        rigidbody *rb = &world->bodies[i];
        fprintf(out, "[body %d] id=%u gen=%u type=%s state=%s\n", i, rb->object_id, rb->object_generation,
                dump_type(rb->type), dump_state(rb));
        fprintf(out, "  pos=(%+.5f,%+.5f,%+.5f) vel=(%+.5f,%+.5f,%+.5f)|v|=%.5f\n", rb->position.x, rb->position.y,
                rb->position.z, rb->velocity.x, rb->velocity.y, rb->velocity.z, vector3_length(rb->velocity));
        fprintf(out, "  acc=(%+.5f,%+.5f,%+.5f)\n", rb->acceleration.x, rb->acceleration.y, rb->acceleration.z);
        fprintf(out, "  quat=(%+.5f,%+.5f,%+.5f,%+.5f) ", rb->orientation.w, rb->orientation.x, rb->orientation.y,
                rb->orientation.z);
        tui_format_euler(b0, sizeof(b0), rb->orientation, "");
        fprintf(out, "eulerXYZdeg=%s\n", b0);
        fprintf(out, "  angvel=(%+.5f,%+.5f,%+.5f)|w|=%.5f angacc=(%+.5f,%+.5f,%+.5f)\n", rb->angular_velocity.x,
                rb->angular_velocity.y, rb->angular_velocity.z, vector3_length(rb->angular_velocity),
                rb->angular_acceleration.x, rb->angular_acceleration.y, rb->angular_acceleration.z);
        fprintf(out, "  mass=%.5f invM=%.7f rest=%.3f fricS=%.3f fricK=%.3f nice=%d\n", rb->mass,
                rb->inverse_mass, rb->restitution, rb->friction_static, rb->friction_kinetic, rb->nice_value);
        fprintf(out, "  radius=%.5f halfLen=%.5f halfExt=(%.4f,%.4f,%.4f) boundR=%.5f\n", rb->radius,
                rb->cylinder_half_length, rb->half_extensions.x, rb->half_extensions.y, rb->half_extensions.z,
                broadphase_bounding_radius(rb));
        math3 il = rb->inertia_tensor_local;
        fprintf(out, "  I_local=diag(%.7f,%.7f,%.7f)\n", il.matrix[0][0], il.matrix[1][1], il.matrix[2][2]);
        math3 r = vector4_to_math3(rb->orientation);
        math3 iw = math3_multiplication(r, math3_multiplication(rb->inertia_tensor_local, math3_transposition(r)));
        tui_format_matrix3(b2, sizeof(b2), iw, "");
        fprintf(out, "  I_world=%s\n", b2);
        tui_format_vector3(b0, sizeof(b0), rb->force_accumulator, "");
        tui_format_vector3(b1, sizeof(b1), rb->torque_accumulator, "");
        fprintf(out, "  Facc=%s Tacc=%s\n", b0, b1);
        vector3 p = vector3_scaling(rb->velocity, rb->mass);
        vector3 l = math3_multiplication_vector3(iw, rb->angular_velocity);
        fprintf(out, "  P=(%+.5f,%+.5f,%+.5f)|P|=%.6f L=(%+.6f,%+.6f,%+.6f)|L|=%.7f KE=%.7f\n", p.x, p.y, p.z,
                vector3_length(p), l.x, l.y, l.z, vector3_length(l), rb_get_kinetic_energy(rb));
        int isl = islands_body_island(world, rb);
        int has = (world->has_contact && i < mpe_max_bodies) ? world->has_contact[i] : -1;
        float rem = (world->ccd_time_remaining && i < mpe_max_bodies) ? world->ccd_time_remaining[i] : -1.0f;
        fprintf(out, "  sleepT=%.3f island=%d awake=%d hasContact=%d ccdRem=%.6f effInvM=%.7f colour=(%.2f,%.2f,%.2f)\n",
                rb->sleep_timer, isl, islands_body_awake(world, rb) ? 1 : 0, has, rem,
                rigidbody_effective_inv_mass(rb), rb->colour.x, rb->colour.y, rb->colour.z);
        fprintf(out, "  axes X=(%+.4f,%+.4f,%+.4f) Y=(%+.4f,%+.4f,%+.4f) Z=(%+.4f,%+.4f,%+.4f)\n",
                rb->cached_axes[0].x, rb->cached_axes[0].y, rb->cached_axes[0].z, rb->cached_axes[1].x,
                rb->cached_axes[1].y, rb->cached_axes[1].z, rb->cached_axes[2].x, rb->cached_axes[2].y,
                rb->cached_axes[2].z);
    }

    /* ---- springs ---- */
    int springs = 0;
    for (int i = 0; i < mpe_max_joints; i++) {
        if (world->spring_joints[i].is_active) {
            springs++;
        }
    }
    fprintf(out, "[springs] active=%d\n", springs);
    for (int i = 0; i < mpe_max_joints; i++) {
        spring_joint *sj = &world->spring_joints[i];
        if (!sj->is_active) {
            continue;
        }
        int ia = dump_index_by_id(world, sj->object_id_a);
        int ib = dump_index_by_id(world, sj->object_id_b);
        if (ia < 0 || ib < 0) {
            fprintf(out, "  spring slot=%d endpoint-missing A(id%u->%d) B(id%u->%d)\n", i, sj->object_id_a, ia,
                    sj->object_id_b, ib);
            continue;
        }
        vector3 d = vector3_subtraction(world->bodies[ib].position, world->bodies[ia].position);
        float len = vector3_length(d);
        vector3 axis = len > 1e-9f ? vector3_scaling(d, 1.0f / len) : (vector3){1.0f, 0.0f, 0.0f};
        float ext = len - sj->equilibrium_length;
        vector3 rv = vector3_subtraction(world->bodies[ib].velocity, world->bodies[ia].velocity);
        float vr = vector3_dot(rv, axis);
        fprintf(out, "  spring slot=%d A=%d(id%u)@(%+.4f,%+.4f,%+.4f) B=%d(id%u)@(%+.4f,%+.4f,%+.4f)\n", i, ia,
                sj->object_id_a, world->bodies[ia].position.x, world->bodies[ia].position.y,
                world->bodies[ia].position.z, ib, sj->object_id_b, world->bodies[ib].position.x,
                world->bodies[ib].position.y, world->bodies[ib].position.z);
        fprintf(out, "    L0=%.4f len=%.4f ext=%+.4f axis=(%+.4f,%+.4f,%+.4f) vrel=%+.4f k=%.3f c=%.3f Hooke=%+.4f\n",
                sj->equilibrium_length, len, ext, axis.x, axis.y, axis.z, vr, sj->spring_constant,
                sj->damping_coefficient, ext * sj->spring_constant + vr * sj->damping_coefficient);
    }

    /* ---- generic constraints ---- */
    int constr = 0;
    for (int i = 0; i < mpe_max_joints; i++) {
        if (world->revolute_constraints[i].is_active) {
            constr++;
        }
    }
    fprintf(out, "[constraints] active=%d\n", constr);
    for (int i = 0; i < mpe_max_joints; i++) {
        constraint *c = &world->revolute_constraints[i];
        if (!c->is_active) {
            continue;
        }
        int ia = dump_index_by_id(world, c->body_id_a);
        int ib = dump_index_by_id(world, c->body_id_b);
        fprintf(out, "  %s slot=%d A=%d(id%u) B=%d(id%u)", dump_ctype(c->type), i, ia, c->body_id_a, ib,
                c->body_id_b);
        if (ia < 0 || ib < 0) {
            fprintf(out, " endpoint-missing\n");
            continue;
        }
        rigidbody *a = &world->bodies[ia];
        rigidbody *b = &world->bodies[ib];
        if (c->type == constraint_revolute) {
            vector3 wa = dump_anchor_world(a, c->p.revolute.anchor_a);
            vector3 wb = dump_anchor_world(b, c->p.revolute.anchor_b);
            vector3 err = vector3_subtraction(wb, wa);
            vector3 axw = vector4_rotate_to_vector3(a->orientation, vector3_normalisation(c->p.revolute.axis_a));
            vector3 relw = vector3_subtraction(b->angular_velocity, a->angular_velocity);
            fprintf(out, " wA=(%+.4f,%+.4f,%+.4f) wB=(%+.4f,%+.4f,%+.4f) |err|=%.6f\n", wa.x, wa.y, wa.z, wb.x,
                    wb.y, wb.z, vector3_length(err));
            fprintf(out, "    axis=(%+.4f,%+.4f,%+.4f) wAxis=%+.5f angle=%+.3fdeg init=%d lim[%d %.2f..%.2f] motor[%d v=%+.3f max=%.2f]\n",
                    axw.x, axw.y, axw.z, vector3_dot(relw, axw),
                    c->p.revolute.accumulated_angle * 180.0f / 3.14159265f,
                    c->p.revolute.angle_initialized ? 1 : 0, c->p.revolute.limits_enabled ? 1 : 0,
                    c->p.revolute.limit_min_rad * 180.0f / 3.14159265f,
                    c->p.revolute.limit_max_rad * 180.0f / 3.14159265f,
                    c->p.revolute.motor_enabled ? 1 : 0, c->p.revolute.motor_target_speed,
                    c->p.revolute.motor_max_torque);
        } else if (c->type == constraint_prismatic) {
            vector3 axw = vector4_rotate_to_vector3(a->orientation, vector3_normalisation(c->p.prismatic.axis_a));
            vector3 ra = dump_anchor_world(a, c->p.prismatic.anchor_a);
            vector3 rb2 = dump_anchor_world(b, c->p.prismatic.anchor_b);
            float cur = vector3_dot(vector3_subtraction(rb2, ra), axw);
            fprintf(out, " axis=(%+.4f,%+.4f,%+.4f) cur=%+.5f track=%+.5f init=%d lim[%d %.3f..%.3f] motor[%d v=%+.3f F=%.2f]\n",
                    axw.x, axw.y, axw.z, cur, c->p.prismatic.accumulated_position,
                    c->p.prismatic.position_initialized ? 1 : 0, c->p.prismatic.limits_enabled ? 1 : 0,
                    c->p.prismatic.limit_min, c->p.prismatic.limit_max,
                    c->p.prismatic.motor_enabled ? 1 : 0, c->p.prismatic.motor_target_speed,
                    c->p.prismatic.motor_max_force);
        } else if (c->type == constraint_fixed) {
            vector3 wa = dump_anchor_world(a, c->p.fixed.anchor_a);
            vector3 wb = dump_anchor_world(b, c->p.fixed.anchor_b);
            fprintf(out, " gap=%.6f\n", vector3_length(vector3_subtraction(wb, wa)));
        } else if (c->type == constraint_distance) {
            vector3 wa = dump_anchor_world(a, c->p.distance.anchor_a);
            vector3 wb = dump_anchor_world(b, c->p.distance.anchor_b);
            float d = vector3_length(vector3_subtraction(wb, wa));
            fprintf(out, " dist=%.5f rest=%.5f err=%+.5f\n", d, c->p.distance.rest_length,
                    d - c->p.distance.rest_length);
        } else if (c->type == constraint_rope) {
            vector3 wa = dump_anchor_world(a, c->p.rope.anchor_a);
            vector3 wb = dump_anchor_world(b, c->p.rope.anchor_b);
            float d = vector3_length(vector3_subtraction(wb, wa));
            fprintf(out, " dist=%.5f max=%.5f %s\n", d, c->p.rope.rest_length,
                    d > c->p.rope.rest_length ? "TAUT" : "slack");
        } else {
            fprintf(out, "\n");
        }
    }

    /* ---- relative positions: all pairs ----
     * LOSSY SUMMARY: for n>24 only 64 pairs are listed, and the min/max
     * scan covers only the first 96 bodies (24/64/96 truncation). This
     * keeps snapshots diffable at stress scale; it is not the full graph. */
    int n = world->body_count;
    int cap = n > 96 ? 96 : n;
    long total_pairs = (long) n * (n - 1) / 2;
    float cmin = 1e30f, cmax = -1e30f;
    int cmini = -1, cminj = -1, cmaxi = -1, cmaxj = -1;
    for (int i = 0; i < cap; i++) {
        for (int j = i + 1; j < cap; j++) {
            float d = vector3_length(
                vector3_subtraction(world->bodies[j].position, world->bodies[i].position));
            if (d < cmin) {
                cmin = d;
                cmini = i;
                cminj = j;
            }
            if (d > cmax) {
                cmax = d;
                cmaxi = i;
                cmaxj = j;
            }
        }
    }
    fprintf(out, "[pairs] total=%ld shown=%s closest=[%d]<->[%d] %.5f farthest=[%d]<->[%d] %.5f\n", total_pairs,
            n <= 24 ? "all" : "truncated", cmini, cminj, cmin, cmaxi, cmaxj, cmax);
    int shown = 0;
    int show_all = n <= 24;
    for (int i = 0; i < cap; i++) {
        for (int j = i + 1; j < cap; j++) {
            if (!show_all && shown >= 64) {
                break;
            }
            vector3 d = vector3_subtraction(world->bodies[j].position, world->bodies[i].position);
            fprintf(out, "  pair %d<->%d dist=%.5f d=(%+.4f,%+.4f,%+.4f)\n", i, j, vector3_length(d), d.x, d.y,
                    d.z);
            shown++;
        }
        if (!show_all && shown >= 64) {
            break;
        }
    }
    if (!show_all) {
        fprintf(out, "  ... truncated to %d pairs (scan covered first %d bodies)\n", shown, cap);
    }

    /* ---- islands ---- */
    fprintf(out, "[islands] count=%d\n", islands_count(world));
    for (int i = 0; i < n; i++) {
        int isl = islands_body_island(world, &world->bodies[i]);
        fprintf(out, "  body %d island=%d awake=%d\n", i, isl, islands_body_awake(world, &world->bodies[i]) ? 1 : 0);
    }

    /* ---- solver / broadphase diagnostics ---- */
    fprintf(out, "[stats] cacheHit=%d cacheMiss=%d cacheCount=%d/%d manifoldOvfl=%d cell=%.4f\n",
            contact_cache_get_hits(world), contact_cache_get_misses(world), world->world_contact_cache_count,
            world->world_contact_cache_capacity, world->manifold_overflow_count,
            broadphase_get_current_cell_size(world));
    fprintf(out, "[stats] bpNodes=%d/%d pairOvfl=%d dedupOvfl=%d bigClamp=%d\n",
            world->broadphase ? world->broadphase->node_count : -1,
            world->broadphase ? world->broadphase->node_pool_capacity : -1,
            broadphase_get_pair_overflow_count(world), broadphase_get_pair_dedupe_overflow_count(world),
            broadphase_get_large_object_clamp_count(world));
    fprintf(out, "[result] %s\n", bad ? "FAIL" : "PASS");
    return bad ? 1 : 0;
}
