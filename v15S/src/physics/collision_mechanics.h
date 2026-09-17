#ifndef collisions_h
#define collisions_h

#include <stdio.h>
#include <math.h>
#include "../core/math3d.h"
#include "../core/rigidbody.h"
struct physics_world; /* MFS_131: forward decl for per-world cache */
typedef struct {
    vector3 position;
    float penetration;
    vector3 local_position_a;
    vector3 local_position_b;
    float accumulated_normal_impulse;
    float accumulated_tangent_impulse;
    float effective_mass_normal;
    float effective_mass_tangent;
    vector3 tangent_vector;
    /* Second Coulomb tangent (disc model): tangent2 = n x tangent1.
     * Solved jointly with a combined |Ft| <= mu*Fn clamp. */
    vector3 tangent2;
    float accumulated_tangent2_impulse;
    float effective_mass_tangent2;
    /* cached_tangent REMOVED (dead field, never read; tangent_vector +
     * tangent2 are the live Coulomb-disc frame). */
    /* Poisson restitution state: Newtonian velocity bias is wrong for
     * multi-contact (it pays bounce per iteration). Instead the compression
     * phase accumulates unbiased impulse; the restitution pass then pays
     * e * (this tick's compression impulse) once. impact_velocity gates the
     * pass to fresh impacts; base_normal_impulse excludes warm start. */
    float impact_velocity;
    float base_normal_impulse;
    /* warmed REMOVED (dead provenance flag; SOR damping deleted with the
     * strict warm-start match — all contacts solve full steps). */
    /* NOTE: there is deliberately NO velocity-level Baumgarte bias field.
     * Penetration is corrected positionally only (split impulse +
     * depenetration pass). A velocity bias would inject approach velocity
     * that the friction clamp and Poisson pass then treat as real impact. */
    vector3 ra;
    vector3 rb;
} contact_point_data;

typedef struct {
    rigidbody *object_a;
    rigidbody *object_b;
    vector3 normal_vector;
    contact_point_data contacts[4];
    int contact_count;
} collision_data;
bool collision_dual_sphere(rigidbody *rigidbody_object_a, rigidbody *rigidbody_object_b,
                           collision_data *collision_output_data);
float project_obb(rigidbody *rigid_body, vector3 axis, vector3 axes[3]);
bool collision_sphere_cube(rigidbody *sphere, rigidbody *cube, collision_data *collision_output_data);
bool collision_dual_cube(rigidbody *cube_a, rigidbody *cube_b, collision_data *collision_output_data);
void collision_prepare_solver(struct physics_world *world, collision_data *source, collision_data *manifold_entry,
                              float dt);
/* Returns the largest impulse magnitude applied this visit (for convergence tests). */
float collision_resolve_iterative(collision_data *manifold_entry, float dt, bool friction_only, int start_index);
/* Support-first manifold order: indices sorted by lowest contact height
 * (floor contacts first, then ascending pairs), ties broken by manifold
 * index for a deterministic total order. Sequential impulse propagates
 * support ~one contact level per sweep in arbitrary order; support-first
 * order carries floor support to the top of a stack in a single sweep, so
 * deep stacks converge in far fewer iterations. Order affects only the
 * sweep sequence (same equations); twin runs agree bit-for-bit. Keys live
 * in the world's scratch. */
void collision_manifold_solve_order(struct physics_world *world, collision_data *manifolds, int manifold_count,
                                    int *order_out);
/* Split impulse (Catto): positional penetration correction applied AFTER
 * the velocity iterations, directly to positions, mass-weighted. Carries
 * no velocity change, so it cannot inflate contact impulses or friction. */
/* Split impulse (Catto): positional penetration correction applied AFTER
 * the velocity iterations, directly to positions, mass-weighted. Carries
 * no velocity change, so it cannot inflate contact impulses or friction. */
void collision_apply_split_impulse(collision_data *manifolds, int manifold_count, float dt);
/* Rolling + spin resistance, once per tick after the velocity solve. */
void collision_apply_rolling_resistance(collision_data *manifolds, int manifold_count, float dt);
/* Poisson restitution: after compression converges, each fresh impact gets
 * e * (this tick's compression impulse) once, then a short relaxation lets
 * friction respond. Correct for multi-contact; Newtonian bias over-pays. */
void collision_apply_poisson_restitution(collision_data *manifolds, int manifold_count);
/* TRUTH P0-2: refresh Poisson gate to post-force-integration velocities.
 * Prepare() runs before gravity/springs/motors are integrated, so the
 * recorded impact_velocity is stale by g*dt + spring/motor deltas.
 * Call after rb_integrate_velocity, before iterations: recomputes
 * vn = (vb+wb×rb − va−wa×ra)·n from current velocities (ra/rb/n unchanged,
 * positions not yet moved). Exact pre-solve approach speed. */
void collision_refresh_impact_velocities(collision_data *manifolds, int manifold_count);
/* CCD swept clamp: for bodies whose per-tick displacement exceeds their
 * contact thickness, analytic time-of-impact against the floor plane,
 * spheres, and static boxes. Clamps the body to the TOI configuration and
 * keeps velocity, so discrete narrowphase then sees penetration≈0 with the
 * true approach velocity (restitution/friction respond correctly).
 * TRUTH P0-3: fills time_remaining_out[i] = dt - toi (dt if unclamped).
 * Post-solve integration MUST advance only the remainder, else toi+dt
 * double-counts. Linear sweep only (angular motion ignored over the tick);
 * dynamic-box obstacles are paired by the swept broadphase but not TOI-clamped. */
int collision_ccd_sweep_clamp(rigidbody *bodies, int body_count, float dt);
int collision_ccd_sweep_clamp_full(rigidbody *bodies, int body_count, float dt, float *time_remaining_out);
void contact_cache_save(struct physics_world *world, collision_data *manifolds, int count); /* MFS_131 */
void contact_cache_clear(struct physics_world *world); /* MFS_131 */

bool collision_static_plane_sphere(rigidbody *sphere, float plane_y, collision_data *collision_output_data);
bool collision_static_plane_cube(rigidbody *cube, float plane_y, collision_data *collision_output_data);
bool collision_static_plane_body(rigidbody *body, float plane_y, collision_data *collision_output_data);

void contact_cache_stats_reset(struct physics_world *world);
int contact_cache_get_hits(const struct physics_world *world);
int contact_cache_get_misses(const struct physics_world *world);
/* Pair-novelty probe: true if this id pair has any entry saved from a prior
 * tick (either order). Drives wake-on-first-touch (see implementation). */
bool contact_cache_has_pair(struct physics_world *world, uint32_t id_a, uint32_t id_b);
/* Cylinder-vs-object narrowphase. TRUE solid-cylinder geometry:
 * flat end-caps (SDF), rim circle, inside SDF branch; segment-OBB convex
 * exact for cube; coaxial face-gap + parallel 2-point for cyl-cyl.
 * Barrel contacts exact; see collision_cylinder.c. */
bool collision_cylinder_sphere(rigidbody *cyl, rigidbody *sph,
                               collision_data *out);
bool collision_cylinder_cube(rigidbody *cyl, rigidbody *cube,
                             collision_data *out);
bool collision_cylinder_cylinder(rigidbody *cyl_a, rigidbody *cyl_b,
                                 collision_data *out);

#endif
