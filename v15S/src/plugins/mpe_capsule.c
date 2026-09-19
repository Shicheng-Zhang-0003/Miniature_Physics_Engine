/* Example foreign physics component: capsule-ish custom shape (id 100).
 * Fully self-contained (no engine collision symbols) so the .so dlopens
 * anywhere. Bounding-sphere contact: true solid-sphere test vs the
 * other body's centre with slop, mirroring collision_dual_sphere.
 * Build: make plugins/mpe_capsule.so ; load via `mod load` or mpe_loader_load.
 */
#include "../core/mpe_module.h"
#include "../core/mpe_registry.h"
#include "../core/rigidbody.h"
#include "../core/physics_world.h"
#include "../physics/collision_mechanics.h"
#include <math.h>

static bool capsule_contact(rigidbody *cap, rigidbody *other, void *out) {
    collision_data *o = (collision_data *)out;
    vector3 d = vector3_subtraction(other->position, cap->position);
    /* Self-contained bounding radius (no engine symbols so .so dlopens
     * anywhere; engine path uses broadphase_bounding_radius). */
    float other_r = other->radius;
    if (other->type == object_cube) {
        other_r = sqrtf(other->half_extensions.x * other->half_extensions.x +
                        other->half_extensions.y * other->half_extensions.y +
                        other->half_extensions.z * other->half_extensions.z);
    } else if (other->type == object_cylinder) {
        other_r = sqrtf(other->radius * other->radius +
                        other->cylinder_half_length * other->cylinder_half_length);
    } else if (other->type == object_custom) {
        other_r = other->radius > 0.0f ? other->radius : 0.5f;
    }
    float rr = cap->radius + other_r;
    float dist2 = vector3_length_squared(d);
    const float slop = 0.01f;
    if (dist2 >= (rr + slop) * (rr + slop)) return false;
    float dist = sqrtf(dist2);
    vector3 n = (dist > 1e-6f) ? vector3_scaling(d, 1.0f / dist) : (vector3){0, 1, 0};
    o->object_a = cap;
    o->object_b = other;
    o->normal_vector = n;
    o->contact_count = 1;
    o->contacts[0].position = vector3_addition(cap->position, vector3_scaling(n, cap->radius));
    o->contacts[0].penetration = rr - dist;
    o->contacts[0].local_position_a = vector3_scaling(n, cap->radius);
    o->contacts[0].local_position_b = vector3_scaling(n, -(other->type == object_sphere ? other->radius : 0.0f));
    return true;
}

static bool capsule_vs_other(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    (void)w; return capsule_contact(a, b, out);
}
static bool capsule_vs_capsule(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    (void)w; return capsule_contact(a, b, out);
}

__attribute__((constructor)) static void capsule_register(void) {
    mpe_register_pair_handler(3, 0, 100, -1, capsule_vs_other, "capsule-sphere");
    mpe_register_pair_handler(3, 1, 100, -1, capsule_vs_other, "capsule-cube");
    mpe_register_pair_handler(3, 2, 100, -1, capsule_vs_other, "capsule-cylinder");
    mpe_register_pair_handler(3, 3, 100, 100, capsule_vs_capsule, "capsule-capsule");
}

const mpe_module_desc_t mpe_module_desc = {
    .abi = MPE_MODULE_ABI,
    .name = "capsule-shape",
    .version = "1.0",
    .kind = "shape",
    .deterministic = true,
    .attach = 0,
    .detach = 0,
    .pre_step = 0,
    .post_step = 0,
};
