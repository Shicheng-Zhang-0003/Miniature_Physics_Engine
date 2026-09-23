/* Foreign capsule shape (custom id 100): segment + radius.
 *
 * Fully self-contained (no engine collision symbols) so the .so dlopens
 * anywhere. Segment = cached_axes[0] * [-h, +h] with h =
 * cylinder_half_length when positive, else a degenerate point (sphere).
 * Slop comes from the owning world's config, never hardcoded.
 * Only IEEE-exact ops (sqrtf/isfinite) — deterministic=true holds.
 *
 * Contact convention (matches engine narrowphase): normal A->B,
 * slop-band negatives clamped to zero penetration (friction-only),
 * degenerate inputs return false (never phantom contacts).
 * Rationale for approximations is documented per branch below.
 * Build: make plugins/mpe_capsule.so ; load via `mod load` or mpe_loader_load.
 */
#include "../core/mpe_module.h"
#include "../core/mpe_registry.h"
#include "../core/rigidbody.h"
#include "../core/physics_world.h"
#include "../physics/collision_mechanics.h"
#include <math.h>

#define CAP_EPS 1e-4f /* degenerate epsilon, aligned with engine narrowphase */

/* Capsule frame + the BOUNDING INVARIANT: the engine owns radius as the
 * bounding radius (broadphase-critical; sanitize preserves it for custom
 * bodies), so the cross-section is DERIVED as rc = sqrt(R^2 - h^2), never
 * read directly. Bodies must satisfy h <= R (segment inside the bounding
 * sphere, or broadphase misses pairs); overshoot clamps to a thin needle
 * rather than going imaginary. h = 0 degenerates exactly to a sphere. */
static int capsule_frame(const rigidbody *cap, vector3 *axle_out, float *half_out, float *rad_out) {
    if (!cap || !axle_out || !half_out || !rad_out) return -1;
    if (!isfinite(cap->position.x) || !isfinite(cap->position.y) || !isfinite(cap->position.z)) return -1;
    if (!isfinite(cap->radius) || cap->radius <= 0.0f) return -1;
    vector3 ax = cap->cached_axes[0];
    if (!isfinite(ax.x) || !isfinite(ax.y) || !isfinite(ax.z)) return -1;
    float len2 = ax.x * ax.x + ax.y * ax.y + ax.z * ax.z;
    if (!(len2 > 1e-12f) || !isfinite(len2)) return -1;
    float inv = 1.0f / sqrtf(len2);
    axle_out->x = ax.x * inv;
    axle_out->y = ax.y * inv;
    axle_out->z = ax.z * inv;
    float R = cap->radius;
    float h = (isfinite(cap->cylinder_half_length) && cap->cylinder_half_length > 0.0f)
                  ? cap->cylinder_half_length
                  : 0.0f;
    if (h > R) h = R; /* malformed body: clamp, never imaginary */
    float rc2 = R * R - h * h;
    if (!(rc2 > 1e-12f)) rc2 = 1e-12f;
    *half_out = h;
    *rad_out = sqrtf(rc2);
    return 0;
}

static float capsule_slop(mpe_world_t *w) {
    const mpe_config_t *C = (w && w->cfg) ? w->cfg : &g_cfg;
    float slop = C->solver.penetration_slop;
    if (!(slop >= 0.0f) || !isfinite(slop)) slop = 0.01f;
    return slop;
}

/* Closest point on segment [c-h*ax, c+h*ax] to p. */
static vector3 seg_closest(vector3 c, vector3 ax, float h, vector3 p) {
    vector3 d = vector3_subtraction(p, c);
    float s = vector3_dot(d, ax);
    if (s < -h) s = -h;
    else if (s > h) s = h;
    return vector3_addition(c, vector3_scaling(ax, s));
}

static bool emit_contact(void *out, rigidbody *a, rigidbody *b, vector3 nrm, vector3 pos, float pen) {
    collision_data *o = (collision_data *)out;
    *o = (collision_data){0};
    o->object_a = a;
    o->object_b = b;
    o->normal_vector = nrm;
    o->contact_count = 1;
    o->contacts[0].position = pos;
    o->contacts[0].penetration = (pen > 0.0f) ? pen : 0.0f;
    /* Lever-arm locals are rebuilt by collision_prepare_solver from
     * position; still write sane values for direct consumers. */
    o->contacts[0].local_position_a = vector3_subtraction(pos, a->position);
    o->contacts[0].local_position_b = vector3_subtraction(pos, b->position);
    return true;
}

/* Exact segment-vs-sphere: closest segment point to the sphere centre. */
static bool capsule_vs_sphere_exact(rigidbody *cap, vector3 ax, float h, float r, rigidbody *sph,
                                    void *out, float slop) {
    if (!isfinite(sph->position.x) || !isfinite(sph->position.y) || !isfinite(sph->position.z)) {
        return false;
    }
    if (!isfinite(sph->radius) || sph->radius <= 0.0f) return false;
    vector3 q = seg_closest(cap->position, ax, h, sph->position);
    vector3 diff = vector3_subtraction(sph->position, q);
    float dist2 = vector3_length_squared(diff);
    if (!isfinite(dist2)) return false;
    float rr = r + sph->radius;
    if (dist2 >= (rr + slop) * (rr + slop)) return false;
    float dist = sqrtf(dist2);
    vector3 nrm;
    vector3 pos;
    if (dist > CAP_EPS) {
        nrm = vector3_scaling(diff, 1.0f / dist);
        pos = vector3_addition(q, vector3_scaling(nrm, r));
    } else {
        /* Centre on the segment: pick a perpendicular (same policy as the
         * engine's coincident-centre fallbacks, but face-derived, never -Y). */
        vector3 ref =
            (fabsf(ax.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f} : (vector3){1.0f, 0.0f, 0.0f};
        nrm = vector3_cross(ax, ref);
        float l2 = vector3_length_squared(nrm);
        if (!(l2 > 1e-12f)) return false;
        nrm = vector3_scaling(nrm, 1.0f / sqrtf(l2));
        pos = vector3_addition(q, vector3_scaling(nrm, r));
    }
    return emit_contact(out, cap, sph, nrm, pos, rr - dist);
}

/* Segment-vs-OBB: sample the segment (endpoints always; interior samples
 * scale with length) as spheres vs the box, keep the deepest. Exact for
 * h=0; for h>0 the sampling error is bounded by (seg_len/(n-1))^2/8r. */
static bool capsule_vs_cube_sampled(rigidbody *cap, vector3 ax, float h, float r, rigidbody *cube,
                                    void *out, float slop) {
    if (!isfinite(cube->half_extensions.x) || !isfinite(cube->half_extensions.y) ||
        !isfinite(cube->half_extensions.z)) {
        return false;
    }
    int n = (h <= 0.0f) ? 1 : (2 * (int)(h / (r > 0.0f ? r : 1.0f)) + 3);
    if (n < 1) n = 1;
    if (n > 9) n = 9;
    float best_pen = -1e30f;
    vector3 best_n = {0, 1, 0};
    vector3 best_p = cap->position;
    int hit = 0;
    for (int i = 0; i < n; i++) {
        float s = (n == 1) ? 0.0f : (-h + 2.0f * h * (float)i / (float)(n - 1));
        vector3 c = vector3_addition(cap->position, vector3_scaling(ax, s));
        /* Sphere-vs-OBB closest point (engine narrowphase parity). */
        vector3 rel = vector3_subtraction(c, cube->position);
        vector3 closest = cube->position;
        for (int a3 = 0; a3 < 3; a3++) {
            vector3 axis = cube->cached_axes[a3];
            if (!isfinite(axis.x) || !isfinite(axis.y) || !isfinite(axis.z)) return false;
            float e = (a3 == 0)   ? cube->half_extensions.x
                      : (a3 == 1) ? cube->half_extensions.y
                                  : cube->half_extensions.z;
            float d = vector3_dot(rel, axis);
            if (d > e) d = e;
            else if (d < -e) d = -e;
            closest = vector3_addition(closest, vector3_scaling(axis, d));
        }
        vector3 diff = vector3_subtraction(c, closest);
        float dist2 = vector3_length_squared(diff);
        if (!isfinite(dist2)) continue;
        float rr = r;
        /* other radius 0: point sample vs box surface */
        if (dist2 >= (rr + slop) * (rr + slop)) continue;
        float dist = sqrtf(dist2);
        vector3 nrm;
        if (dist > CAP_EPS) {
            nrm = vector3_scaling(diff, 1.0f / dist);
        } else {
            /* Sample centre inside the box: escape along least-penetration
             * face (unnormalized rel projected on axes, min clearance). */
            float best_clear = 1e30f;
            vector3 best_ax = {0, 1, 0};
            float best_sign = 1.0f;
            for (int a3 = 0; a3 < 3; a3++) {
                vector3 axis = cube->cached_axes[a3];
                float e = (a3 == 0)   ? cube->half_extensions.x
                          : (a3 == 1) ? cube->half_extensions.y
                                      : cube->half_extensions.z;
                float d = vector3_dot(rel, axis);
                float clear = e - fabsf(d);
                if (clear < best_clear) {
                    best_clear = clear;
                    best_ax = axis;
                    best_sign = (d >= 0.0f) ? 1.0f : -1.0f;
                }
            }
            nrm = vector3_scaling(best_ax, best_sign);
        }
        float pen = rr - dist;
        if (!hit || pen > best_pen) {
            best_pen = pen;
            best_n = nrm;
            best_p = vector3_addition(closest, vector3_scaling(nrm, 0.0f));
            /* contact on the capsule surface toward the box */
            best_p = vector3_addition(c, vector3_scaling(nrm, -r));
            hit = 1;
        }
    }
    if (!hit) return false;
    return emit_contact(out, cap, cube, best_n, best_p, best_pen);
}

/* Point-vs-solid-cylinder SDF for one segment sample; keep deepest over
 * the same sampling as the cube path. Inside picks min-clearance escape. */
static bool capsule_vs_cylinder_sampled(rigidbody *cap, vector3 ax, float h, float r, rigidbody *cyl,
                                        void *out, float slop) {
    if (!isfinite(cyl->radius) || cyl->radius <= 0.0f) return false;
    if (!isfinite(cyl->cylinder_half_length) || cyl->cylinder_half_length <= 0.0f) return false;
    vector3 cax = cyl->cached_axes[0];
    float l2 = vector3_length_squared(cax);
    if (!(l2 > 1e-12f) || !isfinite(l2)) return false;
    cax = vector3_scaling(cax, 1.0f / sqrtf(l2));
    float cr = cyl->radius, ch = cyl->cylinder_half_length;
    int n = (h <= 0.0f) ? 1 : (2 * (int)(h / (r > 0.0f ? r : 1.0f)) + 3);
    if (n < 1) n = 1;
    if (n > 9) n = 9;
    float best_pen = -1e30f;
    vector3 best_n = {0, 1, 0};
    vector3 best_p = cap->position;
    int hit = 0;
    for (int i = 0; i < n; i++) {
        float s = (n == 1) ? 0.0f : (-h + 2.0f * h * (float)i / (float)(n - 1));
        vector3 c = vector3_addition(cap->position, vector3_scaling(ax, s));
        vector3 d = vector3_subtraction(c, cyl->position);
        float xd = vector3_dot(d, cax);
        vector3 radv = vector3_subtraction(d, vector3_scaling(cax, xd));
        float rl = vector3_length(radv);
        bool inside = (fabsf(xd) <= ch) && (rl <= cr);
        vector3 nrm;
        vector3 surf;
        float gap; /* centre-to-surface distance minus capsule radius */
        if (!inside) {
            float cxd = xd;
            if (cxd > ch) cxd = ch;
            else if (cxd < -ch) cxd = -ch;
            vector3 rdir = {0.0f, 1.0f, 0.0f};
            if (rl > 1e-9f) {
                rdir = vector3_scaling(radv, 1.0f / rl);
            }
            if (rl <= 1e-9f) {
                vector3 ref = (fabsf(cax.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f}
                                                     : (vector3){1.0f, 0.0f, 0.0f};
                rdir = vector3_normalisation(
                    vector3_subtraction(ref, vector3_scaling(cax, vector3_dot(ref, cax))));
            }
            vector3 axle_pt = vector3_addition(cyl->position, vector3_scaling(cax, cxd));
            if (fabsf(xd) <= ch) {
                surf = vector3_addition(axle_pt, vector3_scaling(rdir, cr));
            } else if (rl <= cr) {
                surf = vector3_addition(axle_pt, radv);
            } else {
                vector3 cap_c = vector3_addition(cyl->position, vector3_scaling(cax, cxd));
                surf = vector3_addition(cap_c, vector3_scaling(rdir, cr));
            }
            vector3 diff = vector3_subtraction(c, surf);
            float dist = vector3_length(diff);
            if (!isfinite(dist)) continue;
            gap = dist - r;
            if (gap >= slop) continue;
            nrm = (dist > CAP_EPS) ? vector3_scaling(diff, 1.0f / dist) : rdir;
        } else {
            float axial_clear = ch - fabsf(xd);
            float radial_clear = cr - rl;
            vector3 outw;
            if (axial_clear < radial_clear) {
                float sg = (xd >= 0.0f) ? 1.0f : -1.0f;
                outw = vector3_scaling(cax, sg);
                surf = vector3_addition(cyl->position, vector3_scaling(cax, sg * ch));
                surf = vector3_addition(surf, radv);
            } else {
                outw = radv;
                if (rl <= 1e-9f) {
                    vector3 ref = (fabsf(cax.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f}
                                                         : (vector3){1.0f, 0.0f, 0.0f};
                    outw = vector3_normalisation(
                        vector3_subtraction(ref, vector3_scaling(cax, vector3_dot(ref, cax))));
                } else {
                    outw = vector3_scaling(radv, 1.0f / rl);
                }
                vector3 axle_pt = vector3_addition(cyl->position, vector3_scaling(cax, xd));
                surf = vector3_addition(axle_pt, vector3_scaling(outw, cr));
            }
            nrm = vector3_scaling(outw, -1.0f); /* A->B convention */
            float min_clear = (axial_clear < radial_clear) ? axial_clear : radial_clear;
            gap = -min_clear - r;
            if (gap >= slop) continue;
        }
        float pen = -gap;
        if (!hit || pen > best_pen) {
            best_pen = pen;
            best_n = nrm;
            best_p = surf;
            hit = 1;
        }
    }
    if (!hit) return false;
    return emit_contact(out, cap, cyl, best_n, best_p, best_pen);
}

static bool capsule_contact(rigidbody *cap, rigidbody *other, void *out, mpe_world_t *w) {
    if (!cap || !other || !out) return false;
    vector3 ax;
    float h, r;
    if (capsule_frame(cap, &ax, &h, &r) != 0) return false;
    float slop = capsule_slop(w);
    if (other->type == object_sphere) {
        return capsule_vs_sphere_exact(cap, ax, h, r, other, out, slop);
    }
    if (other->type == object_cube) {
        return capsule_vs_cube_sampled(cap, ax, h, r, other, out, slop);
    }
    if (other->type == object_cylinder) {
        return capsule_vs_cylinder_sampled(cap, ax, h, r, other, out, slop);
    }
    /* Foreign-vs-foreign: bounding-sphere fallback (documented; exact
     * segment/segment sweep is out of scope for the example plugin). */
    if (other->type == object_custom) {
        float orad = (isfinite(other->radius) && other->radius > 0.0f) ? other->radius : 0.5f;
        vector3 q = seg_closest(cap->position, ax, h, other->position);
        vector3 diff = vector3_subtraction(other->position, q);
        float dist2 = vector3_length_squared(diff);
        if (!isfinite(dist2)) return false;
        float rr = r + orad;
        if (dist2 >= (rr + slop) * (rr + slop)) return false;
        float dist = sqrtf(dist2);
        vector3 nrm =
            (dist > CAP_EPS) ? vector3_scaling(diff, 1.0f / dist) : (vector3){0.0f, 1.0f, 0.0f};
        vector3 pos = vector3_addition(q, vector3_scaling(nrm, r));
        return emit_contact(out, cap, other, nrm, pos, rr - dist);
    }
    return false;
}

static bool capsule_vs_other(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    return capsule_contact(a, b, out, w);
}

static bool capsule_vs_capsule(rigidbody *a, rigidbody *b, void *out, mpe_world_t *w) {
    return capsule_contact(a, b, out, w);
}

__attribute__((constructor)) static void capsule_register(void) {
    mpe_register_pair_handler(3, 0, 100, -1, capsule_vs_other, "capsule-sphere");
    mpe_register_pair_handler(3, 1, 100, -1, capsule_vs_other, "capsule-cube");
    mpe_register_pair_handler(3, 2, 100, -1, capsule_vs_other, "capsule-cylinder");
    mpe_register_pair_handler(3, 3, 100, 100, capsule_vs_capsule, "capsule-capsule");
}

__attribute__((destructor)) static void capsule_unregister(void) {
    /* Self-unregister so dlclose never leaves dangling fn pointers. */
    mpe_unregister_pair_handler(capsule_vs_other);
    mpe_unregister_pair_handler(capsule_vs_capsule);
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
