/* Cylinder narrowphase (extracted from collision_mechanics.c to shrink the 2.2k-line god file).
 * Axle-segment + radius capsule model: barrel exact, flat caps hemispherical.
 */
#include "collision_cylinder.h"
#include "../core/physics_world.h"
#include "../config/mpe_config.h"
#include <math.h>
#include <stdint.h>

bool collision_static_plane_cylinder(rigidbody *cyl, float plane_y, collision_data *collision_output_data,
                                     const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if (cyl->type != object_cylinder) {
        return false;
    }

    float r = cyl->radius;
    float h = cyl->cylinder_half_length;

    if ((r <= 0.0f) || (h <= 0.0f) || (!isfinite(r)) || (!isfinite(h))) {
        return false;
    }

    vector3 axis = cyl->cached_axes[0];
    float axis_len_sq = vector3_length_squared(axis);

    if (axis_len_sq < 1e-8f) {
        axis = (vector3){1.0f, 0.0f, 0.0f};
        axis_len_sq = 1.0f;
    }

    axis = vector3_scaling(axis, 1.0f / sqrtf(axis_len_sq));

    float ay = axis.y;
    if (ay > 1.0f) {
        ay = 1.0f;
    }
    if (ay < -1.0f) {
        ay = -1.0f;
    }

    /*
     * For a cylinder against a horizontal plane:
     *
     * vertical half-extent =
     *     r * sqrt(1 - axis.y^2)
     *   + h * fabs(axis.y)
     *
     * The first term is the barrel contribution.
     * The second term is the end-cap/axle contribution.
     */
    float horizontal = sqrtf(fmaxf(0.0f, 1.0f - ay * ay));

    /*
     * Radial offset to the lowest barrel point.
     * The plane normal is up, so the downward direction is (0,-1,0).
     * Remove the component parallel to the axle.
     */
    vector3 down = {0.0f, -1.0f, 0.0f};
    float down_along_axis = vector3_dot(down, axis);
    vector3 radial = vector3_subtraction(down, vector3_scaling(axis, down_along_axis));
    float radial_len = vector3_length(radial);

    if (radial_len > 1e-6f) {
        radial = vector3_scaling(radial, r / radial_len);
    } else {
        radial = vector3_zero();
    }

    rigidbody *plane_body = collision_static_plane_body_proxy(plane_y, cfg);

    collision_output_data->object_a = cyl;
    collision_output_data->object_b = plane_body;
    collision_output_data->normal_vector = (vector3){0.0f, -1.0f, 0.0f};
    collision_output_data->contact_count = 0;

    /*
     * Near-horizontal axle:
     * generate two contacts at the axle ends for stability.
     * This is the normal FTC wheel case.
     * Slop-gated (see clip_obb_faces): wheels rolling within slop keep
     * persistent friction contacts instead of flickering support.
     */
    if (fabsf(ay) < 0.35f) {
        float axle_offsets[2] = {-h, h};
        float wheel_slop = C->solver.penetration_slop;

        for (int i = 0; i < 2; i++) {
            vector3 end_center =
                vector3_addition(cyl->position, vector3_scaling(axis, axle_offsets[i]));

            vector3 contact_point = vector3_addition(end_center, radial);
            float local_penetration = plane_y - contact_point.y;

            if ((local_penetration > -wheel_slop) && (collision_output_data->contact_count < 2)) {
                contact_point_data *cp =
                    &collision_output_data->contacts[collision_output_data->contact_count];
                cp->position = contact_point;
                cp->penetration = (local_penetration > 0.0f) ? local_penetration : 0.0f;
                collision_output_data->contact_count++;
            }
        }
    }

    /*
     * Tilted or vertical axle: cap-disc support as a RIM QUAD, never a single
     * center point. A point support has zero lever arm and supplies no
     * restoring torque: upright cylinders balanced on it twitch, spin, and
     * topple (the "erratic cylinder" instability). Four rim legs at 0/90/180/
     * 270 degrees from the downhill direction degrade gracefully: vertical
     * gives a symmetric 4-point table, tilted keeps the downhill legs that
     * truly touch (each slop-gated independently, own depths). All share the
     * plane normal, which is exact for a horizontal plane.
     */
    if (collision_output_data->contact_count == 0) {
        float axle_offset = (ay >= 0.0f) ? -h : h;
        vector3 cap_center = vector3_addition(cyl->position, vector3_scaling(axis, axle_offset));
        /* In-cap-plane downhill direction (lowest rim azimuth). */
        vector3 nhat;
        if (radial_len > 1e-6f) {
            nhat = vector3_scaling(radial, 1.0f / radial_len);
        } else {
            /* Perfectly vertical: arbitrary in-plane reference (quad stays
             * symmetric regardless of azimuth). */
            vector3 ref = (fabsf(axis.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f} : (vector3){1.0f, 0.0f, 0.0f};
            nhat = vector3_normalisation(
                vector3_subtraction(ref, vector3_scaling(axis, vector3_dot(ref, axis))));
        }
        vector3 what = vector3_cross(axis, nhat);
        if (vector3_length_squared(what) < 1e-12f) {
            what = vector3_normalisation(vector3_cross(
                axis, (fabsf(axis.x) < 0.99f) ? (vector3){1.0f, 0.0f, 0.0f} : (vector3){0.0f, 0.0f, 1.0f}));
        } else {
            what = vector3_normalisation(what);
        }
        float rim_slop = C->solver.penetration_slop;
        const float leg_angles[4] = {0.0f, 1.5707963f, 3.1415927f, 4.7123890f};
        for (int leg = 0; leg < 4; leg++) {
            float c = cosf(leg_angles[leg]);
            float s = sinf(leg_angles[leg]);
            vector3 rim_point = vector3_addition(
                cap_center, vector3_scaling(vector3_addition(vector3_scaling(nhat, c), vector3_scaling(what, s)),
                                            r));
            float leg_pen = plane_y - rim_point.y;
            if ((leg_pen > -rim_slop) && (collision_output_data->contact_count < 4)) {
                contact_point_data *cp = &collision_output_data->contacts[collision_output_data->contact_count];
                cp->position = rim_point;
                cp->penetration = (leg_pen > 0.0f) ? leg_pen : 0.0f;
                collision_output_data->contact_count++;
            }
        }
        /* Analytical safety net: if numerical error admits nothing (e.g.
         * degenerate basis), fall back to the exact lowest height so a
         * deeply penetrating cap is never missed. */
        if (collision_output_data->contact_count == 0) {
            float lowest_offset = (r * horizontal) + (h * fabsf(ay));
            float leg_pen = plane_y - (cyl->position.y - lowest_offset);
            if (leg_pen > -rim_slop) {
                contact_point_data *cp = &collision_output_data->contacts[0];
                cp->position = vector3_addition(cap_center, vector3_scaling(nhat, r));
                cp->penetration = (leg_pen > 0.0f) ? leg_pen : 0.0f;
                collision_output_data->contact_count = 1;
            }
        }
    }

    return collision_output_data->contact_count > 0;
}
/* ================================================================
 * MFS_172: Cylinder-vs-object narrowphase
 * ================================================================ */

/* Cylinder vs Sphere — TRUE solid cylinder SDF (flat caps, exact).
 * TRUTH P1-4: replaces capsule (segment+radius) which bulged hemispheres
 * r beyond flat faces and cut corners. Local frame: axle = X, |x|<=h,
 * |yz|<=r. Closest surface point classified barrel/cap/rim/inside;
 * gap = |p_s - closest| - r_s, slop-gated like other paths. */
bool collision_cylinder_sphere(rigidbody *cyl, rigidbody *sph,
                               collision_data *out, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((cyl->type != object_cylinder) || (sph->type != object_sphere)) {
        return false;
    }
    vector3 axis = cyl->cached_axes[0];
    float axis_len_sq = vector3_length_squared(axis);
    if (axis_len_sq < 1e-8f) {
        axis = (vector3){1.0f, 0.0f, 0.0f};
    } else {
        axis = vector3_scaling(axis, 1.0f / sqrtf(axis_len_sq));
    }
    float r_c = cyl->radius;
    float h = cyl->cylinder_half_length;
    float r_s = sph->radius;
    if ((r_c <= 0.0f) || (h <= 0.0f) || (r_s <= 0.0f) ||
        (!isfinite(r_c)) || (!isfinite(h)) || (!isfinite(r_s))) {
        return false;
    }

    vector3 d = vector3_subtraction(sph->position, cyl->position);
    float x = vector3_dot(d, axis);
    vector3 radial_vec = vector3_subtraction(d, vector3_scaling(axis, x));
    float radial_len = vector3_length(radial_vec);
    vector3 radial_dir = (radial_len > 1e-9f)
        ? vector3_scaling(radial_vec, 1.0f / radial_len)
        : (vector3){0.0f, 1.0f, 0.0f};

    vector3 closest;
    vector3 nrm;
    float center_dist; /* |p_s - closest|, center to cylinder surface */
    bool inside = (fabsf(x) <= h) && (radial_len <= r_c);
    if (inside) {
        /* Push out the nearest face: cap vs barrel. */
        float axial_clear = h - fabsf(x);
        float radial_clear = r_c - radial_len;
        if (axial_clear < radial_clear) {
            float s = (x >= 0.0f) ? 1.0f : -1.0f;
            nrm = vector3_scaling(axis, s); /* cyl -> sphere side */
            closest = vector3_addition(cyl->position, vector3_scaling(axis, s * h));
            closest = vector3_addition(closest, radial_vec); /* keep radial offset on cap disc */
        } else {
            nrm = radial_dir;
            if (radial_len <= 1e-9f) {
                /* Center on axle: pick any perpendicular. */
                vector3 up = (fabsf(axis.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f}
                                                    : (vector3){1.0f, 0.0f, 0.0f};
                nrm = vector3_normalisation(vector3_subtraction(
                    up, vector3_scaling(axis, vector3_dot(up, axis))));
            }
            vector3 axle_pt = vector3_addition(cyl->position, vector3_scaling(axis, x));
            closest = vector3_addition(axle_pt, vector3_scaling(nrm, r_c));
        }
        center_dist = 0.0f; /* center inside: gap = -min_clear below */
        float min_clear = (axial_clear < radial_clear) ? axial_clear : radial_clear;
        float gap = -min_clear - r_s;
        float slop = C->solver.penetration_slop;
        if (gap <= -slop) {
            /* Deep: still a contact (discrete owns it). */
        }
        if (gap >= slop) {
            return false;
        }
        out->object_a = cyl;
        out->object_b = sph;
        out->normal_vector = nrm;
        out->contact_count = 1;
        contact_point_data *cp_in = &out->contacts[0];
        cp_in->penetration = (-gap > 0.0f) ? -gap : 0.0f;
        cp_in->position = closest;
        return true;
    }
    if (fabsf(x) <= h) {
        /* Barrel side (exact for both capsule and true cylinder). */
        vector3 axle_pt = vector3_addition(cyl->position, vector3_scaling(axis, x));
        closest = vector3_addition(axle_pt, vector3_scaling(radial_dir, r_c));
    } else if (radial_len <= r_c) {
        /* Flat cap disc (capsule was WRONG here: hemisphere bulge). */
        float s = (x >= 0.0f) ? 1.0f : -1.0f;
        vector3 cap_center = vector3_addition(cyl->position, vector3_scaling(axis, s * h));
        closest = vector3_addition(cap_center, radial_vec);
    } else {
        /* Rim circle: cap edge point. */
        float s = (x >= 0.0f) ? 1.0f : -1.0f;
        vector3 cap_center = vector3_addition(cyl->position, vector3_scaling(axis, s * h));
        closest = vector3_addition(cap_center, vector3_scaling(radial_dir, r_c));
    }
    vector3 diff = vector3_subtraction(sph->position, closest);
    center_dist = vector3_length(diff);
    float gap2 = center_dist - r_s;
    float slop2 = C->solver.penetration_slop;
    if (gap2 >= slop2) {
        return false;
    }
    out->object_a = cyl;
    out->object_b = sph;
    out->contact_count = 1;
    if (center_dist > 0.0001f) {
        out->normal_vector = vector3_scaling(diff, 1.0f / center_dist);
    } else {
        out->normal_vector = radial_dir;
    }
    contact_point_data *cp = &out->contacts[0];
    cp->penetration = (gap2 < 0.0f) ? -gap2 : 0.0f;
    cp->position = closest;
    return true;
}

/* Cylinder vs Cube (OBB).
 * Sample N points along the axle, find the one closest to the OBB
 * surface, then do a sphere-OBB test at that point with the
 * cylinder radius. 5 samples is enough for short axles (wheels). */

/* LIST4 NEW-02 / NEW-03:
 * Correct cylinder-vs-cube contact normal and sample resolution.
 *
 * Convention:
 *   object_a = cylinder
 *   object_b = cube
 *   normal must point from cylinder toward cube.
 *
 * The old version used:
 *   cyl->position - best_on_obb
 * which pointed from the cube surface toward the cylinder center.
 * That inverted the solver response for floor/wall contacts.
 *
 * The corrected version uses:
 *   best_on_obb - best_pt
 * where best_pt is the closest sampled point on the cylinder axle.
 */
/* Helper: squared distance from axle-segment point p(t) to OBB (box local
 * clamp). Convex in t: ternary search converges to exact minimum. */
static float cylcube_seg_obb_dist2(rigidbody *cube, vector3 e1, vector3 seg, float t, vector3 *obb_out) {
    vector3 pt = vector3_addition(e1, vector3_scaling(seg, t));
    vector3 rel = vector3_subtraction(pt, cube->position);
    vector3 *axes = cube->cached_axes;
    float lx = vector3_dot(rel, axes[0]);
    float ly = vector3_dot(rel, axes[1]);
    float lz = vector3_dot(rel, axes[2]);
    float cx = fmaxf(-cube->half_extensions.x, fminf(cube->half_extensions.x, lx));
    float cy = fmaxf(-cube->half_extensions.y, fminf(cube->half_extensions.y, ly));
    float cz = fmaxf(-cube->half_extensions.z, fminf(cube->half_extensions.z, lz));
    vector3 on_obb = cube->position;
    on_obb = vector3_addition(on_obb, vector3_scaling(axes[0], cx));
    on_obb = vector3_addition(on_obb, vector3_scaling(axes[1], cy));
    on_obb = vector3_addition(on_obb, vector3_scaling(axes[2], cz));
    if (obb_out) {
        *obb_out = on_obb;
    }
    return vector3_length_squared(vector3_subtraction(pt, on_obb));
}

bool collision_cylinder_cube(rigidbody *cyl, rigidbody *cube,
                             collision_data *out, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((cyl->type != object_cylinder) || (cube->type != object_cube)) {
        return false;
    }

    float r = cyl->radius;
    float h = cyl->cylinder_half_length;

    if ((r <= 0.0f) || (h <= 0.0f) || (!isfinite(r)) || (!isfinite(h))) {
        return false;
    }

    vector3 axis = cyl->cached_axes[0];
    float axis_len_sq = vector3_length_squared(axis);

    if (axis_len_sq < 1e-8f) {
        axis = (vector3){1.0f, 0.0f, 0.0f};
        axis_len_sq = 1.0f;
    }

    axis = vector3_scaling(axis, 1.0f / sqrtf(axis_len_sq));

    vector3 e1 = vector3_subtraction(cyl->position, vector3_scaling(axis, h));
    vector3 seg = vector3_scaling(axis, 2.0f * h);

    /* TRUTH P1-5: EXACT segment-OBB closest via convex ternary search
     * (replaces 9-sample polling which missed between samples for long
     * rods and shared one normal across faces). dist²(t) is convex;
     * 28 iterations pin t* to ~1e-6 (0.02mm on a 10m axle, 500x below
     * slop). Deterministic, no RNG. */
    float lo = 0.0f, hi = 1.0f;
    for (int it = 0; it < 28; it++) {
        float m1 = lo + (hi - lo) / 3.0f;
        float m2 = hi - (hi - lo) / 3.0f;
        float d1 = cylcube_seg_obb_dist2(cube, e1, seg, m1, NULL);
        float d2 = cylcube_seg_obb_dist2(cube, e1, seg, m2, NULL);
        if (d1 < d2) {
            hi = m2;
        } else {
            lo = m1;
        }
    }
    float t_star = 0.5f * (lo + hi);
    vector3 obb_star;
    float d2_star = cylcube_seg_obb_dist2(cube, e1, seg, t_star, &obb_star);
    float d_star = sqrtf(d2_star);
    float slop = C->solver.penetration_slop;
    if (d_star >= r + slop) {
        return false;
    }
    vector3 pt_star = vector3_addition(e1, vector3_scaling(seg, t_star));

    out->object_a = cyl;
    out->object_b = cube;
    out->contact_count = 0;

    /* Primary contact: exact normal per point (A->B). Inside-segment
     * degenerate (pt inside box): escape along box-local minimum-penetration
     * axis (true OBB SDF), not toward center. */
    vector3 toward = vector3_subtraction(obb_star, pt_star);
    float toward_len = vector3_length(toward);
    vector3 n0;
    if (toward_len > 0.0001f) {
        n0 = vector3_scaling(toward, 1.0f / toward_len);
    } else {
        vector3 rel0 = vector3_subtraction(pt_star, cube->position);
        vector3 *axes0 = cube->cached_axes;
        float l0x = vector3_dot(rel0, axes0[0]);
        float l0y = vector3_dot(rel0, axes0[1]);
        float l0z = vector3_dot(rel0, axes0[2]);
        float px = cube->half_extensions.x - fabsf(l0x);
        float py = cube->half_extensions.y - fabsf(l0y);
        float pz = cube->half_extensions.z - fabsf(l0z);
        if ((px < py) && (px < pz)) {
            n0 = vector3_scaling(axes0[0], (l0x >= 0.0f) ? 1.0f : -1.0f);
        } else if (py < pz) {
            n0 = vector3_scaling(axes0[1], (l0y >= 0.0f) ? 1.0f : -1.0f);
        } else {
            n0 = vector3_scaling(axes0[2], (l0z >= 0.0f) ? 1.0f : -1.0f);
        }
    }
    out->normal_vector = n0;
    {
        contact_point_data *cp = &out->contacts[out->contact_count++];
        float pen;
        if (toward_len > 0.0001f) {
            pen = r - d_star;
        } else {
            /* TRUTH: axle point inside box. d_star==0, but true capsule
             * depth is r + face_clearance (distance to nearest face), not r.
             * Old r-only underestimated by up to half the box. */
            vector3 rel0b = vector3_subtraction(pt_star, cube->position);
            vector3 *axes0b = cube->cached_axes;
            float l0xb = vector3_dot(rel0b, axes0b[0]);
            float l0yb = vector3_dot(rel0b, axes0b[1]);
            float l0zb = vector3_dot(rel0b, axes0b[2]);
            float pxb = cube->half_extensions.x - fabsf(l0xb);
            float pyb = cube->half_extensions.y - fabsf(l0yb);
            float pzb = cube->half_extensions.z - fabsf(l0zb);
            float clearance = pxb;
            if (pyb < clearance) {
                clearance = pyb;
            }
            if (pzb < clearance) {
                clearance = pzb;
            }
            if (!isfinite(clearance) || clearance < 0.0f) {
                clearance = 0.0f;
            }
            pen = r + clearance;
        }
        cp->penetration = (pen > 0.0f) ? pen : 0.0f;
        /* TRUTH: interior contact position must be on the box surface
         * (face point), not the interior axle point, or lever arms are wrong.
         * obb_star==pt_star interior; push to face along n0. */
        if (toward_len <= 0.0001f) {
            vector3 rel0c = vector3_subtraction(pt_star, cube->position);
            vector3 *axes0c = cube->cached_axes;
            float l0xc = vector3_dot(rel0c, axes0c[0]);
            float l0yc = vector3_dot(rel0c, axes0c[1]);
            float l0zc = vector3_dot(rel0c, axes0c[2]);
            float pxc = cube->half_extensions.x - fabsf(l0xc);
            float pyc = cube->half_extensions.y - fabsf(l0yc);
            float pzc = cube->half_extensions.z - fabsf(l0zc);
            float best = pxc;
            int bi = 0;
            if (pyc < best) {
                best = pyc;
                bi = 1;
            }
            if (pzc < best) {
                best = pzc;
                bi = 2;
            }
            float sgn = 0.0f;
            float loc = 0.0f;
            float ext = 0.0f;
            if (bi == 0) {
                sgn = (l0xc >= 0.0f) ? 1.0f : -1.0f;
                loc = l0xc;
                ext = cube->half_extensions.x;
            } else if (bi == 1) {
                sgn = (l0yc >= 0.0f) ? 1.0f : -1.0f;
                loc = l0yc;
                ext = cube->half_extensions.y;
            } else {
                sgn = (l0zc >= 0.0f) ? 1.0f : -1.0f;
                loc = l0zc;
                ext = cube->half_extensions.z;
            }
            float push = ext - sgn * loc;
            if (isfinite(push)) {
                cp->position = vector3_addition(obb_star, vector3_scaling(n0, push));
            } else {
                cp->position = obb_star;
            }
        } else {
            cp->position = obb_star;
        }
    }

    /* Line support: cylinder lying on a face contacts along an interval,
     * not a point (single point rocks). If the axle is near-parallel to the
     * contacted face, clip the axle against the face slab (expanded by slop)
     * and add the far interval end as a second contact with its OWN normal.
     * Exact for face-parallel (the wheel-lying case); no-op otherwise. */
    {
        vector3 rel_s = vector3_subtraction(pt_star, cube->position);
        vector3 *axes = cube->cached_axes;
        float lx = vector3_dot(rel_s, axes[0]);
        float ly = vector3_dot(rel_s, axes[1]);
        float lz = vector3_dot(rel_s, axes[2]);
        float px = cube->half_extensions.x - fabsf(lx);
        float py = cube->half_extensions.y - fabsf(ly);
        float pz = cube->half_extensions.z - fabsf(lz);
        int face = 0;
        float face_dist = px;
        vector3 face_n = vector3_scaling(axes[0], (lx >= 0.0f) ? 1.0f : -1.0f);
        if (py < face_dist) {
            face_dist = py;
            face = 1;
            face_n = vector3_scaling(axes[1], (ly >= 0.0f) ? 1.0f : -1.0f);
        }
        if (pz < face_dist) {
            face_dist = pz;
            face = 2;
            face_n = vector3_scaling(axes[2], (lz >= 0.0f) ? 1.0f : -1.0f);
        }
        float axis_face = fabsf(vector3_dot(axis, face_n));
        if (axis_face < 0.1f) {
            /* Axle parallel to face: walk both directions from t* to the
             * interval ends where dist exceeds r+slop (bisection, 16 iters
             * each, exact to 1e-5 of axle length — 0.1mm on 10m, 100x
             * below slop). */
            float ends[2] = {t_star, t_star};
            for (int dir = 0; dir < 2; dir++) {
                float a = t_star, b = (dir == 0) ? 0.0f : 1.0f;
                /* If endpoint already within band, it IS the end. */
                vector3 obb_e;
                float de = sqrtf(cylcube_seg_obb_dist2(cube, e1, seg, b, &obb_e));
                if (de < r + slop) {
                    ends[dir] = b;
                    continue;
                }
                for (int it = 0; it < 16; it++) {
                    float mid = 0.5f * (a + b);
                    float dm = sqrtf(cylcube_seg_obb_dist2(cube, e1, seg, mid, NULL));
                    if (dm < r + slop) {
                        a = mid;
                    } else {
                        b = mid;
                    }
                }
                ends[dir] = a;
            }
            /* Add the farther end if separated from primary along axle. */
            float t_far = (fabsf(ends[1] - t_star) > fabsf(ends[0] - t_star)) ? ends[1] : ends[0];
            vector3 pt_far = vector3_addition(e1, vector3_scaling(seg, t_far));
            vector3 obb_far;
            float d_far = sqrtf(cylcube_seg_obb_dist2(cube, e1, seg, t_far, &obb_far));
            float axle_sep = fabsf(t_far - t_star) * 2.0f * h;
            if ((d_far < r + slop) && (axle_sep > 0.05f) && (out->contact_count < 4)) {
                vector3 toward_f = vector3_subtraction(obb_far, pt_far);
                float len_f = vector3_length(toward_f);
                /* Per-contact normal (TRUTH: never share primary's). */
                if (len_f > 0.0001f) {
                    vector3 nf = vector3_scaling(toward_f, 1.0f / len_f);
                    /* Only add if roughly same face (dot>0.7): wrap-around
                     * edges get discrete treatment next tick, not forced. */
                    if (vector3_dot(nf, n0) > 0.7f) {
                        contact_point_data *cp2 = &out->contacts[out->contact_count++];
                        float pen2 = r - d_far;
                        cp2->penetration = (pen2 > 0.0f) ? pen2 : 0.0f;
                        cp2->position = obb_far;
                    }
                }
            }
            (void) face;
        } else if ((axis_face > 0.95f) && (out->contact_count < 4)) {
            /* Cap disc standing on the face: rim-ring support. Without it the
             * cap balances on the single primary point (zero lever arm —
             * same twitch/topple disease as the floor point-support). Only
             * valid face-parallel (shared normal n0); tilted caps keep the
             * primary point contact. Rim depths come from the exact OBB SDF
             * per rim point (inside → face clearance, outside → slop-band
             * support only). */
            vector3 e2 = vector3_addition(e1, seg);
            float d0 = vector3_length_squared(vector3_subtraction(e1, obb_star));
            float d1 = vector3_length_squared(vector3_subtraction(e2, obb_star));
            vector3 cap_center = (d0 < d1) ? e1 : e2;
            vector3 ref =
                (fabsf(axis.x) < 0.9f) ? (vector3){1.0f, 0.0f, 0.0f} : (vector3){0.0f, 1.0f, 0.0f};
            vector3 u = vector3_normalisation(
                vector3_subtraction(ref, vector3_scaling(axis, vector3_dot(ref, axis))));
            vector3 v = vector3_normalisation(vector3_cross(axis, u));
            vector3 *baxes = cube->cached_axes;
            const float rim_angles[3] = {0.0f, 2.0943951f, 4.1887902f};
            for (int leg = 0; leg < 3; leg++) {
                if (out->contact_count >= 4) {
                    break;
                }
                vector3 rim = vector3_addition(
                    cap_center,
                    vector3_scaling(vector3_addition(vector3_scaling(u, cosf(rim_angles[leg])),
                                                    vector3_scaling(v, sinf(rim_angles[leg]))),
                                    r));
                vector3 rrel = vector3_subtraction(rim, cube->position);
                float lx = vector3_dot(rrel, baxes[0]);
                float ly = vector3_dot(rrel, baxes[1]);
                float lz = vector3_dot(rrel, baxes[2]);
                float ex = cube->half_extensions.x, ey = cube->half_extensions.y,
                      ez = cube->half_extensions.z;
                float rim_pen;
                vector3 rim_pos;
                if ((fabsf(lx) <= ex) && (fabsf(ly) <= ey) && (fabsf(lz) <= ez)) {
                    /* Inside: penetration = minimum face clearance. */
                    float cx = ex - fabsf(lx), cy = ey - fabsf(ly), cz = ez - fabsf(lz);
                    if ((cx < cy) && (cx < cz)) {
                        rim_pen = cx;
                        rim_pos = vector3_addition(
                            cube->position,
                            vector3_addition(vector3_scaling(baxes[1], ly),
                                             vector3_addition(vector3_scaling(baxes[2], lz),
                                                              vector3_scaling(baxes[0], (lx >= 0.0f) ? ex : -ex))));
                    } else if (cy < cz) {
                        rim_pen = cy;
                        rim_pos = vector3_addition(
                            cube->position,
                            vector3_addition(vector3_scaling(baxes[0], lx),
                                             vector3_addition(vector3_scaling(baxes[2], lz),
                                                              vector3_scaling(baxes[1], (ly >= 0.0f) ? ey : -ey))));
                    } else {
                        rim_pen = cz;
                        rim_pos = vector3_addition(
                            cube->position,
                            vector3_addition(vector3_scaling(baxes[0], lx),
                                             vector3_addition(vector3_scaling(baxes[1], ly),
                                                              vector3_scaling(baxes[2], (lz >= 0.0f) ? ez : -ez))));
                    }
                } else {
                    /* Outside: slop-band support only (primary owns depth). */
                    float cx = fmaxf(-ex, fminf(ex, lx));
                    float cy = fmaxf(-ey, fminf(ey, ly));
                    float cz = fmaxf(-ez, fminf(ez, lz));
                    rim_pos = vector3_addition(
                        cube->position, vector3_addition(vector3_scaling(baxes[0], cx),
                                                         vector3_addition(vector3_scaling(baxes[1], cy),
                                                                          vector3_scaling(baxes[2], cz))));
                    float gap = vector3_length(vector3_subtraction(rim, rim_pos));
                    if (gap >= slop) {
                        continue;
                    }
                    rim_pen = 0.0f;
                }
                contact_point_data *cpr = &out->contacts[out->contact_count++];
                cpr->position = rim_pos;
                cpr->penetration = rim_pen;
            }
        }
    }

    if (out->contact_count == 0) {
        return false;
    }

    return true;
}

/* Cylinder vs Cylinder — TRUE flat-cap handling + parallel 2-point support.
 * TRUTH P1-6: old segment+radius capsule failed coaxially (face-face gap
 * compared against r_a+r_b instead of 0) and gave single-point support for
 * parallel logs (rocks). Barrel-side uses segment closest (exact); coaxial
 * faces use axial gap; parallel sides emit 2 points at overlap ends. */
bool collision_cylinder_cylinder(rigidbody *cyl_a, rigidbody *cyl_b,
                                 collision_data *out, const mpe_config_t *cfg) {
    const mpe_config_t *C = cfg ? cfg : &g_cfg;
    if ((!cyl_a) || (!cyl_b) || (!out)) {
        return false;
    }
    if ((cyl_a->type != object_cylinder) || (cyl_b->type != object_cylinder)) {
        return false;
    }
    /* TRUTH: NaN/0 geometry must reject, like all other cyl entries. */
    if (!isfinite(cyl_a->radius) || !isfinite(cyl_a->cylinder_half_length) || !isfinite(cyl_b->radius) ||
        !isfinite(cyl_b->cylinder_half_length)) {
        return false;
    }
    if (cyl_a->radius <= 0.0f || cyl_a->cylinder_half_length <= 0.0f || cyl_b->radius <= 0.0f ||
        cyl_b->cylinder_half_length <= 0.0f) {
        return false;
    }
    vector3 ax = cyl_a->cached_axes[0];
    vector3 bx = cyl_b->cached_axes[0];
    float ax_len = vector3_length(ax);
    float bx_len = vector3_length(bx);
    if (ax_len < 1e-6f) {
        ax = (vector3){1.0f, 0.0f, 0.0f};
    } else {
        ax = vector3_scaling(ax, 1.0f / ax_len);
    }
    if (bx_len < 1e-6f) {
        bx = (vector3){1.0f, 0.0f, 0.0f};
    } else {
        bx = vector3_scaling(bx, 1.0f / bx_len);
    }
    float ha = cyl_a->cylinder_half_length;
    float hb = cyl_b->cylinder_half_length;
    float slop = C->solver.penetration_slop;

    /* Coaxial / near-coaxial face-face vs barrel-side disambiguation.
     * TRUTH: pick the SHALLOWER penetration (first touch), not just any gap.
     * Face gap = axial_gap (faces interpenetrating axially). Side gap =
     * lateral - (r_a+r_b) (barrels overlapping laterally). Shallow wins:
     * - Axial approach (lateral~0, gap -0.05): face -0.05 vs side -0.10 ->
     *   face wins (old narrow gate missed deep face, side gave (0,1,0)).
     * - Side approach (gap -0.04 const, lateral 0.08): face -0.04 vs side
     *   -0.02 -> side wins (a wide gate claiming face here pushes X while
     *   bodies pass through in Z). */
    float axis_dot = fabsf(vector3_dot(ax, bx));
    if (axis_dot > 0.95f) {
        vector3 delta = vector3_subtraction(cyl_b->position, cyl_a->position);
        float axial = vector3_dot(delta, ax);
        vector3 lateral_vec = vector3_subtraction(delta, vector3_scaling(ax, axial));
        float lateral = vector3_length(lateral_vec);
        float axial_gap = fabsf(axial) - (ha + hb);
        float side_gap = lateral - (cyl_a->radius + cyl_b->radius);
        bool face_candidate = (axial_gap < slop) && (lateral < cyl_a->radius + cyl_b->radius + slop);
        if (face_candidate && (axial_gap >= -2.0f * slop || axial_gap > side_gap)) {
            /* Faces overlap laterally and meet axially: flat-cap contact. */
            float s = (axial >= 0.0f) ? 1.0f : -1.0f;
            vector3 nrm = vector3_scaling(ax, s); /* A -> B */
            out->object_a = cyl_a;
            out->object_b = cyl_b;
            out->normal_vector = nrm;
            out->contact_count = 1;
            /* Contact at midpoint of overlap disc (clamped to smaller cap). */
            vector3 face_a = vector3_addition(cyl_a->position, vector3_scaling(ax, s * ha));
            vector3 face_b = vector3_subtraction(cyl_b->position, vector3_scaling(ax, s * hb));
            vector3 mid = vector3_scaling(vector3_addition(face_a, face_b), 0.5f);
            contact_point_data *cp0 = &out->contacts[0];
            cp0->penetration = (axial_gap < 0.0f) ? -axial_gap : 0.0f;
            cp0->position = mid;
            /* Second point for stable face support (offset toward rim). */
            float min_r = (cyl_a->radius < cyl_b->radius) ? cyl_a->radius : cyl_b->radius;
            if ((min_r > 0.05f) && (out->contact_count < 4)) {
                vector3 perp = (lateral > 1e-6f)
                    ? vector3_scaling(lateral_vec, 1.0f / lateral)
                    : vector3_normalisation(vector3_cross(
                          ax, (fabsf(ax.y) < 0.99f) ? (vector3){0.0f, 1.0f, 0.0f}
                                                   : (vector3){1.0f, 0.0f, 0.0f}));
                vector3 off = vector3_scaling(perp, min_r * 0.5f);
                contact_point_data *cp1 = &out->contacts[out->contact_count++];
                /* contact_count already 1; append second: */
                (void) cp1;
                out->contacts[1].penetration = cp0->penetration;
                out->contacts[1].position = vector3_addition(mid, off);
                out->contact_count = 2;
            }
            return true;
        }
        /* Parallel but axially separated beyond faces: fall through to side
         * test below (may still touch barrel-to-barrel laterally). */
    }

    vector3 a1 = vector3_subtraction(cyl_a->position, vector3_scaling(ax, ha));
    vector3 a2 = vector3_addition(cyl_a->position, vector3_scaling(ax, ha));
    vector3 b1 = vector3_subtraction(cyl_b->position, vector3_scaling(bx, hb));
    vector3 b2 = vector3_addition(cyl_b->position, vector3_scaling(bx, hb));

    /* segment-segment closest points (Ericson, Real-Time Collision Detection) */
    vector3 d1 = vector3_subtraction(a2, a1);
    vector3 d2 = vector3_subtraction(b2, b1);
    vector3 r  = vector3_subtraction(a1, b1);
    float a = vector3_dot(d1, d1);
    float e = vector3_dot(d2, d2);
    float f = vector3_dot(d2, r);
    float s, t;

    if ((a <= 0.000001f) && (e <= 0.000001f)) {
        s = t = 0.0f;
    } else if (a <= 0.000001f) {
        s = 0.0f;
        t = f / e;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
    } else {
        float c = vector3_dot(d1, r);
        if (e <= 0.000001f) {
            t = 0.0f;
            s = -c / a;
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
        } else {
            float b = vector3_dot(d1, d2);
            float denom = a * e - b * b;
            s = (denom > 0.000001f) ? (b * f - c * e) / denom : 0.0f;
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
            t = (b * s + f) / e;
            if (t < 0.0f) { t = 0.0f; s = -c / a; }
            if (t > 1.0f) { t = 1.0f; s = (b - c) / a; }
            if (s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;
        }
    }

    vector3 pa = vector3_addition(a1, vector3_scaling(d1, s));
    vector3 pb = vector3_addition(b1, vector3_scaling(d2, t));
    float dist = vector3_length(vector3_subtraction(pa, pb));
    float min_dist = cyl_a->radius + cyl_b->radius;
    /* TRUTH: slop-gated (was strict <). */
    if (dist >= min_dist + slop) {
        return false;
    }

    out->object_a = cyl_a;
    out->object_b = cyl_b;
    out->contact_count = 1;
    if (dist > 0.0001f) {
        out->normal_vector = vector3_scaling(
            vector3_subtraction(pb, pa), 1.0f / dist);
    } else {
        out->normal_vector = (vector3){0.0f, 1.0f, 0.0f};
    }
    contact_point_data *cp = &out->contacts[0];
    float raw_pen = min_dist - dist;
    cp->penetration = (raw_pen > 0.0f) ? raw_pen : 0.0f;
    cp->position = vector3_scaling(vector3_addition(pa, pb), 0.5f);
    contact_point_data saved_single = out->contacts[0];
    /* TRUTH P1-6: parallel barrels share a LINE, not a point. Emit two
     * contacts at the overlap interval ends (both clamped onto segments,
     * symmetric ±), so stacked logs do not rock on a single point. */
    if (axis_dot > 0.95f) {
        float overlap = fminf(ha, hb);
        if (overlap > 0.05f) {
            vector3 shared = (vector3_dot(ax, bx) >= 0.0f) ? ax : vector3_scaling(ax, -1.0f);
            /* Overlap interval along A: project B's interval onto A. */
            float s_c = vector3_dot(vector3_subtraction(cyl_b->position, cyl_a->position), shared);
            float lo = fmaxf(-ha, s_c - hb);
            float hi = fminf(ha, s_c + hb);
            if (hi > lo + 0.05f) {
                /* Two endpoints, inset by 25% to stay on the barrel. */
                float t0 = lo + (hi - lo) * 0.25f;
                float t1 = lo + (hi - lo) * 0.75f;
                vector3 pa0 = vector3_addition(cyl_a->position, vector3_scaling(shared, t0));
                vector3 pa1 = vector3_addition(cyl_a->position, vector3_scaling(shared, t1));
                /* Closest points on B's axle for each. */
                for (int k = 0; k < 2; k++) {
                    vector3 pak = (k == 0) ? pa0 : pa1;
                    float tb = vector3_dot(vector3_subtraction(pak, cyl_b->position), bx);
                    if (tb < -hb) {
                        tb = -hb;
                    }
                    if (tb > hb) {
                        tb = hb;
                    }
                    vector3 pbk = vector3_addition(cyl_b->position, vector3_scaling(bx, tb));
                    float sepk = vector3_length(vector3_subtraction(pak, pbk));
                    if ((sepk >= min_dist + slop) || out->contact_count >= 4) {
                        continue;
                    }
                    /* Replace single midpoint with two interval ends. */
                    if (k == 0) {
                        out->contact_count = 0;
                    }
                    contact_point_data *cpk = &out->contacts[out->contact_count++];
                    float penk = min_dist - sepk;
                    cpk->penetration = (penk > 0.0f) ? penk : 0.0f;
                    cpk->position = vector3_scaling(vector3_addition(pak, pbk), 0.5f);
                }
                if (out->contact_count <= 0) {
                    /* Interval ends missed (curved ends): restore single. */
                    out->contacts[0] = saved_single;
                    out->contact_count = 1;
                }
            }
        }
    }
    if (out->contact_count <= 0) {
        return false;
    }
    return true;
}
