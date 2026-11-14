/* MIG WP4 4.6b: CAD window view math. See cad_view_math.h for the
 * contract (pure, stateless, gateable headless; engine frame
 * conventions; cold-path trig ruling). */
#include "cad_view_math.h"

/* ---- internals ------------------------------------------------------ */

/* Degrees -> radians through the house fixed double-pi literal. */
static float cad_view_radians (float degrees) {
    return degrees * (float) MATH_PI / 180.0f;
}

/* Engine quats have no conjugate helper — the assembly->local ray
 * transform needs one (rotate by the INVERSE orientation). */
static vector4 cad_view_conjugate (vector4 quaternion) {
    return (vector4) {quaternion.w, -quaternion.x, -quaternion.y,
                      -quaternion.z};
}

static vector3 cad_view_abs3 (vector3 v) {
    return (vector3) {(v.x < 0.0f) ? -v.x : v.x,
                      (v.y < 0.0f) ? -v.y : v.y,
                      (v.z < 0.0f) ? -v.z : v.z};
}

/* ---- the orbit camera -------------------------------------------------- */

vector3 cad_view_forward (const cad_view_camera *camera) {
    if (!camera) { return (vector3) {0.0f, 0.0f, -1.0f}; }
    const float cy = cosf (cad_view_radians (camera -> yaw_deg));
    const float sy = sinf (cad_view_radians (camera -> yaw_deg));
    const float cp = cosf (cad_view_radians (camera -> pitch_deg));
    const float sp = sinf (cad_view_radians (camera -> pitch_deg));
    /* The eye sits at target + distance * (sy*cp, sp, cy*cp), so the
     * look direction is the negation (normalized by construction for
     * the documented pitch clamp: cp > 0 there). */
    return vector3_normalisation (
        (vector3) {-sy * cp, -sp, -cy * cp});
}

vector3 cad_view_right (const cad_view_camera *camera) {
    const vector3 world_up = {0.0f, 1.0f, 0.0f};
    return vector3_normalisation (
        vector3_cross (cad_view_forward (camera), world_up));
}

vector3 cad_view_up (const cad_view_camera *camera) {
    return vector3_cross (cad_view_right (camera), cad_view_forward (camera));
}

vector3 cad_view_eye (const cad_view_camera *camera) {
    if (!camera) { return (vector3) {0.0f, 0.0f, 0.0f}; }
    return vector3_subtraction (camera -> target,
        vector3_scaling (cad_view_forward (camera), camera -> distance));
}

/* ---- rays -------------------------------------------------------------- */

cad_view_ray cad_view_ray_from_screen (const cad_view_camera *camera,
                                       float ndc_x, float ndc_y) {
    cad_view_ray ray;
    ray.origin = cad_view_eye (camera);
    if (!camera || (camera -> aspect <= 0.0f) || (camera -> distance <= 0.0f)) {
        ray.direction = cad_view_forward (camera);
        return ray;
    }
    const float half_tan = tanf (cad_view_radians (camera -> fov_y_deg) * 0.5f);
    /* Edge rays spread tan(fov/2) vertically and tan(fov/2) * aspect
     * horizontally (the wider screen axis carries the aspect). */
    vector3 spread = vector3_addition (
        vector3_scaling (cad_view_right (camera), ndc_x * half_tan * camera -> aspect),
        vector3_scaling (cad_view_up (camera), ndc_y * half_tan));
    ray.direction = vector3_normalisation (
        vector3_addition (cad_view_forward (camera), spread));
    return ray;
}

/* ---- picking ------------------------------------------------------------ */

static cad_view_hit cad_view_no_hit (void) {
    cad_view_hit hit;
    hit.hit = 0;
    hit.t = 0.0f;
    hit.point = (vector3) {0.0f, 0.0f, 0.0f};
    hit.normal = (vector3) {0.0f, 0.0f, 0.0f};
    return hit;
}

static cad_view_hit cad_view_hit_of (float t, vector3 point, vector3 normal) {
    cad_view_hit hit;
    hit.hit = 1;
    hit.t = t;
    hit.point = point;
    hit.normal = normal;
    return hit;
}

/* Ray -> part-local frame: origin re-based to the part center, both
 * endpoints rotated by the INVERSE orientation (unit quats only — the
 * design model validates them; rotate_to_vector3 re-normalizes
 * defensively anyway). */
static void cad_view_ray_to_local (cad_view_ray ray, vector3 center,
                                   vector4 orientation,
                                   vector3 *local_origin, vector3 *local_dir) {
    *local_origin = vector4_rotate_to_vector3 (cad_view_conjugate (orientation),
        vector3_subtraction (ray.origin, center));
    *local_dir = vector4_rotate_to_vector3 (cad_view_conjugate (orientation),
        ray.direction);
}

cad_view_hit cad_view_pick_sphere (cad_view_ray ray, vector3 center,
                                   float radius) {
    if (!(radius > 0.0f)) { return cad_view_no_hit (); }
    /* |o + t d - c|^2 = r^2 with o already re-based (t along a UNIT d):
     * t^2 + 2 t (o.d) + (|o|^2 - r^2) = 0, nearest root >= 0. */
    const vector3 oc = vector3_subtraction (ray.origin, center);
    const float b = vector3_dot (oc, ray.direction);
    const float c = vector3_dot (oc, oc) - radius * radius;
    const float disc = b * b - c; /* a == 1 for a unit direction */
    if (disc < 0.0f) { return cad_view_no_hit (); }
    const float root = sqrtf (disc);
    float t = -b - root;
    if (t < 0.0f) { t = -b + root; } /* inside the sphere */
    if (t < 0.0f) { return cad_view_no_hit (); } /* sphere behind */
    const vector3 point = vector3_addition (ray.origin,
        vector3_scaling (ray.direction, t));
    const vector3 normal = vector3_normalisation (
        vector3_subtraction (point, center));
    return cad_view_hit_of (t, point, normal);
}

cad_view_hit cad_view_pick_box (cad_view_ray ray, vector3 center,
                                vector3 half_extensions, vector4 orientation) {
    const vector3 he = cad_view_abs3 (half_extensions);
    if ((he.x <= 0.0f) || (he.y <= 0.0f) || (he.z <= 0.0f)) {
        return cad_view_no_hit ();
    }
    vector3 o;
    vector3 d;
    cad_view_ray_to_local (ray, center, orientation, &o, &d);
    /* Slab method in the part's local frame. d component == 0: the ray
     * is parallel to that slab pair — inside means the slab survives,
     * outside misses outright. */
    float tmin = -1.0f; /* unbounded below until proven otherwise */
    float tmax = 1.0e30f;
    int tmin_axis = -1; /* 0/1/2 = x/y/z, for the exit normal */
    const float oh [3] = {o.x, o.y, o.z};
    const float dh [3] = {d.x, d.y, d.z};
    const float hh [3] = {he.x, he.y, he.z};
    for (int axis = 0; axis < 3; axis++) {
        if (dh [axis] == 0.0f) {
            if ((oh [axis] < -hh [axis]) || (oh [axis] > hh [axis])) {
                return cad_view_no_hit ();
            }
            continue;
        }
        const float inv = 1.0f / dh [axis];
        float t1 = (-hh [axis] - oh [axis]) * inv;
        float t2 = (hh [axis] - oh [axis]) * inv;
        if (t1 > t2) { float swap = t1; t1 = t2; t2 = swap; }
        if (t1 > tmin) { tmin = t1; tmin_axis = axis; }
        if (t2 < tmax) { tmax = t2; }
        if (tmax < tmin) { return cad_view_no_hit (); }
    }
    float t;
    vector3 local_normal = {0.0f, 0.0f, 0.0f};
    if (tmin > 0.0f) {
        t = tmin;
        float n [3] = {0.0f, 0.0f, 0.0f};
        n [tmin_axis] = (dh [tmin_axis] > 0.0f) ? -1.0f : 1.0f;
        local_normal = (vector3) {n [0], n [1], n [2]};
    } else if (tmax >= 0.0f) {
        t = 0.0f; /* origin inside the box */
    } else {
        return cad_view_no_hit (); /* box entirely behind */
    }
    const vector3 point = vector3_addition (ray.origin,
        vector3_scaling (ray.direction, t));
    const vector3 normal = (tmin > 0.0f)
        ? vector4_rotate_to_vector3 (orientation, local_normal)
        : (vector3) {0.0f, 0.0f, 0.0f};
    return cad_view_hit_of (t, point, normal);
}

cad_view_hit cad_view_pick_cylinder (cad_view_ray ray, vector3 center,
                                     float radius, float half_length,
                                     vector4 orientation) {
    if (!(radius > 0.0f) || !(half_length > 0.0f)) {
        return cad_view_no_hit ();
    }
    vector3 o;
    vector3 d;
    cad_view_ray_to_local (ray, center, orientation, &o, &d);
    /* Axle LOCAL X (MPE_FTC_090): the side surface is
     * y^2 + z^2 = r^2 with |x| <= hl; the caps are the discs x = +-hl. */
    float best_t = 1.0e30f;
    vector3 best_local_normal = {0.0f, 0.0f, 0.0f};
    /* Side: (oy + t dy)^2 + (oz + t dz)^2 = r^2. */
    const float a = d.y * d.y + d.z * d.z;
    const float b = 2.0f * (o.y * d.y + o.z * d.z);
    const float c = o.y * o.y + o.z * o.z - radius * radius;
    if (a > 0.0f) {
        const float disc = b * b - 4.0f * a * c;
        if (disc >= 0.0f) {
            const float root = sqrtf (disc);
            const float candidates [2] = {(-b - root) / (2.0f * a),
                                          (-b + root) / (2.0f * a)};
            for (int i = 0; i < 2; i++) {
                const float t = candidates [i];
                if ((t < 0.0f) || (t >= best_t)) { continue; }
                const float x = o.x + d.x * t;
                if ((x < -half_length) || (x > half_length)) { continue; }
                const float py = o.y + d.y * t;
                const float pz = o.z + d.z * t;
                best_t = t;
                best_local_normal =
                    vector3_normalisation ((vector3) {0.0f, py, pz});
            }
        }
    }
    /* Caps: the x = +-hl discs. */
    if (d.x != 0.0f) {
        const float planes [2] = {-half_length, half_length};
        for (int i = 0; i < 2; i++) {
            const float t = (planes [i] - o.x) / d.x;
            if ((t < 0.0f) || (t >= best_t)) { continue; }
            const float py = o.y + d.y * t;
            const float pz = o.z + d.z * t;
            if ((py * py + pz * pz) > radius * radius) { continue; }
            best_t = t;
            best_local_normal = (vector3) {(i == 0) ? -1.0f : 1.0f, 0.0f, 0.0f};
        }
    }
    if (best_t >= 1.0e30f) { return cad_view_no_hit (); }
    const vector3 point = vector3_addition (ray.origin,
        vector3_scaling (ray.direction, best_t));
    const vector3 normal =
        vector4_rotate_to_vector3 (orientation, best_local_normal);
    return cad_view_hit_of (best_t, point, normal);
}

/* ---- work-plane drag ---------------------------------------------------- */

int cad_view_ray_plane (cad_view_ray ray, vector3 point, vector3 normal,
                        vector3 *out) {
    if (!out) { return 0; }
    const float denom = vector3_dot (ray.direction, normal);
    if (denom == 0.0f) { return 0; } /* parallel: no intersection */
    const float t = vector3_dot (vector3_subtraction (point, ray.origin),
                                 normal) / denom;
    if (t <= 0.0f) { return 0; } /* behind the ray: refuse, don't wrap */
    *out = vector3_addition (ray.origin,
        vector3_scaling (ray.direction, t));
    return 1;
}
