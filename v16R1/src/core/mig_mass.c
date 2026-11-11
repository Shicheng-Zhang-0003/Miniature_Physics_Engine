/* MIG stage 0 implementation (docs/NON_UNIFORM_BODY.md WP0; contract in
 * mig_mass.h). Independent of the body contract: no rigidbody.h include,
 * no config store, no world. WP1 wires this against counted clamps. */
#include "core/mig_mass.h"
#include <math.h>

/* Fixed π literal — identical bits every platform, never math.h's macro. */
#define MIG_PI 3.14159265358979323846

typedef struct { double r [3][3]; } mig_rot3;

static void mig_rot_from_quat (vector4 q, mig_rot3 *R) {
    /* Mirror of vector4_to_math3 (math3d.h) in double, minus normalization:
     * unit input is a caller contract asserted in mig_compose. Same term
     * order so the composite frame agrees with the body frame exactly. */
    const double w = q.w, x = q.x, y = q.y, z = q.z;
    /* x_double = 2x etc. (the engine's variable name, kept for review). */
    const double x2 = 2.0 * x, y2 = 2.0 * y, z2 = 2.0 * z;
    const double x_x = x * x2, x_y = x * y2, x_z = x * z2;
    const double y_y = y * y2, y_z = y * z2, z_z = z * z2;
    const double w_x = w * x2, w_y = w * y2, w_z = w * z2;
    R -> r [0][0] = 1.0 - (y_y + z_z); R -> r [0][1] = x_y - w_z; R -> r [0][2] = x_z + w_y;
    R -> r [1][0] = x_y + w_z; R -> r [1][1] = 1.0 - (x_x + z_z); R -> r [1][2] = y_z - w_x;
    R -> r [2][0] = x_z - w_y; R -> r [2][1] = y_z + w_x; R -> r [2][2] = 1.0 - (x_x + y_y);
}

/* Exact principal tensor of one primitive (double). Returns 0 on success.
 * Formulas duplicate — never re-route through — the rigidbody.c ones:
 * sphere (2/5)mr²; box (m/12)(h²+d²) per axis; cylinder axle-X ½mr² axial,
 * (m/12)(3r²+l²) transverse ("verified, do not fix", rigidbody.h).
 * 2026-11-10 audit tightening (review §2.4): dims and pose validate FIRST
 * in every mode (a malformed dimension is mig_compose_bad_dims even in
 * density mode, where the volume arithmetic used to poison the mass first
 * and report bad_mass); every part's mass is STRICTLY POSITIVE (the frozen
 * spec §4 — a zero-mass ballast part used to pass on a positive total); and
 * an unknown mass_mode enum is mig_compose_bad_mass_mode, never silently
 * read as density. */
static int mig_part_tensor_local (const mig_part *p, mig_mass_mode mode, double *m_out, double diag [3]) {
    double m = 0.0, r = 0.0, hl = 0.0, hx = 0.0, hy = 0.0, hz = 0.0;
    /* Dims and pose first, per type — identical rule in BOTH mass modes. */
    switch (p -> type) {
        case mig_part_sphere: {
            r = p -> radius;
            if (!isfinite (r) || r <= 0.0 || /* radius must be sane for BOTH modes */
                !isfinite (p -> position.x) || !isfinite (p -> position.y) || !isfinite (p -> position.z)) {
                return mig_compose_bad_dims;
            }
            break;
        }
        case mig_part_cube: {
            hx = p -> half_extensions.x; hy = p -> half_extensions.y; hz = p -> half_extensions.z;
            if (!isfinite (hx) || !isfinite (hy) || !isfinite (hz) || hx <= 0.0 || hy <= 0.0 || hz <= 0.0 ||
                !isfinite (p -> position.x) || !isfinite (p -> position.y) || !isfinite (p -> position.z)) {
                return mig_compose_bad_dims;
            }
            break;
        }
        case mig_part_cylinder: {
            r = p -> radius; hl = p -> cylinder_half_length;
            if (!isfinite (r) || r <= 0.0 || !isfinite (hl) || hl <= 0.0 ||
                !isfinite (p -> position.x) || !isfinite (p -> position.y) || !isfinite (p -> position.z)) {
                return mig_compose_bad_dims;
            }
            break;
        }
        default: return mig_compose_bad_type;
    }
    /* Mass second: explicit kg, or density times the exact primitive
     * volume (the SAME formulas the header documents — no second volume
     * math anywhere). Both modes demand strictly positive mass. */
    if (mode == mig_mass_mode_explicit) {
        m = p -> mass;
        if (!isfinite (m) || m <= 0.0) { return mig_compose_bad_mass; }
    } else if (mode == mig_mass_mode_density) {
        switch (p -> type) {
            case mig_part_sphere:    m = p -> mass * (4.0 / 3.0) * MIG_PI * r * r * r; break;
            case mig_part_cylinder:  m = p -> mass * MIG_PI * r * r * ((double) hl * 2.0); break;
            case mig_part_cube:      m = p -> mass * 8.0 * hx * hy * hz; break;
            default: return mig_compose_bad_type; /* unreachable above */
        }
        if (!isfinite (m) || m <= 0.0) { return mig_compose_bad_mass; }
    } else {
        /* Unknown enum: rejected LOUDLY with its own name — the pre-audit
         * else-branch silently read any foreign value as density mode. */
        return mig_compose_bad_mass_mode;
    }
    /* Principal tensor third: dims validated, mass positive. */
    switch (p -> type) {
        case mig_part_sphere:
            diag [0] = diag [1] = diag [2] = 0.4 * m * r * r;
            break;
        case mig_part_cube:
            diag [0] = (m / 12.0) * (4.0 * hy * hy + 4.0 * hz * hz);
            diag [1] = (m / 12.0) * (4.0 * hx * hx + 4.0 * hz * hz);
            diag [2] = (m / 12.0) * (4.0 * hx * hx + 4.0 * hy * hy);
            break;
        case mig_part_cylinder: {
            const double l = hl * 2.0;
            diag [0] = 0.5 * m * r * r; /* axial, local X */
            diag [1] = (m / 12.0) * (3.0 * r * r + l * l); /* transverse */
            diag [2] = (m / 12.0) * (3.0 * r * r + l * l);
            break;
        }
        default: return mig_compose_bad_type; /* unreachable above */
    }
    *m_out = m;
    return mig_compose_ok;
}

static void mig_rotate_tensor (const mig_rot3 *R, const double diag [3], double out [3][3]) {
    /* B = R·diag(I)·Rᵀ with diagonal I: B_ij = Σ_k R_ik R_jk I_k.
     * Multiplication is IEEE-commutative and the sum order over k is fixed,
     * so B_ij and B_ji are computed to identical bits — symmetry is final. */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double s = 0.0;
            for (int k = 0; k < 3; k++) { s += R -> r [i][k] * R -> r [j][k] * diag [k]; }
            out [i][j] = s;
        }
    }
}

int mig_compose (const mig_part *parts, int count, mig_totals *out) {
    if ((!parts) || (!out) || (count <= 0)) { return mig_compose_null; }
    /* Pass 0: input validation, part by part (never mid-composition). */
    for (int i = 0; i < count; i++) {
        const mig_part *p = &parts [i];
        const double n2 = (double) p -> orientation.w * (double) p -> orientation.w +
                          (double) p -> orientation.x * (double) p -> orientation.x +
                          (double) p -> orientation.y * (double) p -> orientation.y +
                          (double) p -> orientation.z * (double) p -> orientation.z;
        const double a = (n2 > 1.0) ? (n2 - 1.0) : (1.0 - n2);
        if (!isfinite (n2) || !isfinite (a) || a > 1e-4) { return mig_compose_bad_quat; }
        double m_dry; double diag_dry [3];
        /* The dry run validates type/dims/mass for every part UP FRONT and
         * propagates the DISTINCT error (the contract the header names);
         * pass 1 recomputes for real. Composition runs only if every part
         * validated — a malformed part invalidates the WHOLE assembly. */
        int e = mig_part_tensor_local (p, p -> mass_mode, &m_dry, diag_dry);
        if (e != mig_compose_ok) { return e; }
    }
    /* Pass 1: total mass + COM, double accumulation, definition order. */
    double M = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
    for (int i = 0; i < count; i++) {
        const mig_part *p = &parts [i];
        double m, diag [3];
        /* Cannot fail here: pass 0 validated identical inputs; the collapse
         * to bad_dims is unreachable defensive dead weight, kept obvious. */
        if (mig_part_tensor_local (p, p -> mass_mode, &m, diag) != mig_compose_ok) { return mig_compose_bad_dims; }
        M += m;
        mx += m * (double) p -> position.x;
        my += m * (double) p -> position.y;
        mz += m * (double) p -> position.z;
    }
    if (!isfinite (M) || M <= 0.0) { return mig_compose_bad_mass; }
    const double inv_M = 1.0 / M;
    mig_totals result;
    result.mass = M;
    /* 2026-11-10 audit fix: the COM stays DOUBLE through the inertia sum —
     * the old code cast it into the float vector3 fields here and then
     * subtracted that rounded value per part, so the Steiner terms carried
     * the rounding of an intermediate, not of the result (the documented
     * "float boundary lands at the end" was not yet true). The struct
     * write below is the one and only narrowing. */
    const double com_x = mx * inv_M;
    const double com_y = my * inv_M;
    const double com_z = mz * inv_M;
    result.com.x = (float) com_x;
    result.com.y = (float) com_y;
    result.com.z = (float) com_z;
    /* Pass 2: inertia about the COM. Double sums in part order; the float
     * boundary lands once, at the end. */
    double I [3][3] = {{0.0}};
    for (int i = 0; i < count; i++) {
        const mig_part *p = &parts [i];
        double m, diag [3];
        (void) mig_part_tensor_local (p, p -> mass_mode, &m, diag); /* validated in pass 0 */
        mig_rot3 R;
        mig_rot_from_quat (p -> orientation, &R);
        double part_I [3][3];
        mig_rotate_tensor (&R, diag, part_I);
        const double dx = (double) p -> position.x - com_x;
        const double dy = (double) p -> position.y - com_y;
        const double dz = (double) p -> position.z - com_z;
        /* Steiner (parallel axis): I += m (d² δ − d⊗d). Off-diagonals here
         * carry the product-of-inertia the diagonal primitives cannot. */
        const double d2 = dx * dx + dy * dy + dz * dz;
        part_I [0][0] += m * (d2 - dx * dx); part_I [0][1] -= m * dx * dy; part_I [0][2] -= m * dx * dz;
        part_I [1][0] -= m * dy * dx; part_I [1][1] += m * (d2 - dy * dy); part_I [1][2] -= m * dy * dz;
        part_I [2][0] -= m * dz * dx; part_I [2][1] -= m * dz * dy; part_I [2][2] += m * (d2 - dz * dz);
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) { I [a][b] += part_I [a][b]; }
        }
    }
    if (!isfinite (I [0][0]) || !isfinite (I [1][1]) || !isfinite (I [2][2])) { return mig_compose_bad_dims; }
    result.inertia.matrix [0][0] = (float) I [0][0]; result.inertia.matrix [0][1] = (float) I [0][1];
    result.inertia.matrix [0][2] = (float) I [0][2];
    result.inertia.matrix [1][0] = (float) I [1][0]; result.inertia.matrix [1][1] = (float) I [1][1];
    result.inertia.matrix [1][2] = (float) I [1][2];
    result.inertia.matrix [2][0] = (float) I [2][0]; result.inertia.matrix [2][1] = (float) I [2][1];
    result.inertia.matrix [2][2] = (float) I [2][2];
    *out = result;
    return mig_compose_ok;
}

void mig_rebase_parts (mig_part *parts, int count, vector3 delta) {
    if (!parts) { return; }
    for (int i = 0; i < count; i++) {
        parts [i].position.x -= delta.x;
        parts [i].position.y -= delta.y;
        parts [i].position.z -= delta.z;
    }
}

int mig_part_mass (const mig_part *part, double *out_mass) {
    if ((!part) || (!out_mass)) { return mig_compose_null; }
    double m;
    double diag [3];
    int err = mig_part_tensor_local (part, part -> mass_mode, &m, diag);
    if (err != mig_compose_ok) { return err; }
    *out_mass = m;
    return mig_compose_ok;
}

/* WP1 1.2: the bridge — compose once, copy, re-anchor the COPY. See the
 * header for the contract (caller-owned buffers, no malloc, write-on-
 * success, no cap: the cap belongs to WP2's pool). The re-anchor calls
 * mig_rebase_parts verbatim, so the pose shift is ONE code path. */
int mig_make_bridge (const mig_part *src, int count, mig_part *out_parts, mig_bridge *out) {
    if ((!src) || (!out_parts) || (!out) || (out_parts == src)) { return mig_compose_null; }
    mig_totals composed;
    int err = mig_compose (src, count, &composed);
    if (err != mig_compose_ok) { return err; }
    for (int i = 0; i < count; i++) { out_parts [i] = src [i]; }
    mig_rebase_parts (out_parts, count, composed.com);
    out -> totals = composed;
    out -> parts = out_parts;
    out -> count = count;
    return mig_compose_ok;
}
