/* MIG WP7 7.0a — the void compose core. Contract in mig_void.h.
 * Pure math, standalone: nothing in the engine calls this TU yet. */
#include "core/mig_void.h"
#include <math.h>

/* Fixed π literal — identical bits every platform, never math.h's macro
 * (the mig_mass.c discipline, duplicated here under this module's own
 * name so the two TUs stay textually independent). */
#define MIG_VOID_PI 3.14159265358979323846

typedef struct { double r [3][3]; } mv_rot3;

static void mv_rot_from_quat (vector4 q, mv_rot3 *R) {
    /* Verbatim mig_rot_from_quat (mig_mass.c): mirror of
     * vector4_to_math3 (math3d.h) in double, minus normalization: unit
     * input is a caller contract asserted in mig_void_compose. Same
     * term order so a signed composition agrees with the engine's
     * composite frame exactly. */
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

static void mv_rotate_tensor (const mv_rot3 *R, const double diag [3], double out [3][3]) {
    /* Verbatim mig_rotate_tensor (mig_mass.c): B = R·diag(I)·Rᵀ with
     * diagonal I: B_ij = Σ_k R_ik R_jk I_k. Multiplication is
     * IEEE-commutative and the sum order over k is fixed, so B_ij and
     * B_ji are computed to identical bits — symmetry is final. */
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double s = 0.0;
            for (int k = 0; k < 3; k++) { s += R -> r [i][k] * R -> r [j][k] * diag [k]; }
            out [i][j] = s;
        }
    }
}

/* Verbatim mig_part_tensor_local (mig_mass.c) with ONE addition: the
 * record's ROLE folds in at the very end (negation is exact — the
 * material path's bits are untouched, which is what makes the parity
 * gate meaningful). Material records: explicit kg or density·volume,
 * strictly positive (the frozen spec §4 rule). Void records: density
 * mode ONLY (explicit mass on a void is meaningless — a named refusal),
 * finite positive density, and the SAME dims/pose rules — the void's
 * mass math is the SAME primitive volume expression, negated. */
static int mv_part_tensor_signed (const mig_part *p, mig_mass_mode mode,
                                  int is_void, double *m_out, double diag [3]) {
    double m = 0.0, r = 0.0, hl = 0.0, hx = 0.0, hy = 0.0, hz = 0.0;
    /* Dims and pose first, per type — identical rule in BOTH roles. */
    switch (p -> type) {
        case mig_part_sphere: {
            r = p -> radius;
            if (!isfinite (r) || r <= 0.0 || /* radius must be sane */
                !isfinite (p -> position.x) || !isfinite (p -> position.y) || !isfinite (p -> position.z)) {
                return mig_void_bad_dims;
            }
            break;
        }
        case mig_part_cube: {
            hx = p -> half_extensions.x; hy = p -> half_extensions.y; hz = p -> half_extensions.z;
            if (!isfinite (hx) || !isfinite (hy) || !isfinite (hz) || hx <= 0.0 || hy <= 0.0 || hz <= 0.0 ||
                !isfinite (p -> position.x) || !isfinite (p -> position.y) || !isfinite (p -> position.z)) {
                return mig_void_bad_dims;
            }
            break;
        }
        case mig_part_cylinder: {
            r = p -> radius; hl = p -> cylinder_half_length;
            if (!isfinite (r) || r <= 0.0 || !isfinite (hl) || hl <= 0.0 ||
                !isfinite (p -> position.x) || !isfinite (p -> position.y) || !isfinite (p -> position.z)) {
                return mig_void_bad_dims;
            }
            break;
        }
        default: return mig_void_bad_type;
    }
    if (is_void) {
        /* The void's density interpretation: the SAME exact primitive
         * volume expressions — no second-source volume math. */
        if (mode != mig_mass_mode_density) { return mig_void_bad_mass_mode; }
        switch (p -> type) {
            case mig_part_sphere:    m = p -> mass * (4.0 / 3.0) * MIG_VOID_PI * r * r * r; break;
            case mig_part_cylinder:  m = p -> mass * MIG_VOID_PI * r * r * ((double) hl * 2.0); break;
            case mig_part_cube:      m = p -> mass * 8.0 * hx * hy * hz; break;
            default: return mig_void_bad_type; /* unreachable above */
        }
        if (!isfinite (m) || m <= 0.0) { return mig_void_bad_density; }
    } else if (mode == mig_mass_mode_explicit) {
        m = p -> mass;
        if (!isfinite (m) || m <= 0.0) { return mig_void_bad_mass; }
    } else if (mode == mig_mass_mode_density) {
        switch (p -> type) {
            case mig_part_sphere:    m = p -> mass * (4.0 / 3.0) * MIG_VOID_PI * r * r * r; break;
            case mig_part_cylinder:  m = p -> mass * MIG_VOID_PI * r * r * ((double) hl * 2.0); break;
            case mig_part_cube:      m = p -> mass * 8.0 * hx * hy * hz; break;
            default: return mig_void_bad_type; /* unreachable above */
        }
        if (!isfinite (m) || m <= 0.0) { return mig_void_bad_mass; }
    } else {
        /* Unknown enum: rejected LOUDLY with its own name (the
         * mig_mass.c rule — never silently read as density). */
        return mig_void_bad_mass_mode;
    }
    /* Principal tensor third: dims validated, |mass| positive. The
     * formulas are linear in m, so the ROLE negation below is exact. */
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
        default: return mig_void_bad_type; /* unreachable above */
    }
    if (is_void) {
        *m_out = -m;
        diag [0] = -diag [0]; diag [1] = -diag [1]; diag [2] = -diag [2];
    } else {
        *m_out = m;
    }
    return mig_void_ok;
}

/* One signed Steiner contribution about a fixed reference point (the
 * total COM): verbatim mig_compose's pass-2 body with the record's
 * already-signed (m, diag). Kept as a helper so the material and void
 * loops are THE SAME code path — the sign is data, not a branch. */
static void mv_steiner_about (const mig_part *p, double m, const double diag [3],
                              double com_x, double com_y, double com_z,
                              double I [3][3]) {
    mv_rot3 R;
    mv_rot_from_quat (p -> orientation, &R);
    double part_I [3][3];
    mv_rotate_tensor (&R, diag, part_I);
    const double dx = (double) p -> position.x - com_x;
    const double dy = (double) p -> position.y - com_y;
    const double dz = (double) p -> position.z - com_z;
    /* Steiner (parallel axis): I += m (d² δ − d⊗d). Off-diagonals here
     * carry the product-of-inertia the diagonal primitives cannot. With
     * a signed m the whole contribution — rotation of the principal
     * tensor AND Steiner — flips exactly, which is the subtraction. */
    const double d2 = dx * dx + dy * dy + dz * dz;
    part_I [0][0] += m * (d2 - dx * dx); part_I [0][1] -= m * dx * dy; part_I [0][2] -= m * dx * dz;
    part_I [1][0] -= m * dy * dx; part_I [1][1] += m * (d2 - dy * dy); part_I [1][2] -= m * dy * dz;
    part_I [2][0] -= m * dz * dx; part_I [2][1] -= m * dz * dy; part_I [2][2] += m * (d2 - dz * dz);
    for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) { I [a][b] += part_I [a][b]; }
    }
}

int mig_void_compose (const mig_part *material, int material_count,
                      const mig_part *voids, int void_count,
                      mig_totals *out) {
    if (!out) { return mig_void_null; }
    if ((material_count < 0) || (void_count < 0)) { return mig_void_null; }
    if (material_count == 0) { return mig_void_no_material; }
    if (!material) { return mig_void_null; }
    if ((void_count > 0) && (!voids)) { return mig_void_null; }
    /* Pass 0: input validation, record by record — materials in order
     * first, then voids in order (never mid-composition). Per record the
     * mig_compose order: quat, then the tensor dry run. */
    for (int i = 0; i < material_count; i++) {
        const mig_part *p = &material [i];
        const double n2 = (double) p -> orientation.w * (double) p -> orientation.w +
                          (double) p -> orientation.x * (double) p -> orientation.x +
                          (double) p -> orientation.y * (double) p -> orientation.y +
                          (double) p -> orientation.z * (double) p -> orientation.z;
        const double a = (n2 > 1.0) ? (n2 - 1.0) : (1.0 - n2);
        if (!isfinite (n2) || !isfinite (a) || a > 1e-4) { return mig_void_bad_quat; }
        double m_dry; double diag_dry [3];
        /* The dry run validates type/dims/mass for every record UP FRONT
         * and propagates the DISTINCT error (the contract the header
         * names); pass 1 recomputes for real. Composition runs only if
         * every record validated — a malformed record invalidates the
         * WHOLE assembly. */
        int e = mv_part_tensor_signed (p, p -> mass_mode, 0, &m_dry, diag_dry);
        if (e != mig_void_ok) { return e; }
    }
    for (int i = 0; i < void_count; i++) {
        const mig_part *p = &voids [i];
        const double n2 = (double) p -> orientation.w * (double) p -> orientation.w +
                          (double) p -> orientation.x * (double) p -> orientation.x +
                          (double) p -> orientation.y * (double) p -> orientation.y +
                          (double) p -> orientation.z * (double) p -> orientation.z;
        const double a = (n2 > 1.0) ? (n2 - 1.0) : (1.0 - n2);
        if (!isfinite (n2) || !isfinite (a) || a > 1e-4) { return mig_void_bad_quat; }
        double m_dry; double diag_dry [3];
        int e = mv_part_tensor_signed (p, p -> mass_mode, 1, &m_dry, diag_dry);
        if (e != mig_void_ok) { return e; }
    }
    /* Pass 1: signed total mass + COM, double accumulation, materials
     * then voids, definition order within each role. */
    double M = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
    for (int i = 0; i < material_count; i++) {
        const mig_part *p = &material [i];
        double m, diag [3];
        /* Cannot fail here: pass 0 validated identical inputs; the
         * collapse to bad_dims is unreachable defensive dead weight,
         * kept obvious (the mig_compose rule). */
        if (mv_part_tensor_signed (p, p -> mass_mode, 0, &m, diag) != mig_void_ok) { return mig_void_bad_dims; }
        M += m;
        mx += m * (double) p -> position.x;
        my += m * (double) p -> position.y;
        mz += m * (double) p -> position.z;
    }
    for (int i = 0; i < void_count; i++) {
        const mig_part *p = &voids [i];
        double m, diag [3];
        if (mv_part_tensor_signed (p, p -> mass_mode, 1, &m, diag) != mig_void_ok) { return mig_void_bad_dims; }
        /* The void's SIGNED contribution: the exact negative of the same
         * primitive's mass moment — subtraction IS the physics (the
         * sealed-cavity truth, WP7 plan §3). */
        M += m;
        mx += m * (double) p -> position.x;
        my += m * (double) p -> position.y;
        mz += m * (double) p -> position.z;
    }
    if (!isfinite (M) || M <= 0.0) { return mig_void_nonpositive; }
    const double inv_M = 1.0 / M;
    mig_totals result;
    result.mass = M;
    /* The COM stays DOUBLE through the inertia sum — the float boundary
     * lands ONCE, at the struct write (the 2026-11-10 audit fix,
     * mirrored verbatim). */
    const double com_x = mx * inv_M;
    const double com_y = my * inv_M;
    const double com_z = mz * inv_M;
    result.com.x = (float) com_x;
    result.com.y = (float) com_y;
    result.com.z = (float) com_z;
    /* Pass 2: inertia about the COM. Double sums, materials then voids,
     * definition order within each role; the float boundary lands once,
     * at the end. */
    double I [3][3] = {{0.0}};
    for (int i = 0; i < material_count; i++) {
        const mig_part *p = &material [i];
        double m, diag [3];
        (void) mv_part_tensor_signed (p, p -> mass_mode, 0, &m, diag); /* validated in pass 0 */
        mv_steiner_about (p, m, diag, com_x, com_y, com_z, I);
    }
    for (int i = 0; i < void_count; i++) {
        const mig_part *p = &voids [i];
        double m, diag [3];
        (void) mv_part_tensor_signed (p, p -> mass_mode, 1, &m, diag); /* validated in pass 0 */
        mv_steiner_about (p, m, diag, com_x, com_y, com_z, I);
    }
    if (!isfinite (I [0][0]) || !isfinite (I [1][1]) || !isfinite (I [2][2])) { return mig_void_bad_dims; }
    /* Positive-definiteness (Sylvester's criterion for a symmetric 3x3,
     * on the DOUBLE truth before the one float narrowing): leading
     * principal minors strictly positive. A physically meaningful rigid
     * tensor satisfies this; a void larger than its material — which a
     * pure sum cannot see geometrically (the documented compose-purity
     * limit) — violates it, and refusing here keeps garbage out of the
     * engine. */
    if (!(I [0][0] > 0.0)) { return mig_void_not_pd; }
    if (!(I [0][0] * I [1][1] - I [0][1] * I [0][1] > 0.0)) { return mig_void_not_pd; }
    const double det = I [0][0] * (I [1][1] * I [2][2] - I [1][2] * I [1][2])
                     - I [0][1] * (I [0][1] * I [2][2] - I [1][2] * I [0][2])
                     + I [0][2] * (I [0][1] * I [1][2] - I [1][1] * I [0][2]);
    if (!isfinite (det) || !(det > 0.0)) { return mig_void_not_pd; }
    result.inertia.matrix [0][0] = (float) I [0][0]; result.inertia.matrix [0][1] = (float) I [0][1];
    result.inertia.matrix [0][2] = (float) I [0][2];
    result.inertia.matrix [1][0] = (float) I [1][0]; result.inertia.matrix [1][1] = (float) I [1][1];
    result.inertia.matrix [1][2] = (float) I [1][2];
    result.inertia.matrix [2][0] = (float) I [2][0]; result.inertia.matrix [2][1] = (float) I [2][1];
    result.inertia.matrix [2][2] = (float) I [2][2];
    *out = result;
    return mig_void_ok;
}
