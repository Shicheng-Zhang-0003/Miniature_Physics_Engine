/* MIG — MPE-Infinity-Generator, stage 0 (WP0 of the v16S train charter,
 * docs/NON_UNIFORM_BODY.md): EXACT mass composition for primitive parts.
 *
 * Pure math. This TU knows nothing about physics_world or rigidbody — it
 * composes total mass / COM / inertia for a caller-defined assembly and
 * reports. It must never silently substitute an input: every malformed
 * input is a distinct named error the caller can count. (WP1 converts
 * these into the counted-clamp discipline when the body contract starts
 * consuming the totals.)
 *
 * Convention: `position` stays the COM of the composed body (design
 * record §2, decision (A)). These functions take part poses in the
 * caller's ASSEMBLY frame and return the COM in that same frame; the
 * caller re-anchors the body origin via mig_rebase_parts so the composite
 * body is born COM-framed and the engine's existing COM-relative
 * machinery (solver lever arms, localized forces, free flight) is used
 * as-is. That re-anchor is caller bookkeeping by design — NOT a physics
 * step — which is exactly why it lives on the caller side.
 *
 * Determinism contract:
 *  - all accumulation in double; part order = CALLER definition order;
 *  - exact IEEE ops only (+ - * / sqrt); fixed π literal; no pow/trig;
 *  - float is the struct boundary (math3/vector3), double only inside;
 *  - per-part tensors are DIAGONAL in the part's principal frame (all
 *    three primitive types are), rotated R·I·Rᵀ per part before the
 *    Steiner translation; diagonal composition forces the off-diagonal
 *    Steiner terms to carry every product-of-inertia in the output;
 *  - unit quaternion input is ASSERTED (`mig_compose_bad_quat` outside
 *    1±1e-4), never renormalized — renormalizing would hide a caller bug
 *    behind a silent substitution. The assembly-frame matrix algebra
 *    mirrors the engine's vector4_to_math3 (math3d.h) term-for-term in
 *    double so the composite frame never disagrees with the body frame.
 */
#ifndef mig_mass_h
#define mig_mass_h
#include "core/math3d.h"

typedef enum {
    mig_part_sphere = 0,
    mig_part_cube = 1,
    mig_part_cylinder = 2
} mig_part_type;

typedef enum {
    mig_mass_mode_explicit = 0, /* `mass` field is kilograms */
    mig_mass_mode_density = 1   /* `mass` field is kg/m^3; volume from exact dims */
} mig_mass_mode;

typedef struct {
    mig_part_type type;
    /* Sphere: radius. Cube: half_extensions. Cylinder: radius + half_length
     * (axle local X, the engine's MPE_FTC_090 convention). */
    float radius, cylinder_half_length;
    vector3 half_extensions;
    /* Part pose in the caller's assembly frame. */
    vector3 position;
    vector4 orientation; /* part -> assembly, engine (w,x,y,z) convention */
    mig_mass_mode mass_mode;
    float mass; /* explicit kg, or density kg/m^3 when density mode */
} mig_part;

typedef struct {
    double mass;   /* kg (double truth; float boundary lands downstream) */
    vector3 com;   /* assembly frame; floats by house struct convention */
    math3 inertia; /* about COM, assembly frame; exact double sum, float cast */
} mig_totals;

typedef enum {
    mig_compose_ok = 0,
    mig_compose_null = 1,      /* NULL list / NULL out / count <= 0 */
    mig_compose_bad_type = 2,  /* part type outside the WP0 primitive family */
    mig_compose_bad_dims = 3,  /* non-finite or non-positive dimension */
    mig_compose_bad_quat = 4,  /* non-finite or non-unit orientation */
    mig_compose_bad_mass = 5   /* negative/non-finite mass, or total mass <= 0 */
} mig_compose_err;

/* Compose the assembly. Parts are iterated in definition order; mass/COM
 * first, then inertia about the computed COM. Returns mig_compose_err.
 * `out` is overwritten ONLY on success. */
int mig_compose (const mig_part *parts, int count, mig_totals *out);

/* Shift every part position by `-delta`, in place. The caller passes
 * `totals.com` to turn an assembly-frame definition into a COM-framed
 * one (the pose re-anchor). Pure translation: no re-normalization, no
 * reordering — part indices stay exactly as the caller defined them. */
void mig_rebase_parts (mig_part *parts, int count, vector3 delta);

/* ---- WP1 1.2 bridge: the handoff contract WP2 will consume ----
 * mig_compose totals plus the RE-ANCHORED copy of the part list, ready
 * for a world to "copy in" to its own storage (WP2's per-world pool).
 * The bridge owns nothing: the caller supplies the destination buffer,
 * the module never mallocs, and `src` is read-only and never modified.
 *
 * Re-anchor: the copy is shifted by -totals.com (mig_rebase_parts), so
 * the stored part poses are COM-framed — the frame the body itself
 * lives in (charter decision (A): position stays the COM). The original
 * definition frame survives in the caller's `src`; indices are untouched.
 *
 * Write-on-success rule (the mig_compose rule) holds here too: `out` and
 * `out_parts` are overwritten only after a fully valid composition.
 * 2026-10-31 sizing note: mig_compose composes EXACTLY over any count —
 * the P2 cap (composite.max_parts = 16) is a property of WP2's parts
 * POOL (where parts are stored and the counted clamp fires), not of
 * pure math, so the bridge carries no cap and fires no counter. */
typedef struct {
    mig_totals totals;     /* composed truth: mass, COM, inertia about COM */
    const mig_part *parts; /* the caller-owned re-anchored copy below */
    int count;
} mig_bridge;

/* Compose once, copy into `out_parts` (which must hold `count` slots and
 * must NOT alias `src` — the copy is then mutated in place by the
 * re-anchor), re-anchor the copy to the COM frame, and record everything
 * in `out`. Returns a mig_compose_err: mig_compose_ok on success, the
 * compose error verbatim on a malformed assembly, mig_compose_null for
 * NULL src/out_parts/out or a src==out_parts aliasing call. */
int mig_make_bridge (const mig_part *src, int count, mig_part *out_parts, mig_bridge *out);
#endif /* mig_mass_h */
