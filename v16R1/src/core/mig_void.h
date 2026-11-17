/* MIG WP7 7.0a: void math — the STANDALONE negative-mass compose core.
 *
 * "Standalone" is a hard property of this brick (the cad_editor.c
 * pattern): this TU compiles into the engine and suite builds but
 * NOTHING in the engine calls it yet — zero behavior change, verified
 * by the full battery being green. The hook-in (compose / add_composite
 * / render / scene consumption) is the NEXT brick, gated on this
 * module's compliance battery per the user's ruling: "make the code
 * separately and do not hook the code into the main system UNTIL we
 * have tested proper compliance and stability".
 *
 * A void is SUBTRACTIVE material: the exact negative of a primitive's
 * mass contribution (WP7 plan, studies/void_objects_plan.md §3 — a
 * sealed cavity's interior is unreachable by collision and invisible to
 * render BY DEFINITION, so mass/COM/inertia subtraction is its COMPLETE
 * physical truth). Void records are plain mig_part geometry records:
 * the type must be in the primitive family, the mass field is the
 * DENSITY of the removed material (density interpretation ONLY —
 * explicit mass on a void is meaningless and refused), pose and
 * orientation exactly as a material part's.
 *
 * The compose is ONE signed pass mirroring mig_compose exactly: same
 * double truth, same term order, the float boundary lands once — so a
 * void-free call is BITWISE-IDENTICAL to mig_compose on the same
 * materials (pinned by the suite gate). Formulas duplicate — never
 * re-route through — mig_mass.c's statics (that file's own documented
 * discipline, in reverse): what to share is the HOOK brick's decision,
 * after compliance is proven.
 *
 * Pure math. No rigidbody.h, no config store, no world, no malloc.
 * Every malformed input is a distinct named error; write-on-success
 * (out untouched on any refusal). Determinism: fixed op order, fixed π
 * literal (never math.h's macro), double sums, one float narrowing.
 */
#ifndef mig_void_h
#define mig_void_h
#include "core/mig_mass.h"

typedef enum {
    mig_void_ok = 0,
    mig_void_null = 1,          /* NULL out, negative counts, NULL material
                                 * with a positive count, or a NULL voids
                                 * array with a positive count */
    mig_void_bad_type = 2,      /* record type outside the WP0 primitive family */
    mig_void_bad_dims = 3,      /* non-finite or non-positive dims / position */
    mig_void_bad_quat = 4,      /* non-finite or non-unit (|n2-1| > 1e-4) */
    mig_void_bad_mass = 5,      /* material mass non-finite or <= 0 */
    mig_void_bad_mass_mode = 6, /* material mode not explicit/density; void
                                 * mode not density */
    mig_void_bad_density = 7,   /* void density non-finite or <= 0 */
    mig_void_no_material = 8,   /* zero material parts (a void alone is no
                                 * body) */
    mig_void_nonpositive = 9,   /* total mass non-finite or <= 0 (the voids
                                 * ate the body) */
    mig_void_not_pd = 10        /* total tensor not positive-definite (a
                                 * mis-placed void larger than its
                                 * material) */
} mig_void_err;

/* Signed compose: `material` composes POSITIVE (the exact mig_compose
 * rules per record: explicit kg or density kg/m^3, strictly positive,
 * quats unit to 1e-4), the `voids` list SUBTRACTS (each record's mass
 * field read as kg/m^3, density mode only, same dims/pose/quat rules).
 * Both lists keep ASSEMBLY-frame poses; the totals are in that same
 * frame (the caller re-anchors, as always). Refusals, in order:
 * material records one by one (quat, then dims, then mass — the
 * mig_compose pass-0 order), then void records the same way; then the
 * totals: mass finite > 0 AND the inertia tensor positive-definite
 * (Sylvester's criterion, on the double truth BEFORE the one float
 * narrowing) — a void larger than its material is a mis-placed void,
 * refused LOUDLY, never a garbage tensor. With void_count == 0 this is
 * mig_compose bit-for-bit on the same materials (the parity gate pins
 * the totals AND the refusal codes). Write-on-success: `out` is
 * untouched on any refusal. */
int mig_void_compose (const mig_part *material, int material_count,
                      const mig_part *voids, int void_count,
                      mig_totals *out);

#endif /* mig_void_h */
