/* mgb_map.c — the ONLY place scale lives. Frame (A) -> sink (mm).
 *
 * Policy: positions multiply by mm_per_a (default MGB_MM_PER_A); nothing
 * else transforms. Radii come from a small display table (vdW-ish,
 * documented per element, NOT a force-field parameter); unknown Z gets a
 * neutral grey fallback radius — never a failure, always labeled by the
 * caller-visible return of mgb_element_display.
 *
 * Double arithmetic throughout; a float sink rounds once at its boundary.
 * mgb_unmap (header) inverts exactly, pinned by tests.
 */
#include "mgb.h"
#include <math.h>
#include <stddef.h>

typedef struct {
    int Z;
    double r_a;         /* display radius, Angstrom */
    float rgb[3];
    const char *note;
} mgb_elem_t;

/* CPK-flavoured display colours; radii are covalent-ish display picks. */
static const mgb_elem_t MGB_ELEMS[] = {
    {  1, 0.32, {1.00f, 1.00f, 1.00f}, "H white" },
    {  6, 0.77, {0.30f, 0.30f, 0.30f}, "C graphite" },
    {  7, 0.75, {0.20f, 0.40f, 1.00f}, "N blue" },
    {  8, 0.73, {1.00f, 0.20f, 0.20f}, "O red" },
    { 11, 1.02, {0.65f, 0.35f, 1.00f}, "Na violet" },
    { 12, 0.72, {0.55f, 1.00f, 0.55f}, "Mg green" },
    { 15, 1.10, {1.00f, 0.55f, 0.00f}, "P orange" },
    { 16, 1.02, {1.00f, 1.00f, 0.30f}, "S yellow" },
    { 17, 0.99, {0.30f, 1.00f, 0.30f}, "Cl green" },
    { 19, 1.38, {0.70f, 0.40f, 1.00f}, "K violet" },
    {  0, 0.00, {0.00f, 0.00f, 0.00f}, NULL },   /* sentinel */
};

int mgb_element_display (int Z, double *r_mm, float rgb[3]) {
    /* mm_per_a intentionally NOT applied here: radius stays in A so the
     * caller (mgb_map) scales radii and positions through one path. */
    for (const mgb_elem_t *e = MGB_ELEMS; e->note; e++) {
        if (e->Z == Z) {
            if (r_mm) *r_mm = e->r_a;
            if (rgb) {
                rgb[0] = e->rgb[0];
                rgb[1] = e->rgb[1];
                rgb[2] = e->rgb[2];
            }
            return 0;
        }
    }
    if (r_mm) *r_mm = 0.80; /* fallback: mid-size grey */
    if (rgb) {
        rgb[0] = 0.60f;
        rgb[1] = 0.60f;
        rgb[2] = 0.60f;
    }
    return -1;
}

int mgb_map (const mgb_frame_t *f, const mgb_sink_t *sink, void *ctx,
             double mm_per_a) {
    if (!f || !sink || !sink->body || !sink->bond || !sink->finish)
        return -1;
    if (!(mm_per_a > 0.0) || !isfinite (mm_per_a)) return -1;
    if (!f->atoms || f->natoms <= 0) return -1;
    for (int i = 0; i < f->natoms; i++) {
        const mgb_atom_t *a = &f->atoms[i];
        if (!isfinite (a->x + a->y + a->z)) return -1;
        double r_a;
        float rgb[3];
        mgb_element_display (a->Z, &r_a, rgb);
        if (sink->body (ctx, i, a->Z, a->x * mm_per_a, a->y * mm_per_a,
                        a->z * mm_per_a, r_a * mm_per_a, rgb) != 0)
            return -1;
    }
    for (int i = 0; i < f->nbonds; i++) {
        if (sink->bond (ctx, f->bonds[i].a, f->bonds[i].b) != 0) return -1;
    }
    return sink->finish (ctx) != 0 ? -1 : 0;
}
