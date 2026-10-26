/* mgb_sink_file.c — file sink: mapped scene as plain text (standalone
 * verification + golden files). One body per line, bonds after.
 *
 * Format (mgb_scene_v1):
 *   MGB_SCENE_V1 mm_per_a=<g> scale_note=<display-only>
 *   body <idx> Z=<z> pos=<x>,<y>,<z> r=<r> rgb=<r>,<g>,<b>   (mm, %.6f)
 *   bond <a> <b>
 *   end bodies=<n> bonds=<m>
 */
#include "mgb.h"
#include <stdio.h>

typedef struct {
    FILE *f;
    double mm_per_a;
    int nbodies, nbonds;
    int failed;
} file_ctx_t;

static int f_body (void *ctx, int idx, int Z, double x, double y, double z,
                   double r, const float rgb[3]) {
    file_ctx_t *c = ctx;
    if (fprintf (c->f, "body %d Z=%d pos=%.6f,%.6f,%.6f r=%.6f rgb=%.3f,%.3f,%.3f\n",
                 idx, Z, x, y, z, r, rgb[0], rgb[1], rgb[2]) < 0)
        c->failed = 1;
    else
        c->nbodies++;
    return 0;
}

static int f_bond (void *ctx, int a, int b) {
    file_ctx_t *c = ctx;
    if (fprintf (c->f, "bond %d %d\n", a, b) < 0)
        c->failed = 1;
    else
        c->nbonds++;
    return 0;
}

static int f_finish (void *ctx) {
    file_ctx_t *c = ctx;
    if (fprintf (c->f, "end bodies=%d bonds=%d\n", c->nbodies, c->nbonds) < 0)
        c->failed = 1;
    return 0;
}

/* Map frame into an open file. 0 ok, -1 on map or stream error. */
int mgb_sink_file_write (FILE *f, const mgb_frame_t *frame, double mm_per_a) {
    if (!f || !frame) return -1;
    if (fprintf (f, "MGB_SCENE_V1 mm_per_a=%.6f scale_note=display-only-not-physics\n",
                 mm_per_a) < 0)
        return -1;
    file_ctx_t c = { f, mm_per_a, 0, 0, 0 };
    static const mgb_sink_t sink = { f_body, f_bond, f_finish };
    if (mgb_map (frame, &sink, &c, mm_per_a) != 0) return -1;
    if (fflush (f) != 0 || ferror (f)) return -1;
    return c.failed ? -1 : 0;
}
