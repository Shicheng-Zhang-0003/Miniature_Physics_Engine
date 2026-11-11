/* MIG WP2.5 2.5.1: convex hull point collection — pure data, no physics.
 *
 * A convex hull is defined by a caller-owned set of 3D points (vertices).
 * This TU knows nothing about physics_world, rigidbody, or the solver — it
 * holds point sets, builds exact tetra/box point sets, and validates that
 * every point is finite. Garbage rejection (non-finite points) is
 * suite-gated: a malformed point set returns mig_hull_bad_point and the
 * output set is left untouched.
 *
 * Discipline (mirrors mig_mass.h):
 *  - all coordinates double; fixed π literal; no pow/trig;
 *  - caller-owned buffers, no malloc, write-on-success;
 *  - every malformed input is a distinct named error.
 */
#ifndef mig_hull_h
#define mig_hull_h
#include <stddef.h>

typedef struct { double x, y, z; } mig_hull_point;

typedef struct {
    mig_hull_point *points; /* caller-owned buffer */
    int count;              /* points actually stored */
    int capacity;           /* slots in the caller buffer */
} mig_hull_point_set;

typedef enum {
    mig_hull_ok = 0,
    mig_hull_null = 1,       /* NULL set / NULL buf / NULL out */
    mig_hull_bad_count = 2,  /* count <= 0 or exceeds capacity */
    mig_hull_bad_point = 3,  /* non-finite coordinate (garbage rejection) */
    mig_hull_bad_index = 4,  /* APPENDED 2.5.3: face index outside the vertex array */
    mig_hull_bad_mesh = 5    /* APPENDED 2.5.3: non-finite density, degenerate
                              * or non-positive-volume (open/inward-wound) mesh */
} mig_hull_err;

/* Initialise a point set over a caller-owned buffer (no malloc).
 * `buf` must hold `cap` slots; `cap` must be > 0. Returns mig_hull_ok. */
mig_hull_err mig_hull_point_set_init (mig_hull_point_set *set,
                                      mig_hull_point *buf, int cap);

/* Add one point; validates finiteness of x, y, z. Fails with
 * mig_hull_bad_point (leaves the set unchanged) on any non-finite
 * coordinate, or mig_hull_bad_count if the set is full. */
mig_hull_err mig_hull_point_set_add (mig_hull_point_set *set,
                                     double x, double y, double z);

/* Build an exact box point set: the 8 corners of
 * [-hx,+hx] × [-hy,+hy] × [-hz,+hz], centred at the origin.
 * All dims must be finite and > 0. */
mig_hull_err mig_hull_build_box (double hx, double hy, double hz,
                                 mig_hull_point_set *out);

/* Build an exact tetra point set: 4 vertices of a regular tetrahedron
 * centred at the origin: (±1,±1,±1) with an even number of minus signs,
 * scaled by `scale` (must be finite and > 0). */
mig_hull_err mig_hull_build_tetra (double scale,
                                   mig_hull_point_set *out);

/* Validate every point in the set is finite. Returns mig_hull_ok or
 * mig_hull_bad_point on the first non-finite coordinate. */
mig_hull_err mig_hull_point_set_validate (const mig_hull_point_set *set);

/* ---- WP2.5 2.5.2: the 2D proof (divergence theorem on polygons) ----
 * The lowest-dimension validation of the divergence-theorem machinery
 * 2.5.3 builds on: per-edge signed contributions (Green's theorem),
 * double accumulation in edge order, no trig, no pow. */

typedef struct {
    double area;       /* ∫∫ 1 dA (signed: positive for CCW winding) */
    double moment_x;   /* ∫∫ x dA */
    double moment_y;   /* ∫∫ y dA */
    double second_xx;  /* ∫∫ x² dA */
    double second_yy;  /* ∫∫ y² dA */
    double second_xy;  /* ∫∫ x·y dA */
} mig_hull_area2;

/* Mass properties of a plane polygon in the x-y plane (the z coordinate
 * of each vertex is IGNORED), counterclockwise winding for positive
 * area. Vertices must be finite; count >= 3. All results are SIGNED
 * integrals — the caller divides by `area` for centroid and densities. */
mig_hull_err mig_hull_polygon_props (const mig_hull_point *verts, int count,
                                     mig_hull_area2 *out);

/* ---- WP2.5 2.5.3: hull mass properties (divergence theorem, 3D) ----
 * The same per-element signed-contribution discipline, one dimension up:
 * each face anchors a tetra at the origin; signed dV = (a×b)·c/6 sums to
 * the enclosed volume for any closed mesh. */

typedef struct {
    double mass;
    double com[3];
    double inertia[3][3]; /* about the COM (L = I·ω convention) */
} mig_hull_mass;

/* Mass properties of a closed triangle mesh (the convex hull's boundary).
 * faces: 3 vertex indices per face (face_count * 3 ints total), wound so
 * the right-hand-rule normal points OUT of the enclosed volume. Vertices
 * must be finite; a closed mesh needs vert_count >= 4 and face_count >= 4.
 * density must be finite and > 0. All results are written only on success. */
mig_hull_err mig_hull_mesh_props (const mig_hull_point *verts, int vert_count,
                                  const int *faces, int face_count,
                                  double density, mig_hull_mass *out);

#endif /* mig_hull_h */
