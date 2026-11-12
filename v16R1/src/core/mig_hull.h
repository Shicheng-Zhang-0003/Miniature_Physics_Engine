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
 *  - all coordinates double; no pow/trig (no hull formula needs π);
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

/* ---- WP2.5 2.5.4: SAT reference pairs (boolean + MTD only) ----
 * No contact manifold is generated here — that is future work. Both
 * functions report one separating-axis result per pair. Axis
 * normalization pre-scales by the max absolute component before
 * squaring, so directions survive ceiling-scale geometry (a raw cross
 * of two ~1e200 edges would overflow len to Inf and fabricate a zero
 * axis). Consistent with the mass paths: refuse (bad_mesh) only when
 * the VERDICT itself would be non-finite — finite-but-huge inputs
 * whose supports stay representable get their true MTD. */

typedef struct {
    int overlap;      /* 1 = penetrating or touching on every axis */
    double axis[3];   /* unit axis pointing FROM the hull TOWARD the primitive */
    double depth;     /* overlap: positive MTD; separated: 0 */
} mig_hull_sat_result;

/* Hull (closed, convex, outward-wound — caller contract, validated by
 * mig_hull_mesh_props) vs an AXIS-ALIGNED box (center + half extents).
 * Axis set: hull face normals, the 3 box axes, hull-edge × box-edge
 * crosses. On overlap, translating the HULL by −axis·depth separates the
 * pair (first min axis wins ties); on separation, `axis` is a proven
 * separating direction and depth is 0. */
mig_hull_err mig_hull_sat_hull_box (const mig_hull_point *verts, int vert_count,
                                    const int *faces, int face_count,
                                    const double box_center[3],
                                    const double box_half[3],
                                    mig_hull_sat_result *out);

/* Hull vs sphere (center + radius). Axis set: hull face normals,
 * edge-to-center perpendiculars, and vertex-to-center directions — the
 * complete separating-axis set for a convex polyhedron against a sphere. */
mig_hull_err mig_hull_sat_hull_sphere (const mig_hull_point *verts, int vert_count,
                                       const int *faces, int face_count,
                                       const double center[3], double radius,
                                       mig_hull_sat_result *out);

/* ---- APPENDED WP2.6 2.6.1: exact-coordinate hull shapes (pure data) ----
 * Transcribed from the prism/pyramid side study (its sections 1-2;
 * conventions unchanged). Extrusion-family shapes put the base at z=0
 * and extend toward +z, matching the box table's ±z-cap convention;
 * the icosahedron is centred at the origin like build_tetra. Every
 * vertex set below is EXACT in literals — no trig, no ring
 * parametrization, no π (the n-gonal ring families land separately,
 * behind their own ruling). Each shape ships its outward-wound face
 * table (right-hand rule): consumers pass (set.points, set.count) and
 * the table straight into mig_hull_mesh_props / the SAT pair
 * functions. The suite gate proves the windings mechanically — an
 * inward or open mesh cannot produce a positive volume, so every
 * closed-form pass through mesh_props is also a winding pass.
 *
 * Vertex/face counts (also the minimum point-set capacity):
 *   prism 6 verts / 8 tris,  pyr4 5 / 6,  pyr3 4 / 4,
 *   icosahedron 12 / 20. */

#define MIG_HULL_PRISM_VERTS 6
#define MIG_HULL_PRISM_FACES 8
#define MIG_HULL_PYR4_VERTS 5
#define MIG_HULL_PYR4_FACES 6
#define MIG_HULL_PYR3_VERTS 4
#define MIG_HULL_PYR3_FACES 4
#define MIG_HULL_ICOSA_VERTS 12
#define MIG_HULL_ICOSA_FACES 20

/* Right triangular prism, equilateral cross-section, side s > 0,
 * height h > 0: base triangle A=(0,0,0), B=(s,0,0), C=(s/2, √3/2·s, 0)
 * in z=0, top copy at z=h. Vertex order 0..5 = A,B,C,A',B',C'.
 * Closed forms (density 1): V = √3/4·s²h, com (s/2, √3/6·s, h/2),
 * Ixx = Iyy = m(h²/12 + s²/24), Izz = m·s²/12. */
mig_hull_err mig_hull_build_prism_equilateral (double s, double h,
                                               mig_hull_point_set *out);
extern const int mig_hull_prism_faces [MIG_HULL_PRISM_FACES * 3];

/* Right square pyramid, base side s > 0 centred at the origin in z=0,
 * apex (0,0,h), h > 0. Vertex order 0..3 = base corners
 * (−s/2,−s/2), (s/2,−s/2), (s/2,s/2), (−s/2,s/2), 4 = apex.
 * Closed forms (density 1): V = s²h/3, com (0,0,h/4),
 * Ixx = Iyy = m(s²/20 + 3h²/80), Izz = m·s²/10. */
mig_hull_err mig_hull_build_pyr4_square (double s, double h,
                                         mig_hull_point_set *out);
extern const int mig_hull_pyr4_faces [MIG_HULL_PYR4_FACES * 3];

/* Right triangular pyramid (equilateral base, side s > 0), apex above
 * the base centroid (s/2, √3/6·s, h), h > 0. Vertices 0..2 = A,B,C,
 * 3 = apex. Closed forms (density 1): V = √3/12·s²h,
 * com (s/2, √3/6·s, h/4), Ixx = Iyy = m(s²/40 + 3h²/80),
 * Izz = m·s²/20. h = √(2/3)·s is the REGULAR tetrahedron
 * (isotropic I = m·s²/20 — pinned by the gate). */
mig_hull_err mig_hull_build_pyr3_equilateral (double s, double h,
                                              mig_hull_point_set *out);
extern const int mig_hull_pyr3_faces [MIG_HULL_PYR3_FACES * 3];

/* Regular icosahedron, edge a > 0, centred at the origin: the classic
 * (0,±1,±φ) permutation vertex set scaled by a/2 (unscaled edge 2),
 * circumradius Rc = (a/2)·√((5+√5)/2), V = (5/12)(3+√5)a³,
 * I = m·a²(3+√5)/20 about every axis through the centre. */
mig_hull_err mig_hull_build_icosahedron (double a, mig_hull_point_set *out);
extern const int mig_hull_icosahedron_faces [MIG_HULL_ICOSA_FACES * 3];

#endif /* mig_hull_h */
