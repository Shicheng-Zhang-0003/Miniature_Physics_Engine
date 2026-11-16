/* MIG WP2.6 2.6.6: renderer unit meshes for the five catalog hull
 * kinds. Built at init from the SAME mig_hull builders the collision
 * pools use (prism/pyr4/pyr3 at s=h=1; icosahedron at edge 1; the ell
 * at semi-axes 0.5 = unit diameters), then rebased to the COM frame
 * exactly like add_hull's rebase. Affine covariance does the rest:
 * stored verts == unit verts · (s, s, h), so the body's T*R*S model
 * matrix reproduces the collision surface up to float display
 * rounding. Face tables are flat triangle indices, already
 * outward-wound (right-hand rule) — face normals come from the
 * winding via the cross product. Wireframe EBO = every unique
 * undirected edge of the mesh (the catalog polyhedra are all-crease
 * shapes so that is the natural edge set; the ell draws its geodesic
 * wire). WP4 4.6q: the ell kind renders a SMOOTH 32×16 parametric
 * mesh with per-vertex normals (= the unit-sphere direction) instead
 * of the faceted 12×8 catalog build, so a scaled instance shades as
 * the true ellipsoid. UI layer: trig/sqrt are free here — the
 * ENGINE-coordinate ban is untouched (nothing here feeds the solver). */
#include "hull_meshing.h"
#include "../core/mig_hull.h"
#include "../config/mpe_constants.h"
#include <epoxy/gl.h>
#include <stdio.h>
#include <string.h>

mesh prism_mesh, pyr4_mesh, pyr3_mesh, icosa_mesh, ell_mesh;

/* Per-face-expanded float vertices (pos + normal, 6 floats): the
 * largest render mesh is the WP4 4.6q smooth ell (960 tris); the
 * collision catalog's own meshes stay at or under the 2.6.3 ell
 * (168 faces × 3 verts), so the scratch sizes to the bigger family. */
#define hull_ell_render_verts 482  /* 32-slice × 16-stack unit sphere */
#define hull_ell_render_faces 960
#define hull_render_max_verts ((hull_ell_render_verts > MIG_HULL_ELL_VERTS) ? hull_ell_render_verts : MIG_HULL_ELL_VERTS)
#define hull_render_max_faces ((hull_ell_render_faces > MIG_HULL_ELL_FACES) ? hull_ell_render_faces : MIG_HULL_ELL_FACES)
#define hull_render_max_tri_verts (hull_render_max_faces * 3)
static float hull_scratch_verts [hull_render_max_tri_verts * 6];
static unsigned int hull_scratch_indices [hull_render_max_tri_verts];
/* Closed sphere-topology mesh: E = V + F − 2 (4.6q ell: 482 + 960 − 2). */
#define hull_render_max_edges (hull_render_max_verts + hull_render_max_faces)
static unsigned int hull_scratch_wire [hull_render_max_edges * 2];
static unsigned char hull_edge_seen [hull_render_max_verts][hull_render_max_verts];

static void hull_mesh_upload (mesh *mesh_object, const mig_hull_point *points, int vert_count,
                              const int *faces, int face_count, const double com [3],
                              const double (*vert_normals) [3]) {
    int tri_vert_count = 0;
    for (int face_index = 0; face_index < face_count; face_index++) {
        const int ia = faces [face_index * 3], ib = faces [face_index * 3 + 1],
                  ic = faces [face_index * 3 + 2];
        if ((ia < 0) || (ib < 0) || (ic < 0) || (ia >= vert_count) || (ib >= vert_count) ||
            (ic >= vert_count)) {
            fprintf (stderr, "[hull_meshing] face index outside the vertex array — mesh not built\n");
            return;
        }
        const double ax = points [ia].x - com [0], ay = points [ia].y - com [1],
                     az = points [ia].z - com [2];
        const double bx = points [ib].x - com [0], by = points [ib].y - com [1],
                     bz = points [ib].z - com [2];
        const double cx = points [ic].x - com [0], cy = points [ic].y - com [1],
                     cz = points [ic].z - com [2];
        /* Outward face normal from the outward winding (u×v). */
        const double ux = bx - ax, uy = by - ay, uz = bz - az;
        const double vx = cx - ax, vy = cy - ay, vz = cz - az;
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const double nlen = sqrt (nx * nx + ny * ny + nz * nz);
        if (nlen > 0.0) { nx /= nlen; ny /= nlen; nz /= nlen; }
        const double px [3] = {ax, bx, cx}, py [3] = {ay, by, cy}, pz [3] = {az, bz, cz};
        for (int corner = 0; corner < 3; corner++) {
            hull_scratch_verts [tri_vert_count * 6 + 0] = (float) px [corner];
            hull_scratch_verts [tri_vert_count * 6 + 1] = (float) py [corner];
            hull_scratch_verts [tri_vert_count * 6 + 2] = (float) pz [corner];
            /* WP4 4.6q: a per-vertex normal table (the smooth ell) wins
             * over the face normal when given. */
            if (vert_normals) {
                hull_scratch_verts [tri_vert_count * 6 + 3] = (float) vert_normals [faces [face_index * 3 + corner]][0];
                hull_scratch_verts [tri_vert_count * 6 + 4] = (float) vert_normals [faces [face_index * 3 + corner]][1];
                hull_scratch_verts [tri_vert_count * 6 + 5] = (float) vert_normals [faces [face_index * 3 + corner]][2];
            } else {
                hull_scratch_verts [tri_vert_count * 6 + 3] = (float) nx;
                hull_scratch_verts [tri_vert_count * 6 + 4] = (float) ny;
                hull_scratch_verts [tri_vert_count * 6 + 5] = (float) nz;
            }
            hull_scratch_indices [tri_vert_count] = (unsigned int) tri_vert_count;
            tri_vert_count++;
        }
    }
    /* Wireframe: every unique undirected edge (lo, hi) of the mesh. */
    memset (hull_edge_seen, 0, sizeof (hull_edge_seen));
    int wire_count = 0;
    for (int face_index = 0; face_index < face_count; face_index++) {
        const int corner [3] = {faces [face_index * 3], faces [face_index * 3 + 1],
                                faces [face_index * 3 + 2]};
        for (int e = 0; e < 3; e++) {
            const int lo = corner [e] < corner [(e + 1) % 3] ? corner [e] : corner [(e + 1) % 3];
            const int hi = corner [e] < corner [(e + 1) % 3] ? corner [(e + 1) % 3] : corner [e];
            if (hull_edge_seen [lo][hi]) { continue; }
            hull_edge_seen [lo][hi] = 1;
            hull_scratch_wire [wire_count++] = (unsigned int) lo;
            hull_scratch_wire [wire_count++] = (unsigned int) hi;
        }
    }
    glGenVertexArrays (1, &mesh_object -> vertex_array_object);
    glGenBuffers (1, &mesh_object -> vertex_buffer_object);
    glGenBuffers (1, &mesh_object -> element_buffer_object);
    glBindVertexArray (mesh_object -> vertex_array_object);
    glBindBuffer (GL_ARRAY_BUFFER, mesh_object -> vertex_buffer_object);
    glBufferData (GL_ARRAY_BUFFER, (GLsizeiptr) tri_vert_count * 6 * sizeof (float),
                  hull_scratch_verts, GL_STATIC_DRAW);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, mesh_object -> element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr) tri_vert_count * sizeof (unsigned int),
                  hull_scratch_indices, GL_STATIC_DRAW);
    glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof (float), (void *) 0);
    glEnableVertexAttribArray (0);
    glVertexAttribPointer (1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof (float), (void *) (3 * sizeof (float)));
    glEnableVertexAttribArray (1);
    glGenBuffers (1, &mesh_object -> wireframe_element_buffer_object);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, mesh_object -> wireframe_element_buffer_object);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr) wire_count * sizeof (unsigned int),
                  hull_scratch_wire, GL_STATIC_DRAW);
    glGenBuffers (1, &mesh_object -> instance_vbo);
    glBindBuffer (GL_ARRAY_BUFFER, mesh_object -> instance_vbo);
    glBufferData (GL_ARRAY_BUFFER, mpe_max_bodies * 19 * sizeof (float), NULL, GL_DYNAMIC_DRAW);
    for (int i = 0; i < 4; i++) {
        glVertexAttribPointer (2 + i, 4, GL_FLOAT, GL_FALSE, 19 * sizeof (float),
                               (void *) (i * 4 * sizeof (float)));
        glEnableVertexAttribArray (2 + i);
        glVertexAttribDivisor (2 + i, 1);
    } glVertexAttribPointer (6, 3, GL_FLOAT, GL_FALSE, 19 * sizeof (float), (void *) (16 * sizeof (float)));
    glEnableVertexAttribArray (6);
    glVertexAttribDivisor (6, 1);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, mesh_object -> element_buffer_object);
    glBindVertexArray (0);
    mesh_object -> index_count = tri_vert_count;
    mesh_object -> wireframe_index_count = wire_count;
}

/* ---- MIG WP4 4.6q: the smooth parametric ell unit mesh ----
 * The catalog's 12×8 inscribed ellipsoid reads as a faceted 86-vertex
 * polyhedron; the RENDER kind draws this instead: a 32-slice ×
 * 16-stack parametric unit sphere at semi-axes 0.5 (unit
 * equatorial/polar DIAMETERS — the same (s, s, h) scale convention),
 * with SMOOTH per-vertex normals = the unit-sphere direction
 * (position / 0.5). The main shader de-skews normals under the
 * instance's non-uniform scale (A3_PATCH_29_SHADER_NORMALS:
 * normalize(rot · (n / scale))), and a sphere's normal IS its
 * position direction, so the shaded surface is the exact ellipsoid.
 * Layout: vert 0 = north pole; ring j (colatitude φ_j = j·π/16,
 * j = 1..15) = 32 verts at 1 + (j−1)·32 + i; vert 481 = south pole.
 * Faces 960 = 32 north-fan + 14 bands × 64 + 32 south-fan. Winding
 * (verified numerically): θ east (x→y), φ north→south; north fan
 * N→r1[i]→r1[i+1]; bands (v(j,i), v(j+1,i), v(j+1,i+1)) and
 * (v(j,i), v(j+1,i+1), v(j,i+1)); south fan S→r15[i+1]→r15[i]. */
#define HULL_ELL_RENDER_SLICES 32
#define HULL_ELL_RENDER_STACKS 16
static void hull_mesh_upload_ell (mesh *mesh_object) {
    static mig_hull_point verts [hull_ell_render_verts];
    static int faces [hull_ell_render_faces * 3];
    static double normals [hull_ell_render_verts] [3];
    const double r = 0.5;
    int vc = 0;
    normals [vc][0] = 0.0; normals [vc][1] = 0.0; normals [vc][2] = 1.0;
    verts [vc].x = 0.0; verts [vc].y = 0.0; verts [vc].z = r;
    vc++; /* north pole */
    for (int j = 1; j < HULL_ELL_RENDER_STACKS; j++) {
        const double phi = MATH_PI * (double) j / (double) HULL_ELL_RENDER_STACKS;
        const double sp = sin (phi), cp = cos (phi);
        for (int i = 0; i < HULL_ELL_RENDER_SLICES; i++) {
            const double theta = 2.0 * MATH_PI * (double) i / (double) HULL_ELL_RENDER_SLICES;
            const double nx = sp * cos (theta), ny = sp * sin (theta), nz = cp;
            verts [vc].x = r * nx; verts [vc].y = r * ny; verts [vc].z = r * nz;
            normals [vc][0] = nx; normals [vc][1] = ny; normals [vc][2] = nz;
            vc++;
        }
    }
    normals [vc][0] = 0.0; normals [vc][1] = 0.0; normals [vc][2] = -1.0;
    verts [vc].x = 0.0; verts [vc].y = 0.0; verts [vc].z = -r;
    vc++; /* south pole */
    int fc = 0;
    const int south = hull_ell_render_verts - 1;
    /* Ring vertex index: ring j (1..15), slice i (wraps mod 32). */
    #define HULL_ELL_RING_V(j, i) (1 + ((j) - 1) * HULL_ELL_RENDER_SLICES + ((i) % HULL_ELL_RENDER_SLICES))
    for (int i = 0; i < HULL_ELL_RENDER_SLICES; i++) {
        faces [fc++] = 0;
        faces [fc++] = HULL_ELL_RING_V (1, i);
        faces [fc++] = HULL_ELL_RING_V (1, i + 1);
    }
    for (int j = 1; j < HULL_ELL_RENDER_STACKS - 1; j++) {
        for (int i = 0; i < HULL_ELL_RENDER_SLICES; i++) {
            const int v00 = HULL_ELL_RING_V (j, i), v10 = HULL_ELL_RING_V (j + 1, i);
            const int v01 = HULL_ELL_RING_V (j, i + 1), v11 = HULL_ELL_RING_V (j + 1, i + 1);
            faces [fc++] = v00; faces [fc++] = v10; faces [fc++] = v11;
            faces [fc++] = v00; faces [fc++] = v11; faces [fc++] = v01;
        }
    }
    for (int i = 0; i < HULL_ELL_RENDER_SLICES; i++) {
        faces [fc++] = south;
        faces [fc++] = HULL_ELL_RING_V (HULL_ELL_RENDER_STACKS - 1, i + 1);
        faces [fc++] = HULL_ELL_RING_V (HULL_ELL_RENDER_STACKS - 1, i);
    }
    #undef HULL_ELL_RING_V
    /* The build is symmetric about the origin: COM = (0, 0, 0). */
    const double com [3] = {0.0, 0.0, 0.0};
    hull_mesh_upload (mesh_object, verts, vc, faces, fc, com, (const double (*) [3]) normals);
}

void hull_meshing_init (void) {
    mig_hull_point scratch [MIG_HULL_MAX_VERTS];
    mig_hull_point_set set;
    mig_hull_mass props;
    /* One build recipe per kind: (builder call, face table, face count).
     * The ell builds at semi-axes 0.5 so the UNIT shape carries unit
     * equatorial/polar DIAMETERS — the same (s, s, h) scale convention
     * as the extrusion family. */
    if (mig_hull_point_set_init (&set, scratch, MIG_HULL_MAX_VERTS) != mig_hull_ok ||
        mig_hull_build_prism_equilateral (1.0, 1.0, &set) != mig_hull_ok ||
        mig_hull_mesh_props (set.points, set.count, mig_hull_prism_faces, MIG_HULL_PRISM_FACES, 1.0,
                             &props) != mig_hull_ok) {
        fprintf (stderr, "[hull_meshing] prism unit mesh build failed (solid hulls will not draw)\n");
        return;
    }
    hull_mesh_upload (&prism_mesh, set.points, set.count, mig_hull_prism_faces, MIG_HULL_PRISM_FACES,
                      props.com, NULL);
    if (mig_hull_point_set_init (&set, scratch, MIG_HULL_MAX_VERTS) != mig_hull_ok ||
        mig_hull_build_pyr4_square (1.0, 1.0, &set) != mig_hull_ok ||
        mig_hull_mesh_props (set.points, set.count, mig_hull_pyr4_faces, MIG_HULL_PYR4_FACES, 1.0,
                             &props) != mig_hull_ok) {
        fprintf (stderr, "[hull_meshing] pyr4 unit mesh build failed (solid hulls will not draw)\n");
        return;
    }
    hull_mesh_upload (&pyr4_mesh, set.points, set.count, mig_hull_pyr4_faces, MIG_HULL_PYR4_FACES,
                      props.com, NULL);
    if (mig_hull_point_set_init (&set, scratch, MIG_HULL_MAX_VERTS) != mig_hull_ok ||
        mig_hull_build_pyr3_equilateral (1.0, 1.0, &set) != mig_hull_ok ||
        mig_hull_mesh_props (set.points, set.count, mig_hull_pyr3_faces, MIG_HULL_PYR3_FACES, 1.0,
                             &props) != mig_hull_ok) {
        fprintf (stderr, "[hull_meshing] pyr3 unit mesh build failed (solid hulls will not draw)\n");
        return;
    }
    hull_mesh_upload (&pyr3_mesh, set.points, set.count, mig_hull_pyr3_faces, MIG_HULL_PYR3_FACES,
                      props.com, NULL);
    if (mig_hull_point_set_init (&set, scratch, MIG_HULL_MAX_VERTS) != mig_hull_ok ||
        mig_hull_build_icosahedron (1.0, &set) != mig_hull_ok ||
        mig_hull_mesh_props (set.points, set.count, mig_hull_icosahedron_faces, MIG_HULL_ICOSA_FACES, 1.0,
                             &props) != mig_hull_ok) {
        fprintf (stderr, "[hull_meshing] icosa unit mesh build failed (solid hulls will not draw)\n");
        return;
    }
    hull_mesh_upload (&icosa_mesh, set.points, set.count, mig_hull_icosahedron_faces,
                      MIG_HULL_ICOSA_FACES, props.com, NULL);
    /* MIG WP4 4.6q: the ell kind draws the SMOOTH parametric unit mesh
     * (32×16, per-vertex normals) instead of the catalog's faceted 12×8
     * inscribed build — the collision pool keeps the 2.6.3 mesh. */
    hull_mesh_upload_ell (&ell_mesh);
}
