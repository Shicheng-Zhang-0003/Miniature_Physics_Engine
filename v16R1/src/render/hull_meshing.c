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
 * shapes so that is the natural edge set; the coarse 12×8 ell draws
 * its geodesic wire). UI layer: trig/sqrt are free here — the
 * ENGINE-coordinate ban is untouched (nothing here feeds the solver). */
#include "hull_meshing.h"
#include "../core/mig_hull.h"
#include "../config/mpe_constants.h"
#include <epoxy/gl.h>
#include <stdio.h>
#include <string.h>

mesh prism_mesh, pyr4_mesh, pyr3_mesh, icosa_mesh, ell_mesh;

/* Per-face-expanded float vertices (pos + normal, 6 floats): the
 * largest catalog mesh is the ell at 168 faces × 3 verts. */
#define hull_render_max_tri_verts (MIG_HULL_ELL_FACES * 3)
static float hull_scratch_verts [hull_render_max_tri_verts * 6];
static unsigned int hull_scratch_indices [hull_render_max_tri_verts];
/* Closed sphere-topology mesh: E = V + F − 2 (ell: 86 + 168 − 2). */
#define hull_render_max_edges (MIG_HULL_ELL_VERTS + MIG_HULL_ELL_FACES)
static unsigned int hull_scratch_wire [hull_render_max_edges * 2];
static unsigned char hull_edge_seen [MIG_HULL_ELL_VERTS][MIG_HULL_ELL_VERTS];

static void hull_mesh_upload (mesh *mesh_object, const mig_hull_point *points, int vert_count,
                              const int *faces, int face_count, const double com [3]) {
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
            hull_scratch_verts [tri_vert_count * 6 + 3] = (float) nx;
            hull_scratch_verts [tri_vert_count * 6 + 4] = (float) ny;
            hull_scratch_verts [tri_vert_count * 6 + 5] = (float) nz;
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
                      props.com);
    if (mig_hull_point_set_init (&set, scratch, MIG_HULL_MAX_VERTS) != mig_hull_ok ||
        mig_hull_build_pyr4_square (1.0, 1.0, &set) != mig_hull_ok ||
        mig_hull_mesh_props (set.points, set.count, mig_hull_pyr4_faces, MIG_HULL_PYR4_FACES, 1.0,
                             &props) != mig_hull_ok) {
        fprintf (stderr, "[hull_meshing] pyr4 unit mesh build failed (solid hulls will not draw)\n");
        return;
    }
    hull_mesh_upload (&pyr4_mesh, set.points, set.count, mig_hull_pyr4_faces, MIG_HULL_PYR4_FACES,
                      props.com);
    if (mig_hull_point_set_init (&set, scratch, MIG_HULL_MAX_VERTS) != mig_hull_ok ||
        mig_hull_build_pyr3_equilateral (1.0, 1.0, &set) != mig_hull_ok ||
        mig_hull_mesh_props (set.points, set.count, mig_hull_pyr3_faces, MIG_HULL_PYR3_FACES, 1.0,
                             &props) != mig_hull_ok) {
        fprintf (stderr, "[hull_meshing] pyr3 unit mesh build failed (solid hulls will not draw)\n");
        return;
    }
    hull_mesh_upload (&pyr3_mesh, set.points, set.count, mig_hull_pyr3_faces, MIG_HULL_PYR3_FACES,
                      props.com);
    if (mig_hull_point_set_init (&set, scratch, MIG_HULL_MAX_VERTS) != mig_hull_ok ||
        mig_hull_build_icosahedron (1.0, &set) != mig_hull_ok ||
        mig_hull_mesh_props (set.points, set.count, mig_hull_icosahedron_faces, MIG_HULL_ICOSA_FACES, 1.0,
                             &props) != mig_hull_ok) {
        fprintf (stderr, "[hull_meshing] icosa unit mesh build failed (solid hulls will not draw)\n");
        return;
    }
    hull_mesh_upload (&icosa_mesh, set.points, set.count, mig_hull_icosahedron_faces,
                      MIG_HULL_ICOSA_FACES, props.com);
    if (mig_hull_point_set_init (&set, scratch, MIG_HULL_MAX_VERTS) != mig_hull_ok ||
        mig_hull_build_ellipsoid (0.5, 0.5, 0.5, &set) != mig_hull_ok ||
        mig_hull_mesh_props (set.points, set.count, mig_hull_ell_faces, MIG_HULL_ELL_FACES, 1.0,
                             &props) != mig_hull_ok) {
        fprintf (stderr, "[hull_meshing] ell unit mesh build failed (solid hulls will not draw)\n");
        return;
    }
    hull_mesh_upload (&ell_mesh, set.points, set.count, mig_hull_ell_faces, MIG_HULL_ELL_FACES,
                      props.com);
}
