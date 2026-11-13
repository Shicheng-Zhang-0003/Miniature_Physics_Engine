#ifndef hull_meshing_h
#define hull_meshing_h
#include "sphere_meshing.h" /* mesh struct */
/* MIG WP2.6 2.6.6: renderer unit meshes for the five catalog hull
 * kinds (prism, pyr4, pyr3, icosa, ell). Built by hull_meshing_init
 * from the SAME mig_hull builders the collision pools use — the unit
 * COM-frame shape at s=h=1 (the ell at semi-axes 0.5, i.e. unit
 * diameters) — so a body's T*R*S with its stamped (s, s, h) lands the
 * drawn surface exactly on the stored collision surface. The
 * ellipsoid reuses no sphere mesh: its collision shape is the fixed
 * 12×8 UV polyhedron, so that exact polyhedron is what gets drawn. */
extern mesh prism_mesh;
extern mesh pyr4_mesh;
extern mesh pyr3_mesh;
extern mesh icosa_mesh;
extern mesh ell_mesh;
void hull_meshing_init (void);
#endif
