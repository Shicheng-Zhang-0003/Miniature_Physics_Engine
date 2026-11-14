/* MIG WP4 4.6b: CAD window VIEW MATH — picking, camera, drag planes.
 *
 * Pure float math for the (later) GTK CAD window; it lives in cad/ next
 * to the design model so the SUITE can gate it headlessly — the window
 * TU itself is GUI-only and suite-unlinked (the 4.4 panel precedent),
 * but the math that decides which part you clicked must not be. Nothing
 * here mutates a cad_design or a physics_world; nothing here reads
 * config; every function is stateless.
 *
 * Conventions (engine-wide):
 *  - world/assembly frame is y-up, the floor plane is y = 0 (the grid
 *    the window draws and drags on);
 *  - quaternions are engine w-first (vector4 {w,x,y,z});
 *  - part orientations are unit (the design model validates them);
 *  - the cylinder's axle is LOCAL X (MPE_FTC_090);
 *  - degrees -> radians go through the house fixed double-pi literal
 *    (math3d.h MATH_PI), never the non-standard M_PI macro.
 *
 * Determinism: rotation uses vector4_rotate_to_vector3 (mul/add +
 * vector3_normalisation's sqrtf — exact IEEE ops) and libm sinf/cosf
 * for the orbit angles. This is click/authoring-path math (a human is
 * already holding the mouse), never the per-tick solver — the engine's
 * coordinate-trig ban is untouched, matching the 4.6 rotate verb's
 * documented cold-path ruling.
 */
#ifndef cad_view_math_h
#define cad_view_math_h
#include "../core/math3d.h"

/* ---- the orbit camera --------------------------------------------------
 * The window's camera orbits `target` at `distance`, y-up. yaw 0 /
 * pitch 0 puts the camera on the +Z axis looking down -Z at the target;
 * positive pitch lifts the eye (+Y); positive yaw swings the eye around
 * +Y toward +X. `pitch_deg` must stay within (-90, 90) — the CALLER
 * clamps (the window clamps to +-85); these functions are pure and do
 * not clamp. `fov_y_deg` is the vertical field of view, `aspect` is
 * width / height. */
typedef struct {
    vector3 target;
    float yaw_deg;
    float pitch_deg;
    float distance;
    float fov_y_deg;
    float aspect;
} cad_view_camera;

/* Unit direction the camera LOOKS (from the eye toward the target). */
vector3 cad_view_forward (const cad_view_camera *camera);

/* Screen-right unit vector (forward x world-up, orthogonalized). */
vector3 cad_view_right (const cad_view_camera *camera);

/* Camera-up unit vector (right x forward — orthogonal to both). */
vector3 cad_view_up (const cad_view_camera *camera);

/* Eye position: target - forward * distance. */
vector3 cad_view_eye (const cad_view_camera *camera);

/* ---- rays -------------------------------------------------------------- */

typedef struct {
    vector3 origin;    /* the eye */
    vector3 direction; /* UNIT */
} cad_view_ray;

/* Perspective unprojection: ndc_x/ndc_y in GL clip space [-1, 1] (x
 * right, y UP — the window converts from pixel coordinates, which have
 * y DOWN). The returned ray has origin = eye and a UNIT direction; the
 * edge rays spread by tan(fov/2) vertically and tan(fov/2) * aspect
 * horizontally. Degenerate camera (aspect <= 0, non-positive distance)
 * returns the center ray — the caller's camera is validated upstream. */
cad_view_ray cad_view_ray_from_screen (const cad_view_camera *camera,
                                       float ndc_x, float ndc_y);

/* A pick result: `hit` 0 = miss. `t` is the ray parameter of the FIRST
 * surface crossed (distance along the UNIT direction; 0 when the ray
 * starts inside the primitive), `point` and `normal` are in the
 * ASSEMBLY frame, normal unit and outward. `t` is exact ONLY for unit
 * directions — the contract below the pick functions. */
typedef struct {
    int hit;
    float t;
    vector3 point;
    vector3 normal;
} cad_view_hit;

/* Ray vs sphere (center, radius > 0). */
cad_view_hit cad_view_pick_sphere (cad_view_ray ray, vector3 center,
                                   float radius);

/* Ray vs a box (center, half extents > 0, part orientation). The slab
 * test runs in the part's LOCAL frame (orientation applied); an origin
 * inside the box hits with t = 0 and a zero normal. */
cad_view_hit cad_view_pick_box (cad_view_ray ray, vector3 center,
                                vector3 half_extensions, vector4 orientation);

/* Ray vs a cylinder (center, radius > 0, half_length > 0, part
 * orientation; axle LOCAL X — sides first, then the two end caps,
 * nearest surface wins). An origin inside the tube hits the far cap or
 * the far side with the nearest valid t. */
cad_view_hit cad_view_pick_cylinder (cad_view_ray ray, vector3 center,
                                     float radius, float half_length,
                                     vector4 orientation);

/* ---- work-plane drag --------------------------------------------------- */

/* Ray vs the plane through `point` with unit `normal`. Returns 1 and
 * writes `out` on a forward hit, 0 when the ray is parallel to the
 * plane or the plane is behind the ray (the window keeps the part in
 * place on a 0 return — no snapping, no mutation). */
int cad_view_ray_plane (cad_view_ray ray, vector3 point, vector3 normal,
                        vector3 *out);

#endif /* cad_view_math_h */
