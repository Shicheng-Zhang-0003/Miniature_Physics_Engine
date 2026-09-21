#ifndef boundary_h
#define boundary_h
#include "../core/math3d.h"
#include "../core/rigidbody.h"
#include "../config/mpe_config.h"

/* TRUTH: per-world config (NULL = global). The old signatures read g_cfg
 * directly, so foreign worlds with different floor_emergency_slop got the
 * wrong safety net. */
void boundary_apply_floor(rigidbody *rigid_body, float floor_y_coordinate);
void boundary_apply_floor_cfg(rigidbody *rigid_body, float floor_y_coordinate, const mpe_config_t *cfg);
void boundary_apply_box(rigidbody *rigid_body, vector3 minimum_bounds, vector3 maximum_bounds);
void boundary_apply_box_cfg(rigidbody *rigid_body, vector3 minimum_bounds, vector3 maximum_bounds,
                            const mpe_config_t *cfg);
#endif
