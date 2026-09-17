#include "../mpe_engine.h"
#include "object_selector.h"
#include <math.h>
int selected_object = -1;
uint32_t selected_object_id = 0;
// Exact OBB Raycast using the Slab Method
static bool ray_obb_intersection(vector3 ray_origin, vector3 ray_dir, rigidbody *obb, float *t_hit) {
    float tmin = -1e30f;
    float tmax = 1e30f;
    vector3 *axes = obb->cached_axes;
    float extents[3] = {obb->half_extensions.x, obb->half_extensions.y, obb->half_extensions.z};
    for (int i = 0; i < 3; i++) {
        float d = vector3_dot(axes[i], ray_dir);
        float e = vector3_dot(axes[i], vector3_subtraction(obb->position, ray_origin));
        if (fabsf(d) > math_epsilon) {
            float t1 = (e - extents[i]) / d;
            float t2 = (e + extents[i]) / d;
            if (t1 > t2) {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }
            if (t1 > tmin) {
                tmin = t1;
            }
            if (t2 < tmax) {
                tmax = t2;
            }
            if (tmin > tmax) {
                return false;
            }
        } else if ((-e > extents[i]) || (-e < -extents[i])) {
            return false;
        }
    }
    *t_hit = tmin > 0 ? tmin : tmax;
    return *t_hit > 0;
}
/* Exact solid-cylinder raycast (axle = local X, flat caps). Side quadric in
 * cylinder-local space plus two cap discs; nearest positive t wins. */
static bool ray_cylinder_intersection(vector3 ray_origin, vector3 ray_dir, rigidbody *cyl, float *t_hit) {
    vector3 ax = cyl->cached_axes[0];
    float ax_len_sq = vector3_length_squared(ax);
    if (ax_len_sq < 1e-8f) {
        return false;
    }
    ax = vector3_scaling(ax, 1.0f / sqrtf(ax_len_sq));
    float r = cyl->radius;
    float h = cyl->cylinder_half_length;
    vector3 oc = vector3_subtraction(ray_origin, cyl->position);
    float dx = vector3_dot(ray_dir, ax);
    float ox = vector3_dot(oc, ax);
    vector3 rd_perp = vector3_subtraction(ray_dir, vector3_scaling(ax, dx));
    vector3 oc_perp = vector3_subtraction(oc, vector3_scaling(ax, ox));
    float best_t = 1e30f;
    bool hit = false;
    /* Barrel side: |oc_perp + t*rd_perp|^2 = r^2 with |ox + t*dx| <= h. */
    float a = vector3_length_squared(rd_perp);
    if (a > 1e-12f) {
        float b = 2.0f * vector3_dot(oc_perp, rd_perp);
        float c = vector3_length_squared(oc_perp) - r * r;
        float disc = b * b - 4.0f * a * c;
        if (disc >= 0.0f) {
            float sq = sqrtf(disc);
            float t_candidates[2] = {(-b - sq) / (2.0f * a), (-b + sq) / (2.0f * a)};
            for (int k = 0; k < 2; k++) {
                float t = t_candidates[k];
                if (t > 0.0f && t < best_t && fabsf(ox + t * dx) <= h) {
                    best_t = t;
                    hit = true;
                }
            }
        }
    }
    /* Flat caps: planes x = +/-h, radial check. */
    if (fabsf(dx) > 1e-9f) {
        for (int s = -1; s <= 1; s += 2) {
            float t = (s * h - ox) / dx;
            if (t > 0.0f && t < best_t) {
                vector3 p = vector3_addition(oc, vector3_scaling(ray_dir, t));
                vector3 radial = vector3_subtraction(p, vector3_scaling(ax, s * h));
                if (vector3_length_squared(radial) <= r * r) {
                    best_t = t;
                    hit = true;
                }
            }
        }
    }
    if (hit) {
        *t_hit = best_t;
    }
    return hit;
}
void select_object_by_index(int object_index) {
    if ((object_index < 0) || (object_index >= (physics_world_get_primary()->body_count))) {
        clear_selection();
        return;
    }

    selected_object = object_index;
    selected_object_id = (physics_world_get_primary()->bodies)[object_index].object_id;
}

void selection_validate(void) {
    if (selected_object_id == 0) {
        clear_selection();
        return;
    }

    int object_index = scene_find_object_index_by_id(selected_object_id);

    if (object_index < 0) {
        clear_selection();
        return;
    }

    selected_object = object_index;
}

uint32_t selection_get_id(void) {
    return selected_object_id;
}

int selector_ray_tracing(void) {
    vector3 ray_origin_position = main_camera_fov.position;
    vector3 ray_direction_vector = vector3_normalisation(main_camera_fov.forward_vector);
    float closest_hit_distance = 1e30f;
    int closest_object_index = -1;
    for (int object_index = 0; object_index < (physics_world_get_primary()->body_count); object_index++) {
        rigidbody *rigid_body_pointer = &(physics_world_get_primary()->bodies)[object_index];
        float t_hit = 0.0f;
        bool hit = false;
        if (rigid_body_pointer->type == object_sphere) {
            vector3 origin_to_center_vector = vector3_subtraction(rigid_body_pointer->position, ray_origin_position);
            float projection_length_along_ray = vector3_dot(origin_to_center_vector, ray_direction_vector);
            if (projection_length_along_ray < 0) {
                continue;
            }
            vector3 closest_point_on_ray_vector = vector3_scaling(ray_direction_vector, projection_length_along_ray);
            vector3 perpendicular_displacement_vector =
                vector3_subtraction(origin_to_center_vector, closest_point_on_ray_vector);
            float perpendicular_distance_squared = vector3_length_squared(perpendicular_displacement_vector);
            if (perpendicular_distance_squared <= (rigid_body_pointer->radius * rigid_body_pointer->radius)) {
                hit = true;
                t_hit = projection_length_along_ray;
            }
        } else if (rigid_body_pointer->type == object_cylinder) {
            hit = ray_cylinder_intersection(ray_origin_position, ray_direction_vector, rigid_body_pointer, &t_hit);
        } else {
            hit = ray_obb_intersection(ray_origin_position, ray_direction_vector, rigid_body_pointer, &t_hit);
        }
        if (hit) {
            if (t_hit < closest_hit_distance) {
                closest_hit_distance = t_hit;
                closest_object_index = object_index;
            }
        }
    }
    select_object_by_index(closest_object_index);
    return closest_object_index;
}
void clear_selection(void) {
    selected_object = -1;
    selected_object_id = 0;
}
void selector_apply_force_impulse(float impulse_magnitude) {
    selection_validate();
    if ((selected_object < 0) || (selected_object >= (physics_world_get_primary()->body_count))) {
        return;
    }
    rigidbody *selected_rigid_body = &(physics_world_get_primary()->bodies)[selected_object];
    vector3 applied_impulse_vector = vector3_scaling(main_camera_fov.forward_vector, impulse_magnitude);
    rb_apply_forces_perfect(selected_rigid_body, applied_impulse_vector);
}
