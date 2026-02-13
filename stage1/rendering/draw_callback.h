#include <stdio.h>
void draw_object (cairo_t *cr, rigidbody *b) {
    float focal_length = 400.0f; // Distance from eye to screen
    float z_depth = b->position.z + 100.0f; // Move objects in general away from camera POV
    if (z_depth <= 0) return; // Behind camera perception
    float screen_x = (b->position.x * focal_length) / z_depth + (SCREEN_WIDTH / 2);
    float screen_y = (b->position.y * focal_length) / z_depth + (SCREEN_HEIGHT / 2);
    // Scale size based on depth
    float projected_radius_view = (b->radius * focal_length) / z_depth;
    cairo_arc (cr, screen_x, screen_y, projected_radius, 0, 2 * M_PI);
    cairo_fill (cr);
}
