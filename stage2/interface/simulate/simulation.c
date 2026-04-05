#include "../../master_header_2.h"
#include "../../../stage3/master_header_3.h"
#include "../../../stage1/master_header.h"
#include <gtk/gtk.h>
#include <stdbool.h>
//World Status right now
frame_timer main_timer;
rigidbody *obj_per_scene = NULL;
int object_count = 0;
int object_capacity = 0;
gboolean physics_step_increment (gpointer user_data_stored) {
    frame_timer_update (&main_timer);
    float dt = main_timer.delta_time;
    if (main_inputs.w_key) {camera_move_w (&main_camera_fov, dt);}
    if (main_inputs.a_key) {camera_move_a (&main_camera_fov, dt);}
    if (main_inputs.s_key) {camera_move_s (&main_camera_fov, dt);}
    if (main_inputs.d_key) {camera_move_d (&main_camera_fov, dt);}
    if (main_inputs.space_key) {
        spawner_launch_sphere (0.5, 1.0, 20.0);
        main_inputs.space_key = false;
    } //Apply forces to all objects within the simulation
    for (int step = 0; step < object_count; step++) {
        vector3 gravity = {0, -9.81, 0};
        force_applicant_gravity_normal (&obj_per_scene [step], gravity, (vector3) {0.0, 1.0, 0.0});
        //Add Forces Later on Input and ETC
    } //Resolve collisions between objects
    for (int step2 = 0; step2 < object_count; step2++) {
        for (int step3 = step2 + 1; step3 < object_count; step3++) {
            collision_data collision;
            if (collision_dual_sphere (&obj_per_scene [step2], &obj_per_scene [step3], &collision)) {collision_resolve (&collision);}
        }
    } //Integrate all objects into render
    for (int step4 = 0; step4 < object_count; step4++) {rb_integrate (&obj_per_scene [step4], dt);} //GTK redraw the window
    for (int step5 = 0; step5 < object_count; step5++) {boundary_apply_floor (&obj_per_scene [step5], 0.0f);}
    gtk_widget_queue_draw (GTK_WIDGET (user_data_stored));
    mouse_lock_reset_centre (GTK_WIDGET (user_data_stored));
    return TRUE;
}
