#include "../../master_header_2.h"
#include "../../../stage1/master_header.h"
#include <complex.h>
#include <gtk/gtk.h>
#include <stdbool.h>
//World Status right now
rigidbody obj_per_scene [10];
int object_count = 0;
gboolean physics_step_increment (gpointer user_data_stored) {
    float dt = 0.016; // 1 / 60 s increment, 16 ms 
    if (main_inputs.w_key) {camera_move_w (&main_camera_fov, dt);}
    if (main_inputs.a_key) {camera_move_a (&main_camera_fov, dt);}
    if (main_inputs.s_key) {camera_move_s (&main_camera_fov, dt);}
    if (main_inputs.d_key) {camera_move_d (&main_camera_fov, dt);}
    //Apply forces to all objects within the simulation
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
    gtk_widget_queue_draw (GTK_WIDGET (user_data_stored));
    return TRUE;
}
