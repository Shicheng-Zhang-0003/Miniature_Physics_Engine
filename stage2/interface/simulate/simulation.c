#include "../../master_header_2.h"
#include "../../../stage1/master_header.h"
#include <gtk/gtk.h>
#include <stdbool.h>

//World Status right now
rigidbody obj_per_scene [10];
int object_count = 0;
static gboolean physics_step_increment (gpointer user_data_stored) {
    float dt = 0.016; // 1 / 60 s increment, 16 ms 
    for (int step = 0; step < object_count; step++) {
        //Apply forces nominally
        //Gravity
        vector3 gravity = {0, -9.81, 0};
        force_applicant_gravity_normal (&obj_per_scene [step], gravity, (vector3) {0, 1, 0});
        //Integrate motion (buffer.h, stage 1)
        rb_integrate (&obj_per_scene [step], dt);
    } //Resolve Collision Sequence (collision_mechanics.h, stage 1)
    for (int step2 = 0; step2 < object_countl step2++) {
        for (int step3 = step2 + 1; step3 < object_count; step3++) {
            collision_data collision;
            if (collision_dual_sphere(&obj_per_scene [step2], &obj_per_scene [step3], &collision)) {collision_resolve (&collision);}
        }
    } //GTK redraw the window
    gtk_widget_queue_draw (GTK_WIDGET (user_data_stored));
    return TRUE;
}
