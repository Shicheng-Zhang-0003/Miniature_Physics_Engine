#ifndef timer_h
#define timer_h
#include <stdbool.h>
#include <gtk/gtk.h>
#include "../../stage1/master_header.h"
//To get a 60 FPS frame game, we will divide 1000 ms / 60 to get a frame render update of every 16 ms
typedef struct {
    rigidbody *objects;
    int number_objects;
    GtkWidget *widget;
} world_physics_setting;
static gboolean tick_function (gpointer user_data_pointer) {
    world_physics_setting *world_settings = (world_physics_setting*) user_data_pointer;
    float delta_time = 0.016; //Time stepping value
    //Application of constant forces
    //Gravity
    vector3 gravity_acceleration = {0, -9.81, 0};
    for (int object_index = 0; object_index < world_settings -> number_objects; object_index++) {force_applicant_gravity_normal (&world_settings -> objects [object_index], gravity_acceleration, (vector3) {0, 1, 0});}
    //Collision Resolve
    //Loop and call upon collusion_dual_sphere function
    for (int object_a_index = 0; object_a_index < world_settings -> number_objects; object_a_index++) {
        for (int object_b_index = object_a_index + 1; object_b_index < world_settings -> number_objects; object_b_index++) {
            collision_data collision;
            if (collision_dual_sphere (&world_settings -> objects [object_a_index], &world_settings -> objects [object_b_index], &collision)) {collision_resolve (&collision);}
        }
    } //Integration
    for (int object_index = 0; object_index < world_settings -> number_objects; object_index++) {
        //buffer.h update logic
        rb_integrate (&world_settings -> objects [object_index], delta_time);
    } //Clear screen using GTK drawing
    gtk_widget_queue_draw (world_settings -> widget); //Redraw all current objects
    return true; //Continue Timer runtime
}
#endif
