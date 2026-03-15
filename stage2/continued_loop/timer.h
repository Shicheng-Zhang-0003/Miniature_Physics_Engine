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
static gboolean tick_function (gpointer data) {
    world_physics_setting *world = (world_physics_setting*) data;
    float dt = 0.016; //Time stepping value
    //Application of constant forces
    //Gravity
    vector3 gravity = {0, -9.81, 0};
    for (int step1 = 0; step1 < number_objects; step1++) {force_applicant_gravity_normal (&world->objects [step1], gravity, (vector3) {0, 1, 0});}
    //Collision Resolve
    //Loop and call upon collusion_dual_sphere function
    //Integration 
    for (int step2 = 0; step2 < number_objects; step2++) {
        //buffer.h update logic
        rb_integrate (&world->objects [step2], dt);
    } //Clear screen using GTK drawing
    gtk_widget_queue_draw (world->widget); //Redraw all current objects
    return TRUE; //Continue Timer runtime
} 
