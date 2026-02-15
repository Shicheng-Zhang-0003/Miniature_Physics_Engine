//#include "../..master_header.h" 
//For when actually integrated with stage 1
#include <gtk/gtk.h>

//To get a 60 FPS frame game, we will divide 1000 ms / 60 to get a frame render update of every 16 ms
static gboolean tick_function (gpointer data) {
    rigidbody *objects = (rigidbody*) data;
    float dt = 0.016; //Time stepping value
    //Application of constant forces
    //Gravity
    vector3 gravity = {0, -9.81, 0};
    for (int step1; step1 < num_objects; step1++) {force_applicant_gravity_normal (&objects [step1], gravity, (vector3) {0, 1, 0});}
    //Collision Resolve
    //Loop and call upon collusion_dual_sphere function
    //Integration 
    for (int step2 = 0; step2 < num_objects; step2++) {
        //buffer.h update logic
        //update_rigidbody (&objects [step2], dt);
    }//Clear screen using GTK drawing
    gtk_widget_queue_draw (GTK_WIDGET (objects->user_data_widget)); //Redraw all current objects
    return TRUE; //Continue Timer runtime
} 
