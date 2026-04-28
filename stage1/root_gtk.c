#include <complex.h>
#include <gtk/gtk.h>
#include "../stage2/master_header_2.h"
#include "../stage3/master_header_3.h"
#include "../stage4/master_header_4.h"
#include "../stage5/master_header_5.h"
#include "master_header.h"
camera main_camera_fov;
input_status main_inputs;
//On Call
static void when_realised (GtkGLArea *gl_area_widget) {
    if (gtk_gl_area_get_error (gl_area_widget) != NULL) {return;}
    gtk_gl_area_make_current (gl_area_widget);
    //Init OpenGL Status
    glEnable (GL_DEPTH_TEST); //Test Depth Signal
    render_init ();
    //Scene Init and Mouse Lock (On Realize)
    scene_init_default ();
    GtkWidget *parent_window_container = gtk_widget_get_toplevel (GTK_WIDGET (gl_area_widget));
    mouse_lock_enable (parent_window_container);
} //On render: Screen Make
static gboolean on_rendered (GtkGLArea *gl_area_widget, GdkGLContext *gl_context_data) {
    int widget_allocated_width = gtk_widget_get_allocated_width (GTK_WIDGET (gl_area_widget));
    int widget_allocated_height = gtk_widget_get_allocated_height (GTK_WIDGET (gl_area_widget));
    render_scene_current (widget_allocated_width, widget_allocated_height);
    return TRUE;
} int main_algorithm (int argc, char *argv []);
int main_algorithm (int argc, char *argv []) {
    gtk_init (&argc, &argv);
    //Camera Init
    initalise_camera (&main_camera_fov, (vector3) {0.0, 0.0, 50.0});
    initialise_input (&main_inputs);
    //Widgeting
    GtkWidget *main_window_container = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget *gl_drawing_area_widget = gtk_gl_area_new ();
    //Keyboard and Mouse Events
    gtk_widget_add_events (main_window_container, GDK_KEY_PRESS_MASK | GDK_KEY_RELEASE_MASK | GDK_POINTER_MOTION_MASK);
    //Signalling
    g_signal_connect (gl_drawing_area_widget, "render", G_CALLBACK (on_rendered), NULL);
    g_signal_connect (gl_drawing_area_widget, "realize", G_CALLBACK (when_realised), NULL);
    g_signal_connect (main_window_container, "key-press-event", G_CALLBACK (on_keypress), &main_inputs);
    g_signal_connect (main_window_container, "key-release-event", G_CALLBACK (on_key_released), &main_inputs);
    g_signal_connect (main_window_container, "motion-notify-event", G_CALLBACK (on_mouse_movements), NULL);
    //Add Objects
    gtk_container_add (GTK_CONTAINER (main_window_container), gl_drawing_area_widget);
    overlay_init (main_window_container, gl_drawing_area_widget);
    //Physics Step Loop (16ms)
    g_timeout_add (16, physics_step_increment, gl_drawing_area_widget);
    //Show Window
    gtk_widget_show_all (main_window_container);
    frame_timer_init (&main_timer);
    joint_init_pool ();
    gtk_main ();
    return 0;
} int main (int argc, char *argv []) {
    main_algorithm (argc, argv);
    return 0;
}

//How to Pass Camera FOV View to GPU
// Inside render loop
//float view_matrix [16];
//camera_get_view_matrix (&my_camera, view_matrix);

// Send it to shader unit
//GLuint viewLoc = glGetUniformLocation (shader_program, "view");
//glUniformMatrix4fv (viewLoc, 1, GL_FALSE, view_matrix);
