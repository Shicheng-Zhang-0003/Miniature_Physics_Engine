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
    //Scene Init (On Realize)
    scene_init_default ();
} //On render: Screen Make
static gboolean on_rendered (GtkGLArea *gl_area_widget, GdkGLContext *gl_context_data) {
    (void) gl_context_data;
    int screen_scale_factor = gtk_widget_get_scale_factor (GTK_WIDGET (gl_area_widget));
    int widget_width = gtk_widget_get_allocated_width (GTK_WIDGET (gl_area_widget)) * screen_scale_factor;
    int widget_height = gtk_widget_get_allocated_height (GTK_WIDGET (gl_area_widget)) * screen_scale_factor;
    render_scene_current (widget_width, widget_height);
    return TRUE;
} int main_algorithm (int argc, char *argv []);
int main_algorithm (int argc, char *argv []) {
    g_setenv ("GDK_BACKEND", "x11", FALSE);
    gtk_init (&argc, &argv);
    //Camera Init
    initalise_camera (&main_camera_fov, (vector3) {0.0f, 20.0f, 50.0f});
    initialise_input (&main_inputs);
    //Widgeting
    GtkWidget *main_window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget *gl_area_widget = gtk_gl_area_new ();
    //Keyboard and Mouse Events
    gtk_widget_add_events (main_window, GDK_KEY_PRESS_MASK | GDK_KEY_RELEASE_MASK | GDK_POINTER_MOTION_MASK | GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK);
    //Signalling
    g_signal_connect (gl_area_widget, "render", G_CALLBACK (on_rendered), NULL);
    g_signal_connect (gl_area_widget, "realize", G_CALLBACK (when_realised), NULL);
    g_signal_connect (main_window, "key-press-event", G_CALLBACK (on_keypress), &main_inputs);
    g_signal_connect (main_window, "key-release-event", G_CALLBACK (on_key_released), &main_inputs);
    g_signal_connect (main_window, "motion-notify-event", G_CALLBACK (on_mouse_movements), NULL);
    g_signal_connect (main_window, "button-press-event", G_CALLBACK (on_button_press), &main_inputs);
    g_signal_connect (main_window, "button-release-event", G_CALLBACK (on_button_release), &main_inputs);
    //Add Objects
    GtkWidget *ui_overlay_layout = overlay_init (gl_area_widget);
    gtk_container_add (GTK_CONTAINER (main_window), ui_overlay_layout);
    //Focus and Event Catching
    gtk_widget_set_can_focus (main_window, TRUE);
    gtk_widget_grab_focus (main_window);
    //Physics Step Loop (16ms)
    g_timeout_add (16, physics_step_increment, gl_area_widget);
    //Show Window
    gtk_widget_show_all (main_window);
    frame_timer_init (&main_timer);
    joint_init_pool ();
    gtk_main ();
    return 0;
} int main (int argc, char *argv []) {
    main_algorithm (argc, argv);
    return 0;
} //How to Pass Camera FOV View to GPU
// Inside render loop
//float view_matrix [16];
//camera_get_view_matrix (&my_camera, view_matrix);
// Send it to shader unit
//GLuint viewLoc = glGetUniformLocation (shader_program, "view");
//glUniformMatrix4fv (viewLoc, 1, GL_FALSE, view_matrix);
