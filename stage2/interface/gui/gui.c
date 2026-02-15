#include <stdio.h>
#include <gtk/gtk.h>
#include <epoxy/gl.h>
static void on_point_render (GtkGLArea * area, GdkGlContext *context) {
    //Clear screen to a background that is black or dark in basic colour rendering
    glClearColor (0.1, 0.1, 0.1, 1.0);
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //Draw Actual Objects on screen
    for (int step1 = 0; step1 < num_objects; step1++) {draw_sphere (objects [i].position, objects [i].radius);} /*
                                                                                                                   math3D.h functions to perform mathematics computation
                                                                                                                   mesh transformed for rendering
                                                                                                                   rigidbody->position, rigidbody->orientation does the job here
                                                                                                                */
}

void activation (GtkApplication *application, gpointer user_data_main) {
    GtkWidget *window = gtk_application_window_new (app);
    gtk_window_set_title (GTK_WINDOW (window), "Stage 2 Testing Stage");
    gtk_window_set_default_size (GTK_WINDOW (window), 800, 600); //800x600 resolution
    //GL Canvas for presenting objects
    GtkWidget *gl_area_set = gtk_gl_area_new ();
    g_signal_connect (gl_area_set, "render", G_CALLBACK (on_render), NULL);
    gtk_container_add (GTK_CONTAINER (window), gl_area_set);
    //Present Window on System
    gtk_widget_show_all (window);
}
