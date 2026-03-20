#include <stdio.h>
#include <gtk/gtk.h>
#include <epoxy/gl.h>
#include "../../master_header_2.h"
extern rigidbody objects [10];
extern int number_objects;
static void on_point_render (GtkGLArea * area, GdkGLContext *context) {
    int width = gtk_widget_get_allocated_width (GTK_WIDGET (area));
    int height = gtk_widget_get_allocated_height (GTK_WIDGET (area));
    render_scene_current (width, height);
} void activation (GtkApplication *application, gpointer user_data_main) {
    GtkWidget *window = gtk_application_window_new (application);
    gtk_window_set_title (GTK_WINDOW (window), "Stage 2 Testing Stage");
    gtk_window_set_default_size (GTK_WINDOW (window), 800, 600); //800x600 resolution
    //GL Canvas for presenting objects
    GtkWidget *gl_area_set = gtk_gl_area_new ();
    g_signal_connect (gl_area_set, "render", G_CALLBACK (on_point_render), NULL);
    gtk_container_add (GTK_CONTAINER (window), gl_area_set);
    //Present Window on System
    g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);
    gtk_widget_show_all (window);
}
