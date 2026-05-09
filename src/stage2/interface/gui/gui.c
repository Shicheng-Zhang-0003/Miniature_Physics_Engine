#include <stdio.h>
#include <gtk/gtk.h>
#include <epoxy/gl.h>
#include "../../master_header_2.h"
static void on_point_render (GtkGLArea *gl_area, GdkGLContext *gl_context) {
    (void) gl_context;
    int widget_width = gtk_widget_get_allocated_width (GTK_WIDGET (gl_area));
    int widget_height = gtk_widget_get_allocated_height (GTK_WIDGET (gl_area));
    render_scene_current (widget_width, widget_height);
} static void render_init_callback (GtkGLArea *gl_area, gpointer user_data) {
    (void) user_data;
    if (gtk_gl_area_get_error (gl_area) != NULL) {return;}
    render_init ();
} void activation (GtkApplication *application_object, gpointer user_data_pointer) {
    (void) user_data_pointer;
    GtkWidget *main_window = gtk_application_window_new (application_object);
    gtk_window_set_title (GTK_WINDOW (main_window), "Stage 2 Testing Stage");
    gtk_window_set_default_size (GTK_WINDOW (main_window), 800, 600); //800x600 resolution
    //GL Canvas for presenting objects
    GtkWidget *gl_area_widget = gtk_gl_area_new ();
    g_signal_connect (gl_area_widget, "render", G_CALLBACK (on_point_render), NULL);
    g_signal_connect (gl_area_widget, "realize", G_CALLBACK (render_init_callback), NULL);
    gtk_container_add (GTK_CONTAINER (main_window), gl_area_widget);
    //Present Window on System
    g_signal_connect (main_window, "destroy", G_CALLBACK (gtk_main_quit), NULL);
    gtk_widget_show_all (main_window);
}
