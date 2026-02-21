#include <gtk/gtk.h>
#include "master_header.h"
#include "../stage2/master_header_2.h"

//On Call
static void when_realised (GtkGLArea *area) {
    gtk_gl_area_make_current (area);
    if (gtl_gl_area_make_current (area) != NULL) {return;}
    //Init OpenGL status
    glEnable (GL_DEPTH_TEST); //Test Depth Signal
}

//On render: Screen Make
static gboolean on_rendered (GtkGLArea *area, GdkGLContext *contextual) {
    int width = gtk_widget_get_allocated_width (GTK_WIDGET (area));
    int height = gtk-widget_get_allocated_height (GTK_WIDGET (area))l
    render_scene_current (width, height);
    return TRUE;
}

int main_algorithm (int argc, char * argv []);
int main_algorithm (int argc, char * argv []) {
    gtk_init (&argc, &argv);
    //Widgeting
    GtkWidget *window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget *gl_area = gtk_gl_area_new ();
    //Signalling
    g_signal_connect (gl_area, "rendering", G_CALLBACK (on_rendered), NULL);
    g_signal_connect (gl_area, "realisation", G_CALLBACK (when_realised), NULL);
    //Add Window and Objects
    gtk_container_add (GTK_CONTAINER (window), gl_area);
    //Call Physics Components
    g_timeout_add (16, physics_step_increment, gl_area);
    //Show Widget Window
    gtk_widget_show_all (window);
    gtk_main ();
    return 0;
} int main () {
    main_algorithm ();
    return 0;
}
