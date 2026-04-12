#include "overlay.h"
#include "../../stage1/master_header.h"
#include <stdio.h>
static GtkWidget *overlay_label = NULL;
void overlay_init (GtkWidget *window, GtkWidget *gl_area) {
    //Gtk overlay allows to stack widgets incrementally over top of each other
    GtkWidget *overlay_container = gtk_overlay_new ();
    //Move gl_area value into a overlay container
    gtk_container_remove (GTK_CONTAINER (window), gl_area);
    gtk_container_add (GTK_CONTAINER (overlay_container), gl_area);
    gtk_container_add (GTK_CONTAINER (window), overlay_container);
    //Create labels and anchor to the top left
    overlay_label = gtk_label_new ("NORMAL");
    gtk_widget_set_halign (overlay_label, GTK_ALIGN_START);
    gtk_widget_set_valign (overlay_label, GTK_ALIGN_START);
    gtk_widget_set_margin_start (overlay_label, 10);
    gtk_widget_set_margin_top (overlay_label, 10);
    //White Text --> shows over dark background of gtk widgets
    GtkCssProvider *css = gtk_css_provider_new ();
    gtk_css_provider_load_from_data (css, "label {color: white; font-size: 14px;}", -1, NULL);
    gtk_style_context_add_provider (gtk_widget_get_style_context (overlay_label), GTK_STYLE_PROVIDER (css), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    gtk_overlay_add_overlay (GTK_OVERLAY (overlay_container), overlay_label);
} void overlay_update (void) {
    if (!overlay_label) {return;}
    char buffer [256];
    if ((selected_object < 0) || (selected_object >= object_count)) {
        gtk_label_set_text (GTK_LABEL (overlay_label), "No object selected | E: select object  F: impulse application  Space: spawn object");
        return;
    } rigidbody *rb = &obj_per_scene [selected_object];
    vector3 gravity = {0.0, -9.81, 0.0};
    state_energy energy = force_to_system_energy_amount (rb, gravity);
    float speed = vector3_length = (rb->velocity);
    snprintf (buffer, sizeof (buffer), "Object %d | Position = (%.1f, %.1f, %.1f) | Speed = %.2f | Ek = %.2f | Epg = %.2f | Em = %.2f", selected_object,
        rb->position.x, rb->position.y, rb->position.z,
        speed,
        energy.ek, energy.epg, energy.em
    ); gtk_label_set_text (GTK_LABEL (overlay_label), buffer);
}
