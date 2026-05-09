#include "mouse_lock.h"
void mouse_lock_enable (GtkWidget *window_widget) {
    GdkWindow *gdk_window_handle = gtk_widget_get_window (window_widget);
    if (! (gdk_window_handle)) {return;}
    gdk_window_set_event_compression (gdk_window_handle, FALSE);
    GdkDisplay *gdk_display_instance = gdk_window_get_display (gdk_window_handle);
    GdkSeat *gdk_seat_instance = gdk_display_get_default_seat (gdk_display_instance);
    GdkCursor *blank_cursor_handle = gdk_cursor_new_for_display (gdk_display_instance, GDK_BLANK_CURSOR);
    // Grab everything: Pointer, owner_events = FALSE (trap it), use blank cursor, and confine to THIS window
    gdk_seat_grab (gdk_seat_instance, gdk_window_handle, GDK_SEAT_CAPABILITY_POINTER, FALSE, blank_cursor_handle, NULL, NULL, NULL);
    g_object_unref (blank_cursor_handle);
} void mouse_lock_disable (GtkWidget *window_widget) {
    (void) window_widget;
    GdkDisplay *gdk_display_instance = gdk_display_get_default ();
    if (! (gdk_display_instance)) {return;}
    GdkSeat *gdk_seat_instance = gdk_display_get_default_seat (gdk_display_instance);
    gdk_seat_ungrab (gdk_seat_instance);
} void mouse_lock_reset_centre (GtkWidget *window_widget) {
    GdkWindow *gdk_window_handle = gtk_widget_get_window (window_widget);
    if (! (gdk_window_handle)) {return;}
    int window_width = gtk_widget_get_allocated_width (window_widget);
    int window_height = gtk_widget_get_allocated_height (window_widget);
    // Get top-left corner of total screen
    int screen_origin_x, screen_origin_y;
    gdk_window_get_origin (gdk_window_handle, &screen_origin_x, &screen_origin_y);
    // gdk_device_warp (): absolute screen coordinates required
    // Add a origin on the screen, warp lands at dead center of window.
    int warp_target_x = screen_origin_x + (window_width / 2);
    int warp_target_y = screen_origin_y + (window_height / 2);
    GdkDisplay *gdk_display_instance = gdk_window_get_display (gdk_window_handle);
    GdkSeat *gdk_seat_instance = gdk_display_get_default_seat (gdk_display_instance);
    GdkDevice *pointer_device_instance = gdk_seat_get_pointer (gdk_seat_instance);
    gdk_device_warp (pointer_device_instance, gdk_window_get_screen (gdk_window_handle), warp_target_x, warp_target_y);
}
