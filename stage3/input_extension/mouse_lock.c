#include "mouse_lock.h"
void mouse_lock_enable (GtkWidget *parent_window_container) {
    GdkWindow *gdk_window_instance_handle = gtk_widget_get_window (parent_window_container);
    if (!gdk_window_instance_handle) {return;}
    GdkDisplay *gdk_display_default_instance = gdk_window_get_display (gdk_window_instance_handle);
    GdkSeat *gdk_seat_default_instance = gdk_display_get_default_seat (gdk_display_default_instance);
    GdkCursor *gdk_blank_cursor_handle = gdk_cursor_new_for_display (gdk_display_default_instance, GDK_BLANK_CURSOR);
    gdk_seat_grab (gdk_seat_default_instance, gdk_window_instance_handle, GDK_SEAT_CAPABILITY_POINTER, TRUE, gdk_blank_cursor_handle, NULL, NULL, NULL);
    g_object_unref (gdk_blank_cursor_handle);
} void mouse_lock_disable (GtkWidget *parent_window_container) {
    GdkDisplay *gdk_display_default_instance = gdk_display_get_default ();
    if (!gdk_display_default_instance) {return;}
    GdkSeat *gdk_seat_default_instance = gdk_display_get_default_seat (gdk_display_default_instance);
    gdk_seat_ungrab (gdk_seat_default_instance);
} void mouse_lock_reset_centre (GtkWidget *parent_window_container) {
    GdkWindow *gdk_window_instance_handle = gtk_widget_get_window (parent_window_container);
    if (!gdk_window_instance_handle) {return;}
    int window_allocated_width = gtk_widget_get_allocated_width (parent_window_container);
    int window_allocated_height = gtk_widget_get_allocated_height (parent_window_container);
    // Get top-left corner of total screen
    int screen_origin_x, screen_origin_y;
    gdk_window_get_origin (gdk_window_instance_handle, &screen_origin_x, &screen_origin_y);
    // gdk_device_warp (): absolute screen coordinates required
    // Add a origin on the screen, warp lands at dead center of window.
    int warp_target_x = screen_origin_x + (window_allocated_width / 2);
    int warp_target_y = screen_origin_y + (window_allocated_height / 2);
    GdkDisplay *gdk_display_default_instance = gdk_window_get_display (gdk_window_instance_handle);
    GdkSeat *gdk_seat_default_instance = gdk_display_get_default_seat (gdk_display_default_instance);
    GdkDevice *gdk_pointer_device_instance = gdk_seat_get_pointer (gdk_seat_default_instance);
    gdk_device_warp (gdk_pointer_device_instance, gdk_window_get_screen (gdk_window_instance_handle), warp_target_x, warp_target_y);
}
