#include "mouse_lock.h"
void mouse_lock_enable (GtkWidget *window) {
    GdkWindow *gdk_win = gtk_widget_get_window (window);
    GdkDisplay *display = gdk_window_get_display (gdk_win);
    GdkSeat *seat = gdk_display_get_default_seat (display);
    GdkCursor *blank_cursor = gdk_cursor_new_from_name (display, "none");
    gdk_seat_grab (seat, gdk_win, GDK_SEAT_CAPABILITY_POINTER, FALSE, blank_cursor, NULL, NULL, NULL);
    g_object_unref (blank_cursor);
} void mouse_lock_disable (GtkWidget *window) {
    GdkDisplay *display = gdk_display_get_default ();
    GdkSeat *seat = gdk_display_get_default_seat (display);
    gdk_seat_ungrab (seat);
} void mouse_lock_recalibrate (GtkWidget *window) {
    GdkWindow *gdk_win = gtk_widget_get_window (window);
    int width = gtk_widget_get_allocated_width (window);
    int height = gtk_widget_get_allocated_height (window);
    // Warp pointer to window centre
    GdkDisplay *display = gdk_window_get_display (gdk_win);
    GdkSeat *seat = gdk_display_get_default_seat (display);
    GdkDevice *pointer = gdk_seat_get_pointer (seat);
    gdk_device_warp (pointer, gdk_window_get_screen (gdk_win), width / 2, height / 2);
}
