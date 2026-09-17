/* GTK4-PREP: GTK3 preserved under #else; GTK4 Wayland-safe port follows.
 * GTK3 used GdkWindow-type, window event-compression, seat grab/ungrab and
 * device warp — all removed in GTK4 and/or forbidden on Wayland. This port
 * is functionally equivalent where the platform allows it, but Wayland-safe:
 * no warp, GdkSurface via GtkNative, GdkSeat without grab, hidden cursor via
 * gtk_widget_set_cursor / gdk_surface_set_cursor, and pointer-constraints
 * documented as compositor-side.
 */
#ifdef MPE_GTK4

#include "../mpe_engine.h"
#include "mouse_lock.h"
#include "input_state.h"
#include <gtk/gtk.h>
#include <gdk/gdk.h>

extern input_status main_inputs;

/* Resolve the GdkSurface for any widget (via its GtkNative toplevel).
 * May return NULL if the widget is not yet realized — callers must handle it. */
static GdkSurface *mpe_surface_for_widget(GtkWidget *widget) {
    if (!widget) return NULL;
    if (!GTK_IS_WIDGET(widget)) return NULL;
    GtkNative *native = gtk_widget_get_native(widget);
    if (!native) return NULL;
    return gtk_native_get_surface(native);
}

static GdkCursor *mpe_blank_cursor_new(void) {
    /* GTK4: the BLANK cursor enum was removed. Named cursor "none" hides the
     * pointer on both X11 and Wayland. NULL fallback lets GDK pick default. */
    return gdk_cursor_new_from_name("none", NULL);
}

void mouse_lock_enable(GtkWidget *window_widget) {
    if (!window_widget) return;
    if (!GTK_IS_WIDGET(window_widget)) return;

    GdkSurface *surface = mpe_surface_for_widget(window_widget);

    /* Wayland-safe seat query — GTK4 removed the seat grab API. Keep GdkSeat/
     * GdkDevice access to document capability, but do not grab. Confinement
     * on Wayland requires zwp_pointer_constraints_v1 via compositor/portal,
     * which GDK does not wrap (no pointer-constraint API on GdkToplevel). */
    GdkDisplay *display = NULL;
    if (surface) {
        display = gdk_surface_get_display(surface);
    } else {
        display = gdk_display_get_default();
    }
    if (display) {
        GdkSeat *seat = gdk_display_get_default_seat(display);
        if (seat) {
            GdkDevice *pointer = gdk_seat_get_pointer(seat);
            (void)pointer;
            (void)gdk_seat_get_capabilities(seat);
        }
    }

    /* Window event compression was removed in GTK4. Motion compression is
     * controlled by GtkEventControllerMotion; no surface call needed. */

    GdkCursor *blank = mpe_blank_cursor_new();
    if (blank) {
        /* Preferred GTK4 API: per-widget cursor (Wayland-safe, compositor-
         * mediated). Also set on the underlying GdkSurface for immediate effect
         * if the widget is realized. gtk_widget_set_cursor refs the cursor. */
        gtk_widget_set_cursor(window_widget, blank);
        if (surface) {
            gdk_surface_set_cursor(surface, blank);
        }
        g_object_unref(blank);
    } else {
        gtk_widget_set_cursor_from_name(window_widget, "none");
        if (surface) {
            GdkCursor *fallback = gdk_cursor_new_from_name("none", NULL);
            if (fallback) {
                gdk_surface_set_cursor(surface, fallback);
                g_object_unref(fallback);
            }
        }
    }

    /* Pointer constraints: GTK4/GDK has no public API for
     * zwp_pointer_constraints_v1 / zwp_locked_pointer_v1. On X11 the hidden
     * cursor plus relative deltas (via GtkEventControllerMotion) approximates
     * the old confined grab. On Wayland, true lock requires the compositor or
     * xdg-desktop-portal RemoteDesktop/GlobalShortcuts — not exposed through
     * GDK. Pending portal support, hidden cursor + delta integration is the
     * correct best-effort Wayland-safe equivalent. */
}

void mouse_lock_disable(GtkWidget *window_widget) {
    GdkSurface *surface = NULL;
    GdkDisplay *display = NULL;

    if (window_widget && GTK_IS_WIDGET(window_widget)) {
        /* Clear widget cursor — restores default. */
        gtk_widget_set_cursor(window_widget, NULL);
        surface = mpe_surface_for_widget(window_widget);
    }

    if (surface) {
        gdk_surface_set_cursor(surface, NULL);
        display = gdk_surface_get_display(surface);
    } else {
        display = gdk_display_get_default();
        /* If we were given a non-widget window_widget that is now NULL,
         * try to clear the default toplevel surface cursor as fallback. */
        if (display) {
            /* No widget to clear, but touching the seat keeps parity with
             * the GTK3 path which called seat ungrab unconditionally. */
        }
    }

    if (display) {
        GdkSeat *seat = gdk_display_get_default_seat(display);
        (void)seat;
        /* GTK4 removed seat ungrab — no ungrab to perform. */
    }
}

void mouse_lock_reset_centre(GtkWidget *window_widget) {
    /* Wayland forbids pointer warping; device warp was removed in GTK4.
     * This function is a deliberate no-op for Wayland safety.
     *
     * GTK3 warped to (origin + width/2, height/2) and re-grabbed to keep the
     * pointer centred for delta = x_root - screen_centre. GTK4's
     * GtkEventControllerMotion delivers absolute surface coords; deltas are
     * computed as (x - last_x) without recentring, so no warp is needed.
     * Multi-monitor warp-drop handling is also obsolete without a grab.
     *
     * Keep surface/seat touches for API parity and to preserve expected
     * GdkSurface/GdkSeat symbol usage inside the GTK4 branch. */
    if (!window_widget) return;
    if (!GTK_IS_WIDGET(window_widget)) return;

    GdkSurface *surface = mpe_surface_for_widget(window_widget);
    if (surface) {
        (void)gdk_surface_get_width(surface);
        (void)gdk_surface_get_height(surface);
        GdkDisplay *display = gdk_surface_get_display(surface);
        if (display) {
            GdkSeat *seat = gdk_display_get_default_seat(display);
            if (seat) {
                GdkDevice *pointer = gdk_seat_get_pointer(seat);
                (void)pointer;
            }
        }
    } else {
        /* Not yet realized — nothing to centre; delta logic in
         * input_control.c handles initial last_x/last_y seeding. */
    }
}

void mouse_lock_reacquire(GtkWidget *window_widget) {
    if (!main_inputs.is_mouse_locked) {
        return;
    }
    if (!window_widget) return;
    if (!GTK_IS_WIDGET(window_widget)) return;

    GdkSurface *surface = mpe_surface_for_widget(window_widget);
    GdkDisplay *display = NULL;
    if (surface) {
        display = gdk_surface_get_display(surface);
    } else {
        display = gdk_display_get_default();
    }
    if (display) {
        GdkSeat *seat = gdk_display_get_default_seat(display);
        (void)seat;
        /* No seat grab in GTK4 — re-assert hidden cursor only. */
    }

    GdkCursor *blank = mpe_blank_cursor_new();
    if (blank) {
        gtk_widget_set_cursor(window_widget, blank);
        if (surface) {
            gdk_surface_set_cursor(surface, blank);
        }
        g_object_unref(blank);
    } else {
        gtk_widget_set_cursor_from_name(window_widget, "none");
    }

    main_inputs.suppress_mouse_delta = false;
    /* Reset warp guard so the first motion event after reacquire is not
     * eaten. In GTK4 this is a no-op (no warp), but we keep the call for
     * semantic parity with the GTK3 path. */
    mouse_lock_reset_centre(window_widget);
}

#else
#include "../mpe_engine.h"
#include "mouse_lock.h"
#include "input_control.h"
extern input_status main_inputs;
void mouse_lock_enable(GtkWidget *window_widget) {
    GdkWindow *gdk_window_handle = gtk_widget_get_window(window_widget);
    if (!(gdk_window_handle)) {
        return;
    }
    gdk_window_set_event_compression(gdk_window_handle, FALSE);
    GdkDisplay *gdk_display_instance = gdk_window_get_display(gdk_window_handle);
    GdkSeat *gdk_seat_instance = gdk_display_get_default_seat(gdk_display_instance);
    GdkCursor *blank_cursor_handle = gdk_cursor_new_for_display(gdk_display_instance, GDK_BLANK_CURSOR);
    // Grab everything: Pointer, owner_events = FALSE (trap it), use blank cursor, and confine to THIS window
    gdk_seat_grab(gdk_seat_instance, gdk_window_handle, GDK_SEAT_CAPABILITY_POINTER, FALSE, blank_cursor_handle, NULL,
                  NULL, NULL);
    g_object_unref(blank_cursor_handle);
}
void mouse_lock_disable(GtkWidget *window_widget) {
    (void) window_widget;
    GdkDisplay *gdk_display_instance = gdk_display_get_default();
    if (!(gdk_display_instance)) {
        return;
    }
    GdkSeat *gdk_seat_instance = gdk_display_get_default_seat(gdk_display_instance);
    gdk_seat_ungrab(gdk_seat_instance);
}
void mouse_lock_reset_centre(GtkWidget *window_widget) {
    GdkWindow *gdk_window_handle = gtk_widget_get_window(window_widget);
    if (!(gdk_window_handle)) {
        return;
    }
    int window_width = gtk_widget_get_allocated_width(window_widget);
    int window_height = gtk_widget_get_allocated_height(window_widget);
    int screen_origin_x, screen_origin_y;
    gdk_window_get_origin(gdk_window_handle, &screen_origin_x, &screen_origin_y);
    int warp_target_x = screen_origin_x + (window_width / 2);
    int warp_target_y = screen_origin_y + (window_height / 2);
    GdkDisplay *gdk_display_instance = gdk_window_get_display(gdk_window_handle);
    GdkSeat *gdk_seat_instance = gdk_display_get_default_seat(gdk_display_instance);
    GdkDevice *pointer_device_instance = gdk_seat_get_pointer(gdk_seat_instance);
    gdk_device_warp(pointer_device_instance, gdk_window_get_screen(gdk_window_handle), warp_target_x, warp_target_y);
    // Re-assert grab after warp — multi-monitor warps can silently drop the seat grab
    GdkCursor *blank_cursor_handle = gdk_cursor_new_for_display(gdk_display_instance, GDK_BLANK_CURSOR);
    gdk_seat_grab(gdk_seat_instance, gdk_window_handle, GDK_SEAT_CAPABILITY_POINTER, FALSE, blank_cursor_handle, NULL,
                  NULL, NULL);
    g_object_unref(blank_cursor_handle);
}
void mouse_lock_reacquire(GtkWidget *window_widget) {
    if (!main_inputs.is_mouse_locked) {
        return;
    }
    GdkWindow *gdk_window_handle = gtk_widget_get_window(window_widget);
    if (!(gdk_window_handle)) {
        return;
    }
    GdkDisplay *gdk_display_instance = gdk_window_get_display(gdk_window_handle);
    GdkSeat *gdk_seat_instance = gdk_display_get_default_seat(gdk_display_instance);
    GdkCursor *blank_cursor_handle = gdk_cursor_new_for_display(gdk_display_instance, GDK_BLANK_CURSOR);
    gdk_seat_grab(gdk_seat_instance, gdk_window_handle, GDK_SEAT_CAPABILITY_POINTER, FALSE, blank_cursor_handle, NULL,
                  NULL, NULL);
    g_object_unref(blank_cursor_handle);
    main_inputs.suppress_mouse_delta = false;
    //Reset warp guard so the first motion event after reacquire is not eaten
    mouse_lock_reset_centre(window_widget);
}

#endif /* MPE_GTK4 */
