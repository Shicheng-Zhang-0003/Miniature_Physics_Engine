/* Mouse lock, GTK4 (v15S). GTK4-only; the GTK3 body was removed 2026-09-29.
 *
 * ROOT CAUSE of "click locks, move far enough and it exits the window and
 * never re-locks" (DESPOT-2026-09-29)
 * ----------------------------------------------------------------------
 * For as long as the first playable release, camera input was derived from
 * the ABSOLUTE cursor position inside the surface:
 *
 *     dx = x - last_x;   dy = y - last_y;
 *
 * and "lock" only hid the cursor. On X11 under GTK3 that worked, because
 * mouse_lock_reset_centre() issued XWarpPointer after every motion event,
 * re-centring the pointer so it could never run away.
 *
 * On Wayland there is no warp (protocol-forbidden) and GTK4 has no seat
 * grab, so reset_centre() was a no-op. The consequences are exactly the
 * reported symptom:
 *
 *   1. The cursor physically travelled across the screen. Spin the camera
 *      and the pointer runs to the screen edge, where the compositor stops
 *      sending motion. The camera stalls -- a 360-degree spin is impossible.
 *   2. Once the pointer left the surface the motion stream stopped
 *      entirely, so the view froze and the window looked "exited".
 *      `is_mouse_locked` stayed true with no events ever arriving again,
 *      so nothing could re-lock it.
 *
 * So this was never a "re-lock bug": lock was never implemented on Wayland.
 * Hiding a cursor is not a lock.
 *
 * A second, independent defect: the X11 warp was guarded on
 * GDK_WINDOWING_X11, a GTK3 macro that GTK4 does not define, so under v15S
 * the warp was compiled out on X11 as well. Mouse lock was therefore
 * broken on BOTH backends, and "it works if I force X11" only ever
 * referred to the old GTK3 build.
 *
 * THE FIX
 * -------
 * Wayland provides exactly the right primitive and GTK4 does not wrap it:
 * the relative-pointer protocol (zwp_relative_pointer_manager_v1). Its own
 * specification states that when "a pointer motion caused the absolute
 * pointer position to be clipped by for example the edge of the monitor,
 * the relative motion is unaffected by the clipping". That is precisely
 * the failure above, and precisely what a first-person camera needs: the
 * on-screen cursor never moves, so screen edges are irrelevant.
 *
 * on_mouse_movements() now asks mouse_lock_take_relative_delta() first;
 * when a relative delta is available it is used and the absolute cursor
 * coordinates are ignored entirely. X11 keeps the warp-and-recentre path.
 *
 * If the compositor does not advertise the protocol, we fall back to the
 * previous absolute-delta behaviour so the game still works -- it just
 * remains edge-limited there, as it always was.
 */
#include "../mpe_engine.h"
#include "mouse_lock.h"
#include "input_state.h"
#include "mouse_look.h"
#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <string.h>
#ifdef MPE_WAYLAND_RELATIVE_POINTER
#include <gdk/wayland/gdkwayland.h>
#include "wayland/relative-pointer-unstable-v1-client-protocol.h"
#include "wayland/pointer-constraints-unstable-v1-client-protocol.h"
#endif
#ifdef MPE_GTK4_X11_WARP
#include <X11/Xlib.h>
#include <gdk/x11/gdkx.h>
#endif
extern input_status main_inputs;
/* --------------------------------------------------------------- helpers */
static GdkSurface *mpe_surface_for_widget (GtkWidget *widget) {
    if (!widget || !GTK_IS_WIDGET (widget))
        return NULL;
    GtkWidget *toplevel = gtk_widget_get_ancestor (widget, GTK_TYPE_WINDOW);
    if (!toplevel)
        toplevel = widget;
    GtkNative *native = gtk_widget_get_native (toplevel);
    if (!native)
        return NULL;
    return gtk_native_get_surface (native);
}
/* Available on every build; the GTK4 Wayland type check needs the GDK
 * wayland backend header, which we only include where it exists. */
static int mpe_display_is_wayland (GdkDisplay *display) {
    if (!display)
        return 0;
#ifdef MPE_WAYLAND_RELATIVE_POINTER
    return GDK_IS_WAYLAND_DISPLAY (display) ? 1 : 0;
#else
    return 0;
#endif
}
static GdkCursor *mpe_blank_cursor_new (void) {
    /* GTK4 removed the BLANK cursor enum. The named cursor "none" hides the
     * pointer on both X11 and Wayland. */
    return gdk_cursor_new_from_name ("none", NULL);
}
#ifdef MPE_WAYLAND_RELATIVE_POINTER
/* ---------------------------------------------------------------- state */
static struct zwp_relative_pointer_manager_v1 *mpe_rel_manager = NULL;
static int mpe_rel_manager_searched = 0; /* one lazy registry bind */
static struct zwp_relative_pointer_v1 *mpe_rel_ptr = NULL;
/* DESPOT-2026-09-29 (user report, second round). The relative pointer ALONE is
 * not a lock.
 *
 * Observed: in a WINDOW, nothing worked at all -- the mouse still left the
 * window after a little movement. Fullscreen only partially worked. That is
 * the exact signature of using one protocol where two are required:
 *
 *   zwp_relative_pointer_v1 reports UNBOUNDED deltas and never moves the
 *   cursor. It is an input device, not a confinement. It says nothing about
 *   where the pointer is allowed to be.
 *
 *   zwp_locked_pointer_v1 is the confinement. Once locked, the pointer cannot
 *   leave the surface AT ALL, and the compositor keeps feeding motion.
 *
 * Without the second, the cursor still wanders out of a small window, and once
 * it is outside, the compositor stops sending anything -- so the camera freezes
 * and the game looks like it exited. In fullscreen the window is the screen, so
 * the cursor has nowhere to escape to and it mostly worked, which is why the
 * bug looked like an odd direction-dependent edge case rather than "the
 * confinement is missing".
 *
 * Both together are the standard combination for a first-person camera:
 * locked_pointer confines and keeps events flowing, relative_pointer supplies
 * the unbounded deltas that confinement would otherwise clip. */
static struct zwp_pointer_constraints_v1 *mpe_pc_manager = NULL;
static struct zwp_locked_pointer_v1 *mpe_locked_ptr = NULL;
static int mpe_locked_active = 0;
/* Accumulated relative motion since the last drain. Written from the
 * Wayland dispatch (GDK's queue, main thread) and read from the GTK
 * handler on the same thread, so no locking is required. */
static double mpe_rel_dx = 0.0, mpe_rel_dy = 0.0;
static int mpe_rel_dirty = 0;
/* Diagnostics. The user-visible symptom of a broken lock is DIRECTIONAL, and a
 * directional symptom is undebuggable without knowing whether the compositor
 * delivered the events at all. These counters separate "the compositor never
 * sent it" from "we received it and converted it wrong". Reset per lock
 * attempt by mouse_lock_diagnostics_reset(). */
static unsigned long mpe_rel_events = 0;
static unsigned long mpe_rel_pos_x = 0, mpe_rel_neg_x = 0;
static unsigned long mpe_rel_pos_y = 0, mpe_rel_neg_y = 0;
/* Forward decl: mouse_lock_init() binds the globals at startup, before this
 * helper's definition. */
static struct zwp_relative_pointer_manager_v1 *mpe_get_rel_manager (struct wl_display *wl_display);
static int mpe_surface_is_wayland (GdkSurface *surface) {
    if (!surface)
        return 0;
    return mpe_display_is_wayland (gdk_surface_get_display (surface));
}
/* zwp_relative_pointer_v1 has exactly one event. */
static void mpe_rel_handle_motion (void *data, struct zwp_relative_pointer_v1 *rp, uint32_t utime_hi, uint32_t utime_lo,
                                   wl_fixed_t dx, wl_fixed_t dy, wl_fixed_t dx_unacc, wl_fixed_t dy_unacc) {
    (void) data;
    (void) rp;
    (void) utime_hi;
    (void) utime_lo;
    (void) dx_unacc;
    (void) dy_unacc;
    const double rx = wl_fixed_to_double (dx);
    const double ry = wl_fixed_to_double (dy);
    mpe_rel_dx += rx;
    mpe_rel_dy += ry;
    mpe_rel_dirty = 1;
    mpe_rel_events++;
    if (rx > 0.0)
        mpe_rel_pos_x++;
    else if (rx < 0.0)
        mpe_rel_neg_x++;
    if (ry > 0.0)
        mpe_rel_pos_y++;
    else if (ry < 0.0)
        mpe_rel_neg_y++;
}
void mouse_lock_diagnostics (unsigned long *events, unsigned long *pos_x, unsigned long *neg_x, unsigned long *pos_y,
                             unsigned long *neg_y) {
    if (events)
        * events = mpe_rel_events;
    if (pos_x)
        * pos_x = mpe_rel_pos_x;
    if (neg_x)
        * neg_x = mpe_rel_neg_x;
    if (pos_y)
        * pos_y = mpe_rel_pos_y;
    if (neg_y)
        * neg_y = mpe_rel_neg_y;
}
void mouse_lock_diagnostics_reset (void) {
    mpe_rel_events = mpe_rel_pos_x = mpe_rel_neg_x = 0;
    mpe_rel_pos_y = mpe_rel_neg_y = 0;
}
static const struct zwp_relative_pointer_v1_listener mpe_rel_listener = {
    mpe_rel_handle_motion,
};
static void mpe_rel_reg_global (void *data, struct wl_registry *registry, uint32_t name, const char *interface,
                                uint32_t version) {
    (void) version;
    struct zwp_relative_pointer_manager_v1 **out = (struct zwp_relative_pointer_manager_v1 **) data;
    if (out && interface && strcmp (interface, zwp_relative_pointer_manager_v1_interface.name) == 0) {
        *out = (struct zwp_relative_pointer_manager_v1 *) wl_registry_bind (
            registry, name, &zwp_relative_pointer_manager_v1_interface, 1);
    }
}
static void mpe_rel_reg_global_remove (void *data, struct wl_registry *r, uint32_t name) {
    (void) data;
    (void) r;
    (void) name;
}
static const struct wl_registry_listener mpe_rel_reg_listener = {
    mpe_rel_reg_global,
    mpe_rel_reg_global_remove,
};
/* DESPOT-2026-09-29: bind BOTH globals once, at STARTUP.
 *
 * The bind needs a wl_display_roundtrip() to actually receive the globals, and
 * doing that lazily from inside a GTK event handler (the old behaviour: the
 * first click-to-lock called it) means re-entering GDK's own event dispatch
 * mid-handler. That is exactly the kind of thing that works on the first click
 * and then behaves differently, so the bind is now done once from application
 * startup, before the event loop exists to re-enter. */
void mouse_lock_init (void) {
    GdkDisplay *display = gdk_display_get_default ();
    if (!display || !GDK_IS_WAYLAND_DISPLAY (display))
        return;
    (void) mpe_get_rel_manager (gdk_wayland_display_get_wl_display (display));
}
/* Bind the global once and cache it: it is a compositor capability, so it
 * cannot change for the lifetime of the display. */
static struct zwp_relative_pointer_manager_v1 *mpe_get_rel_manager (struct wl_display *wl_display) {
    if (mpe_rel_manager_searched)
        return mpe_rel_manager;
    mpe_rel_manager_searched = 1;
    if (!wl_display)
        return NULL;
    struct wl_registry *registry = wl_display_get_registry (wl_display);
    if (!registry)
        return NULL;
    wl_registry_add_listener (registry, &mpe_rel_reg_listener, &mpe_rel_manager);
    /* One round-trip so the globals are actually delivered. At most once
     * per process, on the first click-to-lock. */
    wl_display_roundtrip (wl_display);
    wl_registry_destroy (registry);
    return mpe_rel_manager;
}
int mouse_lock_relative_available (void) {
    GdkDisplay *display = gdk_display_get_default ();
    if (!display || !GDK_IS_WAYLAND_DISPLAY (display))
        return 0;
    return mpe_get_rel_manager (gdk_wayland_display_get_wl_display (display)) != NULL;
}
/* Non-zero when a real relative-pointer lock is attached. The caller must not
 * fall back to absolute cursor coordinates while this is true: with a live
 * relative pointer the cursor is unconstrained and free to travel anywhere, so
 * its position carries no information about how far the hand moved. */
int mouse_lock_relative_active (void) {
    return mpe_rel_ptr != NULL;
}
/* Non-zero when the pointer is genuinely CONFINED to the surface. Reported
 * separately from relative_active() because they are different capabilities and
 * can fail independently: a compositor may offer relative motion without
 * pointer constraints. */
int mouse_lock_confined (void) {
    return mpe_locked_active;
}
/* DESPOT-2026-09-29: the sign convention lives in ui_input/mouse_look.h, as a
 * pure function with no GTK dependency, so the headless suite can assert all
 * four directions. Duplicating it here is what let it be wrong in one
 * direction and right in the other with nothing able to notice. */
float mouse_lock_relative_to_camera (double rdx, double rdy, float *out_x, float *out_y) {
    return mpe_mouse_relative_to_camera (rdx, rdy, out_x, out_y);
}
int mouse_lock_take_relative_delta (double *dx, double *dy) {
    if (!mpe_rel_ptr || !mpe_rel_dirty)
        return 0;
    if (dx)
        * dx = mpe_rel_dx;
    if (dy)
        * dy = mpe_rel_dy;
    mpe_rel_dx = 0.0;
    mpe_rel_dy = 0.0;
    mpe_rel_dirty = 0;
    return 1;
}
static void mpe_rel_pointer_destroy (void) {
    if (mpe_locked_ptr) {
        zwp_locked_pointer_v1_destroy (mpe_locked_ptr);
        mpe_locked_ptr = NULL;
    }
    mpe_locked_active = 0;
    if (mpe_rel_ptr) {
        zwp_relative_pointer_v1_destroy (mpe_rel_ptr);
        mpe_rel_ptr = NULL;
    }
    mpe_rel_dx = mpe_rel_dy = 0.0;
    mpe_rel_dirty = 0;
}
/* Start receiving unbounded relative motion on this surface's pointer. */
static int mpe_rel_pointer_acquire (GdkSurface *surface) {
    if (mpe_rel_ptr)
        return 1;
    if (!mpe_surface_is_wayland (surface))
        return 0;
    GdkDisplay *display = gdk_surface_get_display (surface);
    if (!display)
        return 0;
    GdkSeat *seat = gdk_display_get_default_seat (display);
    if (!seat)
        return 0;
    GdkDevice *dev = gdk_seat_get_pointer (seat);
    if (!dev || !GDK_IS_WAYLAND_DEVICE (dev))
        return 0;
    struct zwp_relative_pointer_manager_v1 *mgr = mpe_get_rel_manager (gdk_wayland_display_get_wl_display (display));
    if (!mgr)
        return 0; /* compositor lacks the protocol */
    struct wl_pointer *wl_pointer = gdk_wayland_device_get_wl_pointer (dev);
    if (!wl_pointer)
        return 0;
    mpe_rel_ptr = zwp_relative_pointer_manager_v1_get_relative_pointer (mgr, wl_pointer);
    if (!mpe_rel_ptr)
        return 0;
    zwp_relative_pointer_v1_add_listener (mpe_rel_ptr, &mpe_rel_listener, NULL);
    /* Confinement. Best-effort: a compositor may implement one protocol and not
     * the other, and losing confinement must not lose the deltas. */
    /* GTK4 note: there is no gdk_surface_get_wayland_surface(); the accessor
     * takes the GdkSurface directly. It is a private GDK header, which is a
     * real fragility, so this is the ONE place the dependency is paid and the
     * result is used immediately. */
    struct wl_surface *wl_surf = NULL;
    if (mpe_pc_manager && GDK_IS_WAYLAND_SURFACE (surface)) {
        wl_surf = gdk_wayland_surface_get_wl_surface (surface);
    }
    if (wl_surf) {
        {
            mpe_locked_ptr = zwp_pointer_constraints_v1_lock_pointer (mpe_pc_manager, wl_surf, wl_pointer, NULL,
                                                                      ZWP_POINTER_CONSTRAINTS_V1_LIFETIME_PERSISTENT);
            if (mpe_locked_ptr)
                mpe_locked_active = 1;
        }
    }
    return 1;
}
#else /* !MPE_WAYLAND_RELATIVE_POINTER */
/* No Wayland relative-pointer support compiled in (e.g. Windows): lock falls
 * back to the historical hide-the-cursor behaviour. */
/* Diagnostics. The user-visible symptom of a broken lock is directional, and
 * a directional symptom is undebuggable without knowing whether the compositor
 * delivered the events at all. These counters separate "the compositor never
 * sent it" from "we received it and converted it wrong". Reset by
 * mouse_lock_enable so each lock attempt is measured on its own. */
static unsigned long mpe_rel_events = 0;
static unsigned long mpe_rel_pos_x = 0, mpe_rel_neg_x = 0;
static unsigned long mpe_rel_pos_y = 0, mpe_rel_neg_y = 0;
void mouse_lock_diagnostics (unsigned long *events, unsigned long *pos_x, unsigned long *neg_x, unsigned long *pos_y,
                             unsigned long *neg_y) {
    if (events)
        * events = mpe_rel_events;
    if (pos_x)
        * pos_x = mpe_rel_pos_x;
    if (neg_x)
        * neg_x = mpe_rel_neg_x;
    if (pos_y)
        * pos_y = mpe_rel_pos_y;
    if (neg_y)
        * neg_y = mpe_rel_neg_y;
}
void mouse_lock_diagnostics_reset (void) {
    mpe_rel_events = mpe_rel_pos_x = mpe_rel_neg_x = 0;
    mpe_rel_pos_y = mpe_rel_neg_y = 0;
}
int mouse_lock_relative_available (void) {
    return 0;
}
/* DESPOT-2026-09-29: the ONE place the sign convention lives.
 *
 * Previously the conversion was inlined in on_mouse_movements(), which made it
 * untestable -- the sign convention is exactly the kind of thing that is wrong
 * in one direction and right in the other, and there was no way to check it
 * without a live compositor and a physical mouse.
 *
 * Wayland surface coordinates are +x right, +y DOWN. Camera pitch is
 * +up. So y is negated and x is not. Exposed as a pure function so the suite
 * can assert all four directions from a headless build.
 *
 * Returns the magnitude of the input so callers can distinguish "no motion"
 * from "motion that happened to be zero on one axis". */
float mouse_lock_relative_to_camera (double rdx, double rdy, float *out_x, float *out_y) {
    if (out_x)
        * out_x = (float) rdx;
    if (out_y)
        * out_y = (float) -rdy;
    return (float) (rdx * rdx + rdy * rdy);
}
/* Non-zero when a real relative-pointer lock is attached. The caller must not
 * fall back to absolute cursor coordinates while this is true: with a live
 * relative pointer the cursor is free to travel anywhere and carries no
 * information about intent. */
int mouse_lock_relative_active (void) {
    return mpe_rel_ptr != NULL;
}
/* Non-zero when the pointer is genuinely CONFINED to the surface. Reported
 * separately from relative_active() because they are different capabilities and
 * can fail independently: a compositor may offer relative motion without
 * pointer constraints. */
int mouse_lock_confined (void) {
    return mpe_locked_active;
}
int mouse_lock_take_relative_delta (double *dx, double *dy) {
    (void) dx;
    (void) dy;
    return 0;
}
static void mpe_rel_pointer_destroy (void) {}
static int mpe_rel_pointer_acquire (GdkSurface *surface) {
    (void) surface;
    return 0;
}
#endif /* MPE_WAYLAND_RELATIVE_POINTER */
/* ------------------------------------------------------------- public API */
void mouse_lock_enable (GtkWidget *widget) {
    if (!widget || !GTK_IS_WIDGET (widget))
        return;
    GdkCursor *blank = mpe_blank_cursor_new ();
    if (blank) {
        gtk_widget_set_cursor (widget, blank);
        g_object_unref (blank);
    } else {
        gtk_widget_set_cursor_from_name (widget, "none");
    }
    GdkSurface *surface = mpe_surface_for_widget (widget);
    if (surface) {
        GdkCursor *blank2 = mpe_blank_cursor_new ();
        if (blank2) {
            gdk_surface_set_cursor (surface, blank2);
            g_object_unref (blank2);
        }
    }
    /* The actual lock. On Wayland this replaces the cursor-hiding fake with
     * real relative motion, so the pointer has nowhere left to go. */
    mpe_rel_pointer_acquire (surface);
}
void mouse_lock_disable (GtkWidget *widget) {
    if (!widget || !GTK_IS_WIDGET (widget))
        return;
    mpe_rel_pointer_destroy ();
    gtk_widget_set_cursor (widget, NULL);
    GdkSurface *surface = mpe_surface_for_widget (widget);
    if (surface)
        gdk_surface_set_cursor (surface, NULL);
}
void mouse_lock_reset_centre (GtkWidget *window_widget) {
    /* X11 only: re-centre so the absolute-delta path stays bounded. On
     * Wayland the relative-pointer protocol makes this unnecessary and
     * warping is protocol-forbidden, so it is a no-op.
     * CRITICAL: never touch gdk_x11_* on a Wayland display (unchecked cast
     * segfault). Guard on the display type. */
    if (!window_widget || !GTK_IS_WIDGET (window_widget))
        return;
    GdkSurface *surface = mpe_surface_for_widget (window_widget);
    if (!surface)
        return;
    GdkDisplay *display = gdk_surface_get_display (surface);
    if (!display)
        return;
    if (mpe_display_is_wayland (display))
        return;
#ifdef MPE_GTK4_X11_WARP
    if (!GDK_IS_X11_DISPLAY (display) || !GDK_IS_X11_SURFACE (surface))
        return;
    Display *xdisplay = gdk_x11_display_get_xdisplay (display);
    if (!xdisplay)
        return;
    int ww = gdk_surface_get_width (surface);
    int wh = gdk_surface_get_height (surface);
    if (ww <= 0 || wh <= 0)
        return;
    XWarpPointer (xdisplay, None, DefaultRootWindow (xdisplay), 0, 0, 0, 0, ww / 2, wh / 2);
    XFlush (xdisplay);
#endif
}
void mouse_lock_reacquire (GtkWidget *window_widget) {
    if (!main_inputs.is_mouse_locked)
        return;
    if (!window_widget || !GTK_IS_WIDGET (window_widget))
        return;
    GdkSurface *surface = mpe_surface_for_widget (window_widget);
    /* Re-establish real lock. If the surface was rebuilt this is required,
     * otherwise the old relative pointer would be attached to a dead surface
     * and the camera would silently stop responding. */
    if (!mpe_rel_ptr)
        mpe_rel_pointer_acquire (surface);
    GdkCursor *blank = mpe_blank_cursor_new ();
    if (blank) {
        gtk_widget_set_cursor (window_widget, blank);
        if (surface)
            gdk_surface_set_cursor (surface, blank);
        g_object_unref (blank);
    } else {
        gtk_widget_set_cursor_from_name (window_widget, "none");
    }
    main_inputs.suppress_mouse_delta = false;
    mouse_lock_reset_centre (window_widget);
}
