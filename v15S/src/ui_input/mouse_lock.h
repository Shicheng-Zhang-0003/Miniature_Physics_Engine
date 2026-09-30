#ifndef mouse_lock_h
#define mouse_lock_h
#include <gtk/gtk.h>
#include <gdk/gdk.h>
//Lock Cursor --> Actual Object and POV Movement Inside Engine
void mouse_lock_enable(GtkWidget *window_widget);
//Disable Cursor Lock
void mouse_lock_disable(GtkWidget *window_widget);
//Cursor Back to window center
void mouse_lock_reset_centre(GtkWidget *window_widget);
//Re-Grab Cursor Status
void mouse_lock_reacquire(GtkWidget *window_widget);
//DESPOT-2026-09-29: Wayland relative-pointer lock. Returns 1 and drains the
// accumulated unbounded relative motion when the compositor supports
// zwp_relative_pointer_manager_v1; the caller must then IGNORE absolute
// cursor coordinates. Returns 0 when unavailable (X11, or a compositor
// without the protocol), in which case the caller falls back to the
// absolute-delta path.
int mouse_lock_take_relative_delta(double *dx, double *dy);
//Non-zero if this display can do real relative-pointer lock.
int mouse_lock_relative_available(void);
//DESPOT-2026-09-29: the single source of truth for the sign convention.
//Wayland is +x right / +y DOWN; camera pitch is +up, so y is negated. Pure, so
//the headless suite can assert all four directions.
float mouse_lock_relative_to_camera(double rdx, double rdy,
                                    float *out_x, float *out_y);
//Non-zero while a real relative-pointer lock is attached. While true, absolute
//cursor coordinates must be ignored entirely.
int mouse_lock_relative_active(void);
//Directional receive counters, so "compositor never sent it" is separable from
//"we received it and converted it wrong". Reset per lock attempt.
void mouse_lock_diagnostics(unsigned long *events, unsigned long *pos_x,
                            unsigned long *neg_x, unsigned long *pos_y,
                            unsigned long *neg_y);
void mouse_lock_diagnostics_reset(void);
#endif
