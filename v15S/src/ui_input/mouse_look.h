/* DESPOT-2026-09-29: the mouse-look sign convention, as a dependency-free pure
 * function so it can be tested from a headless build.
 *
 * Why this is its own file: the convention used to be inlined in
 * on_mouse_movements(), which meant the one piece of mouse-look logic that can
 * be wrong in a directional way was the one piece that could not be tested
 * without a compositor and a physical mouse. It is exactly the sort of thing
 * that is right for two of the four directions.
 *
 * CONVENTION (the thing under test):
 *   Wayland surface coordinates are +x right, +y DOWN.
 *   Camera yaw/pitch as consumed by simulation_camera.c is +x right, +y UP.
 *   Therefore x passes through and y is negated.
 *
 * No GTK, no engine headers: this compiles anywhere, including the headless
 * test suite, so a change to the convention cannot pass CI by never being
 * exercised.
 */
#ifndef mouse_look_h
#define mouse_look_h

/* Convert raw relative-pointer motion into camera-space deltas.
 *
 * Returns the MAGNITUDE of the input (not its square -- the first version
 * returned dx^2+dy^2 while the comment said "magnitude", and the test caught
 * it immediately, which is the sort of contract drift that is invisible when
 * the only caller ignores the return value). It exists so a caller can tell
 * "no motion at all" from "motion that happened to be zero on one axis".
 * Called once per motion event, so the sqrt is free. */
#include <math.h>
static inline float mpe_mouse_relative_to_camera (double rdx, double rdy, float *out_x, float *out_y) {
    if (out_x)
        *out_x = (float) rdx;
    if (out_y)
        *out_y = (float) -rdy;
    return (float) sqrt (rdx * rdx + rdy * rdy);
}

#endif /* mouse_look_h */
