#ifndef ftc_traj_h
#define ftc_traj_h
#include "core/math3d.h"
/* Purple predicted-trajectory line for the ball the robot is about to
 * launch. While robot 0 carries a ball and the arcade league is active,
 * this shows where that ball would fly — including bounces off the floor,
 * the perimeter walls and the bucket — computed by SIMULATING the real
 * physics (a throwaway world that mirrors the primary world's statics with
 * the ball's exact mass/radius/restitution, stepped at the engine dt with
 * the same drag + Magnus aero), not a closed-form parabola. That is the
 * whole point: motor/launch specs are not trustworthy enough for an
 * analytic aim line, so the preview runs the solver itself and cannot
 * disagree with the shot that later fires.
 *
 * The solve is AMORTIZED across frames (a fixed tick budget per frame on a
 * world kept alive between frames) because one full prediction measured
 * 45-146 ms — several times the entire frame budget, which froze the GUI
 * outright. A prediction therefore draws itself in over ~0.2 s at a few ms
 * per frame, and costs nothing once complete. It restarts only when the shot
 * actually changes (aim heading, carried set, real translation), rate-limited
 * by a cooldown. ftc_traj_update() is called from the engine tick;
 * ftc_traj_points() feeds the renderer.
 *
 * Measured (temp/traj_amort.c): worst frame 3.0 ms at 60 fps, and the polyline
 * is bit-identical (0.000000 m deviation) to the live 128-iteration shot —
 * the prediction world runs 8 iterations because it has one ball, no joints
 * and no 40:1 mass ratio (see FTRJ_ITERATIONS). */
void ftc_traj_update (float frame_delta_time);
int ftc_traj_enabled (void);
/* Returns the number of points and a pointer to the cached polyline
 * (world-space centres of the predicted ball). Returns 0 when there is
 * nothing to draw (no robot, no carried ball, no bundle). The buffer stays
 * valid until the next ftc_traj_update(). */
int ftc_traj_points (const vector3 **out_pts);
#endif /* ftc_traj_h */