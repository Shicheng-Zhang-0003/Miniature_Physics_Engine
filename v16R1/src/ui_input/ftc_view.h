#ifndef ftc_view_h
#define ftc_view_h
/* FTC camera views, selected from the terminal with `ftc view [0|1|2]`:
 *   0 = free camera (default; WASD/mouse fly, unchanged)
 *   1 = field-centric follow: camera POSITION tracks robot 0, but the camera
 *       YAWS with the field, not the robot — the usual driver-station
 *       "player" view (robot turning does not spin the view).
 *   2 = robot-centric: the camera sits behind robot 0 and yaws with its
 *       heading, so the robot always faces away from the camera.
 * ftc_view_tick rides the engine frame loop right after the free-camera
 * tick and overrides it while a follow mode is active. */
void ftc_view_set (int mode);
int ftc_view_get (void);
void ftc_view_tick (float frame_delta_time);
#endif /* ftc_view_h */
