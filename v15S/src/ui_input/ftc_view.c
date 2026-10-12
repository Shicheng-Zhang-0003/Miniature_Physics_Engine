/* See ftc_view.h. Keeps the FTC-specific camera logic out of core: the
 * free-camera tick (core/simulation_camera.c) stays generic, and this TU
 * re-targets main_camera_fov onto robot 0 while a follow mode is active.
 * Robot access goes through the bundle (the engine never links FTC); the
 * chassis body index in ftc_robot is all we need, so robot.h is included
 * for its layout only (same as ui_input/gamepad_drive.c). */
#include "ftc_view.h"
#include "../mpe_engine.h"
#include "../core/mpe_platform.h"
#include "../core/mpe_loader.h"
#include "../core/physics_world.h"
#include "../ui_input/camera.h"
#include "../ecosystem/mfs/modules/ftc/submodules/robot.h"
#include <math.h>
/* Bundle identity mirrors term_ftc.c / gamepad_drive.c. */
#define FTCV_HANDLE_PRIMARY "mfs-simulator"
#define FTCV_HANDLE_MODULE "ecosystem/mfs/plugins/mpe_ftc.so"
static void *ftcv_sym (const char *sym) {
    void *p = mpe_loader_symbol (FTCV_HANDLE_PRIMARY, sym);
    if (p)
        return p;
    return mpe_loader_symbol (FTCV_HANDLE_MODULE, sym);
}
/* Chase-camera placement (metres / degrees). */
#define FTCV_DIST 3.5f
#define FTCV_HEIGHT 2.5f
#define FTCV_PITCH (-20.0f)
#define FTCV_FIELD_YAW 90.0f /* +Z: the field's forward (bucket/pieces side) */
static int s_view_mode = 0;
void ftc_view_set (int mode) {
    if (mode < 0)
        mode = 0;
    if (mode > 2)
        mode = 2;
    s_view_mode = mode;
}
int ftc_view_get (void) {return s_view_mode;}
void ftc_view_tick (float frame_delta_time) {
    (void) frame_delta_time;
    if (s_view_mode == 0)
        return; /* free camera: leave whatever the free tick produced */
    physics_world *w = physics_world_get_primary ();
    ftc_robot * (*p_get) (struct physics_world *, int) =
        (ftc_robot * (*) (struct physics_world *, int)) ftcv_sym ("ftc_fleet_get");
    if (!w || !p_get)
        return; /* no world/bundle: nothing to follow, stay on free camera */
    ftc_robot *r = p_get (w, 0);
    if (!r || r -> chassis_body < 0 || r -> chassis_body >= w -> body_count)
        return; /* no robot spawned yet */
    rigidbody *ch = &w -> bodies [r -> chassis_body];
    /* Robot forward = chassis local +Z in world space (cached_axes[2], the
     * same axis gamepad_drive uses for forward velocity). Flatten to the
     * ground plane so rolling/pitch does not tilt the view heading. */
    vector3 fwd = ch -> cached_axes [2];
    fwd.y = 0.0f;
    float fl = sqrtf (fwd.x * fwd.x + fwd.z * fwd.z);
    if (!(fl > 1e-4f) || !isfinite (fl)) {
        fwd = (vector3) {0.0f, 0.0f, 1.0f};
        fl = 1.0f;
    }
    fwd.x /= fl;
    fwd.z /= fl;
    vector3 cam;
    float yaw;
    if (s_view_mode == 2) {
        /* Robot-centric: sit behind the robot, look along its heading. */
        cam.x = ch -> position.x - fwd.x * FTCV_DIST;
        cam.z = ch -> position.z - fwd.z * FTCV_DIST;
        yaw = atan2f (fwd.z, fwd.x) * 57.29578f; /* camera yaw 0 -> +X, 90 -> +Z */
    } else {
        /* Field-centric: fixed world viewing direction (+Z field forward),
         * only the position tracks the robot. */
        cam.x = ch -> position.x;
        cam.z = ch -> position.z - FTCV_DIST;
        yaw = FTCV_FIELD_YAW;
    }
    cam.y = ch -> position.y + FTCV_HEIGHT;
    main_camera_fov.position = cam;
    main_camera_fov.yaw = yaw;
    main_camera_fov.pitch = FTCV_PITCH;
    main_camera_fov.horizontal_velocity = (vector3) {0.0f, 0.0f, 0.0f};
    main_camera_fov.vertical_velocity = 0.0f;
    camera_update_vectors (&main_camera_fov);
}
