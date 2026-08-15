#ifndef mpe_engine_h
#define mpe_engine_h

#include <gtk/gtk.h>
#include <epoxy/gl.h>

/* MPE_TASK_25_CONSTANTS_INCLUDE */
#include "config/mpe_constants.h"
#include "config/mpe_config.h"

#include "core/math3D.h"
#include "core/math4_special.h"
#include "core/rigidbody.h"
#include "core/frame_timer.h"

#include "physics/collision_mechanics.h"
#include "physics/define_forces.h"
#include "physics/broadphase.h"
#include "physics/spring_joint.h"

#include "render/shader_loading.h"
#include "render/sphere_meshing.h"
#include "render/cube_meshing.h"
#include "render/grid.h"
#include "render/wireframe.h"

#include "scene/scene_init.h"
#include "scene/boundary.h"
#include "scene/scene_saving.h"
#include "scene/scene_load.h"

#include "ui_input/input_control.h"
#include "ui_input/camera.h"
#include "ui_input/mouse_lock.h"
#include "ui_input/object_spawner.h"
#include "ui_input/object_selector.h"
#include "ui_input/overlay.h"

#include "ui_input/editor.h"
/* MPE_TASK_18_TERMINAL_INCLUDE_BEGIN */
#include "ui_input/debug_terminal.h"
#include "ui_input/config_menu.h"
/* MPE_TASK_18_TERMINAL_INCLUDE_END */
/* ------------------------------------------------------------------ */
/* Global scene state                                                 */
/* ------------------------------------------------------------------ */

extern rigidbody *obj_per_scene;
extern int object_count;
extern int object_capacity;

/* ------------------------------------------------------------------ */
/* Global application input and camera state                         */
/* ------------------------------------------------------------------ */

extern camera main_camera_fov;
extern input_status main_inputs;

/* ------------------------------------------------------------------ */
/* Global editor, world, and timing state                            */
/* ------------------------------------------------------------------ */

extern int selected_object;



extern frame_timer main_timer;

/* ------------------------------------------------------------------ */
/* Top-level render entry points                                      */
/* ------------------------------------------------------------------ */

void render_init (void);
void render_scene_current (int widget_width, int widget_height);

/* ------------------------------------------------------------------ */
/* Top-level physics tick entry point                                 */
/* ------------------------------------------------------------------ */

gboolean physics_step_increment (gpointer user_data_pointer);

/* ------------------------------------------------------------------ */
/* Optional GTK application activation entry point                    */
/* ------------------------------------------------------------------ */



/* A3_PATCH_24_MENU_STATE_MACHINE */
float open_numerical_input_dialog (GtkWidget *parent, const char *title, float current_value);

/* A3_PATCH_36_DEBUG_COUNTERS */
extern int debug_last_object_count;
extern int debug_last_broadphase_pair_count;
extern int debug_last_manifold_count;
extern float debug_last_frame_time;
/* MPE_TASK_13_LONG_RUN_EXTERN_BEGIN */
extern int long_run_validation_active;
extern int long_run_validation_ticks_remaining;
extern int long_run_validation_total_ticks;
/* MPE_TASK_13_LONG_RUN_EXTERN_END */
/* MPE_TASK_12_SLEEPING_COUNT_EXTERN_BEGIN */
extern int debug_last_sleeping_object_count;
/* MPE_TASK_12_SLEEPING_COUNT_EXTERN_END */
/* MPE_TASK_09_MANIFOLD_OVERFLOW_EXTERN_BEGIN */
extern int debug_last_manifold_overflow_count;
/* MPE_TASK_09_MANIFOLD_OVERFLOW_EXTERN_END */

/* A3_PATCH_41_FINAL_VALIDATION */
#define A3_VERSION_STRING "v15R1" /* v15R1 release candidate */

/* MPE_RELEASE_FREEZE_BEGIN */
#define A3_RELEASE_FREEZE 0
#define A3_RELEASE_FREEZE_NOTE "v15R1 development cycle active"
/* MPE_RELEASE_FREEZE_END */

#endif // mpe_engine_h
