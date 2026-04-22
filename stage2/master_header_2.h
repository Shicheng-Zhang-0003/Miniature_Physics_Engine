#ifndef master_header_2_h
#define master_header_2_h
#include "../stage1/master_header.h"
#include "camera/camera.h"
#include "input_controller/input_control.h"
#include "interface/sphere_object/meshing/sphere_meshing.h"
#include "interface/render/shader_loading.h"
#include "continued_loop/timer.h"
#include <gtk/gtk.h>
// Global scene, interface/simulate/simulation.c
extern rigidbody *obj_per_scene;
extern int object_count;
extern int object_capacity;
// Global camera and input, root_gtk.c
extern camera main_camera_fov;
extern input_status main_inputs;
extern frame_timer main_timer;
// Rendering, interface/render/render.c
void render_init (void);
void render_scene_current (int width, int height);
// Physics interface, interface/simulate/simulation.c
gboolean physics_step_increment (gpointer user_data_stored);
// GUI, interface/gui/gui.c
void activation (GtkApplication *application, gpointer user_data_main);
#endif // master_header_2_h
