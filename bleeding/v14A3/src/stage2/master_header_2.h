#ifndef master_header_2_h
#define master_header_2_h
#include "../../mpe_engine.h"
#include <gtk/gtk.h>

extern rigidbody *obj_per_scene;
extern int object_count;
extern int object_capacity;
extern camera main_camera_fov;
extern input_status main_inputs;

void render_init (void);
void render_scene_current (int widget_width, int widget_height);
gboolean physics_step_increment (gpointer user_data_pointer);

extern float world_gravity_y;
extern float world_drag_coefficient;
extern float world_surface_friction_static;
extern float world_surface_friction_kinetic;

void activation (GtkApplication *application_object, gpointer user_data_pointer);
#endif
