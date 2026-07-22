#ifndef object_selector_h
#define object_selector_h
#include "../core/math3D.h"
#include "../core/buffer.h"

extern int selected_object;
int selector_ray_tracing (void);
void clear_selection (void);
void selector_apply_force_impulse (float impulse_magnitude);
#endif
