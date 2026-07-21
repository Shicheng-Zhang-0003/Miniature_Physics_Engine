#ifndef mpe_engine_h
#define mpe_engine_h

#define MPE_MAX_BODIES 16384
#define MPE_MAX_JOINTS 1024
#define MPE_MAX_BROADPHASE_PAIRS 65536

#include "core/math3D.h"
#include "core/math4_special.h"
#include "core/buffer.h"
#include "core/timer.h"
#include "core/frame_timer.h"

#include "physics/collision_mechanics.h"
#include "physics/define_forces.h"
#include "physics/broadphase.h"
#include "physics/spring_joint.h"
#include "physics/constraint_interface.h"
#include "physics/physics_world.h"

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

#endif // mpe_engine_h
