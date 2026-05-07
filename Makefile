CC = gcc
CFLAGS = $(shell pkg-config --cflags gtk+-3.0 epoxy) -I. -Wall -Wextra
LIBS = $(shell pkg-config --libs gtk+-3.0 epoxy) -lm

SRCS = stage1/root_gtk.c \
       stage2/camera/camera.c \
       stage2/input_controller/input_control.c \
       stage2/interface/simulate/simulation.c \
       stage2/interface/render/shader_loading.c \
       stage2/interface/render/new_render.c \
       stage2/interface/sphere_object/meshing/sphere_meshing.c \
       stage2/interface/gui/gui.c \
       stage3/boundary/boundary.c \
       stage3/scene/scene_init.c \
       stage3/timing_belt/frame_timer.c \
       stage3/input_extension/mouse_lock.c \
       stage4/interaction/spawner/object_spawner.c \
       stage4/interaction/selector/object_selector.c \
       stage4/broadphase/broadphase.c \
       stage4/debug_overlay/overlay.c \
       stage5/rendering/grid.c \
       stage5/rendering/wireframe.c \
       stage5/scene_io/scene_saving.c \
       stage5/scene_io/scene_load.c \
       stage5/constraints/spring_joint.c

engine: $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) $(LIBS) -o engine

clean:
	rm -f engine
