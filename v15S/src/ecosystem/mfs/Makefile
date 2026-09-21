# MFS — distinct folder inside the MPE kernel module-ecosystem handling tree
#
# Location: v15S/src/ecosystem/mfs/
# Parent:   v15S/src/ecosystem/  (kernel MEI: mpe_ecosystem.h/.c + registry)
#
# Contents (canonical, single source — no forks):
#   mfs_module_1.c / .h / _test.c  — MPI "generic" tick module "mfs-simulator"
#   mfs_ecosystem.c                 — MEI wrapper bundling Module 1 internally
#   mfs_internal.c / .h             — private static registry for that bundling
#
# Retired (do NOT resurrect):
#   v15S/mfs_ecosystem/modules/mfs_module_1.* (535-line older fork coupled to
#   mfs_internal via a dlopen constructor that self-DoSed attach; physics
#   identical to the canonical 703-line fork kept here).
#
# Dependencies (NOT moved — stay where they are):
#   v15S/src/{core,physics,config,scene}/  via -I$(MPE_SRC)  (engine kernel)
#   v15S/robotics_backup/robotics/, gamepad/ via -I$(RB)     (FTC robot library)
# Includes in this folder are location-independent on purpose:
#   engine  -> "core/...", "physics/...", "config/..."
#   robots  -> "robotics/...", "gamepad/..."
#   local   -> "mfs_*.h" (same dir)
# Do not reintroduce ../-relative crosses; they break on every move.
#
# Physics-truth note: deterministic=false (libm M_PI/fabsf + live gamepad;
# route trig through core/det_math.h before claiming true).

MPE_SRC = ../..
RB = ../../../robotics_backup

CFLAGS = -I$(MPE_SRC) -I$(RB) -O3 -Wall -Wextra -ffp-contract=off -fPIC -fno-inline -fno-lto -fno-inline-functions-called-once -fvisibility=default \
         -DMPE_MODULE_ABI=1 -DMPE_ECOSYSTEM_ABI=1 -DMPE_GTK4=1

MPE_CORE_SRCS = $(MPE_SRC)/core/physics_world.c $(MPE_SRC)/core/rigidbody.c \
           $(MPE_SRC)/core/mpe_registry.c $(MPE_SRC)/core/mpe_loader.c \
           $(MPE_SRC)/core/det_math.c $(MPE_SRC)/core/mpe_primary.c \
           $(MPE_SRC)/physics/collision_narrowphase.c $(MPE_SRC)/physics/collision_cache.c \
           $(MPE_SRC)/physics/collision_solver.c $(MPE_SRC)/physics/collision_ccd.c \
           $(MPE_SRC)/physics/collision_cylinder.c $(MPE_SRC)/physics/broadphase.c \
           $(MPE_SRC)/physics/constraint.c $(MPE_SRC)/physics/revolute_joint.c \
           $(MPE_SRC)/physics/depenetration.c $(MPE_SRC)/physics/islands.c \
           $(MPE_SRC)/config/mpe_config.c $(MPE_SRC)/config/mpe_config_schema.c \
           $(MPE_SRC)/scene/boundary.c

RB_CORE_SRCS = $(RB)/robotics/robot.c $(RB)/robotics/drivetrain.c \
           $(RB)/robotics/motor.c $(RB)/robotics/motor_presets.c \
           $(RB)/robotics/battery.c \
           $(RB)/gamepad/gamepad.c

MFS_SRCS = mfs_module_1.c mfs_ecosystem.c mfs_internal.c ../mpe_ecosystem.c
MFS_OBJS = $(MFS_SRCS:.c=.o)
ALL_SRCS = $(MPE_CORE_SRCS) $(RB_CORE_SRCS) $(MFS_SRCS)
ALL_OBJS = $(ALL_SRCS:.c=.o)

all: mfs_module_1.so mfs_ecosystem.so

# Fat link (engine + robot stacks statically inside), matching the original
# MFS makefiles. Long-term the thin pattern (mpe_ftc.so: only module objs,
# resolve against the -rdynamic host) avoids version skew; not switched yet.
mfs_module_1.so: $(ALL_OBJS)
	$(CC) $(CFLAGS) -shared $(ALL_OBJS) -lm -ldl -o mfs_module_1.so

mfs_ecosystem.so: $(ALL_OBJS)
	$(CC) $(CFLAGS) -shared $(ALL_OBJS) -lm -ldl -o mfs_ecosystem.so

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

test: $(ALL_OBJS)
	$(CC) $(CFLAGS) -DMFS_MODULE_1_TEST mfs_module_1_test.c $(ALL_OBJS) \
	    -lm -ldl -o test_mfs_module_1
	./test_mfs_module_1

clean:
	rm -f $(MFS_OBJS) mfs_module_1.so mfs_ecosystem.so test_mfs_module_1
