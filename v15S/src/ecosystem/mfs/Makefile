# MFS overarching module ecosystem ("mfs-simulator")
#
# Location: v15S/src/ecosystem/mfs/  (under the kernel MEI tree)
# Full map: see README_MFS.md
#
# Include convention (location-independent, do not use ../ crosses):
#   engine headers -> "core/...", "physics/...", "config/..."  (-I$(MPE_SRC))
#   MFS-internal   -> "modules/...", e.g. "modules/ftc/submodules/robot.h" (-I.)
#
# Physics-truth note: deterministic=false on both bundled modules (libm
# trig in odometry/rotors plus live gamepad polling in module_1; route
# through core/det_math.h before claiming true).

MPE_SRC = ../..
MFS = .

CFLAGS = -I$(MPE_SRC) -I$(MFS) -O3 -Wall -Wextra -ffp-contract=off -fPIC -fno-inline -fno-lto -fno-inline-functions-called-once -fvisibility=default \
         -DMPE_MODULE_ABI=1 -DMPE_ECOSYSTEM_ABI=1 -DMPE_GTK4=1

# ---- engine kernel objects (resolved against the -rdynamic host for .so builds) ----
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

# ---- MFS modules + submodules ----
MOD1_SRCS = modules/module_1/mfs_module_1.c
MOD1_SUB_SRCS = modules/module_1/submodules/gamepad/gamepad.c
FTC_SRCS = modules/ftc/ftc_module.c modules/ftc/ftc_fleet.c modules/ftc/gui_robot_registry.c
FTC_SUB_SRCS = modules/ftc/submodules/robot.c modules/ftc/submodules/drivetrain.c \
           modules/ftc/submodules/motor.c modules/ftc/submodules/motor_presets.c \
           modules/ftc/submodules/battery.c
ECO_SRCS = mfs_ecosystem.c mfs_internal.c

ALL_SRCS = $(MPE_CORE_SRCS) $(MOD1_SRCS) $(MOD1_SUB_SRCS) $(FTC_SRCS) $(FTC_SUB_SRCS) $(ECO_SRCS)
ALL_OBJS = $(ALL_SRCS:.c=.o)

# Thin module .so (FTC stack only; engine symbols resolve against the host,
# same pattern as build_tests.sh mpe_ftc.so; avoids version skew).
FTC_MOD_SRCS = $(FTC_SRCS) $(FTC_SUB_SRCS)
FTC_MOD_OBJS = $(FTC_MOD_SRCS:.c=.o)

all: mfs_module_1.so mfs_ecosystem.so mpe_ftc.so

mfs_module_1.so: $(ALL_OBJS)
	$(CC) $(CFLAGS) -shared $(ALL_OBJS) -lm -ldl -o mfs_module_1.so

mfs_ecosystem.so: $(ALL_OBJS)
	$(CC) $(CFLAGS) -shared $(ALL_OBJS) -lm -ldl -o mfs_ecosystem.so

mpe_ftc.so: $(FTC_MOD_OBJS)
	$(CC) $(CFLAGS) -shared $(FTC_MOD_OBJS) -lm -ldl -o plugins/mpe_ftc.so

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

test: $(ALL_OBJS)
	$(CC) $(CFLAGS) -DMFS_MODULE_1_TEST modules/module_1/mfs_module_1_test.c $(ALL_OBJS) \
	    -lm -ldl -o test_mfs_module_1
	./test_mfs_module_1

clean:
	rm -f mfs_module_1.so mfs_ecosystem.so plugins/mpe_ftc.so test_mfs_module_1
	rm -f $(MOD1_SRCS:.c=.o) $(MOD1_SUB_SRCS:.c=.o) $(FTC_SRCS:.c=.o) $(FTC_SUB_SRCS:.c=.o) $(ECO_SRCS:.c=.o)
