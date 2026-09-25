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
#
# Thin-.so rule: module .so files contain ONLY MFS objects; engine symbols
# resolve against the -rdynamic host at dlopen (same pattern as
# build_tests.sh mpe_ftc.so). Linking engine objects into the .so caused
# version skew and duplicate-symbol failures. Objects live in build/ (never
# scattered across core/ physics/ config/ scene/).

MPE_SRC = ../..
MFS = .
BUILD = build

CC ?= gcc
CFLAGS = -I$(MPE_SRC) -I$(MFS) -O3 -Wall -Wextra -ffp-contract=off -fPIC -fno-inline -fno-lto -fno-inline-functions-called-once -fvisibility=default \
         -DMPE_MODULE_ABI=1 -DMPE_ECOSYSTEM_ABI=1 -DMPE_GTK4=1

include mfs_sources.mk

# MFS-only objects (engine stays in the host). gui_robot_registry.c is
# deliberately EXCLUDED: it includes mpe_engine.h -> gtk/gtk.h and bypasses
# the module system with globals (see audit D19); it is not part of any
# loadable module.
MOD1_SRCS = modules/module_1/mfs_module_1.c modules/module_1/submodules/gamepad/gamepad.c
MFS_MOD1_OBJS = $(patsubst %.c,$(BUILD)/%.o,$(MOD1_SRCS))
# MFS_FTC_SRCS is engine-relative (see mfs_sources.mk); strip the prefix
# for local builds.
MFS_FTC_LOCAL = $(patsubst ecosystem/mfs/%,%,$(MFS_FTC_SRCS))
MFS_FTC_MOD_OBJS = $(patsubst %.c,$(BUILD)/%.o,$(MFS_FTC_LOCAL))
MFS_ECO_SRCS = mfs_ecosystem.c mfs_internal.c
MFS_ECO_OBJS = $(patsubst %.c,$(BUILD)/%.o,$(MFS_ECO_SRCS))

ALL_MFS_OBJS = $(MFS_MOD1_OBJS) $(MFS_FTC_MOD_OBJS) $(MFS_ECO_OBJS)

all: mfs_module_1.so mfs_ecosystem.so plugins/mpe_ftc.so

# NOTE: patterns match with MFS (=.) as CWD. Run make from ecosystem/mfs/.
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# mfs_ecosystem.so is a BUNDLE: it references mfs_module_1_desc and the
# FTC mpe_module_desc directly, so it links those objects in (unlike the
# thin single-module .so files below, which resolve engine symbols against
# the -rdynamic host at dlopen).
MFS_BUNDLE_OBJS = $(MFS_ECO_OBJS) $(MFS_MOD1_OBJS) $(MFS_FTC_MOD_OBJS)

mfs_module_1.so: $(MFS_MOD1_OBJS)
	$(CC) $(CFLAGS) -shared $(MFS_MOD1_OBJS) -lm -ldl -o mfs_module_1.so

mfs_ecosystem.so: $(MFS_BUNDLE_OBJS)
	$(CC) $(CFLAGS) -shared $(MFS_BUNDLE_OBJS) -lm -ldl -o mfs_ecosystem.so

plugins/mpe_ftc.so: $(MFS_FTC_MOD_OBJS)
	@mkdir -p plugins
	$(CC) $(CFLAGS) -shared $(MFS_FTC_MOD_OBJS) -lm -ldl -o plugins/mpe_ftc.so

# module_1 drives through drivetrain_update: link the FTC submodule
# objects too (not ftc_module.o — unneeded here), plus engine objects
# (single test binary, not a plugin: no -rdynamic host to resolve against).
include mfs_sources.mk
MFS_ENGINE_OBJS = $(patsubst %.c,$(BUILD)/eng/%.o,$(MFS_ENGINE_SRCS))
$(BUILD)/eng/%.o: $(MPE_SRC)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

MFS_MOD1_TEST_EXTRA = $(BUILD)/modules/ftc/submodules/robot.o \
    $(BUILD)/modules/ftc/submodules/drivetrain.o \
    $(BUILD)/modules/ftc/submodules/motor.o \
    $(BUILD)/modules/ftc/submodules/motor_presets.o \
    $(BUILD)/modules/ftc/submodules/battery.o

test: $(MFS_MOD1_OBJS) $(MFS_MOD1_TEST_EXTRA) $(MFS_ENGINE_OBJS)
	$(CC) $(CFLAGS) -DMFS_MODULE_1_TEST modules/module_1/mfs_module_1_test.c $(MFS_MOD1_OBJS) \
	    $(MFS_MOD1_TEST_EXTRA) $(MFS_ENGINE_OBJS) -lm -ldl -o $(BUILD)/test_mfs_module_1
	MPE_GAMEPAD_DEVICE=disabled ./$(BUILD)/test_mfs_module_1

clean:
	rm -rf $(BUILD) mfs_module_1.so mfs_ecosystem.so plugins/mpe_ftc.so test_mfs_module_1

.PHONY: all test clean
