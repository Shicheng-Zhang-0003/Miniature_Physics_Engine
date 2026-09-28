# MFS overarching module ecosystem ("mfs-simulator")
#
# DUAL-MODE (DESPOT-2026-09-28): this file is SOURCE-IDENTICAL in both trees
# (standalone 461-MFS root and embedded 475-MPE/v15S/src/ecosystem/mfs/).
# MPE_SRC auto-detects the engine tree; override with
#   make MPE_SRC=/path/to/475-MPE/v15S/src
# or MFS_ENGINE_SRC env. Standalone default assumes the sibling layout
#   projects/461-MFS + projects/475-MPE (same parent folder).
#
# Location (embedded): v15S/src/ecosystem/mfs/  (under the kernel MEI tree)
# Full map: see README_MFS.md
#
# Include convention (location-independent, do not use ../ crosses):
#   engine headers -> "core/...", "physics/...", "config/..."  (-I$(MPE_SRC))
#   MFS-internal   -> "modules/...", e.g. "modules/ftc/submodules/robot.h" (-I.)
#
# Physics-truth note: deterministic=false on both bundled modules (heading
# frame is det_sin/det_cos exact now, but other libm uses remain in the TUs
# plus live gamepad polling in module_1; route the rest through
# core/det_math.h before claiming true).
#
# Thin-plugin rule: module plugins contain ONLY MFS objects; engine symbols
# resolve against the host at load (Linux: -rdynamic host at dlopen;
# Windows: LoadLibrary shim in core/mpe_platform.h). Linking engine objects
# into the plugin caused version skew and duplicate-symbol failures.
# Objects live in build/ (never scattered across core/ physics/ config/ scene/).
# Windows: produces .dll (MSYS2) / .dll (native); Linux: .so. Both kept.

# ---- engine-tree discovery (dual-mode; explicit MPE_SRC always wins) ----
ifeq ($(origin MPE_SRC),undefined)
  ifneq ($(wildcard ../../core/physics_world.c),)
    MPE_SRC = ../..
  else ifneq ($(wildcard ../475-MPE/v15S/src/core/physics_world.c),)
    MPE_SRC = ../475-MPE/v15S/src
  else ifneq ($(MFS_ENGINE_SRC),)
    MPE_SRC = $(MFS_ENGINE_SRC)
  else
    $(error MPE engine not found. Run make from v15S/src/ecosystem/mfs, or set MPE_SRC=/path/to/475-MPE/v15S/src (sibling layout projects/461-MFS + projects/475-MPE also works))
  endif
endif
MFS = .
BUILD = build

CC ?= gcc
# ---- Windows detection (Linux unchanged) ----
MPE_WINDOWS ?= $(strip $(if $(filter Windows_NT,$(OS)),1,$(if $(findstring mingw,$(CC)),1,$(if $(findstring MINGW,$(shell uname -s 2>/dev/null)),1,$(if $(findstring MSYS,$(shell uname -s 2>/dev/null)),1,)))))
ifeq ($(MPE_WINDOWS),1)
  PLUGIN_EXT := .dll
  EXE_EXT := .exe
  MPE_DL_LIBS :=
  MPE_RDYNAMIC :=
  MPE_WIN_LIBS := -lwinmm -lxinput -lws2_32 -static-libgcc -Wl,-Bstatic -lwinpthread -Wl,-Bdynamic
  MPE_FPIC :=
else
  PLUGIN_EXT := .so
  EXE_EXT :=
  MPE_DL_LIBS := -ldl
  MPE_RDYNAMIC := -rdynamic
  MPE_WIN_LIBS :=
  MPE_FPIC := -fPIC
endif
# NOTE (float builds, FIX-AUDIT-DESPOT): -O3 here vs -O2 in build_tests.sh.
# Both pass -ffp-contract=off, but -O3 vectorises/reorders FP math, so .so
# results are not promised bit-identical to the -O2 test binaries
# (IEEE-exact only within one build config + libm).
# -MMD -MP: emit a .d sidecar per object so header edits force a rebuild.
# CRITICAL, not tidiness: these objects are compiled against ENGINE headers
# (core/rigidbody.h defines the rigidbody layout the plugin indexes into
# physics_world->bodies). Without dependency tracking, changing an engine
# header left mfs_ecosystem.so "up to date" with a stale sizeof(rigidbody),
# so the plugin read the bodies array at the wrong stride and robot spawn
# failed with "no fleet attached". Append-only fields do NOT save you here:
# the struct SIZE still changes. See README_MFS.md build-invariants.
CFLAGS = -I$(MPE_SRC) -I$(MFS) -O3 -Wall -Wextra -ffp-contract=off $(MPE_FPIC) -fno-inline -fno-lto -fno-inline-functions-called-once -fvisibility=default \
         -DMPE_MODULE_ABI=1 -DMPE_ECOSYSTEM_ABI=1 -DMPE_GTK4=1 -MMD -MP

include mfs_sources.mk

# MFS-only objects (engine stays in the host). gui_robot_registry.c is
# deliberately EXCLUDED: it includes mpe_engine.h -> gtk/gtk.h and bypasses
# the module system with globals (see audit D19); it is not part of any
# loadable module.
MOD1_SRCS = modules/module_1/mfs_module_1.c modules/module_1/submodules/gamepad/gamepad.c
MFS_MOD1_OBJS = $(patsubst %.c,$(BUILD)/%.o,$(MOD1_SRCS))
# MFS_FTC_SRCS is engine-relative (see mfs_sources.mk); strip the prefix
# for local builds. DUAL-MODE: the strip is a no-op in standalone checkouts
# (paths have no prefix there), so fall back to the explicit MFS-relative
# list when any entry still carries the prefix.
MFS_FTC_LOCAL_TMP = $(patsubst ecosystem/mfs/%,%,$(MFS_FTC_SRCS))
MFS_FTC_STANDALONE_SRCS = modules/ftc/ftc_module.c \
    modules/ftc/ftc_fleet.c \
    modules/ftc/submodules/robot.c \
    modules/ftc/submodules/drivetrain.c \
    modules/ftc/submodules/motor.c \
    modules/ftc/submodules/motor_presets.c \
    modules/ftc/submodules/battery.c
MFS_FTC_LOCAL = $(if $(filter ecosystem/mfs/%,$(MFS_FTC_LOCAL_TMP)),$(MFS_FTC_STANDALONE_SRCS),$(MFS_FTC_LOCAL_TMP))
MFS_FTC_MOD_OBJS = $(patsubst %.c,$(BUILD)/%.o,$(MFS_FTC_LOCAL))
MFS_ECO_SRCS = mfs_ecosystem.c mfs_internal.c
MFS_ECO_OBJS = $(patsubst %.c,$(BUILD)/%.o,$(MFS_ECO_SRCS))

ALL_MFS_OBJS = $(MFS_MOD1_OBJS) $(MFS_FTC_MOD_OBJS) $(MFS_ECO_OBJS)
ALL_MFS_DEPS = $(ALL_MFS_OBJS:.o=.d) $(patsubst %.c,$(BUILD)/eng/%.d,$(MFS_ENGINE_SRCS))

all: mfs_module_1$(PLUGIN_EXT) mfs_ecosystem$(PLUGIN_EXT) plugins/mpe_ftc$(PLUGIN_EXT)

# NOTE: patterns match with MFS (=.) as CWD. Run make from ecosystem/mfs/.
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# mfs_ecosystem is a BUNDLE: it references mfs_module_1_desc and the
# FTC mpe_module_desc directly, so it links those objects in (unlike the
# thin single-module plugins below, which resolve engine symbols against
# the host at load).
MFS_BUNDLE_OBJS = $(MFS_ECO_OBJS) $(MFS_MOD1_OBJS) $(MFS_FTC_MOD_OBJS)

# Windows thin DLLs must link against the host import lib (Linux .so allows
# undefined, Windows DLLs do not). Engine Makefile generates libengine.a
# on Windows (engine.exe export lib). build_tests.sh handles this via the
# suite host lib; `make` here uses the engine lib when present.
ifeq ($(MPE_WINDOWS),1)
  MPE_HOST_LIB := $(MPE_SRC)/libengine.a
else
  MPE_HOST_LIB :=
endif

mfs_module_1$(PLUGIN_EXT): $(MFS_MOD1_OBJS)
ifeq ($(MPE_WINDOWS),1)
	@if [ -f "$(MPE_HOST_LIB)" ]; then $(CC) $(CFLAGS) -shared $(MFS_MOD1_OBJS) "$(MPE_HOST_LIB)" -lm $(MPE_DL_LIBS) $(MPE_WIN_LIBS) -o mfs_module_1$(PLUGIN_EXT); else echo "Windows: need $(MPE_HOST_LIB) (build engine first: make -C ../.. engine) or use build_tests.sh (suite host)"; exit 1; fi
else
	$(CC) $(CFLAGS) -shared $(MFS_MOD1_OBJS) -lm $(MPE_DL_LIBS) $(MPE_WIN_LIBS) -o mfs_module_1$(PLUGIN_EXT)
endif

mfs_ecosystem$(PLUGIN_EXT): $(MFS_BUNDLE_OBJS)
ifeq ($(MPE_WINDOWS),1)
	@if [ -f "$(MPE_HOST_LIB)" ]; then $(CC) $(CFLAGS) -shared $(MFS_BUNDLE_OBJS) "$(MPE_HOST_LIB)" -lm $(MPE_DL_LIBS) $(MPE_WIN_LIBS) -o mfs_ecosystem$(PLUGIN_EXT); else echo "Windows: need $(MPE_HOST_LIB) (build engine first: make -C ../.. engine) or use build_tests.sh (suite host)"; exit 1; fi
else
	$(CC) $(CFLAGS) -shared $(MFS_BUNDLE_OBJS) -lm $(MPE_DL_LIBS) $(MPE_WIN_LIBS) -o mfs_ecosystem$(PLUGIN_EXT)
endif

plugins/mpe_ftc$(PLUGIN_EXT): $(MFS_FTC_MOD_OBJS)
	@mkdir -p plugins
ifeq ($(MPE_WINDOWS),1)
	@if [ -f "$(MPE_HOST_LIB)" ]; then $(CC) $(CFLAGS) -shared $(MFS_FTC_MOD_OBJS) "$(MPE_HOST_LIB)" -lm $(MPE_DL_LIBS) $(MPE_WIN_LIBS) -o plugins/mpe_ftc$(PLUGIN_EXT); else echo "Windows: need $(MPE_HOST_LIB) (build engine first: make -C ../.. engine) or use build_tests.sh (suite host)"; exit 1; fi
else
	$(CC) $(CFLAGS) -shared $(MFS_FTC_MOD_OBJS) -lm $(MPE_DL_LIBS) $(MPE_WIN_LIBS) -o plugins/mpe_ftc$(PLUGIN_EXT)
endif

# Linux-compat aliases (scripts/tests referencing .so keep working on Linux;
# on Windows both .so (MSYS2) and .dll (native) are accepted by the loader).
# Only defined when PLUGIN_EXT differs (Windows) to avoid self-dependency.
ifeq ($(PLUGIN_EXT),.dll)
mfs_module_1.so: mfs_module_1$(PLUGIN_EXT)
	@cp mfs_module_1$(PLUGIN_EXT) mfs_module_1.so 2>/dev/null || true
mfs_ecosystem.so: mfs_ecosystem$(PLUGIN_EXT)
	@cp mfs_ecosystem$(PLUGIN_EXT) mfs_ecosystem.so 2>/dev/null || true
plugins/mpe_ftc.so: plugins/mpe_ftc$(PLUGIN_EXT)
	@cp plugins/mpe_ftc$(PLUGIN_EXT) plugins/mpe_ftc.so 2>/dev/null || true
endif

# module_1 drives through drivetrain_update: link the FTC submodule
# objects too (not ftc_module.o — unneeded here), plus engine objects
# (single test binary, not a plugin: no host to resolve against).
# (FIX-AUDIT-DESPOT: the duplicate `include mfs_sources.mk` that lived here
# is removed; the top include already defines MFS_ENGINE_SRCS/FTC_SRCS.)
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
	    $(MFS_MOD1_TEST_EXTRA) $(MFS_ENGINE_OBJS) -lm $(MPE_DL_LIBS) $(MPE_WIN_LIBS) -pthread -o $(BUILD)/test_mfs_module_1$(EXE_EXT)
	MPE_GAMEPAD_DEVICE=disabled ./$(BUILD)/test_mfs_module_1$(EXE_EXT)

clean:
	rm -rf $(BUILD) mfs_module_1.so mfs_module_1.dll mfs_ecosystem.so mfs_ecosystem.dll plugins/mpe_ftc.so plugins/mpe_ftc.dll test_mfs_module_1 test_mfs_module_1.exe

-include $(ALL_MFS_DEPS)

.PHONY: all test clean
