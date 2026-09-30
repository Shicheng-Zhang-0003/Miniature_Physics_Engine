# MPE engine makefile (MPE-only).
CC = gcc
# ---- Windows / MSYS2 / MinGW detection (Linux unchanged when absent) ----
# Auto-detect: OS=Windows_NT (cmd/mingw32-make), uname MINGW/MSYS/CYGWIN,
# or CC containing mingw (cross x86_64-w64-mingw32-gcc). Override with
# `make MPE_WINDOWS=1` (force Windows) or `make MPE_WINDOWS=0` (force POSIX).
MPE_WINDOWS ?= $(strip $(if $(filter Windows_NT,$(OS)),1,$(if $(findstring mingw,$(CC)),1,$(if $(findstring MINGW,$(shell uname -s 2>/dev/null)),1,$(if $(findstring MSYS,$(shell uname -s 2>/dev/null)),1,)))))
ifeq ($(MPE_WINDOWS),1)
  PLUGIN_EXT := .dll
  EXE_EXT := .exe
  # No -ldl/-rdynamic on Windows (LoadLibrary shim in mpe_platform.h);
  # no X11 (Wayland/X11 code is #ifdef GDK_WINDOWING_X11-guarded).
  # winpthreads provides -pthread on MinGW; static-link it + libgcc so
  # .exe/.dll run without MSYS2 DLLs in PATH (system libs stay dynamic).
  MPE_RDYNAMIC :=
  MPE_WIN_LIBS := -lws2_32 -lwinmm -static-libgcc -Wl,-Bstatic -lwinpthread -Wl,-Bdynamic
else
  PLUGIN_EXT := .so
  EXE_EXT :=
  MPE_RDYNAMIC := -rdynamic
  MPE_WIN_LIBS :=
endif
MPE_TEMP := $(abspath ../../temp)
export TMPDIR := $(MPE_TEMP)
ENABLE_NATIVE ?= 0
NATIVE_FLAG := $(if $(filter 1,$(ENABLE_NATIVE)),-march=native,)
# GTK4 only. The GTK3 code path was removed from the tree on 2026-09-29:
# 6,832 lines of never-compiled duplicate across 18 files in ui_input/ (the
# whole directory carried a full GTK3/GTK4 body copy of every handler). The
# v15R3 GTK3 engine is preserved in git history, and RELEASE_POLICY already
# records v15S as the GTK4 port, so keeping a second unbuildable copy in-tree
# only risked the two copies drifting -- which it had, to the point where a
# one-line fix (the never-cleared `e_key_pressed` latch) had to be applied
# twice, once per copy.
# `make GTK_PKG=gtk+-3.0` now fails loudly instead of silently offering a
# build that can no longer link.
GTK_PKG ?= gtk4
ifeq ($(GTK_PKG),gtk4)
   GTK_CFLAGS := $(shell pkg-config --cflags gtk4 epoxy)
   GTK_LIBS   := $(shell pkg-config --libs gtk4 epoxy)
   GTK_DEFINES = -DMPE_GTK4=1 $(GTK_X11_DEFINES) $(WAYLAND_REL)
ifeq ($(MPE_WINDOWS),1)
   X11_LIBS    :=
   GTK_X11_DEFINES :=
else
   X11_LIBS    := -lX11
   # DESPOT-2026-09-29: the X11 pointer re-centre warp was guarded on
   # GDK_WINDOWING_X11, a GTK3 macro that GTK4 does not define, so under
   # v15S the warp was compiled out on X11 as well -- mouse lock was broken
   # on BOTH backends. Restore it behind an explicit define.
   GTK_X11_DEFINES := -DMPE_GTK4_X11_WARP=1
endif
# Wayland relative-pointer lock (DESPOT-2026-09-29): the root fix for mouse
# lock on Wayland. Not available on Windows.
ifeq ($(MPE_WINDOWS),1)
   WAYLAND_REL    :=
   WAYLAND_LIBS   :=
else
   WAYLAND_REL    := -DMPE_WAYLAND_RELATIVE_POINTER=1
   WAYLAND_LIBS   := -lwayland-client
endif
else
   $(error GTK3 support was removed 2026-09-29; the tree is GTK4-only. Use \
     GTK_PKG=gtk4 (the default). The v15R3 GTK3 engine is in git history.)
endif
ifneq (,$(filter 0,$(MPE_WINDOWS)))
   X11_LIBS    :=
endif
BASE_CFLAGS = -g $(GTK_CFLAGS) -I. $(NATIVE_FLAG) $(GTK_DEFINES) -Wall -Wextra -MMD -MP -ffp-contract=off
ifeq ($(MPE_WINDOWS),1)
BASE_LIBS = $(GTK_LIBS) $(X11_LIBS) -lm -pthread $(MPE_WIN_LIBS)
else
BASE_LIBS = $(GTK_LIBS) $(X11_LIBS) $(WAYLAND_LIBS) -lm -ldl -rdynamic
endif
# Opt-in strict warnings: `make WARNINGS_AS_ERRORS=1` appends -Werror.
WARNINGS_AS_ERRORS ?= 0
WERROR_FLAG := $(if $(filter 1,$(WARNINGS_AS_ERRORS)),-Werror,)
BASE_CFLAGS += $(WERROR_FLAG)
# Determinism flags are hard-appended after $(CFLAGS) on every compile/link
# so environment CFLAGS can never silently drop -ffp-contract=off (bitwise
# twins depend on it). NATIVE builds emit a portability warning (see below).
DETERMINISM_FLAGS = -ffp-contract=off -fno-fast-math
# Command-line CFLAGS/LIBS (e.g. compile_asan) override these defaults.
CFLAGS ?= $(BASE_CFLAGS) -O3
LIBS ?= $(BASE_LIBS)
# TRUTH: hard-append determinism flags so NEITHER environment NOR
# command-line CFLAGS can silently drop -ffp-contract=off (bitwise twins
# depend on it). DESPOT-2026-09-29: plain `CFLAGS +=` is a NO-OP for
# command-line variables, and `CFLAGS ?=` does not assign when CFLAGS comes
# from the ENVIRONMENT -- so both of the ordinary ways a developer or CI sets
# flags (`make CFLAGS="-O2"` and `CFLAGS=-O2 make`) silently disabled FMA
# contraction AND dropped -MMD -MP, breaking header dependency tracking. That
# made the project's headline determinism guarantee opt-out by accident.
# `override` makes the append unconditional, so every one of the 40 recipes
# that reference $(CFLAGS) is covered without touching each of them.
override CFLAGS += $(DETERMINISM_FLAGS)
# DESPOT-2026-09-29: the same trap applied to dependency generation. -MMD -MP
# lives in BASE_CFLAGS, which `CFLAGS ?= $(BASE_CFLAGS) -O3` only reaches when
# CFLAGS is unset -- so `CFLAGS=-O2 make` silently stopped producing .d files,
# and header edits stopped triggering rebuilds. Appended with `override` so it
# survives an environment or command-line CFLAGS.
override CFLAGS += -MMD -MP

BUILD_DIR ?= build
# ENGINE_SRCS uses a wildcard (not an explicit list) so new engine TUs are
# picked up automatically; `make check-new-files` surfaces untracked .c
# files that the wildcard would silently absorb without review. Explicit
# exclusions: tests/ (headless mains with their own #ifdef guards must
# never link into the engine binary).
ENGINE_SRCS := $(wildcard *.c config/*.c core/*.c physics/*.c render/*.c scene/*.c ui_input/*.c) ecosystem/mpe_ecosystem.c
ifneq ($(MPE_WINDOWS),1)
  # wayland-scanner output for zwp_relative_pointer_manager_v1 (mouse lock).
  ENGINE_SRCS += $(wildcard ui_input/wayland/*.c)
endif
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(ENGINE_SRCS))
DEPS := $(OBJECTS:.o=.d)

engine: $(OBJECTS)
ifeq ($(MPE_WINDOWS),1)
	$(CC) $(CFLAGS) $(DETERMINISM_FLAGS) $(OBJECTS) $(LIBS) -Wl,--export-all-symbols -Wl,--out-implib,libengine.a -o engine$(EXE_EXT)
else
	$(CC) $(CFLAGS) $(DETERMINISM_FLAGS) $(OBJECTS) $(LIBS) -o engine$(EXE_EXT)
endif

# ---- Headless test binary (no GTK/GL deps) ----
headless:
	$(CC) $(CFLAGS) -DMPE_HEADLESS headless_main.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_headless$(EXE_EXT)

.PHONY: headless test_two_world

# Debug build (separate object dir so objects never mix with release).
debug:
	$(MAKE) BUILD_DIR=build-debug CFLAGS="$(BASE_CFLAGS) -O0 -g" engine

# Optimised release rebuild from scratch.
release:
	$(MAKE) BUILD_DIR=build-release CFLAGS="$(BASE_CFLAGS) -O3 -DNDEBUG" engine

PREFIX ?= $(HOME)/.local
install: engine mpe-tui plugins/mpe_capsule$(PLUGIN_EXT)
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 0755 engine$(EXE_EXT) $(DESTDIR)$(PREFIX)/bin/mpe-engine$(EXE_EXT)
	install -m 0755 mpe-tui$(EXE_EXT) $(DESTDIR)$(PREFIX)/bin/mpe-tui$(EXE_EXT)
	install -d $(DESTDIR)$(PREFIX)/share/mpe/shaders
	install -m 0644 render/shaders/*.glsl $(DESTDIR)$(PREFIX)/share/mpe/shaders/ || true
	install -d $(DESTDIR)$(PREFIX)/lib/mpe/plugins
	install -m 0644 plugins/*$(PLUGIN_EXT) $(DESTDIR)$(PREFIX)/lib/mpe/plugins/ || true
	install -d $(DESTDIR)$(PREFIX)/share/mpe/status
	@echo "NOTE: run from v15S/src or set MPE_SHADER_DIR=$(PREFIX)/share/mpe/shaders"

uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/mpe-engine $(DESTDIR)$(PREFIX)/bin/mpe-tui
	rm -rf $(DESTDIR)$(PREFIX)/share/mpe $(DESTDIR)$(PREFIX)/lib/mpe

installcheck:
	@echo "--- installcheck: full MPE verification profile ---"
	python3 ../../tools/test_runner.py --profile full

installcheck-v1:
	@echo "--- installcheck-v1: isolated legacy + paranoia regressions ---"
	python3 ../../tools/test_runner.py --profile physics

check-deps:
	@if [ "$(MPE_WINDOWS)" = "1" ]; then pkg-config --exists gtk4 epoxy || (echo "missing deps (Windows): need gtk4 epoxy (MSYS2: pacman -S mingw-w64-x86_64-gtk4 mingw-w64-x86_64-libepoxy)"; exit 1); else pkg-config --exists gtk4 epoxy x11 || (echo "missing deps: need gtk4 epoxy x11"; exit 1); fi
	@echo "deps OK (engine). For mpe-tui also: pkg-config --exists ncurses"

check-deps-tui:
	@pkg-config --exists ncurses || (echo "missing dep for mpe-tui: need ncurses"; exit 1)
	@echo "tui deps OK"

check-flags:
	@if [ "$(ENABLE_NATIVE)" = "1" ] && [ "$(ALLOW_NONPORTABLE)" != "1" ]; then echo "FAIL: ENABLE_NATIVE=1 breaks bitwise determinism; set ALLOW_NONPORTABLE=1 to override"; exit 1; fi
	@if [ "$(ENABLE_NATIVE)" = "1" ]; then echo "WARNING: NATIVE BUILD - NOT BITWISE PORTABLE"; fi
	@echo "$(CFLAGS)" | grep -q "ffp-contract=off" || (echo "FAIL: -ffp-contract=off missing from CFLAGS"; exit 1)
	@echo "flags OK"

check-new-files:
	@git status --porcelain -- '*.c' '*.h' | grep -q '^??' && { echo "new untracked sources (review ENGINE_SRCS wildcard):"; git status --porcelain -- '*.c' '*.h' | grep '^??'; exit 1; } || echo "no untracked sources"

help:
	@echo "targets: engine (default) | debug | release | install | installcheck | clean | distclean"
	@echo "         test_<name> (build+run) | build_<name> (build only) | paranoia_*"
	@echo "         test_runner.py --profile quick|physics|full (default: full, JUnit + JSON reports)"
	@echo "         build_suite | test_suite | test_suite_all (canonical C regression suite)"
	@echo "         mpe-tui (terminal debugger + snapshot suite) | tui-smoke | plugins/*.so"
	@echo "         mfs_ecosystem.so (delegates to ecosystem/mfs/Makefile)"
	@echo "         check-deps | check-deps-tui | check-flags | check-new-files"
	@echo "         WARNINGS_AS_ERRORS=1 for -Werror"
	@echo "         flow: python3 ../../tools/test_runner.py --profile full"

# MFS lives in its distinct folder: ecosystem/mfs/ (see ecosystem/mfs/README_MFS.md).
# It builds itself (ecosystem/mfs/Makefile, objects in ecosystem/mfs/build/);
# the generic $(BUILD_DIR)/%.o rule below is engine-only.

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(DETERMINISM_FLAGS) -c $< -o $@

clean:
	rm -rf build build-debug build-release engine engine.exe test_* mpe-tui mpe-tui.exe *.so *.dll *.a *.lib plugins/*.so plugins/*.dll
	find . -name '*.d' -delete
	@echo "NOTE: user data status/engine.cfg preserved (use distclean to remove)"

distclean: clean
	rm -rf status/engine.cfg status/engine.cfg.backup

# ---- MFS overarching ecosystem (v15S/src/ecosystem/mfs/) ----
# Single build lives in ecosystem/mfs/Makefile (thin .so files, build/ objs).
# This target only delegates and stages the artifact here; do NOT duplicate
# the recipe (a duplicated recipe already drifted once: GUI registry linked
# here, excluded there).
mfs_ecosystem$(PLUGIN_EXT):
	$(MAKE) -C ecosystem/mfs mfs_ecosystem$(PLUGIN_EXT)
	cp ecosystem/mfs/mfs_ecosystem$(PLUGIN_EXT) ./mfs_ecosystem$(PLUGIN_EXT)

# ---- MPE terminal debugger + snapshot suite (TUI) ----
# Terminal-only executable: live ncurses inspector (overview / object+math /
# joints / graph / help) plus pipeable --snapshot/--stream dumps that serve
# as the scriptable debugging output suite. Links the headless-tested core
# plus springs; never calls GTK/GL at runtime (spring GL render is linked
# for its physics symbols but never invoked — output is terminal only).
TUI_SRCS := tui/tui_main.c tui/tui_debugger.c tui/tui_dump.c
TUI_CORE = $(MPE_TEST_CORE) $(SPRING_TEST_SRCS)
TUI_OUT ?= ../../temp/tui_smoke
# epoxy kept: spring_joint.o render path references GL symbols even though
# the TUI never calls it at runtime (physics symbols live in the same TU).
ifeq ($(MPE_WINDOWS),1)
TUI_LIBS := -lm $(shell pkg-config --libs ncursesw 2>/dev/null || pkg-config --libs ncurses 2>/dev/null || echo -lncurses) $(shell pkg-config --libs epoxy 2>/dev/null || echo "") $(MPE_WIN_LIBS)
else
TUI_LIBS := -lm $(shell pkg-config --libs ncurses) $(shell pkg-config --libs epoxy) -ldl
endif
build_tui: $(TUI_SRCS) $(TUI_CORE)
	$(CC) $(CFLAGS) -Dmpe_tui_main $(TUI_SRCS) $(TUI_CORE) $(TUI_LIBS) -o mpe-tui$(EXE_EXT)
mpe-tui: build_tui
tui-smoke: build_tui
	mkdir -p $(TUI_OUT)
	./mpe-tui$(EXE_EXT) --snapshot 60 --scene demo > $(TUI_OUT)/demo_snapshot.log
	./mpe-tui$(EXE_EXT) --stream 120 --every 60 --scene demo > $(TUI_OUT)/demo_stream.log
	./mpe-tui$(EXE_EXT) --snapshot 10 --scene tower > $(TUI_OUT)/tower.log
	./mpe-tui$(EXE_EXT) --snapshot 10 --scene pendulum > $(TUI_OUT)/pendulum.log
	./mpe-tui$(EXE_EXT) --snapshot 10 --scene springlab > $(TUI_OUT)/springlab.log
	./mpe-tui$(EXE_EXT) --snapshot 600 --scene f10 > $(TUI_OUT)/f10.log
	./mpe-tui$(EXE_EXT) --snapshot 600 --scene stress > $(TUI_OUT)/stress.log
	./mpe-tui$(EXE_EXT) --snapshot 60 --scene ccd > $(TUI_OUT)/ccd.log
	./mpe-tui$(EXE_EXT) --snapshot 60 --scene demo --solver seq-impulse --broadphase hash > $(TUI_OUT)/demo_hash.log
	@echo "[PASS] tui-smoke: all scenes dump finite state"

.PHONY: clean distclean debug release install installcheck check-deps check-deps-tui check-flags check-new-files help mpe-tui build_tui tui-smoke plugins mfs_ecosystem$(PLUGIN_EXT) ecosystem/mfs/mfs_ecosystem$(PLUGIN_EXT) headless build_suite test_suite test_suite_all

-include $(DEPS)

# ---- MPE-only headless tests ----
# Convention: build_<name> compiles only (used by tools/test_runner.py so a
# failing test is never misreported as a build failure); test_<name> builds
# and runs (human use).
# TRUE god file split: collision_mechanics.c (2305 lines) → 4 focused TUs
MPE_TEST_CORE := core/physics_world.c core/rigidbody.c core/mpe_registry.c core/mpe_loader.c core/det_math.c core/mpe_primary.c \
	physics/collision_narrowphase.c physics/collision_cache.c physics/collision_solver.c physics/collision_ccd.c physics/collision_cylinder.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c physics/depenetration.c physics/islands.c \
	config/mpe_config.c config/mpe_config_schema.c scene/boundary.c ecosystem/mpe_ecosystem.c
ifeq ($(MPE_WINDOWS),1)
MPE_TEST_LIBS := -lm -pthread $(MPE_WIN_LIBS)
else
MPE_TEST_LIBS := -lm -ldl -pthread
endif

build_two_world: tests/two_world_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_two_world_test tests/two_world_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_two_world$(EXE_EXT)
test_two_world: build_two_world
	./test_two_world$(EXE_EXT)

build_revolute: tests/revolute_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_revolute_test tests/revolute_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_revolute$(EXE_EXT)
test_revolute: build_revolute
	./test_revolute$(EXE_EXT)

build_cylinder_drop: tests/cylinder_drop_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_cylinder_drop_test tests/cylinder_drop_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_cylinder_drop$(EXE_EXT)
test_cylinder_drop: build_cylinder_drop
	./test_cylinder_drop$(EXE_EXT)

build_driven_wheel: tests/driven_wheel_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_driven_wheel_test tests/driven_wheel_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_driven_wheel$(EXE_EXT)
test_driven_wheel: build_driven_wheel
	./test_driven_wheel$(EXE_EXT)

build_math3_inverse: tests/math3_inverse_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_math3_inverse_test tests/math3_inverse_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_math3_inverse$(EXE_EXT)
test_math3_inverse: build_math3_inverse
	./test_math3_inverse$(EXE_EXT)

build_floor_collision_diag: tests/floor_collision_diag.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_floor_diag tests/floor_collision_diag.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_floor_collision_diag$(EXE_EXT)
test_floor_collision_diag: build_floor_collision_diag
	./test_floor_collision_diag$(EXE_EXT)

build_cylinder_sphere: tests/cylinder_sphere_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmfs_cyl_sph_test tests/cylinder_sphere_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_cylinder_sphere$(EXE_EXT)
test_cylinder_sphere: build_cylinder_sphere
	./test_cylinder_sphere$(EXE_EXT)

build_cylinder_cube: tests/cylinder_cube_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmfs_cyl_cube_test tests/cylinder_cube_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_cylinder_cube$(EXE_EXT)
test_cylinder_cube: build_cylinder_cube
	./test_cylinder_cube$(EXE_EXT)

build_cylinder_cylinder: tests/cylinder_cylinder_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmfs_cyl_cyl_test tests/cylinder_cylinder_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_cylinder_cylinder$(EXE_EXT)
test_cylinder_cylinder: build_cylinder_cylinder
	./test_cylinder_cylinder$(EXE_EXT)

build_list4_cylinder_floor: tests/list4_cylinder_floor_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dlist4_cylinder_floor_test tests/list4_cylinder_floor_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_list4_cylinder_floor$(EXE_EXT)
test_list4_cylinder_floor: build_list4_cylinder_floor
	./test_list4_cylinder_floor$(EXE_EXT)

SCENE_TEST_SRCS := scene/scene_load.c scene/scene_saving.c scene/scene_id_remap.c scene/scene_crc.c
build_scene_roundtrip: tests/scene_roundtrip_test.c $(MPE_TEST_CORE) $(SCENE_TEST_SRCS)
	$(CC) $(CFLAGS) -Dmpe_scene_roundtrip_test tests/scene_roundtrip_test.c $(MPE_TEST_CORE) $(SCENE_TEST_SRCS) $(MPE_TEST_LIBS) -o test_scene_roundtrip$(EXE_EXT)
test_scene_roundtrip: build_scene_roundtrip
	mkdir -p ../../temp
	./test_scene_roundtrip$(EXE_EXT)

build_static_hold: tests/static_hold_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_static_hold_test tests/static_hold_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_static_hold$(EXE_EXT)
test_static_hold: build_static_hold
	./test_static_hold$(EXE_EXT)

build_rolling_decay: tests/rolling_decay_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_rolling_decay_test tests/rolling_decay_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_rolling_decay$(EXE_EXT)
test_rolling_decay: build_rolling_decay
	./test_rolling_decay$(EXE_EXT)

build_ccd_sweep: tests/ccd_sweep_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_ccd_sweep_test tests/ccd_sweep_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_ccd_sweep$(EXE_EXT)
test_ccd_sweep: build_ccd_sweep
	./test_ccd_sweep$(EXE_EXT)

build_kinematic: tests/kinematic_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_kinematic_test tests/kinematic_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_kinematic$(EXE_EXT)
test_kinematic: build_kinematic
	./test_kinematic$(EXE_EXT)

build_determinism: tests/determinism_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_determinism_test tests/determinism_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_determinism$(EXE_EXT)
test_determinism: build_determinism
	./test_determinism$(EXE_EXT)

build_momentum: tests/momentum_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_momentum_test tests/momentum_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_momentum$(EXE_EXT)
test_momentum: build_momentum
	./test_momentum$(EXE_EXT)

build_angmom: tests/angmom_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_angmom_test tests/angmom_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_angmom$(EXE_EXT)
test_angmom: build_angmom
	./test_angmom$(EXE_EXT)

SPRING_TEST_SRCS := physics/spring_joint.c
build_spring: tests/spring_test.c $(MPE_TEST_CORE) $(SPRING_TEST_SRCS)
	$(CC) $(CFLAGS) -Dmpe_spring_test tests/spring_test.c $(MPE_TEST_CORE) $(SPRING_TEST_SRCS) $(MPE_TEST_LIBS) $(shell pkg-config --libs epoxy) -o test_spring$(EXE_EXT)
test_spring: build_spring
	./test_spring$(EXE_EXT)

build_projectile: tests/projectile_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_projectile_test tests/projectile_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_projectile$(EXE_EXT)
test_projectile: build_projectile
	./test_projectile$(EXE_EXT)

build_incline_accel: tests/incline_accel_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_incline_accel_test tests/incline_accel_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_incline_accel$(EXE_EXT)
test_incline_accel: build_incline_accel
	./test_incline_accel$(EXE_EXT)

build_pendulum: tests/pendulum_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_pendulum_test tests/pendulum_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_pendulum$(EXE_EXT)
test_pendulum: build_pendulum
	./test_pendulum$(EXE_EXT)

build_bounce_series: tests/bounce_series_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_bounce_series_test tests/bounce_series_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_bounce_series$(EXE_EXT)
test_bounce_series: build_bounce_series
	./test_bounce_series$(EXE_EXT)

build_friction_stop: tests/friction_stop_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_friction_stop_test tests/friction_stop_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_friction_stop$(EXE_EXT)
test_friction_stop: build_friction_stop
	./test_friction_stop$(EXE_EXT)

build_stack: tests/stack_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_stack_test tests/stack_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_stack$(EXE_EXT)
test_stack: build_stack
	./test_stack$(EXE_EXT)

build_f10_long_run: tests/f10_long_run_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_f10_long_run_test tests/f10_long_run_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_f10_long_run$(EXE_EXT)
test_f10_long_run: build_f10_long_run
	./test_f10_long_run$(EXE_EXT)

build_sleep_contact_wake: tests/sleep_contact_wake_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_sleep_contact_wake_test tests/sleep_contact_wake_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_sleep_contact_wake$(EXE_EXT)
test_sleep_contact_wake: build_sleep_contact_wake
	./test_sleep_contact_wake$(EXE_EXT)

build_f11_torture: tests/f11_torture_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_f11_torture_test tests/f11_torture_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_f11_torture$(EXE_EXT)
test_f11_torture: build_f11_torture
	./test_f11_torture$(EXE_EXT)
# Pure math4 (header-only): no MPE_TEST_CORE needed, no GL needed.
build_frustum: tests/frustum_test.c
	$(CC) $(CFLAGS) -Dmpe_frustum_test tests/frustum_test.c -lm -o test_frustum$(EXE_EXT)
test_frustum: build_frustum
	./test_frustum$(EXE_EXT)

build_module: tests/module_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_module_test tests/module_test.c $(MPE_TEST_CORE) $(MPE_TEST_LIBS) -o test_module$(EXE_EXT)
test_module: build_module
	./test_module$(EXE_EXT)

# ---- Foreign physics plugins (.so hot-plug) ----
# NOTE: plugins use the engine's registry (no registry objects linked in);
# engine exports symbols via $(MPE_RDYNAMIC) so dlopened .so resolves
# mpe_register_pair_handler at load time.
plugins/mpe_capsule$(PLUGIN_EXT): plugins/mpe_capsule.c core/mpe_module.h core/mpe_registry.h
	mkdir -p plugins
	$(CC) $(BASE_CFLAGS) -fPIC -shared plugins/mpe_capsule.c -o plugins/mpe_capsule$(PLUGIN_EXT)
# ---- Suite v2: single-binary C regression suite (from-scratch rewrite) ----
# One binary, exact-name dispatch, shared builders (no 30x core recompile).
# Links core + spring physics + scene I/O (stubs for scene_init/camera live
# in tests/mpe_suite_main.c). Replaces the 30-binary v1 layout over time.
MPE_SUITE_SRCS := tests/mpe_suite_main.c tests/mpe_suite_a.c tests/mpe_suite_b.c tests/mpe_suite_c.c tests/mpe_suite_loader.c tests/mpe_suite_ftc.c
MPE_SUITE_DEPS := $(MPE_TEST_CORE) $(SPRING_TEST_SRCS) $(SCENE_TEST_SRCS)
# The ftc_ecosystem test needs the bundle fixture: build it first.
build_suite: $(MPE_SUITE_SRCS) $(MPE_SUITE_DEPS) ecosystem/mfs/mfs_ecosystem$(PLUGIN_EXT) plugins/mpe_capsule$(PLUGIN_EXT)
	$(CC) $(CFLAGS) $(MPE_RDYNAMIC) $(MPE_SUITE_SRCS) $(MPE_TEST_CORE) $(SPRING_TEST_SRCS) $(SCENE_TEST_SRCS) $(MPE_TEST_LIBS) $(shell pkg-config --libs epoxy) -o test_mpe_suite$(EXE_EXT)

ecosystem/mfs/mfs_ecosystem$(PLUGIN_EXT):
	$(MAKE) -C ecosystem/mfs mfs_ecosystem$(PLUGIN_EXT)
test_suite: build_suite
	mkdir -p ../../temp
	./test_mpe_suite$(EXE_EXT)
test_suite_all: build_suite
	mkdir -p ../../temp
	./test_mpe_suite$(EXE_EXT) --all

plugins: plugins/mpe_capsule$(PLUGIN_EXT)

.PHONY: clean build_two_world test_two_world build_revolute test_revolute \
	build_cylinder_drop test_cylinder_drop build_driven_wheel test_driven_wheel \
	build_math3_inverse test_math3_inverse build_floor_collision_diag test_floor_collision_diag \
	build_cylinder_sphere test_cylinder_sphere build_cylinder_cube test_cylinder_cube \
	build_cylinder_cylinder test_cylinder_cylinder build_list4_cylinder_floor test_list4_cylinder_floor \
	build_scene_roundtrip test_scene_roundtrip build_static_hold test_static_hold \
	build_rolling_decay test_rolling_decay build_ccd_sweep test_ccd_sweep \
	build_kinematic test_kinematic build_determinism test_determinism \
	build_momentum test_momentum build_angmom test_angmom build_spring test_spring \
	build_projectile test_projectile build_incline_accel test_incline_accel \
	build_pendulum test_pendulum build_bounce_series test_bounce_series \
	build_friction_stop test_friction_stop build_stack test_stack \
	build_f10_long_run test_f10_long_run build_sleep_contact_wake test_sleep_contact_wake \
	build_f11_torture test_f11_torture \
	build_frustum test_frustum \
	build_paranoia_freeflight_exact test_paranoia_freeflight_exact \
	build_paranoia_depenetration_sleep test_paranoia_depenetration_sleep \
	build_paranoia_contact_solver test_paranoia_contact_solver \
	build_paranoia_constraints test_paranoia_constraints \
	build_paranoia_broadphase test_paranoia_broadphase \
	build_paranoia_energy_momentum test_paranoia_energy_momentum \
	build_paranoia_ccd test_paranoia_ccd \
	build_paranoia_cylinder_collision test_paranoia_cylinder_collision \
	build_paranoia_determinism test_paranoia_determinism \
	build_paranoia_scene_persistence test_paranoia_scene_persistence \
	build_paranoia_spring_joints test_paranoia_spring_joints \
	build_paranoia_integration_edges test_paranoia_integration_edges \
	build_paranoia_fp_edges test_paranoia_fp_edges \
	build_paranoia_config test_paranoia_config

# ---- Paranoia tests (extreme edge cases, mathematical truth verification) ----
PARANOIA_TEST_CORE := $(MPE_TEST_CORE)
PARANOIA_CFLAGS := $(CFLAGS) -DMPE_HEADLESS

build_paranoia_freeflight_exact: tests/paranoia_freeflight_exact.c $(PARANOIA_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_paranoia_freeflight_exact tests/paranoia_freeflight_exact.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_freeflight_exact$(EXE_EXT)
test_paranoia_freeflight_exact: build_paranoia_freeflight_exact
	./test_paranoia_freeflight_exact$(EXE_EXT)

build_paranoia_depenetration_sleep: tests/paranoia_depenetration_sleep.c $(PARANOIA_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_paranoia_depenetration_sleep tests/paranoia_depenetration_sleep.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_depenetration_sleep$(EXE_EXT)
test_paranoia_depenetration_sleep: build_paranoia_depenetration_sleep
	./test_paranoia_depenetration_sleep$(EXE_EXT)

build_paranoia_contact_solver: tests/paranoia_contact_solver.c $(PARANOIA_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_paranoia_contact_solver tests/paranoia_contact_solver.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_contact_solver$(EXE_EXT)
test_paranoia_contact_solver: build_paranoia_contact_solver
	./test_paranoia_contact_solver$(EXE_EXT)

build_paranoia_constraints: tests/paranoia_constraints.c $(PARANOIA_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_paranoia_constraints tests/paranoia_constraints.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_constraints$(EXE_EXT)
test_paranoia_constraints: build_paranoia_constraints
	./test_paranoia_constraints$(EXE_EXT)

build_paranoia_broadphase: tests/paranoia_broadphase.c $(PARANOIA_TEST_CORE)
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_broadphase tests/paranoia_broadphase.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_broadphase$(EXE_EXT)
test_paranoia_broadphase: build_paranoia_broadphase
	./test_paranoia_broadphase$(EXE_EXT)

build_paranoia_energy_momentum: tests/paranoia_energy_momentum.c $(PARANOIA_TEST_CORE)
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_energy_momentum tests/paranoia_energy_momentum.c $(PARANOIA_TEST_CORE) $(SPRING_TEST_SRCS) $(MPE_TEST_LIBS) $(shell pkg-config --libs epoxy) -o test_paranoia_energy_momentum$(EXE_EXT)
test_paranoia_energy_momentum: build_paranoia_energy_momentum
	./test_paranoia_energy_momentum$(EXE_EXT)

build_paranoia_ccd: tests/paranoia_ccd.c $(PARANOIA_TEST_CORE)
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_ccd tests/paranoia_ccd.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_ccd$(EXE_EXT)
test_paranoia_ccd: build_paranoia_ccd
	./test_paranoia_ccd$(EXE_EXT)

build_paranoia_cylinder_collision: tests/paranoia_cylinder_collision.c $(PARANOIA_TEST_CORE)
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_cylinder_collision tests/paranoia_cylinder_collision.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_cylinder_collision$(EXE_EXT)
test_paranoia_cylinder_collision: build_paranoia_cylinder_collision
	./test_paranoia_cylinder_collision$(EXE_EXT)

build_paranoia_determinism: tests/paranoia_determinism.c $(PARANOIA_TEST_CORE)
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_determinism tests/paranoia_determinism.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_determinism$(EXE_EXT)
test_paranoia_determinism: build_paranoia_determinism
	./test_paranoia_determinism$(EXE_EXT)

build_paranoia_scene_persistence: tests/paranoia_scene_persistence.c $(PARANOIA_TEST_CORE) physics/spring_joint.c scene/scene_init.c scene/scene_load.c scene/scene_saving.c scene/scene_id_remap.c scene/scene_crc.c
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_scene_persistence tests/paranoia_scene_persistence.c $(PARANOIA_TEST_CORE) physics/spring_joint.c scene/scene_init.c scene/scene_load.c scene/scene_saving.c scene/scene_id_remap.c scene/scene_crc.c $(MPE_TEST_LIBS) $(shell pkg-config --libs epoxy) -o test_paranoia_scene_persistence$(EXE_EXT)
test_paranoia_scene_persistence: build_paranoia_scene_persistence
	mkdir -p ../../temp
	./test_paranoia_scene_persistence$(EXE_EXT)

build_paranoia_spring_joints: tests/paranoia_spring_joints.c $(PARANOIA_TEST_CORE) physics/spring_joint.c
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_spring_joints tests/paranoia_spring_joints.c $(PARANOIA_TEST_CORE) physics/spring_joint.c $(MPE_TEST_LIBS) $(shell pkg-config --libs epoxy) -o test_paranoia_spring_joints$(EXE_EXT)
test_paranoia_spring_joints: build_paranoia_spring_joints
	./test_paranoia_spring_joints$(EXE_EXT)

build_paranoia_integration_edges: tests/paranoia_integration_edges.c $(PARANOIA_TEST_CORE)
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_integration_edges tests/paranoia_integration_edges.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_integration_edges$(EXE_EXT)
test_paranoia_integration_edges: build_paranoia_integration_edges
	./test_paranoia_integration_edges$(EXE_EXT)

build_paranoia_fp_edges: tests/paranoia_fp_edges.c $(PARANOIA_TEST_CORE)
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_fp_edges tests/paranoia_fp_edges.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_fp_edges$(EXE_EXT)
test_paranoia_fp_edges: build_paranoia_fp_edges
	./test_paranoia_fp_edges$(EXE_EXT)

build_paranoia_config: tests/paranoia_config.c $(PARANOIA_TEST_CORE)
	$(CC) $(PARANOIA_CFLAGS) -Dmpe_paranoia_config tests/paranoia_config.c $(PARANOIA_TEST_CORE) $(MPE_TEST_LIBS) -o test_paranoia_config$(EXE_EXT)
test_paranoia_config: build_paranoia_config
	mkdir -p ../../temp
	./test_paranoia_config$(EXE_EXT)
