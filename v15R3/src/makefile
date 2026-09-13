# MPE engine makefile (MPE-only).
CC = gcc
ENABLE_NATIVE ?= 0
NATIVE_FLAG := $(if $(filter 1,$(ENABLE_NATIVE)),-march=native,)
BASE_CFLAGS = $(shell pkg-config --cflags gtk+-3.0 epoxy) -I. $(NATIVE_FLAG) -Wall -Wextra -MMD -MP -ffp-contract=off
BASE_LIBS = $(shell pkg-config --libs gtk+-3.0 epoxy) -lm
# Command-line CFLAGS/LIBS (e.g. compile_asan) override these defaults.
CFLAGS ?= $(BASE_CFLAGS) -O3
LIBS ?= $(BASE_LIBS)

BUILD_DIR ?= build
# Explicit engine sources: everything except tests/ (headless mains with
# their own #ifdef guards must never link into the engine binary).
ENGINE_SRCS := $(wildcard *.c config/*.c core/*.c physics/*.c render/*.c scene/*.c ui_input/*.c)
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(ENGINE_SRCS))
DEPS := $(OBJECTS:.o=.d)

engine: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(LIBS) -o engine

# Debug build (separate object dir so objects never mix with release).
debug:
	$(MAKE) BUILD_DIR=build-debug CFLAGS="$(BASE_CFLAGS) -O0 -g" engine

# Optimised release rebuild from scratch.
release:
	$(MAKE) BUILD_DIR=build-release CFLAGS="$(BASE_CFLAGS) -O3 -DNDEBUG" engine

PREFIX ?= $(HOME)/.local
install: engine
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 0755 engine $(DESTDIR)$(PREFIX)/bin/mpe-engine

help:
	@echo "targets: engine (default) | debug | release | install | clean"
	@echo "         test_<name> (build+run) | build_<name> (build only)"
	@echo "         flow: python3 ../../tools/test_runner.py"

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf build build-debug build-release engine test_*

.PHONY: clean debug release install help

-include $(DEPS)

# ---- MPE-only headless tests ----
# Convention: build_<name> compiles only (used by tools/test_runner.py so a
# failing test is never misreported as a build failure); test_<name> builds
# and runs (human use).
MPE_TEST_CORE := core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c physics/depenetration.c physics/islands.c \
	config/mpe_config.c config/mpe_config_schema.c scene/boundary.c

build_two_world: tests/two_world_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_two_world_test tests/two_world_test.c $(MPE_TEST_CORE) -lm -o test_two_world
test_two_world: build_two_world
	./test_two_world

build_revolute: tests/revolute_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_revolute_test tests/revolute_test.c $(MPE_TEST_CORE) -lm -o test_revolute
test_revolute: build_revolute
	./test_revolute

build_cylinder_drop: tests/cylinder_drop_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_cylinder_drop_test tests/cylinder_drop_test.c $(MPE_TEST_CORE) -lm -o test_cylinder_drop
test_cylinder_drop: build_cylinder_drop
	./test_cylinder_drop

build_driven_wheel: tests/driven_wheel_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_driven_wheel_test tests/driven_wheel_test.c $(MPE_TEST_CORE) -lm -o test_driven_wheel
test_driven_wheel: build_driven_wheel
	./test_driven_wheel

build_math3_inverse: tests/math3_inverse_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_math3_inverse_test tests/math3_inverse_test.c $(MPE_TEST_CORE) -lm -o test_math3_inverse
test_math3_inverse: build_math3_inverse
	./test_math3_inverse

build_floor_collision_diag: tests/floor_collision_diag.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_floor_diag tests/floor_collision_diag.c $(MPE_TEST_CORE) -lm -o test_floor_collision_diag
test_floor_collision_diag: build_floor_collision_diag
	./test_floor_collision_diag

build_cylinder_sphere: tests/cylinder_sphere_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmfs_cyl_sph_test tests/cylinder_sphere_test.c $(MPE_TEST_CORE) -lm -o test_cylinder_sphere
test_cylinder_sphere: build_cylinder_sphere
	./test_cylinder_sphere

build_cylinder_cube: tests/cylinder_cube_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmfs_cyl_cube_test tests/cylinder_cube_test.c $(MPE_TEST_CORE) -lm -o test_cylinder_cube
test_cylinder_cube: build_cylinder_cube
	./test_cylinder_cube

build_cylinder_cylinder: tests/cylinder_cylinder_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmfs_cyl_cyl_test tests/cylinder_cylinder_test.c $(MPE_TEST_CORE) -lm -o test_cylinder_cylinder
test_cylinder_cylinder: build_cylinder_cylinder
	./test_cylinder_cylinder

build_list4_cylinder_floor: tests/list4_cylinder_floor_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dlist4_cylinder_floor_test tests/list4_cylinder_floor_test.c $(MPE_TEST_CORE) -lm -o test_list4_cylinder_floor
test_list4_cylinder_floor: build_list4_cylinder_floor
	./test_list4_cylinder_floor

SCENE_TEST_SRCS := scene/scene_load.c scene/scene_saving.c scene/scene_id_remap.c scene/scene_crc.c
build_scene_roundtrip: tests/scene_roundtrip_test.c $(MPE_TEST_CORE) $(SCENE_TEST_SRCS)
	$(CC) $(CFLAGS) -Dmpe_scene_roundtrip_test tests/scene_roundtrip_test.c $(MPE_TEST_CORE) $(SCENE_TEST_SRCS) -lm -o test_scene_roundtrip
test_scene_roundtrip: build_scene_roundtrip
	./test_scene_roundtrip

build_static_hold: tests/static_hold_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_static_hold_test tests/static_hold_test.c $(MPE_TEST_CORE) -lm -o test_static_hold
test_static_hold: build_static_hold
	./test_static_hold

build_rolling_decay: tests/rolling_decay_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_rolling_decay_test tests/rolling_decay_test.c $(MPE_TEST_CORE) -lm -o test_rolling_decay
test_rolling_decay: build_rolling_decay
	./test_rolling_decay

build_ccd_sweep: tests/ccd_sweep_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_ccd_sweep_test tests/ccd_sweep_test.c $(MPE_TEST_CORE) -lm -o test_ccd_sweep
test_ccd_sweep: build_ccd_sweep
	./test_ccd_sweep

build_kinematic: tests/kinematic_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_kinematic_test tests/kinematic_test.c $(MPE_TEST_CORE) -lm -o test_kinematic
test_kinematic: build_kinematic
	./test_kinematic

build_determinism: tests/determinism_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_determinism_test tests/determinism_test.c $(MPE_TEST_CORE) -lm -o test_determinism
test_determinism: build_determinism
	./test_determinism

build_momentum: tests/momentum_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_momentum_test tests/momentum_test.c $(MPE_TEST_CORE) -lm -o test_momentum
test_momentum: build_momentum
	./test_momentum

build_angmom: tests/angmom_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_angmom_test tests/angmom_test.c $(MPE_TEST_CORE) -lm -o test_angmom
test_angmom: build_angmom
	./test_angmom

SPRING_TEST_SRCS := physics/spring_joint.c
build_spring: tests/spring_test.c $(MPE_TEST_CORE) $(SPRING_TEST_SRCS)
	$(CC) $(CFLAGS) -Dmpe_spring_test tests/spring_test.c $(MPE_TEST_CORE) $(SPRING_TEST_SRCS) -lm $(shell pkg-config --libs epoxy) -o test_spring
test_spring: build_spring
	./test_spring

build_projectile: tests/projectile_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_projectile_test tests/projectile_test.c $(MPE_TEST_CORE) -lm -o test_projectile
test_projectile: build_projectile
	./test_projectile

build_incline_accel: tests/incline_accel_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_incline_accel_test tests/incline_accel_test.c $(MPE_TEST_CORE) -lm -o test_incline_accel
test_incline_accel: build_incline_accel
	./test_incline_accel

build_pendulum: tests/pendulum_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_pendulum_test tests/pendulum_test.c $(MPE_TEST_CORE) -lm -o test_pendulum
test_pendulum: build_pendulum
	./test_pendulum

build_bounce_series: tests/bounce_series_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_bounce_series_test tests/bounce_series_test.c $(MPE_TEST_CORE) -lm -o test_bounce_series
test_bounce_series: build_bounce_series
	./test_bounce_series

build_friction_stop: tests/friction_stop_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_friction_stop_test tests/friction_stop_test.c $(MPE_TEST_CORE) -lm -o test_friction_stop
test_friction_stop: build_friction_stop
	./test_friction_stop

build_stack: tests/stack_test.c $(MPE_TEST_CORE)
	$(CC) $(CFLAGS) -Dmpe_stack_test tests/stack_test.c $(MPE_TEST_CORE) -lm -o test_stack
test_stack: build_stack
	./test_stack

# Pure math4 (header-only): no MPE_TEST_CORE needed, no GL needed.
build_frustum: tests/frustum_test.c
	$(CC) $(CFLAGS) -Dmpe_frustum_test tests/frustum_test.c -lm -o test_frustum
test_frustum: build_frustum
	./test_frustum

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
	build_frustum test_frustum
