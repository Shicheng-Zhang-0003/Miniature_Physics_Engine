# A3_PATCH_31_INCREMENTAL_MAKEFILE
CC = gcc
# A3_PATCH_32_PORTABLE_GRID_RESTORE
ENABLE_NATIVE ?= 0
NATIVE_FLAG := $(if $(filter 1,$(ENABLE_NATIVE)),-march=native,)
CFLAGS = $(shell pkg-config --cflags gtk+-3.0 epoxy) -I. -O3 $(NATIVE_FLAG) -Wall -Wextra -MMD -MP
LIBS = $(shell pkg-config --libs gtk+-3.0 epoxy) -lm

BUILD_DIR := build
SOURCES := $(shell find . -name '*.c' | sed 's|^\./||')
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SOURCES))
DEPS := $(OBJECTS:.o=.d)

engine: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(LIBS) -o engine

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) engine test_*

.PHONY: clean

-include $(DEPS)

# MPE_FTC_059D: headless two-world independence test
TWO_WORLD_SOURCES := tests/two_world_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	config/mpe_config.c config/mpe_config_schema.c
test_two_world: $(TWO_WORLD_SOURCES)
	$(CC) $(CFLAGS) -DMPE_TWO_WORLD_TEST $(TWO_WORLD_SOURCES) -lm -o test_two_world
	./test_two_world

# MPE_FTC_068: headless revolute pendulum test
REVOLUTE_TEST_SOURCES := tests/revolute_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	config/mpe_config.c config/mpe_config_schema.c
test_revolute: $(REVOLUTE_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_REVOLUTE_TEST $(REVOLUTE_TEST_SOURCES) -lm -o test_revolute
	./test_revolute

# MPE_FTC_077: headless teleop drive test
TELEOP_TEST_SOURCES := tests/teleop_drive_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c
test_teleop_drive: $(TELEOP_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_TELEOP_DRIVE_TEST $(TELEOP_TEST_SOURCES) -lm -o test_teleop_drive
	./test_teleop_drive

# MPE_FTC_078: ASan+UBSan teleop drive test for heap-corruption diagnosis
test_teleop_drive_asan: $(TELEOP_TEST_SOURCES)
	$(CC) $(CFLAGS) -O1 -g -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer \
	  -DMPE_TELEOP_DRIVE_TEST $(TELEOP_TEST_SOURCES) -lm \
	  -fsanitize=address -fsanitize=undefined -o test_teleop_drive_asan
	./test_teleop_drive_asan

# MPE_FTC_080: headless mecanum drive test
MECANUM_TEST_SOURCES := tests/mecanum_drive_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c
test_mecanum_drive: $(MECANUM_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_MECANUM_DRIVE_TEST $(MECANUM_TEST_SOURCES) -lm -o test_mecanum_drive
	./test_mecanum_drive

# MPE_FTC_092: cylinder drop test
CYLINDER_DROP_TEST_SOURCES = tests/cylinder_drop_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_cylinder_drop: $(CYLINDER_DROP_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_CYLINDER_DROP_TEST $(CYLINDER_DROP_TEST_SOURCES) -lm -o test_cylinder_drop
	./test_cylinder_drop

# MPE_FTC_093a: driven-wheel propulsion proof
DRIVEN_WHEEL_TEST_SOURCES = tests/driven_wheel_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_driven_wheel: $(DRIVEN_WHEEL_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_DRIVEN_WHEEL_TEST $(DRIVEN_WHEEL_TEST_SOURCES) -lm -o test_driven_wheel
	./test_driven_wheel

# MPE_FTC_093e: math3_inverse diagnostic
MATH3_INV_TEST_SOURCES = tests/math3_inverse_test.c core/physics_world.c core/rigidbody.c physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c config/mpe_config.c config/mpe_config_schema.c

test_math3_inverse: $(MATH3_INV_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMPE_MATH3_INVERSE_TEST $(MATH3_INV_TEST_SOURCES) -lm -o test_math3_inverse
	./test_math3_inverse

# MFS_FTC_INTEGRATION: full FTC robot integration test
FTC_INTEGRATION_SOURCES := tests/ftc_integration_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c

test_ftc_integration: $(FTC_INTEGRATION_SOURCES)
	$(CC) $(CFLAGS) -DMPE_FTC_INTEGRATION_TEST $(FTC_INTEGRATION_SOURCES) -lm -o test_ftc_integration
	./test_ftc_integration

# MFS_FTC_DEBUG: diagnostic test
FTC_DEBUG_SOURCES := tests/ftc_debug_test.c core/physics_world.c core/rigidbody.c \
	physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
	robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
	config/mpe_config.c config/mpe_config_schema.c

test_ftc_debug: $(FTC_DEBUG_SOURCES)
	$(CC) $(CFLAGS) -DMPE_FTC_DEBUG_TEST $(FTC_DEBUG_SOURCES) -lm -o test_ftc_debug
	./test_ftc_debug

# MFS_133: Full physics truth test suite
PHYSICS_TRUTH_SOURCES := tests/physics_truth_test.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
config/mpe_config.c config/mpe_config_schema.c
test_physics_truth: $(PHYSICS_TRUTH_SOURCES)
	$(CC) $(CFLAGS) -DMPE_PHYSICS_TRUTH_TEST $(PHYSICS_TRUTH_SOURCES) -lm -o test_physics_truth
	./test_physics_truth

# MFS_135: diagnostic
DIAG_SOURCES := tests/physics_truth_diag.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
config/mpe_config.c config/mpe_config_schema.c
test_physics_truth_diag: $(DIAG_SOURCES)
	$(CC) $(CFLAGS) -DMPE_PHYSICS_TRUTH_DIAG $(DIAG_SOURCES) -lm -o test_physics_truth_diag
	./test_physics_truth_diag

# MFS_138: floor collision diagnostic
FLOOR_DIAG_SOURCES := tests/floor_collision_diag.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
config/mpe_config.c config/mpe_config_schema.c
test_floor_collision_diag: $(FLOOR_DIAG_SOURCES)
	$(CC) $(CFLAGS) -DMPE_FLOOR_DIAG $(FLOOR_DIAG_SOURCES) -lm -o test_floor_collision_diag
	./test_floor_collision_diag

# MFS_142: idle wheel-spin diagnostic
IDLE_DIAG_SOURCES := tests/idle_spin_diag.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
config/mpe_config.c config/mpe_config_schema.c
test_idle_spin_diag: $(IDLE_DIAG_SOURCES)
	$(CC) $(CFLAGS) -DMFS_IDLE_DIAG $(IDLE_DIAG_SOURCES) -lm -o test_idle_spin_diag
	./test_idle_spin_diag

# MFS_143: deep idle-spin diagnostic
IDLE_DEEP_SOURCES := tests/idle_spin_deep_diag.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
config/mpe_config.c config/mpe_config_schema.c
test_idle_spin_deep_diag: $(IDLE_DEEP_SOURCES)
	$(CC) $(CFLAGS) -DMFS_IDLE_DEEP_DIAG $(IDLE_DEEP_SOURCES) -lm -o test_idle_spin_deep_diag
	./test_idle_spin_deep_diag

# MFS_148: idle root-cause diagnostic
IDLE_RC_SOURCES := tests/idle_rootcause_diag.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
config/mpe_config.c config/mpe_config_schema.c
test_idle_rootcause_diag: $(IDLE_RC_SOURCES)
	$(CC) $(CFLAGS) -DMFS_IDLE_ROOTCAUSE_DIAG $(IDLE_RC_SOURCES) -lm -o test_idle_rootcause_diag
	./test_idle_rootcause_diag

# MFS_151: odometry diagnostic
ODOM_DIAG_SOURCES := tests/odometry_diag.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
config/mpe_config.c config/mpe_config_schema.c
test_odometry_diag: $(ODOM_DIAG_SOURCES)
	$(CC) $(CFLAGS) -DMFS_ODOM_DIAG $(ODOM_DIAG_SOURCES) -lm -o test_odometry_diag
	./test_odometry_diag

# MFS_167: tank drive differential turning test
TANK_TURN_TEST_SOURCES := tests/tank_turn_test.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
config/mpe_config.c config/mpe_config_schema.c
test_tank_turn: $(TANK_TURN_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMFS_TANK_TURN_TEST $(TANK_TURN_TEST_SOURCES) -lm -o test_tank_turn
	./test_tank_turn

# MFS_167: odometry accuracy test
ODOMETRY_ACC_TEST_SOURCES := tests/odometry_accuracy_test.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
robotics/motor.c robotics/motor_presets.c robotics/battery.c robotics/robot.c robotics/drivetrain.c \
config/mpe_config.c config/mpe_config_schema.c
test_odometry_accuracy: $(ODOMETRY_ACC_TEST_SOURCES)
	$(CC) $(CFLAGS) -DMFS_ODOMETRY_ACCURACY_TEST $(ODOMETRY_ACC_TEST_SOURCES) -lm -o test_odometry_accuracy
	./test_odometry_accuracy

# MFS_174: cylinder collision tests
CYL_SPH_SOURCES := tests/cylinder_sphere_test.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
config/mpe_config.c config/mpe_config_schema.c
test_cylinder_sphere: $(CYL_SPH_SOURCES)
	$(CC) $(CFLAGS) -DMFS_CYL_SPH_TEST $(CYL_SPH_SOURCES) -lm -o test_cylinder_sphere
	./test_cylinder_sphere

CYL_CUBE_SOURCES := tests/cylinder_cube_test.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
config/mpe_config.c config/mpe_config_schema.c
test_cylinder_cube: $(CYL_CUBE_SOURCES)
	$(CC) $(CFLAGS) -DMFS_CYL_CUBE_TEST $(CYL_CUBE_SOURCES) -lm -o test_cylinder_cube
	./test_cylinder_cube

CYL_CYL_SOURCES := tests/cylinder_cylinder_test.c core/physics_world.c core/rigidbody.c \
physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \
config/mpe_config.c config/mpe_config_schema.c
test_cylinder_cylinder: $(CYL_CYL_SOURCES)
	$(CC) $(CFLAGS) -DMFS_CYL_CYL_TEST $(CYL_CYL_SOURCES) -lm -o test_cylinder_cylinder
	./test_cylinder_cylinder
