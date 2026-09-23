# Canonical file lists (engine-relative paths). build_tests.sh embeds
# copies (MFS_ lists are MFS-relative there since it compiles from v15S/src
# with an ecosystem/mfs/ prefix) — update both files together.
# The Makefile builds thin MFS-only .so files and needs no engine objects.
MFS_ENGINE_SRCS = core/physics_world.c core/rigidbody.c \
    core/mpe_registry.c core/mpe_loader.c \
    core/det_math.c core/mpe_primary.c \
    physics/collision_narrowphase.c physics/collision_cache.c \
    physics/collision_solver.c physics/collision_ccd.c \
    physics/collision_cylinder.c physics/broadphase.c \
    physics/constraint.c physics/revolute_joint.c \
    physics/depenetration.c physics/islands.c \
    config/mpe_config.c config/mpe_config_schema.c \
    scene/boundary.c ecosystem/mpe_ecosystem.c
MFS_FTC_SRCS = ecosystem/mfs/modules/ftc/ftc_module.c \
    ecosystem/mfs/modules/ftc/ftc_fleet.c \
    ecosystem/mfs/modules/ftc/submodules/robot.c \
    ecosystem/mfs/modules/ftc/submodules/drivetrain.c \
    ecosystem/mfs/modules/ftc/submodules/motor.c \
    ecosystem/mfs/modules/ftc/submodules/motor_presets.c \
    ecosystem/mfs/modules/ftc/submodules/battery.c
