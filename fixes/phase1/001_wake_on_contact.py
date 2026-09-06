#!/usr/bin/env python3
"""
MPE Phase 1.2-prep #1: Wake-on-contact extraction
==================================================
Ports the legacy simulation_physics_loop.c wake-on-contact logic into
physics_world.c as a standalone helper.

WHY THIS IS REQUIRED BEFORE KILLING LEGACY:
physics_world_step does NOT staticize sleeping bodies (the hack was removed).
So a sleeping body keeps real mass and the solver WILL apply impulses to it,
but rb_integrate_velocity / rb_integrate_position skip sleeping bodies.
Without waking it here, you get velocity-without-motion (silent corruption)
and F6 (sleep/wake) breaks. This extraction restores correct behavior.

Idempotent. Run: python3 fixes/phase1/001_wake_on_contact.py
"""
import sys, subprocess
from pathlib import Path

DRY_RUN = "--dry-run" in sys.argv

def find_root():
    p = Path(__file__).resolve().parent
    while p != p.parent:
        if (p / "v15R3" / "src").exists():
            return p
        p = p.parent
    return None

ROOT = find_root()
if ROOT is None:
    print("FATAL: cannot locate project root containing v15R3/src")
    sys.exit(1)
SRC = ROOT / "v15R3" / "src"

MARKER_HELPER = "MPE_WAKE_ON_CONTACT_HELPER"
MARKER_CALL   = "MPE_WAKE_ON_CONTACT_CALL"

HELPER = '''/* MPE_WAKE_ON_CONTACT_HELPER: wake sleeping bodies struck by active bodies.
* Standalone extraction from the legacy simulation_physics_loop.c narrowphase.
* Required because physics_world_step does NOT staticize sleeping bodies, so a
* sleeping body in a contact manifold must be woken or it receives solver
* impulses it never integrates. */
static void physics_world_wake_on_contact(rigidbody *body_a, rigidbody *body_b) {
    bool a_was_sleeping = body_a->is_sleeping;
    bool b_was_sleeping = body_b->is_sleeping;
    float wake_linear_sq = g_cfg.sleep.wake_linear_thresh_sq;
    float wake_angular_sq = g_cfg.sleep.wake_angular_thresh_sq;
    bool a_is_active = (!a_was_sleeping) &&
        ((vector3_length_squared(body_a->velocity) > wake_linear_sq) ||
         (vector3_length_squared(body_a->angular_velocity) > wake_angular_sq));
    bool b_is_active = (!b_was_sleeping) &&
        ((vector3_length_squared(body_b->velocity) > wake_linear_sq) ||
         (vector3_length_squared(body_b->angular_velocity) > wake_angular_sq));
    if (a_was_sleeping && (!body_b->static_state) && b_is_active) {
        rigidbody_wake(body_a);
    }
    if (b_was_sleeping && (!body_a->static_state) && a_is_active) {
        rigidbody_wake(body_b);
    }
}

'''

TEST_C = r'''/* MPE Phase 1.2-prep #1: wake-on-contact test.
* A sleeping cube is struck by a fast projectile. The cube must wake. */
#ifdef MPE_SLEEP_WAKE_CONTACT_TEST
#include <stdio.h>
#include <math.h>
#include "core/physics_world.h"
#include "physics/constraint.h"
#include "config/mpe_config.h"

int main(void) {
    mpe_config_init();
    constraint_pool_init();
    physics_world world;
    physics_world_init(&world);

    /* Static floor (not strictly needed, kept for realism) */
    physics_world_add_cube(&world,
        (vector3){0.0f, -0.5f, 0.0f}, (vector3){10.0f, 0.5f, 10.0f}, 0.0f);

    /* Sleeping target cube, floating well above the floor */
    int target = physics_world_add_cube(&world,
        (vector3){0.0f, 2.0f, 0.0f}, (vector3){0.5f, 0.5f, 0.5f}, 2.0f);
    world.bodies[target].velocity = vector3_zero();
    world.bodies[target].angular_velocity = vector3_zero();
    world.bodies[target].is_sleeping = true;
    world.bodies[target].sleep_timer = 2.0f;

    /* Fast projectile aimed at the target along -Z */
    int projectile = physics_world_add_sphere(&world, 0.35f, 3.0f,
        (vector3){0.0f, 2.0f, 2.0f});
    world.bodies[projectile].velocity = (vector3){0.0f, 0.0f, -8.0f};

    const float dt = 1.0f / 60.0f;
    int woke = 0;
    int fail = 0;
    for (int t = 0; t < 60 && !fail; t++) {
        physics_world_step(&world, dt);
        for (int i = 0; i < world.body_count; i++) {
            if (!isfinite(world.bodies[i].position.x) ||
                !isfinite(world.bodies[i].position.y) ||
                !isfinite(world.bodies[i].position.z)) {
                printf("[FAIL] NaN at tick %d\n", t);
                fail = 1; break;
            }
        }
        if (!world.bodies[target].is_sleeping) {
            woke = 1;
        }
    }
    if (fail) return 1;
    if (!woke) {
        printf("[FAIL] sleeping cube was never woken by projectile impact\n");
        return 1;
    }
    printf("[PASS] wake-on-contact: sleeping cube woken by impact\n");
    physics_world_cleanup(&world);
    return 0;
}
#endif /* MPE_SLEEP_WAKE_CONTACT_TEST */
'''

MAKEFILE_TARGET = '''
# MPE Phase 1.2-prep #1: wake-on-contact test
WAKE_CONTACT_SOURCES := tests/sleep_wake_contact_test.c core/physics_world.c core/rigidbody.c \\
    physics/collision_mechanics.c physics/broadphase.c physics/constraint.c physics/revolute_joint.c \\
    config/mpe_config.c config/mpe_config_schema.c
test_sleep_wake_contact: $(WAKE_CONTACT_SOURCES)
\t$(CC) $(CFLAGS) -DMPE_SLEEP_WAKE_CONTACT_TEST $(WAKE_CONTACT_SOURCES) -lm -o test_sleep_wake_contact
\t./test_sleep_wake_contact
'''

def write(path, text):
    if DRY_RUN:
        print(f"[DRY] would write {path.relative_to(ROOT)} ({len(text)} bytes)")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    print(f"[WRITE] {path.relative_to(ROOT)}")

def patch_physics_world():
    pw = SRC / "core" / "physics_world.c"
    content = pw.read_text()
    changed = False

    # 1. Insert helper before physics_world_step
    if MARKER_HELPER not in content:
        anchor = "void physics_world_step(physics_world *world, float dt) {"
        if anchor not in content:
            print("[FAIL] could not find physics_world_step anchor")
            return False
        content = content.replace(anchor, HELPER + anchor, 1)
        changed = True
        print("[OK] inserted physics_world_wake_on_contact helper")
    else:
        print("[SKIP] helper already present")

    # 2. Insert call before collision_prepare_solver in narrowphase
    if MARKER_CALL not in content:
        old = "            collision_prepare_solver(&narrowphase_collision, &world_manifolds[manifold_count]);"
        new = ("            physics_world_wake_on_contact(body_a, body_b); /* MPE_WAKE_ON_CONTACT_CALL */\n"
               "            collision_prepare_solver(&narrowphase_collision, &world_manifolds[manifold_count]);")
        if old not in content:
            print("[FAIL] could not find collision_prepare_solver anchor")
            return False
        content = content.replace(old, new, 1)
        changed = True
        print("[OK] inserted wake call into narrowphase dispatch")
    else:
        print("[SKIP] wake call already present")

    if changed and not DRY_RUN:
        pw.write_text(content)
    return True

def main():
    print("=" * 60)
    print("MPE Phase 1.2-prep #1: Wake-on-contact extraction")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN **\n")

    if not patch_physics_world():
        return 1
    write(SRC / "tests" / "sleep_wake_contact_test.c", TEST_C)

    mk = SRC / "makefile"
    mk_content = mk.read_text()
    if "test_sleep_wake_contact:" not in mk_content:
        if DRY_RUN:
            print("[DRY] would append makefile target")
        else:
            mk.write_text(mk_content + MAKEFILE_TARGET)
            print("[WRITE] makefile target appended")
    else:
        print("[SKIP] makefile target already present")

    if not DRY_RUN:
        print("\n[RUN] building + running wake-on-contact test")
        rc = subprocess.call(["make", "test_sleep_wake_contact"], cwd=str(SRC))
        if rc != 0:
            print("[FAIL] wake-on-contact test failed")
            return 1
        print("\n[PASS] wake-on-contact extraction verified")
    return 0

if __name__ == "__main__":
    sys.exit(main())
