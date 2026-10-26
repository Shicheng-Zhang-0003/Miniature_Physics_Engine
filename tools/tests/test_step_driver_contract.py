"""Guard the single-step-driver invariant established 2026-10-23.

The GUI callback used to carry a second, hand-maintained copy of the entire
physics pipeline. That copy drifted repeatedly (unclamped drag, stale sleep
maxima, integrate-before-narrow, late manifolds after split). It is now a thin
fixed-timestep accumulator over ``physics_world_step()``.

These tests are structural on purpose: they do not try to re-derive physics,
they fail if a stage sequence is reintroduced into the GUI driver, or if the
canonical step stops publishing the overlay telemetry the driver forwards.
Behavioral equivalence was separately proven by a 600-tick GUI-vs-canonical
twin hash (three-cube stack, identical state hash and pair/manifold counts).
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / "v16R1" / "src" / "core" / "simulation_physics_loop.c"
CANONICAL = ROOT / "v16R1" / "src" / "core" / "physics_world.c"
WORLD_HEADER = ROOT / "v16R1" / "src" / "core" / "physics_world.h"

# Physics stages that must live in physics_world_step(), not in a driver.
FORBIDDEN_STAGE_CALLS = (
    "broadphase_generate_pairing",
    "physics_world_process_pair",
    "collision_prepare_solver",
    "collision_resolve_iterative",
    "collision_apply_poisson",
    "collision_apply_split_impulse",
    "collision_apply_rolling_resistance",
    "constraint_solve_all",
    "constraint_pre_step_all",
    "islands_build",
    "rb_integrate_velocity",
    "rb_integrate_position",
    "a3_positional_depenetration_pass",
    "collision_ccd_sweep_clamp_world",
)


class StepDriverContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.driver = DRIVER.read_text(encoding="utf-8")
        self.canonical = CANONICAL.read_text(encoding="utf-8")
        self.header = WORLD_HEADER.read_text(encoding="utf-8")

    def test_driver_calls_the_canonical_step_exactly_once(self) -> None:
        calls = re.findall(r"\bphysics_world_step\s*\(world,", self.driver)
        self.assertEqual(len(calls), 1, "the GUI driver must have exactly one step call site")
        self.assertIn("while (physics_time_accumulator >= (double) fixed_physics_dt)", self.driver)

    def test_driver_contains_no_physics_stage_sequence(self) -> None:
        for symbol in FORBIDDEN_STAGE_CALLS:
            self.assertNotIn(symbol, self.driver, f"{symbol} belongs in physics_world_step()")
        # A reintroduced pipeline cannot stay this small.
        self.assertLess(len(self.driver.splitlines()), 150,
                        "GUI driver grew past an accumulator; move physics into the canonical step")

    def test_canonical_step_publishes_overlay_telemetry(self) -> None:
        self.assertIn("world -> last_broadphase_pair_count = pair_count;", self.canonical)
        self.assertIn("world -> last_manifold_count = manifold_count;", self.canonical)
        # Reset before the degraded-path early returns so stale values cannot leak.
        reset = self.canonical.index("world -> last_broadphase_pair_count = 0;")
        self.assertLess(reset, self.canonical.index("if ((!world -> bodies)"))
        self.assertIn("int last_broadphase_pair_count;", self.header)
        self.assertIn("int last_manifold_count;", self.header)

    def test_driver_forwards_canonical_counts_to_legacy_overlay(self) -> None:
        self.assertIn("debug_last_broadphase_pair_count = world -> last_broadphase_pair_count;", self.driver)
        self.assertIn("debug_last_manifold_count = world -> last_manifold_count;", self.driver)
        self.assertIn("debug_last_manifold_overflow_count", self.driver)


if __name__ == "__main__":
    unittest.main()
