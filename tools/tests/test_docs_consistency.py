"""Cross-check user-facing docs against the code they describe.

These are deliberately narrow: each test pins one drift that was found in the
2026-10-23 review. They run inside the runner-contract phase, so a future
change that updates the code but not the prose fails the quick profile instead
of shipping a plausible lie.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
V15S = ROOT / "v15S"


def read(rel: str) -> str:
    return (ROOT / rel).read_text(encoding="utf-8")


class DocsConsistencyTests(unittest.TestCase):
    def test_shooter_default_matches_the_code_and_user_guide(self) -> None:
        intake = read("v15S/src/ecosystem/mfs/modules/ftc/submodules/intake.c")
        match = re.search(r"s_launch_elev_deg\s*=\s*([0-9.]+)f", intake)
        self.assertIsNotNone(match, "intake.c launch-elevation default not found")
        default = match.group(1).rstrip("0").rstrip(".")
        guide = read("v15S/how_to_use.md")
        self.assertIn(f"| Shooter launch elevation (deg) | {default} |", guide)
        # The stock aim text used to say 50° after the code moved to 65°.
        self.assertNotRegex(guide, r"\b50°")
        self.assertNotIn("defaults 1.0, 50", guide)

    def test_windows_support_story_is_consistent(self) -> None:
        guide = read("v15S/how_to_use.md")
        limits = read("v15S/KNOWN_LIMITATIONS.md")
        self.assertIn("Windows is unsupported as a release target", guide)
        self.assertIn("Windows build exists", limits)
        self.assertNotIn("Windows is not supported.", guide)

    def test_tui_stage_swap_commands_are_not_stubs(self) -> None:
        tui = read("v15S/src/tui/tui_shell.c")
        self.assertNotIn("(registry-based, stubbed)", tui)
        self.assertIn("mpe_find_broadphase", tui)
        self.assertIn("mpe_find_solver", tui)

    def test_joint_pool_status_is_not_contradictory(self) -> None:
        work = read("v15S/REMAINING_WORK.md")
        self.assertIn("- [x] Joint pools cleared inside `physics_world_init`", work)
        self.assertNotIn("- [ ] Uninitialized joint pools", work)

    def test_reference_materials_comment_matches_the_ignore_rule(self) -> None:
        ignore = read(".gitignore")
        self.assertNotIn("references stay", ignore)
        self.assertRegex(ignore, r"(?m)^reference_materials/\s*$")

    def test_empty_world_memory_claim_matches_the_allocator(self) -> None:
        header = read("v15S/src/core/physics_world.h")
        architecture = read("v15S/ARCHITECTURE.md")
        self.assertNotIn("costs kilobytes instead of megabytes", header)
        self.assertNotIn("An empty world costs kilobytes", architecture)
        self.assertIn("low single-digit MB", header)

    def test_step_path_docs_describe_one_canonical_step(self) -> None:
        limits = read("v15S/KNOWN_LIMITATIONS.md")
        work = read("v15S/REMAINING_WORK.md")
        self.assertNotIn("Unification is a maintenance item", limits)
        self.assertIn("both now call the same `physics_world_step()`", limits)
        self.assertIn("Superseded 2026-10-23", work)
        self.assertIn("- [x] Replace the duplicate GUI and headless physics pipelines", work)
        self.assertNotIn("Unification stays a\n  maintenance item", work)


if __name__ == "__main__":
    unittest.main()
