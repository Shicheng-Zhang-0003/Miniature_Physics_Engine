"""Contract tests for the MPE verification harness itself."""

from __future__ import annotations

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

from tools import test_runner as runner


class RegistryContractTests(unittest.TestCase):
    def test_c_registry_has_unique_names_and_diag_tags(self) -> None:
        entries = runner.discover_suite_entries('''
            {"alpha", test_alpha, 0},
            {"diagnostic", test_diag, 1},
        ''')
        self.assertEqual(entries, [("alpha", False), ("diagnostic", True)])

    def test_c_registry_rejects_duplicate_and_empty_registries(self) -> None:
        with self.assertRaisesRegex(ValueError, "duplicate"):
            runner.discover_suite_entries('{"same", f1, 0}, {"same", f2, 1},')
        with self.assertRaisesRegex(ValueError, "empty"):
            runner.discover_suite_entries("/* no cases */")

    def test_makefile_targets_split_isolated_suites(self) -> None:
        legacy, paranoia = runner.discover_make_targets('''
            build_alpha: tests/alpha_test.c
            build_paranoia_edge: tests/paranoia_edge.c
        ''')
        self.assertEqual(legacy, ["alpha"])
        self.assertEqual(paranoia, ["paranoia_edge"])

    def test_makefile_target_discovery_rejects_empty_inventory(self) -> None:
        with self.assertRaisesRegex(ValueError, "empty group"):
            runner.discover_make_targets("# no tests")


class OutputContractTests(unittest.TestCase):
    def test_suite_output_requires_every_registered_case_once(self) -> None:
        expected = [("alpha", False), ("diagnostic", True)]
        output = "  [PASS] alpha (checks failed: 0)\n  [PASS] diagnostic (checks failed: 0)\n"
        self.assertEqual(runner.parse_suite_output(output, expected), [
            ("PASS", "alpha", 0), ("PASS", "diagnostic", 0),
        ])

    def test_suite_output_rejects_missing_duplicate_and_unexpected_cases(self) -> None:
        expected = [("alpha", False)]
        with self.assertRaisesRegex(ValueError, "missing"):
            runner.parse_suite_output("", expected)
        with self.assertRaisesRegex(ValueError, "duplicate"):
            runner.parse_suite_output("  [PASS] alpha (checks failed: 0)\n  [PASS] alpha (checks failed: 0)", expected)
        with self.assertRaisesRegex(ValueError, "unexpected"):
            runner.parse_suite_output("  [PASS] other (checks failed: 0)", expected)

    def test_mfs_summary_requires_a_valid_gated_and_info_count(self) -> None:
        self.assertEqual(runner.parse_mfs_summary("FTC RESULT: gated pass=11 fail=0 info-diags=5"), (11, 0, 5))
        with self.assertRaisesRegex(ValueError, "summary"):
            runner.parse_mfs_summary("FTC RESULT: all good")

    def test_tui_snapshot_validation_catches_nonfinite_and_failed_frames(self) -> None:
        with tempfile.TemporaryDirectory(dir=runner.PROJECT_TEMP) as scratch:
            path = Path(scratch) / "snapshot.log"
            path.write_text("### MPE-TUI snapshot tick=1 time=0.0167 dt=0.01667 bodies=2 result=PASS\n", encoding="utf-8")
            self.assertEqual(runner.validate_tui_file(path), (True, "1 finite PASS frame(s)"))
            path.write_text("### MPE-TUI snapshot tick=1 time=nan dt=0.01667 bodies=2 result=PASS\n", encoding="utf-8")
            valid, detail = runner.validate_tui_file(path)
            self.assertFalse(valid)
            self.assertIn("NaN", detail)
            path.write_text("### MPE-TUI snapshot tick=1 time=0.1 dt=0.01 bodies=2 result=FAIL\n", encoding="utf-8")
            valid, detail = runner.validate_tui_file(path)
            self.assertFalse(valid)
            self.assertIn("failing frame", detail)

    def test_counts_separate_informational_results_from_gates(self) -> None:
        results = [
            runner.Result("gate-pass", "physics"),
            runner.Result("gate-fail", "physics", status="FAIL"),
            runner.Result("diag", "diagnostics", severity="info"),
        ]
        self.assertEqual(runner.result_counts(results), {
            "total": 3, "passed": 2, "failed": 1, "informational": 1, "blocking_failures": 1,
        })

    def test_json_and_junit_reports_are_valid(self) -> None:
        result = runner.Result("case", "suite", log="temp/qa_runs/example/case.log")
        with tempfile.TemporaryDirectory(dir=runner.PROJECT_TEMP) as scratch:
            out = Path(scratch)
            runner.write_reports(out, {"schema": "mpe-test-run/v1"}, [result])
            report = json.loads((out / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(report["counts"]["passed"], 1)
            self.assertEqual(report["results"][0]["name"], "case")
            self.assertEqual(runner.ET.parse(out / "junit.xml").getroot().tag, "testsuites")

    def test_unlaunchable_command_becomes_a_reported_failure(self) -> None:
        with tempfile.TemporaryDirectory(dir=runner.PROJECT_TEMP) as scratch:
            instance = runner.Runner("quick", Path(scratch))
            with contextlib.redirect_stdout(io.StringIO()):
                result, output = instance.command("missing-tool", "harness", ["mpe-command-that-does-not-exist"])
            self.assertEqual(result.status, "FAIL")
            self.assertIsNone(result.return_code)
            self.assertIn("failed to start", output)


if __name__ == "__main__":
    unittest.main()
