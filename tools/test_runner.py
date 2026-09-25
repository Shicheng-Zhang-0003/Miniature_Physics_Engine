#!/usr/bin/env python3
"""MPE verification runner.

Profiles:
  quick    Python runner-contract tests and the canonical C regression suite.
  physics  quick + every isolated legacy case and all paranoia cases.
  full     physics + ASan/UBSan suite, MFS robotics, TUI snapshots, engine build.

All run artifacts are written below the project-local temp/qa_runs directory.
This module uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence


PROJECT_ROOT = Path(__file__).resolve().parent.parent
PROJECT_TEMP = PROJECT_ROOT / "temp"
SRC_DIR = PROJECT_ROOT / "v15S" / "src"
MAKEFILE = SRC_DIR / "makefile"
SUITE_SOURCE = SRC_DIR / "tests" / "mpe_suite_main.c"
LEGACY_TARGET = re.compile(r"^[ \t]*build_([A-Za-z0-9_]+):\s+tests/([A-Za-z0-9_]+\.c)(?:\s|$)", re.M)
SUITE_ENTRY = re.compile(r'^\s*\{"([a-z0-9_]+)",\s*[^,]+,\s*([01])\},\s*$', re.M)
SUITE_RESULT = re.compile(r"^\s+\[(PASS|FAIL)\]\s+([a-z0-9_]+)\s+\(checks failed: (\d+)\)\s*$", re.M)
TUI_HEADER = re.compile(r"^### MPE-TUI snapshot tick=(\d+) time=([^ ]+) dt=([^ ]+) bodies=(\d+) result=(PASS|FAIL)\s*$", re.M)
NONFINITE_WORD = re.compile(r"(?<![A-Za-z])(?:nan|[+-]?inf(?:inity)?)(?![A-Za-z])", re.I)


@dataclass
class Result:
    name: str
    suite: str
    kind: str = "test"
    severity: str = "gate"
    status: str = "PASS"
    duration_s: float = 0.0
    return_code: int | None = 0
    log: str = ""
    detail: str = ""

    @property
    def blocking(self) -> bool:
        return self.severity == "gate" and self.status == "FAIL"


def discover_suite_entries(source: str) -> list[tuple[str, bool]]:
    """Read the single authoritative C registry and retain diag classification."""
    entries = [(name, diag == "1") for name, diag in SUITE_ENTRY.findall(source)]
    names = [name for name, _ in entries]
    if not entries or len(names) != len(set(names)):
        raise ValueError("canonical C suite registry is empty or has duplicate names")
    return entries


def discover_make_targets(makefile: str) -> tuple[list[str], list[str]]:
    """Find build-only test targets directly from their source prerequisites."""
    targets = [name for name, _source in LEGACY_TARGET.findall(makefile)]
    if len(targets) != len(set(targets)):
        raise ValueError("duplicate build-only test target in makefile")
    legacy = sorted(name for name in targets if not name.startswith("paranoia_"))
    paranoia = sorted(name for name in targets if name.startswith("paranoia_"))
    if not legacy or not paranoia:
        raise ValueError("makefile test target discovery found an empty group")
    return legacy, paranoia


def parse_suite_output(output: str, expected: Sequence[tuple[str, bool]]) -> list[tuple[str, str, int]]:
    """Require one result line per registered test; missing output is a failure."""
    rows = [(status, name, int(failures)) for status, name, failures in SUITE_RESULT.findall(output)]
    got = [name for _status, name, _failures in rows]
    want = [name for name, _diag in expected]
    if len(got) != len(set(got)):
        raise ValueError("canonical suite emitted duplicate test result lines")
    if set(got) != set(want):
        missing = sorted(set(want) - set(got))
        unexpected = sorted(set(got) - set(want))
        raise ValueError(f"canonical suite result mismatch; missing={missing}, unexpected={unexpected}")
    return rows


def parse_mfs_summary(output: str) -> tuple[int, int, int]:
    match = re.search(r"FTC RESULT: gated pass=(\d+) fail=(\d+) info-diags=(\d+)", output)
    if not match:
        raise ValueError("MFS runner did not emit its gated/info summary")
    return tuple(map(int, match.groups()))


def validate_tui_file(path: Path) -> tuple[bool, str]:
    if not path.is_file() or path.stat().st_size == 0:
        return False, "snapshot artifact missing or empty"
    content = path.read_text(encoding="utf-8", errors="replace")
    headers = TUI_HEADER.findall(content)
    if not headers:
        return False, "no machine-readable snapshot header"
    bad = [row for row in headers if row[4] != "PASS"]
    if bad:
        return False, f"snapshot reported {len(bad)} failing frame(s)"
    if NONFINITE_WORD.search(content):
        return False, "snapshot contains NaN or infinity"
    for row in headers:
        try:
            float(row[1])
            float(row[2])
            int(row[3])
        except ValueError:
            return False, "snapshot header has malformed numeric fields"
    return True, f"{len(headers)} finite PASS frame(s)"


def result_counts(results: Iterable[Result]) -> dict[str, int]:
    rows = list(results)
    return {
        "total": len(rows),
        "passed": sum(r.status == "PASS" for r in rows),
        "failed": sum(r.status == "FAIL" for r in rows),
        "informational": sum(r.severity == "info" for r in rows),
        "blocking_failures": sum(r.blocking for r in rows),
    }


def write_reports(run_dir: Path, metadata: dict, results: Sequence[Result]) -> None:
    counts = result_counts(results)
    report = {**metadata, "counts": counts, "results": [asdict(r) for r in results]}
    (run_dir / "summary.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    root = ET.Element("testsuites")
    by_suite: dict[str, list[Result]] = {}
    for result in results:
        by_suite.setdefault(result.suite, []).append(result)
    for suite_name, suite_results in sorted(by_suite.items()):
        suite = ET.SubElement(root, "testsuite", {
            "name": suite_name,
            "tests": str(len(suite_results)),
            "failures": str(sum(r.status == "FAIL" for r in suite_results)),
            "time": f"{sum(r.duration_s for r in suite_results):.6f}",
        })
        for result in suite_results:
            case = ET.SubElement(suite, "testcase", {
                "classname": result.suite,
                "name": result.name,
                "time": f"{result.duration_s:.6f}",
            })
            props = ET.SubElement(case, "properties")
            ET.SubElement(props, "property", {"name": "severity", "value": result.severity})
            ET.SubElement(props, "property", {"name": "kind", "value": result.kind})
            if result.status == "FAIL":
                failure = ET.SubElement(case, "failure", {
                    "message": result.detail or f"{result.name} failed",
                    "type": "TestFailure",
                })
                failure.text = f"return_code={result.return_code}; log={result.log}"
            elif result.status == "SKIP":
                ET.SubElement(case, "skipped", {"message": result.detail or "not selected"})
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(run_dir / "junit.xml", encoding="utf-8", xml_declaration=True)


class Runner:
    def __init__(self, profile: str, run_dir: Path, test_filter: str | None = None):
        self.profile = profile
        self.run_dir = run_dir
        self.test_filter = test_filter
        self.results: list[Result] = []
        self.env = os.environ.copy()
        self.env.update({
            "TMPDIR": str(run_dir),
            "TMP": str(run_dir),
            "TEMP": str(run_dir),
            "MPE_GAMEPAD_DEVICE": "disabled",
        })
        self.env.pop("MAKEFLAGS", None)

    @staticmethod
    def sanitizer_env() -> dict[str, str]:
        return {
            "ASAN_OPTIONS": "detect_leaks=1:halt_on_error=1",
            "UBSAN_OPTIONS": "halt_on_error=1:print_stacktrace=1",
        }

    @staticmethod
    def sanitizer_cflags() -> str:
        return "$(BASE_CFLAGS) -O1 -fno-omit-frame-pointer -fsanitize=address,undefined"

    def add(self, name: str, suite: str, status: str, *, kind: str = "test", severity: str = "gate",
            duration: float = 0.0, return_code: int | None = 0, log: str = "", detail: str = "") -> Result:
        row = Result(name, suite, kind, severity, status, duration, return_code, log, detail)
        self.results.append(row)
        badge = "INFO" if severity == "info" and status == "PASS" else status
        print(f"  [{badge}] {name} ({duration:.2f}s)")
        if status == "FAIL" and detail:
            print(f"    {detail}")
        return row

    def command(self, name: str, suite: str, command: Sequence[str], *, cwd: Path = PROJECT_ROOT,
                timeout: int = 600, extra_env: dict[str, str] | None = None,
                kind: str = "phase") -> tuple[Result, str]:
        safe_name = re.sub(r"[^A-Za-z0-9_.-]", "_", name)
        log_path = self.run_dir / f"{safe_name}.log"
        env = self.env.copy()
        if extra_env:
            env.update(extra_env)
        started = time.monotonic()
        try:
            proc = subprocess.Popen(
                list(command), cwd=cwd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as error:
            duration = time.monotonic() - started
            output = f"failed to start command: {error}\n"
            log_path.write_text(output, encoding="utf-8")
            row = self.add(name, suite, "FAIL", kind=kind, duration=duration, return_code=None,
                           log=str(log_path.relative_to(PROJECT_ROOT)), detail=output.strip())
            return row, output
        timed_out = False
        try:
            raw, _ = proc.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                raw, _ = proc.communicate(timeout=3)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                raw, _ = proc.communicate()
        duration = time.monotonic() - started
        output = (raw or b"").decode("utf-8", "replace")
        log_path.write_text(output, encoding="utf-8")
        status = "FAIL" if timed_out or proc.returncode != 0 else "PASS"
        detail = f"timed out after {timeout}s" if timed_out else (f"exit code {proc.returncode}" if proc.returncode else "")
        row = self.add(name, suite, status, kind=kind, duration=duration, return_code=proc.returncode,
                       log=str(log_path.relative_to(PROJECT_ROOT)), detail=detail)
        return row, output

    def python_contracts(self) -> bool:
        print("\n--- Test runner contract tests ---")
        row, _ = self.command(
            "runner-contracts", "harness", [sys.executable, "-m", "unittest", "discover", "-s", "tools/tests", "-p", "test_*.py", "-v"],
            timeout=120,
        )
        return row.status == "PASS"

    def source_registries(self) -> tuple[list[tuple[str, bool]], list[str], list[str]]:
        suite_entries = discover_suite_entries(SUITE_SOURCE.read_text(encoding="utf-8"))
        legacy, paranoia = discover_make_targets(MAKEFILE.read_text(encoding="utf-8"))
        return suite_entries, legacy, paranoia

    def core_suite(self, entries: Sequence[tuple[str, bool]], *, sanitizer: bool = False) -> bool:
        name = "suite-v2-asan-ubsan" if sanitizer else "suite-v2-build"
        print(f"\n--- {'ASan + UBSan ' if sanitizer else ''}canonical C suite ---")
        if sanitizer:
            build, _ = self.command(
                name, "sanitizers", ["make", "build_suite", f"CFLAGS={self.sanitizer_cflags()}"], cwd=SRC_DIR,
                timeout=900, kind="phase",
            )
            binary = SRC_DIR / "test_mpe_suite"
        else:
            build, _ = self.command(name, "suite-v2", ["make", "build_suite"], cwd=SRC_DIR, timeout=900)
            binary = SRC_DIR / "test_mpe_suite"
        if build.status != "PASS":
            return False

        list_result, listing = self.command(
            "suite-v2-list" if not sanitizer else "suite-v2-sanitizer-list", "suite-v2", [str(binary), "--list"],
            cwd=SRC_DIR, timeout=30,
        )
        if list_result.status != "PASS":
            return False
        listed: list[tuple[str, bool]] = []
        for line in listing.splitlines():
            m = re.fullmatch(r"\s+([a-z0-9_]+)(?:\s+\(diag\))?", line)
            if m:
                listed.append((m.group(1), "(diag)" in line))
        if listed != list(entries):
            self.add("suite-v2-registry", "suite-v2", "FAIL", kind="contract", return_code=None,
                     detail=f"binary registry differs from source registry: binary={listed!r}, source={list(entries)!r}")
            return False
        if self.test_filter:
            selected = [row for row in entries if row[0] == self.test_filter]
            if selected:
                cmd = [str(binary), self.test_filter]
                expected = selected
            else:
                return False
        else:
            cmd = [str(binary), "--all"]
            expected = entries
        run_name = "suite-v2-run" if not sanitizer else "suite-v2-sanitizer-run"
        run, output = self.command(
            run_name, "suite-v2" if not sanitizer else "sanitizers", cmd, cwd=SRC_DIR,
            timeout=900, extra_env=self.sanitizer_env() if sanitizer else None,
        )
        try:
            rows = parse_suite_output(output, expected)
        except ValueError as error:
            self.add("suite-v2-output-contract", "suite-v2", "FAIL", kind="contract", return_code=None,
                     detail=str(error), log=run.log)
            return False
        diag = dict(expected)
        for status, case, failures in rows:
            ok = status == "PASS" and failures == 0
            self.add(case if not sanitizer else f"asan-ubsan/{case}", "suite-v2" if not sanitizer else "sanitizers",
                     "PASS" if ok else "FAIL", severity="info" if diag[case] else "gate",
                     duration=0.0, return_code=0 if ok else 1, log=run.log,
                     detail="C suite reported a failed assertion" if not ok else "")
        summary_match = re.search(r"Total:\s+(\d+)\s+\|\s+Blocking failures:\s+(\d+)", output)
        if not self.test_filter and (not summary_match or int(summary_match.group(1)) != len(entries) or int(summary_match.group(2)) != 0):
            self.add("suite-v2-summary-contract", "suite-v2", "FAIL", kind="contract", return_code=None,
                     detail="suite summary missing, count mismatched, or has blocking failures", log=run.log)
            return False
        return run.status == "PASS" and all(status == "PASS" and failures == 0 for status, _case, failures in rows)

    def build_and_run_targets(self, names: Sequence[str], suite: str, *, sanitizer: bool = False) -> bool:
        if self.test_filter:
            names = [self.test_filter] if self.test_filter in names else []
        if not names:
            return False
        targets = [f"build_{name}" for name in names]
        report_suite = "sanitizers" if sanitizer else suite
        prefix = "asan-ubsan/" if sanitizer else ""
        print(f"\n--- Build {len(targets)} {'ASan + UBSan ' if sanitizer else ''}{suite} targets ---")
        command = ["make", "-j4", *targets]
        if sanitizer:
            command.append(f"CFLAGS={self.sanitizer_cflags()}")
        build, _ = self.command(f"{prefix}{suite}-build", report_suite, command, cwd=SRC_DIR,
                                timeout=max(900, len(targets) * 45))
        okay = build.status == "PASS"
        if not okay:
            return False
        print(f"\n--- Run {len(names)} {'ASan + UBSan ' if sanitizer else ''}{suite} cases in isolated processes ---")
        for name in names:
            binary = SRC_DIR / f"test_{name}"
            row, output = self.command(f"{prefix}{suite}-{name}", report_suite, [str(binary)], cwd=SRC_DIR,
                                       timeout=120, extra_env=self.sanitizer_env() if sanitizer else None)
            if sanitizer and ("ERROR: AddressSanitizer" in output or "runtime error:" in output):
                row.status = "FAIL"
                row.detail = "sanitizer reported an error"
            okay = okay and row.status == "PASS"
        return okay

    def mfs(self, *, sanitizer: bool = False) -> bool:
        print(f"\n--- {'ASan + UBSan ' if sanitizer else ''}MFS robotics tests ---")
        out = self.run_dir / ("mfs-sanitizers" if sanitizer else "mfs")
        out.mkdir(parents=True, exist_ok=True)
        result, output = self.command(
            "asan-ubsan-mfs-suite" if sanitizer else "mfs-suite", "sanitizers" if sanitizer else "mfs",
            ["./build_tests.sh"], cwd=SRC_DIR / "ecosystem" / "mfs", timeout=1500,
            extra_env={"OUTDIR": str(out), **({
                "MFS_TEST_CFLAGS": "-O1 -fno-omit-frame-pointer -fsanitize=address,undefined",
                "MFS_ASAN_HOTLOAD_ODR_SUPPRESS": "1",
                **self.sanitizer_env(),
            } if sanitizer else {})},
        )
        try:
            gated_pass, gated_fail, info = parse_mfs_summary(output)
            if gated_pass + gated_fail == 0:
                raise ValueError("MFS suite reported no gated checks")
            if gated_fail:
                raise ValueError(f"MFS suite reports {gated_fail} gated failures")
        except ValueError as error:
            self.add("mfs-summary-contract", "mfs", "FAIL", kind="contract", return_code=None,
                     log=result.log, detail=str(error))
            return False
        rows = re.findall(r"^\[(PASS|BUILD-OK|INFO)\]\s+(.+?)(?:\s+\(diagnostic, ungated\))?$", output, re.M)
        sanitizer_error = False
        if sanitizer:
            for log_path in out.glob("*.run.log"):
                run_text = log_path.read_text(encoding="utf-8", errors="replace")
                if "ERROR: AddressSanitizer" in run_text or "runtime error:" in run_text:
                    sanitizer_error = True
                    self.add(f"asan-ubsan/{log_path.stem}", "sanitizers", "FAIL", kind="sanitizer",
                             return_code=1, log=str(log_path.relative_to(PROJECT_ROOT)),
                             detail="sanitizer reported an error")
        for status, case in rows:
            self.add(("asan-ubsan/" if sanitizer else "") + case,
                     "sanitizers" if sanitizer else "mfs", "PASS", severity="info" if status == "INFO" else "gate",
                     log=result.log, detail="informational diagnostic")
        if len([1 for status, _ in rows if status in ("PASS", "BUILD-OK")]) != gated_pass:
            self.add("mfs-count-contract", "mfs", "FAIL", kind="contract", return_code=None, log=result.log,
                     detail="per-case gated output does not match summary count")
            return False
        if len([1 for status, _ in rows if status == "INFO"]) != info:
            self.add("mfs-info-count-contract", "mfs", "FAIL", kind="contract", return_code=None, log=result.log,
                     detail="per-case informational output does not match summary count")
            return False
        return result.status == "PASS" and not sanitizer_error

    def tui(self, *, sanitizer: bool = False) -> bool:
        print(f"\n--- {'ASan + UBSan ' if sanitizer else ''}TUI headless snapshot tests ---")
        smoke_dir = self.run_dir / ("tui-sanitizers" if sanitizer else "tui")
        smoke_dir.mkdir(parents=True, exist_ok=True)
        command = ["make", "tui-smoke", f"TUI_OUT={smoke_dir}"]
        if sanitizer:
            command.append(f"CFLAGS={self.sanitizer_cflags()}")
        result, _ = self.command("asan-ubsan-tui-smoke" if sanitizer else "tui-smoke-build-run",
                                 "sanitizers" if sanitizer else "tui", command, cwd=SRC_DIR, timeout=900,
                                 extra_env=self.sanitizer_env() if sanitizer else None)
        expected = [
            "demo_snapshot.log", "demo_stream.log", "tower.log", "pendulum.log", "springlab.log",
            "f10.log", "stress.log", "ccd.log", "demo_hash.log",
        ]
        okay = result.status == "PASS"
        for file_name in expected:
            valid, detail = validate_tui_file(smoke_dir / file_name)
            self.add(("asan-ubsan/" if sanitizer else "") + f"snapshot/{file_name}",
                     "sanitizers" if sanitizer else "tui", "PASS" if valid else "FAIL", kind="artifact-check",
                     return_code=0 if valid else 1, log=str((smoke_dir / file_name).relative_to(PROJECT_ROOT))
                     if (smoke_dir / file_name).exists() else "", detail=detail if not valid else detail)
            okay = okay and valid
        return okay

    def engine_build(self) -> bool:
        print("\n--- Full GTK engine build and deterministic flags ---")
        build, _ = self.command("engine-clean-rebuild", "engine", ["make", "-B", "engine"], cwd=SRC_DIR,
                                timeout=1200)
        flags, _ = self.command("engine-check-flags", "engine", ["make", "check-flags"], cwd=SRC_DIR,
                                timeout=60)
        version_ok = False
        header = SRC_DIR / "mpe_engine.h"
        if header.is_file():
            version_ok = 'a3_version_string "v15S-dev"' in header.read_text(encoding="utf-8")
        self.add("engine-version-contract", "engine", "PASS" if version_ok else "FAIL", kind="contract",
                 return_code=0 if version_ok else 1, detail="expected v15S-dev version macro" if not version_ok else "")
        return build.status == "PASS" and flags.status == "PASS" and version_ok

    def run_profile(self) -> bool:
        try:
            entries, legacy, paranoia = self.source_registries()
        except (OSError, ValueError) as error:
            self.add("test-registry-discovery", "harness", "FAIL", kind="contract", return_code=None, detail=str(error))
            return False

        okay = self.python_contracts()
        if self.test_filter and self.test_filter not in {name for name, _ in entries}:
            if self.test_filter in paranoia:
                return self.build_and_run_targets(paranoia, "paranoia")
            if self.test_filter in legacy:
                return self.build_and_run_targets(legacy, "legacy")
            self.add("test-filter", "harness", "FAIL", kind="contract", return_code=None,
                     detail=f"unknown exact test name: {self.test_filter}")
            return False
        stage_ok = self.core_suite(entries)
        okay = stage_ok and okay
        if self.profile in {"physics", "full"} and not self.test_filter:
            stage_ok = self.build_and_run_targets(legacy, "legacy")
            okay = stage_ok and okay
            stage_ok = self.build_and_run_targets(paranoia, "paranoia")
            okay = stage_ok and okay
        if self.profile == "full" and not self.test_filter:
            stage_ok = self.core_suite(entries, sanitizer=True)
            okay = stage_ok and okay
            stage_ok = self.build_and_run_targets(legacy, "legacy", sanitizer=True)
            okay = stage_ok and okay
            stage_ok = self.build_and_run_targets(paranoia, "paranoia", sanitizer=True)
            okay = stage_ok and okay
            stage_ok = self.engine_build()
            okay = stage_ok and okay
            stage_ok = self.mfs()
            okay = stage_ok and okay
            stage_ok = self.mfs(sanitizer=True)
            okay = stage_ok and okay
            stage_ok = self.tui()
            okay = stage_ok and okay
            stage_ok = self.tui(sanitizer=True)
            okay = stage_ok and okay
        return okay


def create_run_dir() -> tuple[str, Path]:
    if PROJECT_TEMP.is_symlink():
        raise RuntimeError("project temp directory must not be a symlink")
    PROJECT_TEMP.mkdir(parents=True, exist_ok=True)
    base = PROJECT_TEMP.resolve()
    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"-{os.getpid()}"
    run_dir = base / "qa_runs" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    if not run_dir.resolve().is_relative_to(base):
        raise RuntimeError("refusing to write test artifacts outside the project temp directory")
    return run_id, run_dir


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", choices=("quick", "physics", "full"), default="full",
                        help="quick: canonical; physics: canonical + isolated legacy/paranoia; full: everything (default)")
    parser.add_argument("--test", help="run one exact canonical, legacy, or paranoia test")
    parser.add_argument("--list", action="store_true", help="list dynamically discovered tests and profiles")
    parser.add_argument("--suite", nargs="?", const="all", help="compatibility alias for the canonical C suite")
    parser.add_argument("--include-paranoia", action="store_true", help="compatibility alias for --profile physics")
    parser.add_argument("--all", action="store_true", help="compatibility option; canonical suite includes diagnostics")
    return parser


def list_tests() -> int:
    try:
        entries = discover_suite_entries(SUITE_SOURCE.read_text(encoding="utf-8"))
        legacy, paranoia = discover_make_targets(MAKEFILE.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        print(f"Cannot discover tests: {error}", file=sys.stderr)
        return 2
    print("Profiles: quick, physics, full (default)")
    print("\nCanonical suite v2:")
    for name, diag in entries:
        print(f"  {name}{' [informational]' if diag else ''}")
    print("\nIsolated legacy suite:")
    for name in legacy:
        print(f"  {name}")
    print("\nParanoia suite:")
    for name in paranoia:
        print(f"  {name}")
    print("\nAdditional full-profile checks: ASan/UBSan, MFS, TUI snapshots, GTK engine build")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.list:
        return list_tests()
    if args.include_paranoia:
        args.profile = "physics"
    if args.suite is not None:
        if args.profile != "full" and args.profile != "quick":
            print("--suite cannot be combined with --profile physics/full", file=sys.stderr)
            return 2
        if args.suite != "all":
            args.test = args.suite
        args.profile = "quick"
    if args.all:
        args.profile = "quick"
    try:
        run_id, run_dir = create_run_dir()
    except (OSError, RuntimeError) as error:
        print(f"Cannot create project-local test run directory: {error}", file=sys.stderr)
        return 2
    metadata = {
        "schema": "mpe-test-run/v1",
        "run_id": run_id,
        "profile": args.profile,
        "project_root": str(PROJECT_ROOT),
        "source_tree": "v15S/src",
        "started_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "python": sys.version.split()[0],
    }
    runner = Runner(args.profile, run_dir, args.test)
    print(f"MPE test runner | profile={args.profile} | run={run_id}")
    print(f"Artifacts: {run_dir}")
    okay = runner.run_profile()
    write_reports(run_dir, metadata, runner.results)
    counts = result_counts(runner.results)
    print("\n=== MPE verification summary ===")
    print(f"Checks: {counts['total']} | passed: {counts['passed']} | failed: {counts['failed']} | "
          f"informational: {counts['informational']} | blocking failures: {counts['blocking_failures']}")
    print(f"JSON: {run_dir / 'summary.json'}")
    print(f"JUnit: {run_dir / 'junit.xml'}")
    return 0 if okay and counts["blocking_failures"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
