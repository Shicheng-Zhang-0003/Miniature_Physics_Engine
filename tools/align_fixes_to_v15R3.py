#!/usr/bin/env python3
"""
MPE v15R3 Fix Script Alignment Tool
===================================

Purpose:
    Align the historical/active fix scripts from the v15R2 repair campaign
    so they target v15R3 after the source folder has been renamed:

        v15R2/  ->  v15R3/

    This script is specifically designed for the R2 -> R3 transition.

What it changes:
    - Script path references:
          v15R2/src/...              -> v15R3/src/...
          ROOT / "v15R2" / "src"     -> ROOT / "v15R3" / "src"
          TARGET="v15R2/..."         -> TARGET="v15R3/..."

    - Version-transition logic:
          v15R1 -> v15R2
      becomes:
          v15R2 -> v15R3

      Example:
          sed -i 's/v15R1/v15R2/g' "$TARGET"

      becomes:
          sed -i 's/v15R2/v15R3/g' "$TARGET"

    - Documentation strings inside fix scripts that describe the current
      version/head are shifted one release forward.

Safety:
    - Dry-run by default.
    - Use --apply to write changes.
    - Creates *.pre_r3_align backups before modifying files.
    - Skips files already containing v15R3 unless --force is supplied.
    - Only edits text scripts by default: *.sh and *.py under fixes/.
    - Optionally also updates root runner scripts like run_all.sh and verify.sh.

Usage:
    # Preview changes
    python3 tools/align_fixes_to_v15R3.py

    # Apply changes
    python3 tools/align_fixes_to_v15R3.py --apply

    # Also update root run_all.sh / verify.sh if present
    python3 tools/align_fixes_to_v15R3.py --apply --include-root-runners

    # Include markdown/text files under fixes/
    python3 tools/align_fixes_to_v15R3.py --apply --include-docs

    # Re-run even if a file already mentions v15R3
    python3 tools/align_fixes_to_v15R3.py --apply --force
"""

from __future__ import annotations

import argparse
import difflib
import shutil
import sys
from pathlib import Path


OLD_PREVIOUS = "v15R1"
OLD_HEAD = "v15R2"
NEW_PREVIOUS = "v15R2"
NEW_HEAD = "v15R3"

BACKUP_SUFFIX = ".pre_r3_align"

SCRIPT_EXTENSIONS = {".sh", ".py"}
DOC_EXTENSIONS = {".md", ".txt"}

PLACEHOLDER_PREVIOUS = "__MPE_R3_ALIGN_PREVIOUS__"
PLACEHOLDER_HEAD = "__MPE_R3_ALIGN_HEAD__"


def is_probably_text(path: Path) -> bool:
    try:
        data = path.read_bytes()
    except OSError:
        return False

    if b"\x00" in data:
        return False

    try:
        data.decode("utf-8")
        return True
    except UnicodeDecodeError:
        return False


def should_skip_path(path: Path) -> bool:
    name = path.name

    if BACKUP_SUFFIX in name:
        return True

    if ".pre_" in name:
        return True

    if name.endswith(".bak") or name.endswith(".backup") or name.endswith(".orig"):
        return True

    if "__pycache__" in path.parts:
        return True

    return False


def collect_targets(
    root: Path,
    include_docs: bool,
    include_root_runners: bool,
) -> list[Path]:
    targets: list[Path] = []

    fixes = root / "fixes"
    if fixes.exists():
        allowed = set(SCRIPT_EXTENSIONS)
        if include_docs:
            allowed |= DOC_EXTENSIONS

        for path in sorted(fixes.rglob("*")):
            if not path.is_file():
                continue
            if should_skip_path(path):
                continue
            if path.suffix not in allowed:
                continue
            if not is_probably_text(path):
                continue
            targets.append(path)

    if include_root_runners:
        for name in ("run_all.sh", "verify.sh"):
            path = root / name
            if path.exists() and path.is_file() and is_probably_text(path):
                targets.append(path)

    return targets


def chain_shift_versions(text: str) -> str:
    """
    Shift version references one release forward without cascading.

    We want:

        v15R1 -> v15R2
        v15R2 -> v15R3

    But a naive replacement would cascade v15R1 all the way to v15R3.
    So we use placeholders.
    """

    text = text.replace(OLD_PREVIOUS, PLACEHOLDER_PREVIOUS)
    text = text.replace(OLD_HEAD, PLACEHOLDER_HEAD)

    text = text.replace(PLACEHOLDER_PREVIOUS, NEW_PREVIOUS)
    text = text.replace(PLACEHOLDER_HEAD, NEW_HEAD)

    return text


def targeted_repairs(text: str) -> str:
    """
    Repair specific scripts whose version bump logic cannot be correctly
    handled by raw version shifting alone.

    Main known case:
        016_fix_evolution.sh

    Original v15R2 campaign:
        add v15R1 after v14A3

    R3 campaign should:
        add v15R2 after v15R1, fallback to v14A3 if v15R1 is missing.
    """

    # After chain-shifting, the old line:
    #
    #   sed -i '/^v14A3$/a v15R1' "$TARGET"
    #
    # becomes:
    #
    #   sed -i '/^v14A3$/a v15R2' "$TARGET"
    #
    # That is not ideal for R3. We want v15R2 placed after v15R1.
    old_line = "sed -i '/^v14A3$/a v15R2' \"$TARGET\""

    new_block = """if grep -q '^v15R1$' "$TARGET"; then
    sed -i '/^v15R1$/a v15R2' "$TARGET"
else
    sed -i '/^v14A3$/a v15R2' "$TARGET"
fi"""

    if old_line in text:
        text = text.replace(old_line, new_block)

    # Repair the nearby comment if present.
    text = text.replace(
        "# Add v15R2 to the Prev Versions list (after v14A3 line)",
        "# Add v15R2 to the Prev Versions list (after v15R1 line; fallback to v14A3)",
    )

    text = text.replace(
        "# Add v15R2 to the Prev Versions list (after v14A3)",
        "# Add v15R2 to the Prev Versions list (after v15R1; fallback to v14A3)",
    )

    return text


def transform_text(text: str) -> str:
    text = chain_shift_versions(text)
    text = targeted_repairs(text)
    return text


def make_backup(path: Path) -> Path:
    backup = path.with_name(path.name + BACKUP_SUFFIX)

    if backup.exists():
        return backup

    shutil.copy2(path, backup)
    return backup


def short_diff(path: Path, old: str, new: str, max_lines: int = 160) -> str:
    diff = list(
        difflib.unified_diff(
            old.splitlines(),
            new.splitlines(),
            fromfile=str(path),
            tofile=str(path) + " [aligned]",
            lineterm="",
        )
    )

    if len(diff) > max_lines:
        shown = diff[:max_lines]
        shown.append(f"... diff truncated; {len(diff) - max_lines} more lines ...")
        return "\n".join(shown)

    return "\n".join(diff)


def contains_old_path_reference(text: str) -> bool:
    """
    Remaining v15R2 mentions are not automatically wrong after this migration,
    because v15R2 is now the previous release.

    But path-like v15R2 references are wrong if the active source folder is v15R3.
    """

    bad_patterns = [
        "v15R2/",
        "'v15R2/",
        '"v15R2/',
        'ROOT / "v15R2"',
        "ROOT / 'v15R2'",
        'Path("v15R2")',
        "Path('v15R2')",
    ]

    return any(pattern in text for pattern in bad_patterns)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Align fix scripts from v15R2 campaign to v15R3."
    )

    parser.add_argument(
        "--apply",
        action="store_true",
        help="Write changes. Without this, the script only previews changes.",
    )

    parser.add_argument(
        "--force",
        action="store_true",
        help="Transform files even if they already mention v15R3.",
    )

    parser.add_argument(
        "--include-docs",
        action="store_true",
        help="Also update *.md and *.txt files under fixes/.",
    )

    parser.add_argument(
        "--include-root-runners",
        action="store_true",
        help="Also update root run_all.sh and verify.sh if present.",
    )

    parser.add_argument(
        "--root",
        default=".",
        help="Project root. Default: current directory.",
    )

    args = parser.parse_args()

    root = Path(args.root).resolve()

    if not (root / "fixes").exists():
        print(f"[FATAL] No fixes/ directory found under {root}")
        return 2

    if not (root / "v15R3").exists():
        print(f"[WARN] {root / 'v15R3'} does not exist.")
        print("       If you have not renamed v15R2 -> v15R3 yet, do that first.")
        print()

    if (root / "v15R2").exists():
        print(f"[WARN] {root / 'v15R2'} still exists.")
        print("       This migration assumes the active tree is now v15R3/.")
        print()

    targets = collect_targets(
        root=root,
        include_docs=args.include_docs,
        include_root_runners=args.include_root_runners,
    )

    changed: list[Path] = []
    skipped_already_aligned: list[Path] = []
    unchanged: list[Path] = []
    warnings: list[str] = []

    print("=" * 72)
    print("MPE v15R3 Fix Script Alignment")
    print("=" * 72)
    print(f"Root:                  {root}")
    print(f"Mode:                  {'APPLY' if args.apply else 'DRY-RUN'}")
    print(f"Targets found:         {len(targets)}")
    print(f"Include docs:          {args.include_docs}")
    print(f"Include root runners:  {args.include_root_runners}")
    print(f"Force:                 {args.force}")
    print()

    for path in targets:
        old = path.read_text(encoding="utf-8")

        if NEW_HEAD in old and not args.force:
            skipped_already_aligned.append(path)
            continue

        new = transform_text(old)

        if new == old:
            unchanged.append(path)
            continue

        changed.append(path)

        print("-" * 72)
        print(f"[CHANGE] {path.relative_to(root)}")
        print(short_diff(path.relative_to(root), old, new))
        print()

        if contains_old_path_reference(new):
            warnings.append(
                f"{path.relative_to(root)} still appears to contain path-like v15R2 references."
            )

        if args.apply:
            backup = make_backup(path)
            path.write_text(new, encoding="utf-8")
            print(f"[WRITE]  {path.relative_to(root)}")
            print(f"[BACKUP] {backup.relative_to(root)}")
            print()

    print("=" * 72)
    print("Summary")
    print("=" * 72)
    print(f"Changed:               {len(changed)}")
    print(f"Unchanged:             {len(unchanged)}")
    print(f"Skipped already R3:    {len(skipped_already_aligned)}")
    print(f"Warnings:              {len(warnings)}")
    print()

    if skipped_already_aligned:
        print("Skipped as already aligned:")
        for path in skipped_already_aligned:
            print(f"  - {path.relative_to(root)}")
        print()

    if warnings:
        print("Warnings:")
        for warning in warnings:
            print(f"  - {warning}")
        print()
        print("Review these manually. Some remaining v15R2 references are expected,")
        print("because v15R2 is now the previous release. Path-like references are")
        print("the suspicious ones.")
        print()

    if not args.apply:
        print("[DRY-RUN] No files were modified.")
        print("Run with --apply to write changes.")
    else:
        print("[DONE] Fix scripts aligned for v15R3.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
