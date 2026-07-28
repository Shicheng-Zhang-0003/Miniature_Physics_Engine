#!/usr/bin/env python3
"""
Task 1: Enforce a strict RC freeze policy for v14A3 -> v14S.

Run this from the directory that contains src/.

This script:
  1. Creates RELEASE_POLICY.md.
  2. Inserts or updates a release-freeze notice in README.md.
  3. Adds a compile-time freeze marker to src/mpe_engine.h.

It does not change runtime engine behaviour.
"""

import re
import sys
from pathlib import Path


ROOT = Path.cwd()
SRC_DIR = ROOT / "src"

if not SRC_DIR.is_dir():
    print("ERROR: run this script from the folder that contains src/", file=sys.stderr)
    sys.exit(1)


README_BEGIN = "<!-- MPE_RELEASE_FREEZE_NOTICE_BEGIN -->"
README_END = "<!-- MPE_RELEASE_FREEZE_NOTICE_END -->"

HEADER_BEGIN = "/* MPE_RELEASE_FREEZE_BEGIN */"
HEADER_END = "/* MPE_RELEASE_FREEZE_END */"


POLICY_TEXT = """# MPE v14A3 Release Freeze Policy

This tree is in **v14A3 RC freeze**.

The purpose of this stage is to stabilise the engine for the upcoming `v14S`
stable release.

## Freeze Rule

Until `v14S` is tagged, the following rule applies:

> No new features are to be added to this branch.

Only the following change classes are accepted:

1. Correctness fixes.
2. Crash fixes.
3. Stability fixes.
4. Validation and testing improvements.
5. Documentation corrections.
6. Build and repository hygiene.
7. Small performance fixes only where they remove obvious waste or instability.

## Explicitly Deferred

The following are deferred until after `v14S`:

- New physics features.
- New rendering features.
- New editor systems.
- New constraint types.
- Large architectural refactors.
- Full global-state removal.
- Multithreading.
- Continuous collision detection.
- Generic constraint framework.
- Scene format version 2.

## Release Goal

The goal of `v14S` is not to make the engine perfect.

The goal is to make the current engine:

- build cleanly,
- run predictably,
- fail visibly,
- pass validation,
- and be release-worthy as the stable form of `v14A3`.
"""


README_NOTICE = f"""{README_BEGIN}
> **Release freeze notice:** This tree is in `v14A3` RC freeze. Until `v14S`, only correctness, stability, validation, documentation, and repository hygiene changes are accepted. No new features.
{README_END}"""


HEADER_BLOCK = f"""{HEADER_BEGIN}
#define A3_RELEASE_FREEZE 1
#define A3_RELEASE_FREEZE_NOTE "v14A3 RC freeze: fixes/validation/cleanup only until v14S"
{HEADER_END}"""


def upsert_block(text, begin, end, block):
    """
    Replace an existing marked block if present.
    Returns updated text, or None if no existing block was found.
    """
    pattern = re.compile(re.escape(begin) + r".*?" + re.escape(end), re.DOTALL)

    if pattern.search(text):
        return pattern.sub(lambda _: block, text)

    return None


def write_policy_file():
    policy_path = ROOT / "RELEASE_POLICY.md"
    policy_path.write_text(POLICY_TEXT.lstrip() + "\n", encoding="utf-8")
    print(f"written:  {policy_path}")


def update_readme():
    readme_path = ROOT / "README.md"

    if not readme_path.exists():
        print("skipped:  README.md not found")
        return

    text = readme_path.read_text(encoding="utf-8")

    updated = upsert_block(text, README_BEGIN, README_END, README_NOTICE)

    if updated is not None:
        readme_path.write_text(updated.rstrip() + "\n", encoding="utf-8")
        print(f"updated:  {readme_path}")
        return

    lines = text.splitlines()

    insert_index = 0

    for i, line in enumerate(lines):
        if line.startswith("# "):
            insert_index = i + 1
            break

    if insert_index < len(lines) and lines[insert_index].startswith("## "):
        insert_index += 1

    while insert_index < len(lines) and lines[insert_index].strip() == "":
        insert_index += 1

    lines.insert(insert_index, "")
    lines.insert(insert_index + 1, README_NOTICE)
    lines.insert(insert_index + 2, "")

    new_text = "\n".join(lines).rstrip() + "\n"
    readme_path.write_text(new_text, encoding="utf-8")

    print(f"modified: {readme_path}")


def update_engine_header():
    header_path = SRC_DIR / "mpe_engine.h"

    if not header_path.exists():
        print("skipped:  src/mpe_engine.h not found")
        return

    text = header_path.read_text(encoding="utf-8")

    updated = upsert_block(text, HEADER_BEGIN, HEADER_END, HEADER_BLOCK)

    if updated is not None:
        header_path.write_text(updated.rstrip() + "\n", encoding="utf-8")
        print(f"updated:  {header_path}")
        return

    endif_index = text.rfind("#endif")

    if endif_index != -1:
        new_text = (
            text[:endif_index].rstrip()
            + "\n\n"
            + HEADER_BLOCK
            + "\n\n"
            + text[endif_index:]
        )
    else:
        new_text = text.rstrip() + "\n\n" + HEADER_BLOCK + "\n"

    header_path.write_text(new_text.rstrip() + "\n", encoding="utf-8")
    print(f"modified: {header_path}")


def main():
    print("Task 1: release freeze policy")

    write_policy_file()
    update_readme()
    update_engine_header()

    print("done.")


if __name__ == "__main__":
    main()
