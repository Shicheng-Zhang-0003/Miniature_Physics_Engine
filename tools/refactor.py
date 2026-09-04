#!/usr/bin/env python3
"""
MPE Safe Refactoring Tool
==========================
Replaces all bash/sed/awk fix scripts with a safe, context-aware,
dry-run-capable Python transformation engine.

Usage:
    from refactor import Refactor
    r = Refactor("v15R2/src/simulation.c")
    r.insert_after(
        anchor="static bool editor_dialog_active = false;",
        text="/* new code */\nint x = 0;",
        label="Add robot registry globals"
    )
    r.replace(
        old="old_function_call()",
        new="new_function_call()",
        label="Rename function call"
    )
    r.apply(dry_run=True)   # Preview changes
    r.apply(dry_run=False)  # Actually write
"""

import os
import re
import shutil
import difflib
from pathlib import Path
from typing import Optional


class RefactorError(Exception):
    """Raised when a refactoring operation cannot be safely completed."""
    pass


class Refactor:
    """Safe, atomic, context-aware C source file transformer."""

    def __init__(self, filepath: str):
        self.filepath = Path(filepath)
        if not self.filepath.exists():
            raise RefactorError(f"File not found: {self.filepath}")
        self._original_lines = self.filepath.read_text().splitlines(keepends=True)
        self._lines = list(self._original_lines)
        self._ops: list[dict] = []

    def insert_after(self, anchor: str, text: str, label: str = "",
                     occurrence: int = 1) -> "Refactor":
        """Insert text on the line immediately after the line matching anchor."""
        indices = self._find_anchor_indices(anchor)
        if len(indices) < occurrence:
            raise RefactorError(
                f"[{label}] Anchor found {len(indices)} time(s), "
                f"need occurrence #{occurrence}: {anchor!r}"
            )
        target_idx = indices[occurrence - 1]
        new_lines = [line + "\n" if not line.endswith("\n") else line
                     for line in text.split("\n")]
        self._ops.append({
            "type": "insert_after",
            "index": target_idx,
            "lines": new_lines,
            "label": label,
            "anchor": anchor,
        })
        return self

    def insert_before(self, anchor: str, text: str, label: str = "",
                      occurrence: int = 1) -> "Refactor":
        """Insert text on the line immediately before the line matching anchor."""
        indices = self._find_anchor_indices(anchor)
        if len(indices) < occurrence:
            raise RefactorError(
                f"[{label}] Anchor found {len(indices)} time(s), "
                f"need occurrence #{occurrence}: {anchor!r}"
            )
        target_idx = indices[occurrence - 1]
        new_lines = [line + "\n" if not line.endswith("\n") else line
                     for line in text.split("\n")]
        self._ops.append({
            "type": "insert_before",
            "index": target_idx,
            "lines": new_lines,
            "label": label,
            "anchor": anchor,
        })
        return self

    def replace(self, old: str, new: str, label: str = "",
                count: int = 0, regex: bool = False) -> "Refactor":
        """Replace occurrences of old with new."""
        self._ops.append({
            "type": "replace",
            "old": old,
            "new": new,
            "count": count,
            "regex": regex,
            "label": label,
        })
        return self

    def delete_range(self, start_anchor: str, end_anchor: str,
                     label: str = "", inclusive: bool = True) -> "Refactor":
        """Delete lines from start_anchor to end_anchor."""
        start_indices = self._find_anchor_indices(start_anchor)
        end_indices = self._find_anchor_indices(end_anchor)
        if not start_indices:
            raise RefactorError(f"[{label}] Start anchor not found: {start_anchor!r}")
        if not end_indices:
            raise RefactorError(f"[{label}] End anchor not found: {end_anchor!r}")
        start_idx = start_indices[0]
        end_idx = end_indices[0]
        if end_idx < start_idx:
            raise RefactorError(
                f"[{label}] End anchor (line {end_idx}) is before "
                f"start anchor (line {start_idx})"
            )
        self._ops.append({
            "type": "delete_range",
            "start": start_idx,
            "end": end_idx,
            "inclusive": inclusive,
            "label": label,
        })
        return self

    def add_include(self, include_line: str, label: str = "",
                    after_include: Optional[str] = None) -> "Refactor":
        """Add an #include line idempotently."""
        stripped = include_line.strip()
        for line in self._lines:
            if line.strip() == stripped:
                self._ops.append({
                    "type": "noop",
                    "label": f"{label} (already present)",
                })
                return self

        if after_include:
            indices = self._find_anchor_indices(after_include)
            if indices:
                target_idx = indices[-1]
            else:
                raise RefactorError(
                    f"[{label}] after_include anchor not found: {after_include!r}"
                )
        else:
            target_idx = -1
            for i, line in enumerate(self._lines):
                if line.strip().startswith("#include"):
                    target_idx = i
            if target_idx == -1:
                raise RefactorError(f"[{label}] No existing #include found")

        new_line = include_line if include_line.endswith("\n") else include_line + "\n"
        self._ops.append({
            "type": "insert_after",
            "index": target_idx,
            "lines": [new_line],
            "label": label,
            "anchor": after_include or "(last #include)",
        })
        return self

    def apply(self, dry_run: bool = False) -> str:
        """Apply all queued operations. Returns unified diff string."""
        working = list(self._lines)

        indexed_ops = []
        replace_ops = []
        for op in self._ops:
            if op["type"] == "replace":
                replace_ops.append(op)
            elif op["type"] == "noop":
                continue
            else:
                indexed_ops.append(op)

        for op in replace_ops:
            working = self._apply_replace(working, op)

        indexed_ops.sort(key=lambda o: o.get("index", 0), reverse=True)

        for op in indexed_ops:
            if op["type"] == "insert_after":
                idx = op["index"]
                for new_line in reversed(op["lines"]):
                    working.insert(idx + 1, new_line)
            elif op["type"] == "insert_before":
                idx = op["index"]
                for new_line in reversed(op["lines"]):
                    working.insert(idx, new_line)
            elif op["type"] == "delete_range":
                start = op["start"]
                end = op["end"]
                if op["inclusive"]:
                    del working[start:end + 1]
                else:
                    del working[start + 1:end]

        diff = difflib.unified_diff(
            self._original_lines, working,
            fromfile=str(self.filepath),
            tofile=str(self.filepath),
            lineterm=""
        )
        diff_str = "\n".join(diff)

        if not dry_run:
            backup_path = self.filepath.with_suffix(self.filepath.suffix + ".bak")
            tmp_path = self.filepath.with_suffix(self.filepath.suffix + ".tmp")
            tmp_path.write_text("".join(working))
            if not backup_path.exists():
                shutil.copy2(self.filepath, backup_path)
            tmp_path.rename(self.filepath)

        return diff_str

    def diff(self) -> str:
        """Preview changes without writing."""
        return self.apply(dry_run=True)

    def _find_anchor_indices(self, anchor: str) -> list[int]:
        return [i for i, line in enumerate(self._lines) if anchor in line]

    def _apply_replace(self, lines: list[str], op: dict) -> list[str]:
        result = []
        total_replacements = 0
        max_count = op["count"]
        for line in lines:
            if max_count > 0 and total_replacements >= max_count:
                result.append(line)
                continue
            if op["regex"]:
                new_line, n = re.subn(op["old"], op["new"], line,
                                       count=max_count - total_replacements if max_count > 0 else 0)
            else:
                if max_count > 0:
                    remaining = max_count - total_replacements
                    new_line = line.replace(op["old"], op["new"], remaining)
                    n = min(line.count(op["old"]), remaining)
                else:
                    n = line.count(op["old"])
                    new_line = line.replace(op["old"], op["new"])
            total_replacements += n
            result.append(new_line)
        return result


if __name__ == "__main__":
    print("MPE Refactor Tool")
    print("Import and use programmatically:")
    print("  from refactor import Refactor")
    print("  r = Refactor('path/to/file.c')")
    print("  r.insert_after(...).replace(...).apply(dry_run=True)")
