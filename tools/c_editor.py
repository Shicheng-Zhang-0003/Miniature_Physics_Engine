#!/usr/bin/env python3
"""
c_editor.py - Bulletproof C code mutator.
Replaces sed/awk and fragile string.replace() with structural awareness.
"""
import re
import shutil
from pathlib import Path

class CEditorError(Exception):
    pass

class CEditor:
    def __init__(self, filepath, dry_run=False):
        self.path = Path(filepath)
        if not self.path.exists():
            raise CEditorError(f"File not found: {self.path}")
        self.text = self.path.read_text()
        self.original_text = self.text
        self.dry_run = dry_run
        self.changes_made = 0

    def has_marker(self, marker):
        """Check if a fix has already been applied."""
        return marker in self.text

    def replace_exact(self, old, new, marker):
        """Safe exact-string replacement with idempotency marker."""
        if self.has_marker(marker):
            return False
        if old not in self.text:
            raise CEditorError(f"Anchor missing for marker {marker}")

        # Inject the marker into the new string (preferably at the end or in a comment)
        if '/*' in new or '//' in new:
            new = new.replace('*/', f' {marker} */', 1) if '*/' in new else new + f' /* {marker} */'
        else:
            new = new + f' /* {marker} */'

        self.text = self.text.replace(old, new, 1)
        self.changes_made += 1
        return True

    def replace_function_body(self, func_signature_regex, new_body, marker):
        """
        Replaces an entire C function body by counting braces.
        func_signature_regex: e.g., r'void\s+rb_integrate_velocity\s*\('
        new_body: The new C code for the function (including the signature).
        """
        if self.has_marker(marker):
            return False

        # Find the start of the function
        match = re.search(func_signature_regex, self.text)
        if not match:
            raise CEditorError(f"Function signature not found: {func_signature_regex}")

        # Find the opening brace
        start_idx = self.text.find('{', match.start())
        if start_idx == -1:
            raise CEditorError(f"Opening brace not found for {func_signature_regex}")

        # Count braces to find the end of the function
        brace_count = 1
        i = start_idx + 1
        while i < len(self.text) and brace_count > 0:
            if self.text[i] == '{':
                brace_count += 1
            elif self.text[i] == '}':
                brace_count -= 1
            i += 1

        if brace_count != 0:
            raise CEditorError(f"Unbalanced braces in function {func_signature_regex}")

        # Replace the entire function
        old_func = self.text[match.start():i]
        new_func = new_body + f'\n/* {marker} */\n'

        self.text = self.text.replace(old_func, new_func, 1)
        self.changes_made += 1
        return True

    def save(self):
        if self.changes_made == 0:
            print(f"  [SKIP] {self.path.name}: No changes needed.")
            return

        if self.dry_run:
            print(f"  [DRY RUN] {self.path.name}: {self.changes_made} change(s) would be applied.")
            return

        # Create backup
        backup_path = self.path.with_suffix(self.path.suffix + ".bak")
        if not backup_path.exists():
            shutil.copy2(self.path, backup_path)

        self.path.write_text(self.text)
        print(f"  [OK] {self.path.name}: Applied {self.changes_made} change(s).")

