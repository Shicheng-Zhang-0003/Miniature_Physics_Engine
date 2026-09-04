#!/usr/bin/env python3
"""
MFS 141: Register rolling_resistance_coeff in the config schema (quick win)
===========================================================================
The param currently works only via a hardcoded fallback in drivetrain.c
(scripts 132/134 both failed the schema anchor). This makes it a real,
tunable physical parameter: visible in the config menu, saved/loaded in
engine.cfg, settable via terminal `export`.

Approach: brace-extract the existing floor_friction_k entry, clone it,
rename key/pointer, fix label/desc/default, insert after it. Robust to
whatever the real schema formatting is. If extraction is uncertain it
prints the format and aborts safely (build stays green).

Usage:
    cd <project_root>
    python3 fixes/phase_mfs/141_register_rolling_resistance.py [--dry-run]
"""
import re
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent.parent
SRC = ROOT / "v15R2" / "src"
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
DRY_RUN = "--dry-run" in sys.argv

def log(msg): print(f"  [141] {msg}")

SCHEMA = SRC / "config" / "mpe_config_schema.c"
CONFIG_H = SRC / "config" / "mpe_config.h"
DRIVETRAIN = SRC / "robotics" / "drivetrain.c"


def replace_nth_string(text, n, new_value):
    matches = list(re.finditer(r'"(?:[^"\\]|\\.)*"', text))
    if n >= len(matches):
        return text
    m = matches[n]
    return text[:m.start()] + '"' + new_value + '"' + text[m.end():]


def extract_entry(content, key_fragment):
    """Return (entry_text, insert_index_after_comma) or (None, None)."""
    idx = content.find(key_fragment)
    if idx < 0:
        return None, None
    # walk back to the '{' that opens this entry
    i = idx
    while i > 0 and content[i] != '{':
        i -= 1
    if content[i] != '{':
        return None, None
    start = i
    depth = 0
    j = start
    while j < len(content):
        if content[j] == '{':
            depth += 1
        elif content[j] == '}':
            depth -= 1
            if depth == 0:
                break
        j += 1
    if depth != 0:
        return None, None
    end = j
    # include trailing comma
    k = end + 1
    while k < len(content) and content[k] in ' \t\r\n':
        k += 1
    if k < len(content) and content[k] == ',':
        return content[start:end + 1], k + 1
    return content[start:end + 1], end + 1


def main():
    print("=" * 60)
    print("MFS 141: Register rolling_resistance_coeff")
    print("=" * 60)
    if DRY_RUN:
        print("  ** DRY RUN — no files modified **\n")

    content = SCHEMA.read_text()

    # Idempotency
    if "rolling_resistance_coeff" in content:
        log("[SKIP] already registered in schema")
        registered = True
    else:
        entry, insert_at = extract_entry(content, "floor_friction_k")
        if entry is None:
            # Print format around the key so we can adapt
            kidx = content.find("floor_friction_k")
            if kidx >= 0:
                log("[WARN] brace-extraction failed. Actual context:")
                print("  ---- schema context ----")
                print(content[max(0, kidx - 200):kidx + 400])
                print("  ------------------------")
            else:
                log("[WARN] 'floor_friction_k' not found in schema at all.")
                log("       Printing first 1200 chars of schema for inspection:")
                print(content[:1200])
            return 1

        log("Extracted floor_friction_k entry:")
        print("  " + entry.replace("\n", "\n  "))

        new_entry = entry.replace("floor_friction_k", "rolling_resistance_coeff")
        new_entry = replace_nth_string(new_entry, 0, "world.rolling_resistance_coeff")
        new_entry = replace_nth_string(new_entry, 1, "Rolling Resistance Coeff")
        new_entry = replace_nth_string(new_entry, 2, "Rolling resistance coefficient for wheels on floor (0 = free roll)")
        # Set default to 0.02 (first number after the pointer)
        new_entry = re.sub(
            r"(&g_cfg\.world\.rolling_resistance_coeff\s*,\s*)[-+0-9.eE]+",
            r"\g<1>0.02f",
            new_entry, count=1)

        log("New entry:")
        print("  " + new_entry.replace("\n", "\n  "))

        # preserve the original entry's leading indentation
        line_start = content.rfind("\n", 0, insert_at) + 1
        indent = ""
        for ch in content[line_start:insert_at]:
            if ch in " \t":
                indent += ch
            else:
                break
        content = content[:insert_at] + "\n" + indent + new_entry + "," + content[insert_at:]
        if not DRY_RUN:
            SCHEMA.write_text(content)
        log("[OK] rolling_resistance_coeff registered in schema")
        registered = True

    # Verify struct field exists in mpe_config.h
    ch = CONFIG_H.read_text()
    if "rolling_resistance_coeff" not in ch:
        log("[FAIL] mpe_config.h missing rolling_resistance_coeff field")
        return 1
    log("[OK] mpe_config.h has the struct field")

    # Remove the hardcoded fallback in drivetrain.c (only if registered)
    dt = DRIVETRAIN.read_text()
    if registered and "MFS_134_CONFIG_FALLBACK" in dt:
        old = ('float c_rr = g_cfg.world.rolling_resistance_coeff; /* MFS_134_CONFIG_FALLBACK */\n'
               'if (c_rr <= 0.0f) { c_rr = 0.02f; } /* fallback if config not initialized */')
        new = 'float c_rr = g_cfg.world.rolling_resistance_coeff; /* MFS_141: real config param, default 0.02 */'
        if old in dt:
            dt = dt.replace(old, new, 1)
            if not DRY_RUN:
                DRIVETRAIN.write_text(dt)
            log("[OK] hardcoded fallback removed from drivetrain.c")
        else:
            log("[WARN] fallback pattern not matched exactly — leaving as-is")
    elif "MFS_141" in dt:
        log("[SKIP] fallback already removed")

    if DRY_RUN:
        log("[DRY RUN] skipping build/test")
        return 0

    log("Build check...")
    r = subprocess.run([sys.executable, str(TOOLS / "build_check.py"), "--quick"],
                       cwd=ROOT, capture_output=True, text=True, timeout=180)
    if r.returncode != 0:
        print(r.stdout[-2500:]); print(r.stderr[-2500:])
        log("[FAIL] build failed")
        return 1
    log("[PASS] build clean")

    log("Running physics truth suite...")
    r = subprocess.run(["make", "-C", str(SRC), "test_physics_truth"],
                       capture_output=True, text=True, timeout=180)
    tail = r.stdout[-1200:]
    print(tail)
    if "Failed: 0" not in tail:
        log("[WARN] truth suite has failures — review")
        return 1
    log("[PASS] truth suite still 24/24")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
