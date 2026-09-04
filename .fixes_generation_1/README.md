# Historical Fix Scripts

This directory contains historical bash/sed/awk fix scripts from the MPE v15R2 repair campaign.

These scripts are retained for project archaeology only.

Do not run these scripts against the active codebase.

Current development policy:

- use `tools/build_check.py` for builds
- use `tools/test_runner.py` for tests
- use `tools/refactor.py` for scripted source transformations
- prefer direct C edits for normal feature work

Reason:

The old mutation scripts caused structural C corruption when anchors or syntax context changed.
The project has now moved to Python-based tooling and direct source maintenance.
