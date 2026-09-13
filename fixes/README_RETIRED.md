# fixes/ — RETIRED (history only)

These `*.py` scripts were the per-phase fix runners used during the v15R2/v15R3
MFS and cylinder-physics work. They are **not part of the build or test flow**.

- Current flow is MPE-only: `cd v15R3/src && make && python3 ../../tools/test_runner.py`
  (see `run_all.sh`).
- Robotics/MFS code those scripts touched is parked in
  `v15R3/robotics_backup/` (see its `README_PARKED.md`).
- Kept for archaeology only. Do not run, do not add new scripts here.
