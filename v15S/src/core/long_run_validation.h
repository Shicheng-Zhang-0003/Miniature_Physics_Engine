#ifndef mfs_long_run_validation_h
#define mfs_long_run_validation_h

/* MFS_PHASE_A: public interface for the long-run validation module. */

extern int long_run_validation_active;
extern int long_run_validation_ticks_remaining;
extern int long_run_validation_total_ticks;
extern int long_run_validation_restore_config; /* MPE_TASK_39_FIX */
extern int long_run_validation_is_torture; /* TRUTH: F11 mode, corruption gates only */

void long_run_validation_start(int duration_ticks);
void long_run_validation_tick_update(void);

/* DESPOT-2026-10-04 (torture-leak closure): the 2026-10-04 live incident
 * proved status/engine.cfg can hold full F11 torture values (gravity -17,
 * drag 0.63, sleep off, 9-ton spawner masses, safety net off) because the
 * restore only ran at normal run completion while every exit/save path
 * wrote g_cfg unconditionally. These three functions close it:
 * - torture_live: true while tortured values are live in g_cfg.
 * - snapshot_clean: copy the pre-torture g_cfg into memory (call BEFORE
 *   randomizing). File backup remains, but a missing/corrupt backup file
 *   can never again leave torture live.
 * - cancel_restore: restore clean state NOW (memory first, backup file
 *   second, compiled defaults last; torture never survives). Call from
 *   every exit path before any config save. */
int long_run_validation_torture_live(void);
void long_run_validation_snapshot_clean(void);
void long_run_validation_cancel_restore(void);

#endif
