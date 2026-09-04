#ifndef mfs_long_run_validation_h
#define mfs_long_run_validation_h

/* MFS_PHASE_A: public interface for the long-run validation module. */

extern int long_run_validation_active;
extern int long_run_validation_ticks_remaining;
extern int long_run_validation_total_ticks;
extern int long_run_validation_restore_config; /* MPE_TASK_39_FIX */

void long_run_validation_start(int duration_ticks);
void long_run_validation_tick_update(void);

#endif
