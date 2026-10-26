#ifndef debug_counters_h
#define debug_counters_h
/* GTK4-PREP: debug/validation counters were declared in mpe_engine.h
 * (gtk-dependent). Core validation/reporting needs them without GUI. */
extern int debug_last_object_count;
extern int debug_last_broadphase_pair_count;
extern int debug_last_manifold_count;
extern float debug_last_frame_time;
extern int debug_last_sleeping_object_count;
extern int debug_last_manifold_overflow_count;
extern int long_run_validation_active;
extern int long_run_validation_ticks_remaining;
extern int long_run_validation_total_ticks;
#endif /* debug_counters_h */
