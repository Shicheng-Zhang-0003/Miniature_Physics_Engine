/* MPE_TASK_25_CONSTANTS_MANIFEST_BEGIN */
#ifndef mpe_constants_h
#define mpe_constants_h

/* ==================================================================
 * MPE Compile-Time Constants Manifest
 *
 * This header is the SINGLE SOURCE OF TRUTH for every compile-time
 * constant that defines memory layout, array sizes, and structural
 * limits. CAPACITY entries are CEILINGS: pools start at their
 * *_INITIAL size and double on demand (see physics_world growers),
 * so an empty world costs kilobytes while the layout guarantees hold.
 *
 * Behavioural/tunable constants (cell sizes, slops, thresholds)
 * remain in their domain files until v15R1 Task 29-33 migrates
 * them into the mpe_config_t runtime store.
 * ================================================================== */

/* ------------------------------------------------------------------
 * CAPACITY — object and joint pool limits (ceilings + initial sizes)
 * ------------------------------------------------------------------ */
#define mpe_max_bodies 16384
#define mpe_initial_bodies 512
#define mpe_max_joints 1024
#define mpe_max_broadphase_pairs 65536
#define a3_max_manifolds 8192

/* ------------------------------------------------------------------
 * BROADPHASE — spatial hash grid structure
 * ------------------------------------------------------------------ */
#define hash_table_size 8192
#define max_objects mpe_max_bodies /* alias for clarity in broadphase.c */
#define a3_pair_hash_table_size (1 << 18)
#define a3_pair_hash_mask (a3_pair_hash_table_size - 1)

/* ------------------------------------------------------------------
 * CONTACT CACHE — warm-starting impulse cache (ceiling + initial)
 * ------------------------------------------------------------------ */
#define max_cached_contacts 65536
#define mpe_initial_contacts 4096
#define mpe_id_cache_size 2048

/* ------------------------------------------------------------------
 * DEBUG TERMINAL — history buffer dimensions
 * ------------------------------------------------------------------ */
#define term_history_size 64
#define term_history_length 511

/* ------------------------------------------------------------------
 * SCENE I/O — binary format identification
 * ------------------------------------------------------------------ */
#define mpe_magic 0x4D504533 /* "MPE3" */
#define mpe_version 200 /* v200: LE fields, stable IDs, revolute section, CRC32 footer.
                         * v153 and older keep their native-order legacy reader. */

/* ------------------------------------------------------------------
 * VALIDATION — built-in test durations
 * ------------------------------------------------------------------ */
#define a3_long_run_validation_ticks 3600 /* 60 seconds at 60 Hz */

/* ------------------------------------------------------------------
 * PHYSICS NUMERICS — centralized epsilon / threshold constants.
 * Previously scattered as raw literals (0.98f, 1.01f, 0.0001f, ...).
 * ------------------------------------------------------------------ */
#define mpe_eps_contact 1e-4f
#define mpe_eps_normal 1e-6f
#define mpe_eps_parallel 1e-4f
#define mpe_eps_singular 1e-12f
/* mpe_face_hysteresis REMOVED (dead since face-hysteresis deletion; the
 * 15-axis minimum is reported raw). */
#define mpe_wireframe_scale 1.01f
#define mpe_sleep_wake_depth 0.02f
#define mpe_depen_early_out 5e-4f
#define mpe_wake_corr_thresh 0.01f
#define mpe_boundary_epsilon_sq 1e-6f
#define mpe_ray_tmin_neg -1e30f
#define mpe_ray_tmax_pos 1e30f
#define mpe_spawn_nudge 0.005f
#define mpe_spawn_max_move 1.0f

#endif /* mpe_constants_h */
/* MPE_TASK_25_CONSTANTS_MANIFEST_END */
