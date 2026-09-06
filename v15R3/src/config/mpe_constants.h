/* MPE_TASK_25_CONSTANTS_MANIFEST_BEGIN */
#ifndef mpe_constants_h
#define mpe_constants_h

/* ==================================================================
 * MPE Compile-Time Constants Manifest
 *
 * This header is the SINGLE SOURCE OF TRUTH for every compile-time
 * constant that defines memory layout, array sizes, and structural
 * limits. These are LOCKED: they cannot change at runtime.
 *
 * Behavioural/tunable constants (cell sizes, slops, thresholds)
 * remain in their domain files until v15R1 Task 29-33 migrates
 * them into the mpe_config_t runtime store.
 * ================================================================== */

/* ------------------------------------------------------------------
 * CAPACITY — object and joint pool limits
 * ------------------------------------------------------------------ */
#define mpe_max_bodies 16384
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
 * CONTACT CACHE — warm-starting impulse cache
 * ------------------------------------------------------------------ */
#define max_cached_contacts 65536

/* ------------------------------------------------------------------
 * DEBUG TERMINAL — history buffer dimensions
 * ------------------------------------------------------------------ */
#define term_history_size 64
#define term_history_length 511

/* ------------------------------------------------------------------
 * SCENE I/O — binary format identification
 * ------------------------------------------------------------------ */
#define mpe_magic 0x4D504533 /* "MPE3" */
#define mpe_version 151 /* R3-04: bumped to add cylinder_half_length to body format */

/* ------------------------------------------------------------------
 * VALIDATION — built-in test durations
 * ------------------------------------------------------------------ */
#define a3_long_run_validation_ticks 3600 /* 60 seconds at 60 Hz */

#endif /* mpe_constants_h */
/* MPE_TASK_25_CONSTANTS_MANIFEST_END */
