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
#define MPE_MAX_BODIES            16384
#define MPE_MAX_JOINTS            1024
#define MPE_MAX_BROADPHASE_PAIRS  65536
#define A3_MAX_MANIFOLDS          8192

/* ------------------------------------------------------------------
 * BROADPHASE — spatial hash grid structure
 * ------------------------------------------------------------------ */
#define HASH_TABLE_SIZE           8192
#define MAX_OBJECTS               MPE_MAX_BODIES  /* alias for clarity in broadphase.c */
#define A3_PAIR_HASH_TABLE_SIZE   (1 << 18)
#define A3_PAIR_HASH_MASK         (A3_PAIR_HASH_TABLE_SIZE - 1)

/* ------------------------------------------------------------------
 * CONTACT CACHE — warm-starting impulse cache
 * ------------------------------------------------------------------ */
#define MAX_CACHED_CONTACTS       16384

/* ------------------------------------------------------------------
 * DEBUG TERMINAL — history buffer dimensions
 * ------------------------------------------------------------------ */
#define TERM_HISTORY_SIZE         64
#define TERM_HISTORY_LENGTH       511

/* ------------------------------------------------------------------
 * SCENE I/O — binary format identification
 * ------------------------------------------------------------------ */
#define MPE_MAGIC                 0x4D504533  /* "MPE3" */
#define MPE_VERSION               140

/* ------------------------------------------------------------------
 * VALIDATION — built-in test durations
 * ------------------------------------------------------------------ */
#define A3_LONG_RUN_VALIDATION_TICKS 3600  /* 60 seconds at 60 Hz */

#endif /* mpe_constants_h */
/* MPE_TASK_25_CONSTANTS_MANIFEST_END */
