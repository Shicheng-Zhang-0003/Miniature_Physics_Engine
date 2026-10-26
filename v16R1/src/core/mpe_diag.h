/* Engine-wide self-reporting diagnostics.
 *
 * WHY THIS EXISTS: every failure path in the engine used to collapse into
 * one of a handful of bare integers (-1, -2, "busy") or a guessed sentence
 * at the UI layer. When `mod attach` returned -1 the terminal could only
 * print "table full / attach hook" — a GUESS, with the six real causes
 * (null world, ABI skew, table full, foreign attach hook refusal, wrong
 * world, ...) indistinguishable. The operator was told a guess about a
 * subsystem they could not see, which is how a live "attach failed" took
 * several rebuilds to localise.
 *
 * This module is the single place where an error is NAMED:
 *   1. the subsystem ("source") that raised it,
 *   2. a stable machine code,
 *   3. the exact call site (file:line) inside the engine,
 *   4. a human message with the runtime values that caused it.
 *
 * Every record is fanned out INSTANTLY to, in one locked pass:
 *   - stderr (the terminal log the operator is already watching), flushed
 *     so it appears before the prompt returns;
 *   - event_log (overlay + any consumer of the existing ring);
 *   - an optional sink registered by the UI, which is how the same record
 *     lands in the debug terminal without core/ ever linking ui_input/.
 *
 * core/ must never depend on ui_input/, hence the sink callback instead of
 * a direct term_err() call. Physics workers may emit from any thread, so
 * the ring and the per-source totals are mutex-guarded (same contract as
 * event_log: never hot-path, always serialised).
 */
#ifndef mpe_diag_h
#define mpe_diag_h
#include <stdbool.h>
#include <stddef.h>
#include <time.h>

typedef enum { mpe_diag_info, mpe_diag_warn, mpe_diag_error, mpe_diag_fatal } mpe_diag_level;

/* Ring capacity: deep enough to hold a whole failed session's complaints
 * (the operator is usually mid-task when they open the terminal), shallow
 * enough that rendering stays a memcpy. */
#define MPE_DIAG_RING_CAPACITY 128
#define MPE_DIAG_SOURCE_CAPACITY 48
#define MPE_DIAG_MESSAGE_LEN 256
#define MPE_DIAG_SOURCE_LEN 32
#define MPE_DIAG_CODE_LEN 24
#define MPE_DIAG_SITE_LEN 48

typedef struct {
    unsigned long seq;
    mpe_diag_level level;
    char source [MPE_DIAG_SOURCE_LEN];
    char code [MPE_DIAG_CODE_LEN];
    char site [MPE_DIAG_SITE_LEN];
    char message [MPE_DIAG_MESSAGE_LEN];
    time_t when;
    unsigned repeats; /* coalesced identical repeats of the newest record */
} mpe_diag_record;

typedef struct {
    char source [MPE_DIAG_SOURCE_LEN];
    unsigned fatal;
    unsigned error;
    unsigned warn;
    unsigned info;
    unsigned long first_seq;
    unsigned long last_seq;
} mpe_diag_source_total;

void mpe_diag_init (void);
void mpe_diag_emit (mpe_diag_level level, const char *source, const char *code, const char *file, int line,
                    const char *format, ...);

/* NOTE on spacing: macro DEFINITIONS take no space before '(' (a space
 * would make this object-like, which is exactly the trap in this codebase's
 * `name (args)` call style). Call sites still use the spaced house style. */
#define MPE_DIAG_AT(level, source, code, ...) mpe_diag_emit ((level), (source), (code), __FILE__, __LINE__, __VA_ARGS__)
#define MPE_DIAG_ERROR(source, code, ...) MPE_DIAG_AT (mpe_diag_error, (source), (code), __VA_ARGS__)
#define MPE_DIAG_WARN(source, code, ...) MPE_DIAG_AT (mpe_diag_warn, (source), (code), __VA_ARGS__)
#define MPE_DIAG_INFO(source, code, ...) MPE_DIAG_AT (mpe_diag_info, (source), (code), __VA_ARGS__)

const char *mpe_diag_level_name (mpe_diag_level level);

/* Live mirror into the debug terminal. The UI registers a sink; when
 * enabled every emit also arrives there, tagged, in emission order. */
typedef void (*mpe_diag_sink_fn) (const char *line, void *user_data);
void mpe_diag_set_sink (mpe_diag_sink_fn sink, void *user_data);
void mpe_diag_set_live_mirror (bool enabled);
bool mpe_diag_live_mirror (void);

int mpe_diag_ring_count (void);
const mpe_diag_record *mpe_diag_ring_at (int index); /* chronological, 0 = oldest kept */
int mpe_diag_total (void);
int mpe_diag_count_level (mpe_diag_level level);
int mpe_diag_source_total_count (void);
const mpe_diag_source_total *mpe_diag_source_total_at (int index); /* busiest first */
void mpe_diag_clear (void);

/* Render for the debug terminal: per-source totals ("all sources of all
 * errors") followed by the most recent records. Never fails; always
 * NUL-terminates when cap > 0. Returns bytes written (excluding NUL). */
int mpe_diag_render (char *out, size_t cap, int max_records);
#endif /* mpe_diag_h */