/* Engine-wide self-reporting diagnostics — implementation.
 * See mpe_diag.h for the contract. Everything here is low-frequency
 * (error paths only): locking and formatting costs never touch the
 * physics tick, so records can be as verbose as diagnosis needs. */
#include "mpe_diag.h"
#include "event_log.h"
#include "mpe_platform.h"
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static mpe_diag_record g_ring [MPE_DIAG_RING_CAPACITY];
static int g_head = 0;
static int g_count = 0;
static unsigned long g_seq = 0;
static mpe_diag_source_total g_sources [MPE_DIAG_SOURCE_CAPACITY];
static int g_source_count = 0;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static mpe_diag_sink_fn g_sink = NULL;
static void *g_sink_user = NULL;
static bool g_live_mirror = true;

const char *mpe_diag_level_name (mpe_diag_level level) {
    switch (level) {
        case mpe_diag_fatal: return ("FATAL");
        case mpe_diag_error: return ("ERROR");
        case mpe_diag_warn: return ("WARN");
        default: return ("INFO");
    }
}
static void site_from_file (const char *file, int line, char *out, size_t cap) {
    const char *base = file ? file : "?";
    const char *slash = strrchr (base, '/');
    if (slash && *(slash + 1)) { base = slash + 1; }
#ifdef MPE_OS_WINDOWS
    const char *back = strrchr (base, '\\');
    if (back && *(back + 1)) { base = back + 1; }
#endif
    if (line > 0)
        snprintf (out, cap, "%s:%d", base, line);
    else
        snprintf (out, cap, "%s", base);
}
static int source_index_locked (const char *source) {
    for (int i = 0; i < g_source_count; i++)
        if (strcmp (g_sources [i].source, source) == 0)
            return (i);
    if (g_source_count >= MPE_DIAG_SOURCE_CAPACITY) { return (-1); }
    memset (&g_sources [g_source_count], 0, sizeof (mpe_diag_source_total));
    snprintf (g_sources [g_source_count].source, MPE_DIAG_SOURCE_LEN, "%s", source);
    g_sources [g_source_count].first_seq = g_seq + 1;
    return (g_source_count++);
}
/* Coalescing: a fault that repeats every frame must not drown the log, but
 * it must never be silently swallowed either. Print the first occurrence,
 * then every power of two, then every 64th — so both "it happened twice"
 * and "it is still happening 4096 times later" are visible. */
static bool should_reprint (unsigned repeats) {
    if (repeats <= 1) { return (true); }
    if ((repeats & (repeats - 1)) == 0) { return (true); }
    return ((repeats % 64) == 0);
}
void mpe_diag_init (void) {
    pthread_mutex_lock (&g_lock);
    g_head = 0;
    g_count = 0;
    g_seq = 0;
    g_source_count = 0;
    memset (g_sources, 0, sizeof (g_sources));
    memset (g_ring, 0, sizeof (g_ring));
    g_live_mirror = true;
    pthread_mutex_unlock (&g_lock);
}
void mpe_diag_set_sink (mpe_diag_sink_fn sink, void *user_data) {
    pthread_mutex_lock (&g_lock);
    g_sink = sink;
    g_sink_user = user_data;
    pthread_mutex_unlock (&g_lock);
}
void mpe_diag_set_live_mirror (bool enabled) {
    pthread_mutex_lock (&g_lock);
    g_live_mirror = enabled;
    pthread_mutex_unlock (&g_lock);
}
bool mpe_diag_live_mirror (void) {
    pthread_mutex_lock (&g_lock);
    bool enabled = g_live_mirror;
    pthread_mutex_unlock (&g_lock);
    return (enabled);
}
void mpe_diag_emit (mpe_diag_level level, const char *source, const char *code, const char *file, int line,
                    const char *format, ...) {
    if (!format) { return; }
    char message [MPE_DIAG_MESSAGE_LEN];
    va_list args;
    va_start (args, format);
    vsnprintf (message, MPE_DIAG_MESSAGE_LEN, format, args);
    va_end (args);
    message [MPE_DIAG_MESSAGE_LEN - 1] = '\0';

    char line_buf [MPE_DIAG_MESSAGE_LEN + 256];
    mpe_diag_sink_fn sink = NULL;
    void *sink_user = NULL;
    bool print = false;

    pthread_mutex_lock (&g_lock);
    const char *src = (source && *source) ? source : "?";
    const char *cd = (code && *code) ? code : "?";
    /* Coalesce against the newest slot before allocating a new one. */
    int newest = (g_head - 1 + MPE_DIAG_RING_CAPACITY) % MPE_DIAG_RING_CAPACITY;
    if (g_count > 0 && g_ring [newest].level == level && strcmp (g_ring [newest].source, src) == 0 &&
        strcmp (g_ring [newest].code, cd) == 0 && strcmp (g_ring [newest].message, message) == 0) {
        g_ring [newest].repeats++;
        print = should_reprint (g_ring [newest].repeats);
        if (print)
            snprintf (line_buf, sizeof (line_buf), "[mpe][%s] %s/%s [%s] %s (repeat #%u)\n",
                      mpe_diag_level_name (level), src, cd, g_ring [newest].site, message, g_ring [newest].repeats);
    } else {
        mpe_diag_record *slot = &g_ring [g_head];
        memset (slot, 0, sizeof (*slot));
        slot->seq = ++g_seq;
        slot->level = level;
        slot->when = time (NULL);
        snprintf (slot->source, MPE_DIAG_SOURCE_LEN, "%s", src);
        snprintf (slot->code, MPE_DIAG_CODE_LEN, "%s", cd);
        site_from_file (file, line, slot->site, MPE_DIAG_SITE_LEN);
        snprintf (slot->message, MPE_DIAG_MESSAGE_LEN, "%s", message);
        slot->repeats = 1;
        g_head = (g_head + 1) % MPE_DIAG_RING_CAPACITY;
        if (g_count < MPE_DIAG_RING_CAPACITY) { g_count++; }
        int si = source_index_locked (src);
        if (si >= 0) {
            mpe_diag_source_total *st = &g_sources [si];
            st->last_seq = slot->seq;
            if (level == mpe_diag_fatal) st->fatal++;
            else if (level == mpe_diag_error) st->error++;
            else if (level == mpe_diag_warn) st->warn++;
            else st->info++;
        }
        print = true;
        snprintf (line_buf, sizeof (line_buf), "[mpe][%s] %s/%s [%s] %s\n", mpe_diag_level_name (level), src, cd,
                  slot->site, slot->message);
    }
    if (print && g_live_mirror) {
        sink = g_sink;
        sink_user = g_sink_user;
    }
    pthread_mutex_unlock (&g_lock);

    if (!print) { return; }
    /* Instant, unbuffered, on the stream the level belongs to. */
    FILE *out = (level >= mpe_diag_error) ? stderr : stdout;
    fputs (line_buf, out);
    fflush (out);
    /* event_log keeps the overlay and any existing ring consumer honest. */
    event_log_push (level == mpe_diag_fatal   ? log_error
                        : level == mpe_diag_error ? log_error
                                                   : level == mpe_diag_warn ? log_warn
                                                                           : log_info,
                    "%s %s", src, message);
    if (sink) { sink (line_buf, sink_user); }
}
int mpe_diag_ring_count (void) {
    pthread_mutex_lock (&g_lock);
    int n = g_count;
    pthread_mutex_unlock (&g_lock);
    return (n);
}
const mpe_diag_record *mpe_diag_ring_at (int index) {
    pthread_mutex_lock (&g_lock);
    const mpe_diag_record *rec = NULL;
    if (index >= 0 && index < g_count)
        rec = &g_ring [(g_head - g_count + index + MPE_DIAG_RING_CAPACITY * 2) % MPE_DIAG_RING_CAPACITY];
    pthread_mutex_unlock (&g_lock);
    return (rec);
}
int mpe_diag_total (void) {
    pthread_mutex_lock (&g_lock);
    int n = (int) g_seq;
    pthread_mutex_unlock (&g_lock);
    return (n);
}
int mpe_diag_count_level (mpe_diag_level level) {
    int n = 0;
    pthread_mutex_lock (&g_lock);
    for (int i = 0; i < g_count; i++) {
        if (g_ring [i].level == level || (level == mpe_diag_error && g_ring [i].level == mpe_diag_fatal)) { n++; }
    }
    pthread_mutex_unlock (&g_lock);
    return (n);
}
int mpe_diag_source_total_count (void) {
    pthread_mutex_lock (&g_lock);
    int n = g_source_count;
    pthread_mutex_unlock (&g_lock);
    return (n);
}
const mpe_diag_source_total *mpe_diag_source_total_at (int index) {
    pthread_mutex_lock (&g_lock);
    const mpe_diag_source_total *st = NULL;
    if (index >= 0 && index < g_source_count) { st = &g_sources [index]; }
    pthread_mutex_unlock (&g_lock);
    return (st);
}
void mpe_diag_clear (void) {
    pthread_mutex_lock (&g_lock);
    g_head = 0;
    g_count = 0;
    g_source_count = 0;
    memset (g_sources, 0, sizeof (g_sources));
    pthread_mutex_unlock (&g_lock);
}
/* Busiest source first: the operator asking "all sources of all errors"
 * wants the loudest offender at the top, not registration order. */
static int compare_totals (const void *lhs, const void *rhs) {
    const mpe_diag_source_total *a = (const mpe_diag_source_total *) lhs;
    const mpe_diag_source_total *b = (const mpe_diag_source_total *) rhs;
    unsigned as = a->fatal + a->error, bs = b->fatal + b->error;
    if (as != bs) { return (as < bs) ? 1 : -1; }
    unsigned aw = a->warn, bw = b->warn;
    if (aw != bw) { return (aw < bw) ? 1 : -1; }
    return (strcmp (a->source, b->source));
}
int mpe_diag_render (char *out, size_t cap, int max_records) {
    if (!out || cap == 0) { return (0); }
    out [0] = '\0';
    size_t used = 0;
#define DIAG_APPEND(...)                                                                                                                \
    do {                                                                                                                                 \
        int _w = snprintf (out + used, (cap > used) ? cap - used : 0, __VA_ARGS__);                                                      \
        if (_w > 0) used += (size_t) _w;                                                                                                 \
        if (used >= cap) { used = cap - 1; goto done; }                                                                                 \
    } while (0)

    mpe_diag_source_total totals [MPE_DIAG_SOURCE_CAPACITY];
    mpe_diag_record records [MPE_DIAG_RING_CAPACITY];
    int source_count = 0, record_count = 0, total_seq = 0, fatals = 0, errors = 0, warns = 0;

    pthread_mutex_lock (&g_lock);
    source_count = g_source_count;
    memcpy (totals, g_sources, sizeof (mpe_diag_source_total) * (size_t) source_count);
    record_count = g_count;
    total_seq = (int) g_seq;
    for (int i = 0; i < record_count; i++) {
        records [i] = g_ring [(g_head - g_count + i + MPE_DIAG_RING_CAPACITY * 2) % MPE_DIAG_RING_CAPACITY];
        if (records [i].level == mpe_diag_fatal) fatals++;
        else if (records [i].level == mpe_diag_error) errors++;
        else if (records [i].level == mpe_diag_warn) warns++;
    }
    pthread_mutex_unlock (&g_lock);

    if (source_count > 1) { qsort (totals, (size_t) source_count, sizeof (mpe_diag_source_total), compare_totals); }

    DIAG_APPEND ("mpe diagnostics: %d record(s) total — %d fatal, %d error, %d warn (ring keeps %d/%d)\n", total_seq,
                 fatals, errors, warns, record_count, MPE_DIAG_RING_CAPACITY);
    if (source_count == 0) { DIAG_APPEND ("  no errors or warnings recorded\n"); }
    for (int i = 0; i < source_count; i++) {
        if (totals [i].fatal == 0 && totals [i].error == 0 && totals [i].warn == 0) { continue; }
        DIAG_APPEND ("  %-14s fatal=%u error=%u warn=%u info=%u  last=#%lu\n", totals [i].source, totals [i].fatal,
                     totals [i].error, totals [i].warn, totals [i].info, totals [i].last_seq);
    }
    if (max_records > 0 && record_count > 0) {
        int first = (max_records < record_count) ? record_count - max_records : 0;
        if (first > 0) { DIAG_APPEND ("  ... %d older record(s) elided\n", first); }
        for (int i = first; i < record_count; i++) {
            const mpe_diag_record *r = &records [i];
            char body [MPE_DIAG_MESSAGE_LEN + 160];
            char suffix [24];
            if (r->repeats > 1)
                snprintf (suffix, sizeof (suffix), "  (x%u)", r->repeats);
            else
                snprintf (suffix, sizeof (suffix), "%s", "");
            snprintf (body, sizeof (body), "  #%-5lu %-5s %s/%s [%s] %s%s\n", r->seq, mpe_diag_level_name (r->level),
                      r->source, r->code, r->site, r->message, suffix);
            DIAG_APPEND ("%s", body);
        }
    }
done:
#undef DIAG_APPEND
    out [cap - 1] = '\0';
    return ((int) used);
}