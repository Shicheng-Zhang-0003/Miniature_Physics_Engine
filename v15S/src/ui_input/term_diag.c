/* `diag` — the engine's self-report, on demand.
 *
 * Usage:
 *   diag              per-source totals ("all sources of all errors") + last 25 records
 *   diag tail [n]     last n records only
 *   diag sources      per-source totals only
 *   diag on|off       live mirror: every new record is echoed here as it happens
 *   diag clear        forget every record (totals included)
 *
 * Everything here reads the ring owned by core/mpe_diag.c. The terminal is
 * a VIEW: it never becomes the place an error is first noticed, because the
 * same record is already on stderr and in the event log by the time the
 * operator can type anything.
 */
#include "../core/mpe_platform.h"
#include "term_priv.h"
#include "../core/mpe_diag.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void diag_render_into_term (int max_records) {
    /* Sized for the whole ring at its widest: totals + every record. */
    static char out [16384];
    int n = mpe_diag_render (out, sizeof (out), max_records);
    if (n <= 0) {
        term_err ("mpe: diag: nothing recorded yet\n");
        return;
    }
    term_out (out);
}
void cmd_diag (int argc, char **argv) {
    if (argc < 2) {
        diag_render_into_term (25);
        return;
    }
    if (term_str_eq (argv [1], "tail")) {
        int n = (argc >= 3) ? atoi (argv [2]) : 25;
        if (n <= 0) {
            term_err ("mpe: diag: tail needs a positive count\n");
            return;
        }
        if (n > mpe_diag_ring_count ()) {
            char b [256];
            snprintf (b, sizeof (b), "mpe: diag: only %d record(s) kept; showing all\n", mpe_diag_ring_count ());
            term_dim (b);
            n = mpe_diag_ring_count ();
        }
        diag_render_into_term (n);
        return;
    }
    if (term_str_eq (argv [1], "sources")) {
        char b [512];
        int totals = mpe_diag_total ();
        snprintf (b, sizeof (b), "mpe: diag: %d record(s); %d fatal, %d error, %d warn\n", totals,
                  mpe_diag_count_level (mpe_diag_fatal), mpe_diag_count_level (mpe_diag_error),
                  mpe_diag_count_level (mpe_diag_warn));
        term_out (b);
        int n = mpe_diag_source_total_count ();
        for (int i = 0; i < n; i++) {
            const mpe_diag_source_total *st = mpe_diag_source_total_at (i);
            if (!st) { continue; }
            snprintf (b, sizeof (b), "  %-14s fatal=%u error=%u warn=%u info=%u\n", st->source, st->fatal, st->error,
                      st->warn, st->info);
            term_out (b);
        }
        if (n == 0) {
            term_out ("  (no subsystem has reported anything)\n");
        }
        return;
    }
    if (term_str_eq (argv [1], "on") || term_str_eq (argv [1], "off")) {
        bool on = term_str_eq (argv [1], "on");
        mpe_diag_set_live_mirror (on);
        char b [128];
        snprintf (b, sizeof (b), "mpe: diag: live mirror %s\n", on ? "ON (new records echo here)" : "OFF");
        term_ok (b);
        return;
    }
    if (term_str_eq (argv [1], "clear")) {
        mpe_diag_clear ();
        term_ok ("mpe: diag: cleared\n");
        return;
    }
    if (term_str_eq (argv [1], "status")) {
        char b [256];
        snprintf (b, sizeof (b), "mpe: diag: live mirror %s | %d kept of %d | %d source(s)\n",
                  mpe_diag_live_mirror () ? "on" : "off", mpe_diag_ring_count (), MPE_DIAG_RING_CAPACITY,
                  mpe_diag_source_total_count ());
        term_out (b);
        return;
    }
    term_err ("mpe: diag: usage: diag [tail N|sources|on|off|clear|status]\n");
}