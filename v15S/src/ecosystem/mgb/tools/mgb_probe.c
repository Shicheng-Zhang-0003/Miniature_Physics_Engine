/* mgb_probe.c — CLI exerciser: spawn s2tui, configure a scene, snapshot,
 * parse, map to a file sink. Exercises link+configure+frame+map end to end.
 *
 * Usage:
 *   mgb_probe --s2tui PATH --scene petri --out scene.mgb [--frame frame.s2]
 *             [--timeout S] [--mm-per-a X]
 * Scenes: petri (dd if=petri of=world). Exit 0 mapped ok, 1 any failure.
 */
#include "mgb.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* staged in mgb_sink_file.c */
int mgb_sink_file_write (FILE *f, const mgb_frame_t *frame, double mm_per_a);

static void usage (const char *p) {
    fprintf (stderr,
             "usage: %s --s2tui PATH --scene petri --out scene.mgb "
             "[--frame frame.s2] [--timeout S] [--mm-per-a X]\n", p);
}

int main (int argc, char **argv) {
    const char *s2tui = NULL, *scene = NULL, *out = NULL, *framepath = NULL;
    int timeout = 60;
    double mmpa = MGB_MM_PER_A;
    for (int i = 1; i < argc; i++) {
        if (!strcmp (argv[i], "--s2tui") && i + 1 < argc) s2tui = argv[++i];
        else if (!strcmp (argv[i], "--scene") && i + 1 < argc) scene = argv[++i];
        else if (!strcmp (argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp (argv[i], "--frame") && i + 1 < argc) framepath = argv[++i];
        else if (!strcmp (argv[i], "--timeout") && i + 1 < argc) timeout = atoi (argv[++i]);
        else if (!strcmp (argv[i], "--mm-per-a") && i + 1 < argc) mmpa = atof (argv[++i]);
        else { usage (argv[0]); return 1; }
    }
    if (!s2tui || !scene || !out || timeout <= 0 || !(mmpa > 0.0)) {
        usage (argv[0]);
        return 1;
    }
    if (strcmp (scene, "petri") != 0) {
        fprintf (stderr, "mgb_probe: unknown scene '%s' (only petri in Phase 0)\n", scene);
        return 1;
    }
    mgb_session_t *s = mgb_spawn (s2tui, timeout);
    if (!s) {
        fprintf (stderr, "mgb_probe: spawn/handshake failed for '%s'\n", s2tui);
        return 1;
    }
    int rc = 1;
    mgb_frame_t fr;
    memset (&fr, 0, sizeof fr);
    char *snap = NULL;
    FILE *of = NULL;
    /* configure: build the petri world through the shell S2 already has */
    if (mgb_cmd (s, "dd if=petri of=world", timeout) != 0) {
        fprintf (stderr, "mgb_probe: configure failed (dd petri)\n");
        goto done;
    }
    snap = mgb_snapshot (s, timeout);
    if (!snap) {
        fprintf (stderr, "mgb_probe: snapshot failed\n");
        goto done;
    }
    if (framepath) {
        /* keep the raw frame too (audit trail) */
        FILE *fs = fopen (framepath, "w");
        FILE *fi = fopen (snap, "r");
        if (fs && fi) {
            char b[8192];
            size_t n;
            while ((n = fread (b, 1, sizeof b, fi)) > 0)
                fwrite (b, 1, n, fs);
        }
        if (fi) fclose (fi);
        if (fs) fclose (fs);
    }
    if (mgb_frame_parse (snap, &fr) != 0) {
        fprintf (stderr, "mgb_probe: frame parse failed for '%s'\n", snap);
        goto done;
    }
    of = fopen (out, "w");
    if (!of) {
        fprintf (stderr, "mgb_probe: cannot write '%s'\n", out);
        goto done;
    }
    if (mgb_sink_file_write (of, &fr, mmpa) != 0) {
        fprintf (stderr, "mgb_probe: map failed\n");
        goto done;
    }
    printf ("mgb_probe: atoms=%d bonds=%d restraints=%d mm_per_a=%.3f -> %s\n",
            fr.natoms, fr.nbonds, fr.nrestraints, mmpa, out);
    rc = 0;
done:
    if (of) fclose (of);
    if (rc != 0 && out) remove (out);   /* no half scenes */
    mgb_frame_free (&fr);
    free (snap);
    mgb_close (s);
    return rc;
}
