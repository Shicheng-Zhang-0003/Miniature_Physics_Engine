/* mgb_frame.c — strict S2SAVE1 parser. No repair, no guessing.
 *
 * Grammar (observed from s2tui `sync`, pinned by tests):
 *   S2SAVE1\n
 *   seed %lu\n
 *   dt %lg cutoff %lg dielectric %lg temp %lg thermostat %d tau %lg nu %lg\n
 *   atoms %d\n  (0..MGB_MAX_ATOMS)
 *   Z x y z q eps sigma vx vy vz\n  (x natoms, 10 fields, strtod exact)
 *   bonds %d\n    (0..MGB_MAX_BONDS)
 *   a b order r0 k\n                 (x nbonds, 5 fields)
 *   restraints %d\n (0..MGB_MAX_RESTRAINTS; 5- or 6-col rows, counted only)
 *
 * Any deviation (bad magic, short file, count mismatch, trailing garbage
 * tolerated? NO — extra lines after restraints are rejected) fails the
 * whole parse with -1 and frees everything. A corrupt frame must never
 * become half a scene.
 */
#include "mgb.h"
#define _POSIX_C_SOURCE 200809L /* strtok_r */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void mgb_frame_free (mgb_frame_t *f) {
    if (!f) return;
    free (f->atoms);
    free (f->bonds);
    memset (f, 0, sizeof *f);
}

static int read_line (FILE *fp, char *buf, size_t sz) {
    if (!fgets (buf, sz, fp)) return -1;
    size_t n = strlen (buf);
    if (n && buf[n - 1] == '\n') buf[n - 1] = '\0';
    else return -1;     /* overlong line or missing newline: corrupt */
    return 0;
}

/* strict double: full consumption, finite */
static int parse_double (const char *s, double *out) {
    char *e = NULL;
    double v = strtod (s, &e);
    if (!e || e == s || !isfinite (v)) return -1;
    while (*e == ' ' || *e == '\t') e++;
    if (*e != '\0') return -1;
    *out = v;
    return 0;
}

static int parse_long (const char *s, long *out) {
    char *e = NULL;
    long v = strtol (s, &e, 10);
    if (!e || e == s) return -1;
    while (*e == ' ' || *e == '\t') e++;
    if (*e != '\0') return -1;
    *out = v;
    return 0;
}

/* split line into exactly want tokens (space-separated); -1 otherwise */
static int split (char *line, char *tok[], int want) {
    int n = 0;
    char *save = NULL;
    for (char *t = strtok_r (line, " \t", &save); t;
         t = strtok_r (NULL, " \t", &save)) {
        if (n >= want) return -1;
        tok[n++] = t;
    }
    return n == want ? 0 : -1;
}

int mgb_frame_parse (const char *path, mgb_frame_t *f) {
    if (!path || !f) return -1;
    memset (f, 0, sizeof *f);
    FILE *fp = fopen (path, "r");
    if (!fp) return -1;
    char lb[1024];
    char *tok[12];
    int rc = -1;

    if (read_line (fp, lb, sizeof lb) != 0 || strcmp (lb, "S2SAVE1") != 0)
        goto done;
    /* seed line: `seed %lu` */
    if (read_line (fp, lb, sizeof lb) != 0 || split (lb, tok, 2) != 0 ||
        strcmp (tok[0], "seed") != 0)
        goto done;
    { long sd; if (parse_long (tok[1], &sd) != 0 || sd < 0) goto done;
      f->seed = (double)sd; }
    /* settings line: dt/cutoff/dielectric/temp/thermostat/tau/nu = 14 toks */
    if (read_line (fp, lb, sizeof lb) != 0) goto done;
    {
        char *t2[16];
        char *save = NULL;
        int n = 0;
        for (char *t = strtok_r (lb, " \t", &save); t;
             t = strtok_r (NULL, " \t", &save)) {
            if (n >= 16) goto done;
            t2[n++] = t;
        }
        /* dt v cutoff v dielectric v temp v thermostat i tau v nu v */
        if (n != 14 || strcmp (t2[0], "dt") != 0 ||
            strcmp (t2[2], "cutoff") != 0 || strcmp (t2[4], "dielectric") != 0 ||
            strcmp (t2[6], "temp") != 0 || strcmp (t2[8], "thermostat") != 0 ||
            strcmp (t2[10], "tau") != 0 || strcmp (t2[12], "nu") != 0)
            goto done;
        double v[7];
        const int vi[] = {1, 3, 5, 7, 11, 13};
        for (int i = 0; i < 6; i++)
            if (parse_double (t2[vi[i]], &v[i]) != 0) goto done;
        long th;
        if (parse_long (t2[9], &th) != 0 || th < 0 || th > 3) goto done;
        f->dt = v[0]; f->cutoff = v[1]; f->dielectric = v[2];
        f->temp = v[3]; f->thermostat = (int)th; f->tau = v[4]; f->nu = v[5];
    }
    if (read_line (fp, lb, sizeof lb) != 0 || split (lb, tok, 2) != 0 ||
        strcmp (tok[0], "atoms") != 0)
        goto done;
    { long na; if (parse_long (tok[1], &na) != 0 || na < 0 ||
                   na > MGB_MAX_ATOMS) goto done;
      f->natoms = (int)na; }
    if (f->natoms > 0) {
        f->atoms = calloc ((size_t)f->natoms, sizeof (mgb_atom_t));
        if (!f->atoms) goto done;
        for (int i = 0; i < f->natoms; i++) {
            if (read_line (fp, lb, sizeof lb) != 0 ||
                split (lb, tok, 10) != 0)
                goto done;
            long z;
            double v[9];
            if (parse_long (tok[0], &z) != 0 || z < 1 || z > 118)
                goto done;
            for (int k = 0; k < 9; k++)
                if (parse_double (tok[k + 1], &v[k]) != 0) goto done;
            f->atoms[i].Z = (int)z;
            f->atoms[i].x = v[0]; f->atoms[i].y = v[1];
            f->atoms[i].z = v[2]; f->atoms[i].q = v[3];
            f->atoms[i].lj_eps = v[4]; f->atoms[i].lj_sigma = v[5];
            f->atoms[i].vx = v[6]; f->atoms[i].vy = v[7];
            f->atoms[i].vz = v[8];
        }
    }
    if (read_line (fp, lb, sizeof lb) != 0 || split (lb, tok, 2) != 0 ||
        strcmp (tok[0], "bonds") != 0)
        goto done;
    { long nb; if (parse_long (tok[1], &nb) != 0 || nb < 0 ||
                   nb > MGB_MAX_BONDS) goto done;
      f->nbonds = (int)nb; }
    if (f->nbonds > 0) {
        f->bonds = calloc ((size_t)f->nbonds, sizeof (mgb_bond_t));
        if (!f->bonds) goto done;
        for (int i = 0; i < f->nbonds; i++) {
            if (read_line (fp, lb, sizeof lb) != 0 ||
                split (lb, tok, 5) != 0)
                goto done;
            long a, b, o;
            double r0, k;
            if (parse_long (tok[0], &a) != 0 ||
                parse_long (tok[1], &b) != 0 ||
                parse_long (tok[2], &o) != 0 ||
                parse_double (tok[3], &r0) != 0 ||
                parse_double (tok[4], &k) != 0)
                goto done;
            if (a < 0 || b < 0 || a >= f->natoms || b >= f->natoms ||
                a == b || !isfinite (r0) || !isfinite (k))
                goto done;
            f->bonds[i].a = (int)a; f->bonds[i].b = (int)b;
            f->bonds[i].order = (int)o;
            f->bonds[i].r0 = r0; f->bonds[i].k = k;
        }
    }
    if (read_line (fp, lb, sizeof lb) != 0 || split (lb, tok, 2) != 0 ||
        strcmp (tok[0], "restraints") != 0)
        goto done;
    { long nr; if (parse_long (tok[1], &nr) != 0 || nr < 0 ||
                   nr > MGB_MAX_RESTRAINTS) goto done;
      f->nrestraints = (int)nr; }
    for (int i = 0; i < f->nrestraints; i++) {
        /* 5 cols (no flat) or 6 cols (with flat): counted, not retained */
        if (read_line (fp, lb, sizeof lb) != 0) goto done;
        char *t2[8];
        char *save = NULL;
        int n = 0;
        for (char *t = strtok_r (lb, " \t", &save); t;
             t = strtok_r (NULL, " \t", &save)) {
            if (n >= 8) goto done;
            t2[n++] = t;
        }
        if (n != 5 && n != 6) goto done;
        for (int k = 0; k < n; k++) {
            double d;
            if (parse_double (t2[k], &d) != 0) goto done;
        }
    }
    /* trailing garbage rejected: EOF must follow */
    if (fgetc (fp) != EOF) goto done;
    rc = 0;
done:
    fclose (fp);
    if (rc != 0) mgb_frame_free (f);
    return rc;
}
