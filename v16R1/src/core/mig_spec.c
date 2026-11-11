/* MIG WP4 step 4.2 — frozen grammar reader implementation.
 * See mig_spec.h for the contract and docs/MIG_SPEC.md for the grammar. */
#include "core/mig_spec.h"
#include "core/mpe_diag.h"
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- small helpers ------------------------------------------------- */

static int is_name_char (int c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
           (c >= '0' && c <= '9') || c == '_' || c == '-';
}

/* Strict C-locale number: optional sign, digits with optional one '.',
 * optional exponent ('e'|'E' with optional sign). Full token consumption
 * required; non-finite rejected. No hex, no thousands separators. */
static int parse_number (const char *tok, double *out) {
    if (!tok [0]) { return 0; }
    char *end = NULL;
    errno = 0;
    double v = strtod (tok, &end);
    if (end == tok || *end != '\0') { return 0; }
    if (errno == ERANGE) { return 0; }
    if (!isfinite (v)) { return 0; }
    *out = v;
    return 1;
}

static int parse_int (const char *tok, long *out) {
    if (!tok [0]) { return 0; }
    char *end = NULL;
    errno = 0;
    long v = strtol (tok, &end, 10);
    if (end == tok || *end != '\0' || errno == ERANGE) { return 0; }
    *out = v;
    return 1;
}

/* Shortest round-trip text for a double: try low precisions first, fall
 * back to %.17g (guaranteed round-trip for every double). */
static int format_number (double v, char *buf, size_t cap) {
    static const int precs [] = {6, 9, 12, 17};
    char trial [64];
    int best_len = -1;
    char best [64] = {0};
    for (size_t i = 0; i < sizeof (precs) / sizeof (precs [0]); i++) {
        int n = snprintf (trial, sizeof (trial), "%.*g", precs [i], v);
        if (n < 0 || (size_t) n >= sizeof (trial)) { continue; }
        double w;
        if (!parse_number (trial, &w) || w != v) { continue; } /* not round-trip */
        if (best_len < 0 || n < best_len) {
            best_len = n;
            memcpy (best, trial, (size_t) n + 1);
        }
        if (best_len == 1) { break; }
    }
    if (best_len < 0) {
        best_len = snprintf (best, sizeof (best), "%.17g", v);
    }
    if ((size_t) best_len >= cap) { return (int) cap; }
    memcpy (buf, best, (size_t) best_len + 1);
    return best_len;
}

static const char *code_name (int code) {
    switch (code) {
        case mig_spec_empty_file:     return "mig_spec_empty_file";
        case mig_spec_no_object:      return "mig_spec_no_object";
        case mig_spec_dup_object:     return "mig_spec_dup_object";
        case mig_spec_bad_name:       return "mig_spec_bad_name";
        case mig_spec_dup_kw:         return "mig_spec_dup_kw";
        case mig_spec_bad_frame:      return "mig_spec_bad_frame";
        case mig_spec_bad_part_index: return "mig_spec_bad_part_index";
        case mig_spec_part_cap:       return "mig_spec_part_cap";
        case mig_spec_no_parts:       return "mig_spec_no_parts";
        case mig_spec_field_outside_part: return "mig_spec_field_outside_part";
        case mig_spec_unknown_kw:     return "mig_spec_unknown_kw";
        case mig_spec_bad_type:       return "mig_spec_bad_type";
        case mig_spec_missing_type:   return "mig_spec_missing_type";
        case mig_spec_dims_wrong_type: return "mig_spec_dims_wrong_type";
        case mig_spec_missing_dims:   return "mig_spec_missing_dims";
        case mig_spec_bad_dims:       return "mig_spec_bad_dims";
        case mig_spec_missing_pose:   return "mig_spec_missing_pose";
        case mig_spec_bad_pose:       return "mig_spec_bad_pose";
        case mig_spec_bad_quat:       return "mig_spec_bad_quat";
        case mig_spec_bad_material:   return "mig_spec_bad_material";
        case mig_spec_mass_xor_density: return "mig_spec_mass_xor_density";
        case mig_spec_missing_mass:   return "mig_spec_missing_mass";
        case mig_spec_bad_mass:       return "mig_spec_bad_mass";
        case mig_spec_bad_colour:     return "mig_spec_bad_colour";
        case mig_spec_bad_number:     return "mig_spec_bad_number";
        case mig_spec_trailing:       return "mig_spec_trailing";
        default:                      return "mig_spec_unknown";
    }
}

/* ---- diagnostic emission ------------------------------------------ */

typedef struct {
    FILE *f;
    char path [512];
    int line_no;
    int err;
    int err_line;
    char err_token [128];
    int stopped;
} mig_spec_ctx;

static void diag (mig_spec_ctx *c, int code, const char *token,
                  const char *extra) {
    if (c -> stopped) { return; }
    c -> stopped = 1;
    c -> err = code;
    c -> err_line = c -> line_no;
    snprintf (c -> err_token, sizeof (c -> err_token), "%s", token ? token : "");
    char msg [256] = {0};
    size_t m = 0;
    m += snprintf (msg + m, sizeof (msg) - m, "%s:%d: %s",
                   c -> path, c -> line_no, code_name (code));
    if (token && token [0]) {
        m += snprintf (msg + m, sizeof (msg) - m, " (token '%s')", token);
    }
    if (extra && extra [0]) {
        m += snprintf (msg + m, sizeof (msg) - m, " — %s", extra);
    }
    mpe_diag_emit (mpe_diag_error, "mig_spec", code_name (code), __FILE__, __LINE__, "%s", msg);
}

/* ---- tokenization ------------------------------------------------- */

#define MAX_TOKENS 64
#define MAX_TOKEN_LEN 128

typedef struct {
    char toks [MAX_TOKENS][MAX_TOKEN_LEN];
    int count;
} mig_spec_toks;

/* Split a line into whitespace-separated tokens. '#' begins a comment. */
static int tokenize_line (char *line, mig_spec_toks *toks) {
    char *hash = strchr (line, '#');
    if (hash) { *hash = '\0'; }
    toks -> count = 0;
    char *p = line;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') { p++; }
        if (!*p) { break; }
        if (toks -> count >= MAX_TOKENS) { return -1; }
        char *d = toks -> toks [toks -> count];
        size_t used = 0;
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
            if (used < MAX_TOKEN_LEN - 1) { *d++ = *p; used++; }
            p++;
        }
        *d = '\0';
        toks -> count++;
    }
    return toks -> count;
}

/* ---- part field parsing ------------------------------------------- */

static int parse_dims (mig_spec_toks *t, mig_spec_part *part) {
    if (part -> type == 0) { /* sphere: dims radius r */
        if (t -> count != 3 || strcmp (t -> toks [1], "radius") != 0) {
            return mig_spec_bad_dims;
        }
        double v;
        if (!parse_number (t -> toks [2], &v) || !isfinite (v) || v <= 0.0) {
            return mig_spec_bad_dims;
        }
        part -> radius = v;
        part -> has_dims = 1;
        return mig_spec_ok;
    } else if (part -> type == 1) { /* cube: dims hx a hy b hz c */
        if (t -> count != 7 || strcmp (t -> toks [1], "hx") != 0 ||
            strcmp (t -> toks [3], "hy") != 0 || strcmp (t -> toks [5], "hz") != 0) {
            return mig_spec_bad_dims;
        }
        double v [3];
        for (int i = 0; i < 3; i++) {
            if (!parse_number (t -> toks [2 + i * 2], &v [i]) || !isfinite (v [i]) || v [i] <= 0.0) {
                return mig_spec_bad_dims;
            }
        }
        part -> half_extensions_x = v [0];
        part -> half_extensions_y = v [1];
        part -> half_extensions_z = v [2];
        part -> has_dims = 1;
    } else { /* cylinder: dims radius r half hl */
        if (t -> count != 5 || strcmp (t -> toks [1], "radius") != 0 ||
            strcmp (t -> toks [3], "half") != 0) {
            return mig_spec_bad_dims;
        }
        double vr, vh;
        if (!parse_number (t -> toks [2], &vr) || !isfinite (vr) || vr <= 0.0 ||
            !parse_number (t -> toks [4], &vh) || !isfinite (vh) || vh <= 0.0) {
            return mig_spec_bad_dims;
        }
        part -> radius = vr;
        part -> cylinder_half_length = vh;
        part -> has_dims = 1;
    }
    return mig_spec_ok;
}

static int parse_pose (mig_spec_toks *t, mig_spec_part *part) {
    if (t -> count != 8) { return mig_spec_bad_pose; }
    double v [7];
    for (int i = 0; i < 7; i++) {
        if (!parse_number (t -> toks [1 + i], &v [i]) || !isfinite (v [i])) {
            return mig_spec_bad_pose;
        }
    }
    part -> position_x = v [0];
    part -> position_y = v [1];
    part -> position_z = v [2];
    part -> orientation_w = v [3];
    part -> orientation_x = v [4];
    part -> orientation_y = v [5];
    part -> orientation_z = v [6];
    part -> has_pose = 1;
    double n2 = v[3]*v[3] + v[4]*v[4] + v[5]*v[5] + v[6]*v[6];
    double a = n2 > 1.0 ? n2 - 1.0 : 1.0 - n2;
    if (!isfinite (n2) || a > 1e-4) { return mig_spec_bad_quat; }
    return mig_spec_ok;
}

static int parse_material (mig_spec_toks *t, mig_spec_part *part) {
    double mu_s = -1, mu_k = -1, rest = -1;
    int got_s = 0, got_k = 0, got_r = 0;
    for (int i = 1; i < t -> count; i++) {
        if (strcmp (t -> toks [i], "mu_s") == 0) {
            if (got_s || i + 1 >= t -> count) { return mig_spec_bad_material; }
            if (!parse_number (t -> toks [i + 1], &mu_s) || !isfinite (mu_s) || mu_s < 0.0) {
                return mig_spec_bad_material;
            }
            got_s = 1;
            i++;
        } else if (strcmp (t -> toks [i], "mu_k") == 0) {
            if (got_k || i + 1 >= t -> count) { return mig_spec_bad_material; }
            if (!parse_number (t -> toks [i + 1], &mu_k) || !isfinite (mu_k) || mu_k < 0.0) {
                return mig_spec_bad_material;
            }
            got_k = 1;
            i++;
        } else if (strcmp (t -> toks [i], "rest") == 0) {
            if (got_r || i + 1 >= t -> count) { return mig_spec_bad_material; }
            if (!parse_number (t -> toks [i + 1], &rest) || !isfinite (rest) || rest < 0.0 || rest > 1.0) {
                return mig_spec_bad_material;
            }
            got_r = 1;
            i++;
        } else {
            return mig_spec_bad_material;
        }
    }
    if (!got_s || !got_k || !got_r) { return mig_spec_bad_material; }
    part -> has_material = 1;
    part -> mu_s = mu_s;
    part -> mu_k = mu_k;
    part -> restitution = rest;
    return mig_spec_ok;
}

static int parse_mass_or_density (mig_spec_toks *t, mig_spec_part *part) {
    if (part -> has_mass) { return mig_spec_mass_xor_density; }
    part -> has_mass = 1;
    int is_density = strcmp (t -> toks [0], "density") == 0;
    if (t -> count != 2) { return mig_spec_bad_mass; }
    double v;
    if (!parse_number (t -> toks [1], &v) || !isfinite (v) || v <= 0.0) {
        return mig_spec_bad_mass;
    }
    part -> mass_mode = is_density ? 1 : 0;
    part -> mass = v;
    return mig_spec_ok;
}

static int parse_colour (mig_spec_toks *t, mig_spec_part *part) {
    if (t -> count != 4) { return mig_spec_bad_colour; }
    double v [3];
    for (int i = 0; i < 3; i++) {
        if (!parse_number (t -> toks [1 + i], &v [i]) || !isfinite (v [i]) || v [i] < 0.0 || v [i] > 1.0) {
            return mig_spec_bad_colour;
        }
    }
    part -> has_colour = 1;
    part -> colour_r = v [0];
    part -> colour_g = v [1];
    part -> colour_b = v [2];
    return mig_spec_ok;
}

/* ---- normalized emit ---------------------------------------------- */

static const char *types [] = {"sphere", "cube", "cylinder"};

/* End the current statement line: trailing space -> newline. */
static size_t emit_stmt_end (char *buf, size_t cap, size_t off) {
    if (off > 0 && buf [off - 1] == ' ') { off--; }
    if (off + 1 > cap) { return cap; }
    buf [off++] = '\n';
    return off;
}

static size_t emit_word (const char *s, char *buf, size_t cap, size_t off) {
    size_t l = strlen (s);
    if (off + l + 1 > cap) { return cap; }
    memcpy (buf + off, s, l);
    off += l;
    buf [off++] = ' ';
    return off;
}

static size_t emit_number (double v, char *buf, size_t cap, size_t off) {
    char num [64];
    int n = format_number (v, num, sizeof (num));
    size_t m = (size_t) n;
    if (off + m + 1 > cap) { return cap; }
    memcpy (buf + off, num, m);
    off += m;
    buf [off++] = ' ';
    return off;
}

size_t mig_spec_emit_normalized (const mig_spec *spec, char *buf, size_t cap) {
    size_t off = 0;
    /* Line 1: mig <name> */
    off = emit_word ("mig", buf, cap, off);
    if (off >= cap) { return cap; }
    off = emit_word (spec -> name, buf, cap, off);
    if (off >= cap) { return cap; }
    off = emit_stmt_end (buf, cap, off);
    if (off >= cap) { return cap; }
    /* Line 2: frame assembly (always emitted) */
    off = emit_word ("frame", buf, cap, off);
    if (off >= cap) { return cap; }
    off = emit_word ("assembly", buf, cap, off);
    if (off >= cap) { return cap; }
    off = emit_stmt_end (buf, cap, off);
    if (off >= cap) { return cap; }
    for (int i = 0; i < spec -> count; i++) {
        const mig_spec_part *p = &spec -> parts [i];
        char idx [16];
        int n = snprintf (idx, sizeof (idx), "%d", i);
        /* part <i> type <t> */
        off = emit_word ("part", buf, cap, off);
        if (off >= cap) { return cap; }
        if ((size_t) n + 1 > cap - off) { return cap; }
        memcpy (buf + off, idx, (size_t) n);
        off += (size_t) n;
        buf [off++] = ' ';
        off = emit_word ("type", buf, cap, off);
        if (off >= cap) { return cap; }
        off = emit_word (types [p -> type], buf, cap, off);
        if (off >= cap) { return cap; }
        off = emit_stmt_end (buf, cap, off);
        if (off >= cap) { return cap; }
        /* dims */
        if (p -> type == 0) {
            off = emit_word ("dims", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_word ("radius", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> radius, buf, cap, off);
        } else if (p -> type == 1) {
            off = emit_word ("dims", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_word ("hx", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> half_extensions_x, buf, cap, off);
            off = emit_word ("hy", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> half_extensions_y, buf, cap, off);
            off = emit_word ("hz", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> half_extensions_z, buf, cap, off);
        } else {
            off = emit_word ("dims", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_word ("radius", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> radius, buf, cap, off);
            off = emit_word ("half", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> cylinder_half_length, buf, cap, off);
        }
        if (off >= cap) { return cap; }
        off = emit_stmt_end (buf, cap, off);
        if (off >= cap) { return cap; }
        /* pose */
        off = emit_word ("pose", buf, cap, off);
        if (off >= cap) { return cap; }
        off = emit_number (p -> position_x, buf, cap, off);
        off = emit_number (p -> position_y, buf, cap, off);
        off = emit_number (p -> position_z, buf, cap, off);
        off = emit_number (p -> orientation_w, buf, cap, off);
        off = emit_number (p -> orientation_x, buf, cap, off);
        off = emit_number (p -> orientation_y, buf, cap, off);
        off = emit_number (p -> orientation_z, buf, cap, off);
        if (off >= cap) { return cap; }
        off = emit_stmt_end (buf, cap, off);
        if (off >= cap) { return cap; }
        /* material (only if authored) */
        if (p -> has_material) {
            off = emit_word ("material", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_word ("mu_s", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> mu_s, buf, cap, off);
            off = emit_word ("mu_k", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> mu_k, buf, cap, off);
            off = emit_word ("rest", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> restitution, buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_stmt_end (buf, cap, off);
            if (off >= cap) { return cap; }
        }
        /* mass XOR density */
        if (p -> mass_mode == 0) {
            off = emit_word ("mass", buf, cap, off);
        } else {
            off = emit_word ("density", buf, cap, off);
        }
        if (off >= cap) { return cap; }
        off = emit_number (p -> mass, buf, cap, off);
        if (off >= cap) { return cap; }
        off = emit_stmt_end (buf, cap, off);
        if (off >= cap) { return cap; }
        /* colour (only if authored) */
        if (p -> has_colour) {
            off = emit_word ("colour", buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_number (p -> colour_r, buf, cap, off);
            off = emit_number (p -> colour_g, buf, cap, off);
            off = emit_number (p -> colour_b, buf, cap, off);
            if (off >= cap) { return cap; }
            off = emit_stmt_end (buf, cap, off);
            if (off >= cap) { return cap; }
        }
    }
    if (cap > 0) {
        if (off < cap) { buf [off] = '\0'; } else { buf [cap - 1] = '\0'; }
    }
    return off;
}

/* Check that a completed part has all required fields. Returns 0 on
 * success, or the first missing-field reject code. */
static int missing_field (mig_spec_part *p) {
    if (!p -> has_dims) { return mig_spec_missing_dims; }
    if (!p -> has_pose) { return mig_spec_missing_pose; }
    if (!p -> has_mass) { return mig_spec_missing_mass; }
    return mig_spec_ok;
}

/* ---- main parse --------------------------------------------------- */

int mig_spec_parse_file (const char *path, int max_parts, mig_spec *out) {
    if (!path || !out) { return mig_spec_no_object; }
    if (max_parts < 1) { max_parts = 1; }
    if (max_parts > MIG_SPEC_MAX_PARTS) { max_parts = MIG_SPEC_MAX_PARTS; }

    mig_spec_ctx c;
    memset (&c, 0, sizeof (c));
    snprintf (c.path, sizeof (c.path), "%s", path);
    c.line_no = 0;

    FILE *f = fopen (path, "r");
    if (!f) {
        mpe_diag_emit (mpe_diag_error, "mig_spec", code_name (mig_spec_empty_file),
                       __FILE__, __LINE__, "cannot open file %s", path);
        return mig_spec_empty_file;
    }

    mig_spec spec;
    memset (&spec, 0, sizeof (spec));
    int seen_mig = 0, seen_frame = 0;
    int next_part = 0;       /* expected index of the next part */
    int current = -1;        /* index of the part currently being authored */
    int any_statement = 0;
    char line [MIG_SPEC_LINE_CAP + 2];

    while (fgets (line, sizeof (line), f)) {
        c.line_no++;
        size_t llen = strlen (line);
        int too_long = (llen == sizeof (line) - 1 && line [llen - 1] != '\n');
        if (too_long) {
            diag (&c, mig_spec_trailing, "", "line exceeds 512 bytes");
            int ch;
            while ((ch = fgetc (f)) != '\n' && ch != EOF) { }
            break;
        }
        mig_spec_toks t;
        int ntok = tokenize_line (line, &t);
        if (ntok < 0) {
            diag (&c, mig_spec_trailing, "", "too many tokens");
            break;
        }
        if (ntok == 0) { continue; } /* blank / comment-only line */
        any_statement = 1;

        const char *kw = t.toks [0];
        if (strcmp (kw, "mig") == 0) {
            if (seen_mig) { diag (&c, mig_spec_dup_object, kw, ""); break; }
            if (t.count != 2) { diag (&c, mig_spec_bad_name, kw, ""); break; }
            const char *name = t.toks [1];
            size_t nl = strlen (name);
            if (nl < 1 || nl > 63) { diag (&c, mig_spec_bad_name, name, ""); break; }
            int ok = 1;
            for (size_t i = 0; i < nl; i++) {
                if (!is_name_char ((unsigned char) name [i])) { ok = 0; break; }
            }
            if (!ok) { diag (&c, mig_spec_bad_name, name, ""); break; }
            seen_mig = 1;
            snprintf (spec.name, sizeof (spec.name), "%s", name);
            continue;
        }

        if (!seen_mig) {
            diag (&c, mig_spec_no_object, kw, "");
            break;
        }

        if (strcmp (kw, "frame") == 0) {
            if (seen_frame) { diag (&c, mig_spec_dup_kw, kw, ""); break; }
            if (next_part > 0) { diag (&c, mig_spec_bad_frame, kw, "after parts"); break; }
            if (t.count != 2 || strcmp (t.toks [1], "assembly") != 0) {
                diag (&c, mig_spec_bad_frame, t.count >= 2 ? t.toks [1] : "", "");
                break;
            }
            seen_frame = 1;
            continue;
        }

        if (strcmp (kw, "part") == 0) {
            /* Validate the previous part before starting a new one —
             * dims, pose and mass/density are all required. */
            if (current >= 0) {
                int rc = missing_field (&spec.parts [current]);
                if (rc != mig_spec_ok) { diag (&c, rc, "part", "incomplete"); break; }
            }
            if (t.count != 4 || strcmp (t.toks [2], "type") != 0) {
                diag (&c, mig_spec_missing_type, t.count >= 2 ? t.toks [1] : "", "");
                break;
            }
            long idx;
            if (!parse_int (t.toks [1], &idx) || idx < 0 || idx != next_part) {
                diag (&c, mig_spec_bad_part_index, t.toks [1], "");
                break;
            }
            if (next_part >= max_parts) {
                diag (&c, mig_spec_part_cap, t.toks [1], "");
                break;
            }
            int pt;
            if (strcmp (t.toks [3], "sphere") == 0) { pt = 0; }
            else if (strcmp (t.toks [3], "cube") == 0) { pt = 1; }
            else if (strcmp (t.toks [3], "cylinder") == 0) { pt = 2; }
            else { diag (&c, mig_spec_bad_type, t.toks [3], ""); break; }
            current = (int) idx;
            mig_spec_part *p = &spec.parts [current];
            memset (p, 0, sizeof (*p));
            p -> type = pt;
            spec.count = current + 1;
            next_part++;
            continue;
        }

        /* Part-scoped fields. */
        if (current < 0) {
            diag (&c, mig_spec_field_outside_part, kw, "");
            break;
        }
        mig_spec_part *p = &spec.parts [current];
        int rc;
        if (strcmp (kw, "dims") == 0) {
            rc = parse_dims (&t, p);
        } else if (strcmp (kw, "pose") == 0) {
            rc = parse_pose (&t, p);
        } else if (strcmp (kw, "material") == 0) {
            rc = parse_material (&t, p);
        } else if (strcmp (kw, "mass") == 0 || strcmp (kw, "density") == 0) {
            rc = parse_mass_or_density (&t, p);
        } else if (strcmp (kw, "colour") == 0) {
            rc = parse_colour (&t, p);
        } else {
            diag (&c, mig_spec_unknown_kw, kw, "");
            break;
        }
        if (rc != mig_spec_ok) { diag (&c, rc, kw, ""); break; }
    }
    fclose (f);

    if (!c.stopped) {
        /* Validate the last part (dims, pose and mass/density required). */
        if (current >= 0) {
            int rc = missing_field (&spec.parts [current]);
            if (rc != mig_spec_ok) { diag (&c, rc, "part", "incomplete"); }
        }
        if (!c.stopped) {
            if (!any_statement) { c.err = mig_spec_empty_file; c.err_line = c.line_no; c.stopped = 1; }
            else if (!seen_mig) { c.err = mig_spec_no_object; c.err_line = c.line_no; c.stopped = 1; }
            else if (spec.count == 0) { c.err = mig_spec_no_parts; c.err_line = c.line_no; c.stopped = 1; }
        }
    }

    if (c.stopped) {
        return c.err;
    }

    memcpy (out, &spec, sizeof (spec));
    return mig_spec_ok;
}
