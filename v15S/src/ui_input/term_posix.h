#ifndef term_posix_h
#define term_posix_h
/* POSIX replacements for the GLib calls the debug terminal used to make.
 * Command logic must build on libc/POSIX only (no libglib): strdup,
 * explicit ASCII case handling (locale-independent, unlike
 * tolower/strcasecmp), a quote-aware argv splitter, and CLOCK_MONOTONIC
 * time. GTK/GDK stay strictly at the display/input edge (signal
 * signatures, text buffers, key events) — nothing here replaces those.
 * All functions are static inline: zero link surface, no ODR risk
 * across the duplicated GTK4/GTK3 blocks that include this header.
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>

/* ASCII-only lowercase (locale-independent, unlike tolower(3)). */
static inline int term_ascii_tolower(int c) {
    return (c >= 'A' && c <= 'Z') ? (c + ('a' - 'A')) : c;
}

/* Locale-independent strcasecmp for command/variable names.
 * NULL-tolerant (NULL sorts before any string); 0 means equal. */
static inline int term_ascii_strcasecmp(const char *a, const char *b) {
    if (a == b) {
        return 0;
    }
    if (!a) {
        return -1;
    }
    if (!b) {
        return 1;
    }
    while (*a && *b) {
        int d = term_ascii_tolower((unsigned char)*a) - term_ascii_tolower((unsigned char)*b);
        if (d != 0) {
            return d;
        }
        a++;
        b++;
    }
    return term_ascii_tolower((unsigned char)*a) - term_ascii_tolower((unsigned char)*b);
}

/* Bounded variant: compares at most n chars. */
static inline int term_ascii_strncasecmp(const char *a, const char *b, size_t n) {
    if (n == 0) {
        return 0;
    }
    if (a == b) {
        return 0;
    }
    if (!a) {
        return -1;
    }
    if (!b) {
        return 1;
    }
    while (n-- > 0) {
        int d = term_ascii_tolower((unsigned char)*a) - term_ascii_tolower((unsigned char)*b);
        if (d != 0) {
            return d;
        }
        if (*a == '\0') {
            return 0;
        }
        a++;
        b++;
    }
    return 0;
}

/* malloc'd ASCII-lowercased copy (NULL in -> NULL out). */
static inline char *term_ascii_strdown(const char *s) {
    if (!s) {
        return NULL;
    }
    size_t n = strlen(s);
    char *out = (char *)malloc(n + 1u);
    if (!out) {
        return NULL;
    }
    for (size_t i = 0; i <= n; i++) {
        out[i] = (char)term_ascii_tolower((unsigned char)s[i]);
    }
    return out;
}

/* NULL-tolerant strdup/strndup (g_strdup crashes on NULL; these return it). */
static inline char *term_strdup(const char *s) {
    if (!s) {
        return NULL;
    }
    size_t n = strlen(s);
    char *out = (char *)malloc(n + 1u);
    if (!out) {
        return NULL;
    }
    memcpy(out, s, n + 1u);
    return out;
}

static inline char *term_strndup(const char *s, size_t n) {
    if (!s) {
        return NULL;
    }
    size_t len = 0;
    while (len < n && s[len] != '\0') {
        len++;
    }
    char *out = (char *)malloc(len + 1u);
    if (!out) {
        return NULL;
    }
    memcpy(out, s, len);
    out[len] = '\0';
    return out;
}

static inline void term_strfree(char *s) {
    free(s);
}

/* malloc'd printf (replaces g_strdup_printf). NULL on OOM/encoding error. */
static inline char *term_format(const char *fmt, ...) {
    char stack[256];
    va_list ap;
    va_start(ap, fmt);
    int need = vsnprintf(stack, sizeof(stack), fmt, ap);
    va_end(ap);
    if (need < 0) {
        return NULL;
    }
    if ((size_t)need < sizeof(stack)) {
        return term_strdup(stack);
    }
    char *out = (char *)malloc((size_t)need + 1u);
    if (!out) {
        return NULL;
    }
    va_start(ap, fmt);
    vsnprintf(out, (size_t)need + 1u, fmt, ap);
    va_end(ap);
    return out;
}

/* Prefix test (replaces g_str_has_prefix). */
static inline int term_str_has_prefix(const char *s, const char *prefix) {
    if (!s || !prefix) {
        return 0;
    }
    size_t n = strlen(prefix);
    return strncmp(s, prefix, n) == 0;
}

/* Split on every occurrence of delim, KEEPING empty tokens (matches the
 * g_strsplit(s, "/", -1) contract the path parser relies on: a leading
 * delimiter yields a leading empty token). NULL-terminated array;
 * NULL input yields {NULL}. Free with term_strfreev. */
static inline char **term_strsplit(const char *s, char delim) {
    if (!s) {
        char **out = (char **)malloc(sizeof(char *));
        if (out) {
            out[0] = NULL;
        }
        return out;
    }
    size_t count = 1u;
    for (const char *p = s; *p; p++) {
        if (*p == delim) {
            count++;
        }
    }
    char **out = (char **)malloc((count + 1u) * sizeof(char *));
    if (!out) {
        return NULL;
    }
    size_t idx = 0;
    const char *start = s;
    for (;;) {
        if (*s == delim || *s == '\0') {
            size_t len = (size_t)(s - start);
            char *tok = (char *)malloc(len + 1u);
            if (!tok) {
                for (size_t k = 0; k < idx; k++) {
                    free(out[k]);
                }
                free(out);
                return NULL;
            }
            memcpy(tok, start, len);
            tok[len] = '\0';
            out[idx++] = tok;
            if (*s == '\0') {
                break;
            }
            start = s + 1;
        }
        s++;
    }
    out[idx] = NULL;
    return out;
}

/* Free a NULL-terminated string vector (NULL-tolerant, like g_strfreev). */
static inline void term_strfreev(char **v) {
    if (!v) {
        return;
    }
    for (size_t i = 0; v[i]; i++) {
        free(v[i]);
    }
    free(v);
}

/* Quote-aware command-line splitter replacing g_shell_parse_argv for the
 * terminal's needs: single/double quotes group words, backslash escapes
 * the next char (inside and outside quotes). No $ expansion, no tilde,
 * no comments, no command substitution — deliberately a smaller,
 * predictable contract. Unmatched quote is an error with errbuf set.
 * Returns 1 on success (*argv_out NULL-terminated, free with
 * term_strfreev), 0 on error/OOM. */
static inline int term_parse_argv(const char *cmd, int *argc_out, char ***argv_out, char *errbuf,
                                  int errlen) {
    if (argc_out) {
        *argc_out = 0;
    }
    if (argv_out) {
        *argv_out = NULL;
    }
    if (!cmd) {
        return 1;
    }
    size_t cap = 8u, argc = 0;
    char **argv = (char **)malloc(cap * sizeof(char *));
    if (!argv) {
        return 0;
    }
    size_t bcap = 64u, blen = 0;
    char *buf = (char *)malloc(bcap);
    if (!buf) {
        free(argv);
        return 0;
    }
    int in_word = 0, in_squote = 0, in_dquote = 0, esc = 0, fail = 0;
    const char *p = cmd;
    for (;;) {
        char c = *p;
        int end = (c == '\0');
        if (esc) {
            esc = 0;
        } else if (c == '\\' && !in_squote) {
            esc = 1;
            p++;
            continue;
        } else if (c == '\'' && !in_dquote) {
            in_squote = !in_squote;
            in_word = 1;
            p++;
            continue;
        } else if (c == '"' && !in_squote) {
            in_dquote = !in_dquote;
            in_word = 1;
            p++;
            continue;
        } else if (!in_squote && !in_dquote && (c == ' ' || c == '\t' || c == '\n' || end)) {
            if (in_word) {
                if (blen + 1u >= bcap) {
                    fail = 1;
                    break;
                }
                buf[blen] = '\0';
                if (argc + 1u >= cap) {
                    size_t ncap = cap * 2u;
                    char **nargv = (char **)realloc(argv, ncap * sizeof(char *));
                    if (!nargv) {
                        fail = 1;
                        break;
                    }
                    argv = nargv;
                    cap = ncap;
                }
                argv[argc] = term_strdup(buf);
                if (!argv[argc]) {
                    fail = 1;
                    break;
                }
                argc++;
                blen = 0;
                in_word = 0;
            }
            if (end) {
                break;
            }
            p++;
            continue;
        }
        if (end) {
            break;
        }
        if (blen + 1u >= bcap) {
            size_t nbcap = bcap * 2u;
            char *nbuf = (char *)realloc(buf, nbcap);
            if (!nbuf) {
                fail = 1;
                break;
            }
            buf = nbuf;
            bcap = nbcap;
        }
        buf[blen++] = c;
        in_word = 1;
        p++;
    }
    if (!fail && (in_squote || in_dquote)) {
        fail = 1;
        if (errbuf && errlen > 0) {
            snprintf(errbuf, (size_t)errlen, "unmatched %s quote",
                     in_squote ? "single" : "double");
        }
    }
    if (fail) {
        for (size_t k = 0; k < argc; k++) {
            free(argv[k]);
        }
        free(argv);
        free(buf);
        return 0;
    }
    if (argc + 1u > cap) {
        char **nargv = (char **)realloc(argv, (argc + 1u) * sizeof(char *));
        if (!nargv) {
            for (size_t k = 0; k < argc; k++) {
                free(argv[k]);
            }
            free(argv);
            free(buf);
            return 0;
        }
        argv = nargv;
    }
    argv[argc] = NULL;
    free(buf);
    if (argc_out) {
        *argc_out = (int)argc;
    }
    if (argv_out) {
        *argv_out = argv;
    } else {
        term_strfreev(argv);
    }
    return 1;
}

/* Monotonic microseconds (replaces g_get_monotonic_time). */
static inline int64_t term_monotonic_us(void) {
    struct timespec ts;
#if defined(CLOCK_MONOTONIC)
    if (clock_gettime(CLOCK_MONOTONIC, &ts) == 0) {
        return (int64_t)ts.tv_sec * 1000000LL + (int64_t)(ts.tv_nsec / 1000L);
    }
#endif
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
        return (int64_t)ts.tv_sec * 1000000LL + (int64_t)(ts.tv_nsec / 1000L);
    }
    return 0;
}

#endif /* term_posix_h */
