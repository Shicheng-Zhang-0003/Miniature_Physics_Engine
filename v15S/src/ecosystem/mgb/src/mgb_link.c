/* mgb_link.c — own an s2tui child: spawn, handshake, command, snapshot.
 *
 * Transport: THREE pipes. Child stdin down (commands). Child stdout up
 * (command output; drained to a transcript file — content informational
 * only). Child stderr up (banner/prompts/diagnostics; the FRAMING
 * channel). This split is s2tui's own contract (display.h): stdout is
 * DATA, stderr is display — and the `s2>` prompt is printed to stderr
 * with fflush, so prompt framing MUST read stderr. Reading stdout for
 * the prompt was the first bug in this file (fixed before first probe).
 *
 * Completion signal: the next `s2>` prompt on stderr. Causal order in the
 * child (run_line prints stdout, loop prints prompt) plus a final
 * non-blocking stdout drain after the prompt keeps the transcript whole.
 * The snapshot frame itself travels via `sync` FILE (see mgb_snapshot),
 * never via stdout — so stdout buffering can delay transcript text but
 * can never corrupt a frame.
 *
 * Any timeout, EOF without prompt, or missing frame file marks the
 * session dead: further calls fail immediately, mgb_close kills + reaps.
 * A dead child can never yield half a frame.
 */
#define _POSIX_C_SOURCE 200809L
#include "mgb.h"
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

struct mgb_session {
    pid_t pid;
    int to_child;       /* write end: child stdin */
    int from_out;       /* read end: child stdout (transcript only) */
    int from_err;       /* read end: child stderr (framing) */
    char transcript[256];
    char tmpdir[256];
    int dead;
    unsigned long seq;
};

static void mgb_mark_dead (mgb_session_t *s) {
    if (s) s->dead = 1;
}

static void set_nonblock (int fd) {
    int fl = fcntl (fd, F_GETFL, 0);
    if (fl >= 0) fcntl (fd, F_SETFL, fl | O_NONBLOCK);
}

/* Drain whatever stdout has available into the transcript (non-blocking). */
static void drain_stdout (mgb_session_t *s, FILE *log) {
    char tmp[2048];
    for (;;) {
        ssize_t n = read (s->from_out, tmp, sizeof tmp);
        if (n <= 0) break;
        if (log) {
            fwrite (tmp, 1, (size_t)n, log);
            fflush (log);
        }
    }
}

/* Read stderr until the `s2>` prompt (select timeout). Returns bytes of
 * stderr kept in *keep (may be NULL to discard). -1 on timeout/EOF. */
static long mgb_read_prompt (mgb_session_t *s, FILE *log, char **keep,
                             int timeout_sec) {
    size_t cap = 4096, len = 0;
    char *buf = malloc (cap);
    if (!buf) return -1;
    for (;;) {
        fd_set rf;
        FD_ZERO (&rf);
        FD_SET (s->from_err, &rf);
        FD_SET (s->from_out, &rf);
        int mx = s->from_err > s->from_out ? s->from_err : s->from_out;
        struct timeval tv = { timeout_sec, 0 };
        int r = select (mx + 1, &rf, NULL, NULL, &tv);
        if (r < 0) {
            if (errno == EINTR) continue;
            free (buf);
            return -1;
        }
        if (r == 0) { free (buf); return -1; }  /* timeout */
        if (FD_ISSET (s->from_out, &rf)) drain_stdout (s, log);
        if (FD_ISSET (s->from_err, &rf)) {
            char tmp[1024];
            ssize_t n = read (s->from_err, tmp, sizeof tmp);
            if (n <= 0) { free (buf); return -1; }  /* EOF/err */
            if (log) {
                fwrite (tmp, 1, (size_t)n, log);
                fflush (log);
            }
            if (len + (size_t)n + 1 > cap) {
                cap = (len + (size_t)n + 1) * 2;
                char *nb = realloc (buf, cap);
                if (!nb) { free (buf); return -1; }
                buf = nb;
            }
            memcpy (buf + len, tmp, (size_t)n);
            len += (size_t)n;
            buf[len] = '\0';
            /* Prompt forms: "s2> " at buffer start (first prompt after
             * banner line) or "\ns2> " (subsequent). Suffix check covers
             * prompt split across reads. */
            if (strstr (buf, "\ns2> ") || strstr (buf, "s2> ") == buf ||
                (len >= 4 && memcmp (buf + len - 4, "s2> ", 4) == 0))
                break;
        }
    }
    /* final stdout drain: prompt is causal-after stdout, but the pipe may
     * lag it by a scheduling quantum (block buffering). */
    drain_stdout (s, log);
    if (keep) *keep = buf; else free (buf);
    return (long)len;
}

/* Thread-local last failure: mgb_spawn still returns NULL for every cause
 * (callers rely on that), and the reason travels beside it. */
static _Thread_local mgb_spawn_status_t g_spawn_status = mgb_spawn_ok;
static _Thread_local int g_spawn_errno = 0;
const char *mgb_spawn_strerror (mgb_spawn_status_t st) {
    switch (st) {
        case mgb_spawn_ok: return ("ok");
        case mgb_spawn_e_no_path: return ("empty path");
        case mgb_spawn_e_pipe: return ("pipe() failed");
        case mgb_spawn_e_fork: return ("fork() failed");
        case mgb_spawn_e_notfound: return ("no such file");
        case mgb_spawn_e_noexec: return ("exec() failed");
        case mgb_spawn_e_noexecbit: return ("file exists but is not executable");
        case mgb_spawn_e_handshake: return ("s2tui started but never sent the expected banner/prompt");
        case mgb_spawn_e_alloc: return ("out of memory");
        default: return ("unknown spawn failure");
    }
}
mgb_spawn_status_t mgb_spawn_reason (int *why_errno) {
    if (why_errno) *why_errno = g_spawn_errno;
    return (g_spawn_status);
}
static int fail_spawn (mgb_spawn_status_t st, int e) {
    g_spawn_status = st;
    g_spawn_errno = e;
    return 0;
}
mgb_session_t *mgb_spawn (const char *s2tui_path, int timeout_sec) {
    if (!s2tui_path || !*s2tui_path) { fail_spawn (mgb_spawn_e_no_path, 0); return NULL; }
    if (timeout_sec <= 0) timeout_sec = 15;
    /* Executable bit first. fopen-style "can I read it" checks pass for a
     * non-executable file and then exec fails with a bare 127 exit, which is
     * exactly the case that used to be indistinguishable from a timeout. */
    if (access (s2tui_path, X_OK) != 0) {
        mgb_spawn_status_t st = mgb_spawn_e_noexec;
        if (errno == EACCES) st = mgb_spawn_e_noexecbit;
        else if (errno == ENOENT || errno == ENOTDIR) st = mgb_spawn_e_notfound;
        fail_spawn (st, errno);
        return NULL;
    }
    g_spawn_status = mgb_spawn_ok;
    g_spawn_errno = 0;
    int to_c[2], out_c[2], err_c[2];
    if (pipe (to_c) != 0 || pipe (out_c) != 0 || pipe (err_c) != 0)
        { fail_spawn (mgb_spawn_e_pipe, errno); return NULL; }
    /* Exec-status channel: the child writes errno here if execl fails, so
     * "exec failed: ENOENT" is reported instead of the child vanishing. */
    int exec_err[2];
    if (pipe (exec_err) != 0) { fail_spawn (mgb_spawn_e_pipe, errno); return NULL; }
    fcntl (exec_err[1], F_SETFD, FD_CLOEXEC);
    pid_t pid = fork ();
    if (pid < 0) {
        close (to_c[0]); close (to_c[1]);
        close (out_c[0]); close (out_c[1]);
        close (err_c[0]); close (err_c[1]);
        return NULL;
    }
    if (pid == 0) {
        dup2 (to_c[0], STDIN_FILENO);
        dup2 (out_c[1], STDOUT_FILENO);
        dup2 (err_c[1], STDERR_FILENO);
        close (to_c[0]); close (to_c[1]);
        close (out_c[0]); close (out_c[1]);
        close (err_c[0]); close (err_c[1]);
        execl (s2tui_path, "s2tui", (char *)NULL);
        { int e = errno; ssize_t w = write (exec_err[1], &e, sizeof e); (void) w; }
        _exit (127);
    }
    close (to_c[0]); close (out_c[1]); close (err_c[1]); close (exec_err[1]);
    set_nonblock (out_c[0]);
    mgb_session_t *s = calloc (1, sizeof *s);
    if (!s) {
        close (to_c[1]); close (out_c[0]); close (err_c[0]);
        kill (pid, SIGKILL);
        waitpid (pid, NULL, 0);
        return NULL;
    }
    s->pid = pid;
    s->to_child = to_c[1];
    s->from_out = out_c[0];
    s->from_err = err_c[0];
    const char *td = getenv ("TMPDIR");
    /* TMPDIR is env-controlled: overlong values previously truncated
     * silently into unintended paths. Reject, don't truncate. */
    if (!td || !*td || strlen (td) > 200 || strchr (td, '\n')) td = "/tmp";
    snprintf (s->tmpdir, sizeof s->tmpdir, "%s", td);
    if (snprintf (s->transcript, sizeof s->transcript,
                  "%s/mgb_transcript_%d.log", s->tmpdir,
                  (int)pid) >= (int)sizeof s->transcript) {
        mgb_close (s);
        return NULL;
    }
    FILE *log = fopen (s->transcript, "w");
    /* Did exec actually succeed? The CLOEXEC pipe closes with EOF on a good
     * exec and carries the real errno on a bad one. Non-blocking so a
     * successful exec (EOF immediately) never stalls the handshake. */
    int exec_errno = 0;
    {
        /* Race-free by construction. On a SUCCESSFUL exec the CLOEXEC write
         * end closes, so poll() reports POLLHUP and read() returns 0 at once.
         * On a FAILED exec the child writes errno and poll() reports
         * POLLIN. An earlier non-blocking read raced the child and turned
         * every exec failure into a bogus "handshake timeout" — the same
         * misdiagnosis this whole change exists to remove. The timeout only
         * guards a child that dies between fork and exec, where neither
         * event ever fires. */
        struct pollfd pfd;
        pfd.fd = exec_err[0];
        pfd.events = POLLIN;
        pfd.revents = 0;
        if (poll (&pfd, 1, 500) > 0 && (pfd.revents & POLLIN)) {
            int e = 0;
            if (read (exec_err[0], &e, sizeof e) == (ssize_t) sizeof e) exec_errno = e;
        }
        close (exec_err[0]);
    }
    if (exec_errno != 0) {
        if (log) fclose (log);
        fail_spawn (mgb_spawn_e_noexec, exec_errno);
        mgb_close (s);
        return NULL;
    }
    /* handshake: banner (identity) + first prompt, both on stderr */
    char *banner = NULL;
    long n = mgb_read_prompt (s, log, &banner, timeout_sec);
    if (log) fclose (log);
    int ok = n > 0 && banner && strstr (banner, "s2tui");
    if (!ok && banner == NULL && n <= 0) fail_spawn (mgb_spawn_e_handshake, 0);
    free (banner);
    if (!ok) { mgb_close (s); return NULL; }
    return s;
}

int mgb_cmd (mgb_session_t *s, const char *line, int timeout_sec) {
    if (!s || s->dead || !line) return -1;
    if (timeout_sec <= 0) timeout_sec = 30;
    size_t n = strlen (line);
    char *msg = malloc (n + 2);
    if (!msg) { mgb_mark_dead (s); return -1; }
    memcpy (msg, line, n);
    msg[n] = '\n';
    msg[n + 1] = '\0';
    size_t off = 0;
    while (off < n + 1) {
        ssize_t w = write (s->to_child, msg + off, n + 1 - off);
        if (w <= 0) {
            free (msg);
            mgb_mark_dead (s);
            return -1;
        }
        off += (size_t)w;
    }
    free (msg);
    s->seq++;
    FILE *log = fopen (s->transcript, "a");
    long r = mgb_read_prompt (s, log, NULL, timeout_sec);
    if (log) fclose (log);
    if (r < 0) { mgb_mark_dead (s); return -1; }
    return 0;
}

char *mgb_snapshot (mgb_session_t *s, int timeout_sec) {
    if (!s || s->dead) return NULL;
    if (timeout_sec <= 0) timeout_sec = 60;
    char path[300];
    snprintf (path, sizeof path, "%s/mgb_frame_%d_%lu.s2",
              s->tmpdir, (int)s->pid, s->seq);
    char cmd[340];
    snprintf (cmd, sizeof cmd, "sync %s", path);
    if (mgb_cmd (s, cmd, timeout_sec) != 0) return NULL;
    /* `sync` prints `saved N atoms`; completion already framed. Verify
     * the file exists and is non-empty before handing out. */
    FILE *f = fopen (path, "r");
    if (!f) { mgb_mark_dead (s); return NULL; }
    int c = fgetc (f);
    fclose (f);
    if (c == EOF) { mgb_mark_dead (s); return NULL; }
    char *out = malloc (strlen (path) + 1);
    if (!out) { mgb_mark_dead (s); return NULL; }
    strcpy (out, path);
    return out;
}

void mgb_close (mgb_session_t *s) {
    if (!s) return;
    /* EOF the child's stdin (s2tui prints bye and exits), then enforce */
    close (s->to_child);
    int st;
    for (int i = 0; i < 50; i++) {
        pid_t r = waitpid (s->pid, &st, WNOHANG);
        if (r == s->pid) break;
        if (r < 0 && errno == ECHILD) break;
        struct timespec ts = { 0, 20000000L };
        nanosleep (&ts, NULL);
        if (i == 25) kill (s->pid, SIGTERM);
    }
    if (waitpid (s->pid, &st, WNOHANG) != s->pid) {
        kill (s->pid, SIGKILL);
        waitpid (s->pid, &st, 0);
    }
    close (s->from_out);
    close (s->from_err);
    free (s);
}

int mgb_dead (const mgb_session_t *s) {
    return !s || s->dead;
}
