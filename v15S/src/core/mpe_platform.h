/* mpe_platform.h — central OS/compiler portability layer.
 *
 * Goals:
 *  - Linux behaviour is UNCHANGED (all POSIX paths preserved verbatim).
 *  - Windows (MSYS2/MinGW GCC + native MSVC) compiles and runs.
 *  - MinGW GCC keeps using winpthreads + GCC __attribute__ (all supported).
 *  - MSVC gets drop-in shims for pthread, dlfcn, POSIX file/time/string.
 *
 * Usage: #include "core/mpe_platform.h" (engine) or #include "mfs_platform.h"
 * (standalone 461-MFS, identical copy). Include it BEFORE any POSIX header.
 */
#ifndef mpe_platform_h
#define mpe_platform_h
/* ------------------------------------------------------------------ */
/* 1. OS / compiler detection                                          */
/* ------------------------------------------------------------------ */
#if defined(_WIN32) || defined(_WIN64) || defined(__WIN32__) || defined(__MINGW32__) || defined(__MINGW64__)
#ifndef MPE_OS_WINDOWS
#define MPE_OS_WINDOWS 1
#endif
#else
#ifndef MPE_OS_POSIX
#define MPE_OS_POSIX 1
#endif
#endif
#if defined(_MSC_VER)
#define MPE_COMPILER_MSVC 1
#endif
#if defined(__MINGW32__) || defined(__MINGW64__)
#define MPE_COMPILER_MINGW 1
#endif
#if defined(__GNUC__) && !defined(__clang__)
#define MPE_COMPILER_GCC 1
#endif
#if defined(__clang__)
#define MPE_COMPILER_CLANG 1
#endif
#if defined(__GNUC__) || defined(__clang__)
#define MPE_HAS_GNUC_ATTR 1
#endif
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef MPE_OS_WINDOWS
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <direct.h>
#include <io.h>
#include <process.h>
#include <share.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#ifndef PATH_MAX
#ifdef MAX_PATH
#define PATH_MAX 4096
#else
#define PATH_MAX 4096
#endif
#endif
#else
#include <limits.h>
#ifndef PATH_MAX
#define PATH_MAX 4096
#endif
#endif
/* ------------------------------------------------------------------ */
/* 2. Plugin / executable extensions, path separators                  */
/* ------------------------------------------------------------------ */
#ifdef MPE_OS_WINDOWS
#define MPE_PLUGIN_EXT ".dll"
#define MPE_PLUGIN_EXT_NODOT "dll"
#define MPE_EXE_EXT ".exe"
#define MPE_PATH_SEP '\\'
#define MPE_PATH_SEP_STR "\\"
#else
#define MPE_PLUGIN_EXT ".so"
#define MPE_PLUGIN_EXT_NODOT "so"
#define MPE_EXE_EXT ""
#define MPE_PATH_SEP '/'
#define MPE_PATH_SEP_STR "/"
#endif
/* True if c is a path separator on this platform (Windows accepts both). */
static inline int mpe_is_path_sep (char c) {
#ifdef MPE_OS_WINDOWS
    return c == '/' || c == '\\';
#else
    return c == '/';
#endif
}
/* ------------------------------------------------------------------ */
/* 3. Symbol visibility / attribute macros (Linux unchanged)           */
/* ------------------------------------------------------------------ */
#ifdef MPE_HAS_GNUC_ATTR
#define MPE_USED __attribute__ ((used))
#define MPE_WEAK __attribute__ ((weak))
#define MPE_CTOR __attribute__ ((constructor))
#define MPE_DTOR __attribute__ ((destructor))
#define MPE_WEAK_SUPPORTED 1
#else
/* MSVC: no weak/ctor/dtor. Callers must use MPE_WEAK_SUPPORTED guards
    * and explicit init/fini exports (see mpe_capsule.c). */
#define MPE_USED
#define MPE_WEAK
#define MPE_CTOR
#define MPE_DTOR
#define MPE_WEAK_SUPPORTED 0
#endif
#ifdef MPE_OS_WINDOWS
#ifdef MPE_BUILDING_DLL
#define MPE_EXPORT __declspec (dllexport)
#else
#define MPE_EXPORT
#endif
#define MPE_IMPORT __declspec (dllimport)
#else
#ifdef MPE_HAS_GNUC_ATTR
#define MPE_EXPORT __attribute__ ((visibility ("default")))
#else
#define MPE_EXPORT
#endif
#define MPE_IMPORT
#endif
/* ------------------------------------------------------------------ */
/* 4. Threading: pthread.h on POSIX + MinGW, shim on MSVC              */
/* ------------------------------------------------------------------ */
#if defined(MPE_OS_WINDOWS) && defined(MPE_COMPILER_MSVC) && !defined(__GNUC__)
/* ---- Minimal MSVC pthread replacement (mutex + cond + once) ---- */
typedef struct {
    CRITICAL_SECTION cs;
} mpe_pthread_mutex_t_shim;
#define pthread_mutex_t mpe_pthread_mutex_t_shim
#define PTHREAD_MUTEX_INITIALIZER                                                                                      \
    { 0 }
#define pthread_mutex_lock(m)                                                                                          \
    do {                                                                                                               \
        static int _init = 0;                                                                                          \
        (void) _init;                                                                                                  \
        EnterCriticalSection (&(m)->cs);                                                                               \
    } while (0)
#define pthread_mutex_unlock(m) LeaveCriticalSection (&(m)->cs)
/* NOTE: static initializers need runtime init; call mpe_platform_init()
 * once at startup on MSVC to InitializeCriticalSection all statics.
 * Simpler robust path: use SRWLOCK-based mpe_mutex_* API below. */
typedef struct {
    CONDITION_VARIABLE cv;
} mpe_pthread_cond_t_shim;
#define pthread_cond_t mpe_pthread_cond_t_shim
#define PTHREAD_COND_INITIALIZER                                                                                       \
    { 0 }
#define pthread_cond_wait(c, m) SleepConditionVariableCS (&(c)->cv, &(m)->cs, INFINITE)
#define pthread_cond_broadcast(c) WakeAllConditionVariable (&(c)->cv)
typedef struct {
    INIT_ONCE once;
} mpe_pthread_once_t_shim;
#define pthread_once_t mpe_pthread_once_t_shim
#define PTHREAD_ONCE_INIT                                                                                              \
    { 0 }
static BOOL CALLBACK mpe_once_cb (PINIT_ONCE o, PVOID p, PVOID *c) {
    (void) o;
    (void) c;
    ((void (*) (void)) p) ();
    return TRUE;
}
#define pthread_once(o, f) InitOnceExecuteOnce (&(o)->once, mpe_once_cb, (PVOID) (f), NULL)
#else
#include <pthread.h>
#endif
/* Portable wrappers usable from new code (both POSIX and Windows). */
typedef pthread_mutex_t mpe_mutex_t;
typedef pthread_cond_t mpe_cond_t;
#define MPE_MUTEX_INITIALIZER PTHREAD_MUTEX_INITIALIZER
#define MPE_COND_INITIALIZER PTHREAD_COND_INITIALIZER
static inline void mpe_mutex_lock (mpe_mutex_t *m) {
    pthread_mutex_lock (m);
}
static inline void mpe_mutex_unlock (mpe_mutex_t *m) {
    pthread_mutex_unlock (m);
}
/* ------------------------------------------------------------------ */
/* 5. Dynamic loading: dlfcn.h on POSIX, Win32 shim on Windows         */
/* ------------------------------------------------------------------ */
#ifndef MPE_OS_WINDOWS
#include <dlfcn.h>
#else
/* ---- dlfcn-win32 compatible shim (MinGW + MSVC) ---- */
#ifndef RTLD_NOW
#define RTLD_NOW 0
#endif
#ifndef RTLD_LOCAL
#define RTLD_LOCAL 0
#endif
#ifndef RTLD_NOLOAD
#define RTLD_NOLOAD 0x04
#endif
#ifndef RTLD_GLOBAL
#define RTLD_GLOBAL 0
#endif
typedef struct {
    HMODULE h;
} *mpe_dl_handle_inner;
#ifndef _MPE_DLFCN_SHIM_DEFINED
#define _MPE_DLFCN_SHIM_DEFINED 1
#include <stdlib.h>
static char mpe_dl_errbuf[1024];
static inline const char *mpe_dl_strerror_win (DWORD e, char *buf, size_t n) {
    DWORD f =
        FormatMessageA (FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, e, 0, buf, (DWORD) n, NULL);
    if (f == 0 || f >= n) {
        snprintf (buf, n, "Win32 error %lu", (unsigned long) e);
    } else {
        /* strip trailing CRLF */
        while (f > 0 && (buf[f - 1] == '\n' || buf[f - 1] == '\r'))
            buf[--f] = '\0';
    }
    return buf;
}
static inline void *mpe_win_dlopen (const char *path, int flags) {
    (void) flags;
    if (!path || !*path)
        return NULL;
    /* RTLD_NOLOAD emulation: only resolve if already loaded. */
    if (flags & RTLD_NOLOAD) {
        HMODULE h = GetModuleHandleA (path);
        if (h)
            return (void *) h;
        /* also try basename probe via loaded-module enumeration fallback:
         * GetModuleHandleA fails for bare filenames not yet loaded — that
         * is the correct NOLOAD failure. */
        SetLastError (ERROR_MOD_NOT_FOUND);
        snprintf (mpe_dl_errbuf, sizeof (mpe_dl_errbuf), "module not loaded: %s", path);
        return NULL;
    }
    HMODULE h = LoadLibraryA (path);
    if (!h) {
        mpe_dl_strerror_win (GetLastError (), mpe_dl_errbuf, sizeof (mpe_dl_errbuf));
    }
    return (void *) h;
}
static inline void *mpe_win_dlsym (void *h, const char *sym) {
    if (!h || !sym)
        return NULL;
    FARPROC p = GetProcAddress ((HMODULE) h, sym);
    if (!p) {
        mpe_dl_strerror_win (GetLastError (), mpe_dl_errbuf, sizeof (mpe_dl_errbuf));
        return NULL;
    }
    return (void *) p;
}
static inline int mpe_win_dlclose (void *h) {
    if (!h)
        return -1;
    return FreeLibrary ((HMODULE) h) ? 0 : -1;
}
static inline const char *mpe_win_dlerror (void) {
    if (!mpe_dl_errbuf[0])
        return NULL;
    /* dlerror() consumes the error (POSIX semantics). */
    static char out[1024];
    snprintf (out, sizeof (out), "%s", mpe_dl_errbuf);
    mpe_dl_errbuf[0] = '\0';
    return out;
}
typedef struct {
    const char *dli_fname;
    void *dli_fbase;
    const char *dli_sname;
    void *dli_saddr;
} Dl_info;
static inline int mpe_win_dladdr (const void *addr, Dl_info *info) {
    HMODULE h = NULL;
    if (!addr || !info)
        return 0;
    if (!GetModuleHandleExA (GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                             (LPCSTR) addr, &h))
        return 0;
    static char fname[MAX_PATH * 2];
    DWORD n = GetModuleFileNameA (h, fname, sizeof (fname));
    if (n == 0 || n >= sizeof (fname))
        return 0;
    info->dli_fname = fname;
    info->dli_fbase = (void *) h;
    info->dli_sname = NULL;
    info->dli_saddr = NULL;
    return 1;
}
#define dlopen mpe_win_dlopen
#define dlsym mpe_win_dlsym
#define dlclose mpe_win_dlclose
#define dlerror mpe_win_dlerror
#define dladdr mpe_win_dladdr
#endif /* _MPE_DLFCN_SHIM_DEFINED */
#endif /* MPE_OS_WINDOWS */
/* ------------------------------------------------------------------ */
/* 6. POSIX file/time/string shims for Windows (Linux: pass-through)   */
/* ------------------------------------------------------------------ */
#ifdef MPE_OS_WINDOWS
#include <errno.h>
#ifndef O_RDONLY
#define O_RDONLY 0
#endif
#ifndef O_WRONLY
#define O_WRONLY 1
#endif
#ifndef O_RDWR
#define O_RDWR 2
#endif
#ifndef O_CREAT
#define O_CREAT 0x0100
#endif
#ifndef O_EXCL
#define O_EXCL 0x0400
#endif
#ifndef O_NONBLOCK
#define O_NONBLOCK 0x4000
#endif
/* Not present in MinGW baselines: map to 0 (flag ignored). */
#ifndef O_NOFOLLOW
#define O_NOFOLLOW 0
#endif
#ifndef O_DIRECTORY
#define O_DIRECTORY 0
#endif
#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif
#ifndef F_OK
#define F_OK 0
#endif
#ifndef R_OK
#define R_OK 4
#endif
#ifndef W_OK
#define W_OK 2
#endif
#ifndef X_OK
#define X_OK 1
#endif
#ifndef STDIN_FILENO
#define STDIN_FILENO 0
#endif
#ifndef STDOUT_FILENO
#define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#define STDERR_FILENO 2
#endif
#ifndef S_IRWXU
#define S_IRWXU 0700
#endif
/* mkdir(path, mode): POSIX takes mode, Win32 _mkdir takes 1 arg. */
static inline int mpe_mkdir (const char *path,
#ifdef _MSC_VER
                             int mode
#else
                             mode_t mode
#endif
) {
    (void) mode;
#ifdef _MSC_VER
    return _mkdir (path);
#else
    return mkdir (path);
#endif
}
/* Keep bare mkdir() calls compiling on MinGW (1-arg) and MSVC. */
#ifdef mkdir
#undef mkdir
#endif
#define mkdir(path, mode) mpe_mkdir ((path), (mode))
/* fsync: flush OS buffers. _commit on MSVC/MinGW. */
static inline int mpe_fsync (int fd) {
#ifdef _MSC_VER
    return _commit (fd);
#else
    /* MinGW provides _commit; fsync may be missing. */
    return _commit (fd);
#endif
}
#ifdef fsync
#undef fsync
#endif
#define fsync(fd) mpe_fsync (fd)
/* access: map to _access. */
static inline int mpe_access (const char *p, int mode) {
    return _access (p, mode);
}
#ifdef access
#undef access
#endif
#define access(p, m) mpe_access ((p), (m))
/* MSVC POSIX names -> underscore variants. MinGW already provides both. */
#ifdef _MSC_VER
#ifndef _O_RDONLY
#define _O_RDONLY 0
#endif
#define O_RDONLY _O_RDONLY
#define O_WRONLY _O_WRONLY
#define O_RDWR _O_RDWR
#define O_CREAT _O_CREAT
#define O_EXCL _O_EXCL
#define O_TRUNC _O_TRUNC
#define O_APPEND _O_APPEND
#define O_BINARY _O_BINARY
#define open _open
#define close _close
#define read _read
#define write _write
#define fileno _fileno
#define fdopen _fdopen
#define unlink _unlink
#define rmdir _rmdir
#define strdup _strdup
#endif
/* getpid / isatty / fileno / fdopen / fchmod / mkstemp / realpath */
static inline int mpe_getpid (void) {
    return (int) _getpid ();
}
#ifdef getpid
#undef getpid
#endif
#define getpid() mpe_getpid ()
static inline int mpe_isatty (int fd) {
    return _isatty (fd);
}
#ifdef isatty
#undef isatty
#endif
#define isatty(fd) mpe_isatty (fd)
static inline int mpe_fchmod (int fd, int mode) {
    (void) fd;
    (void) mode;
    /* Windows has no fchmod; file created with default ACLs. Best effort:
     * mark read-only removal is out of scope — return success. */
    return 0;
}
#ifdef fchmod
#undef fchmod
#endif
#define fchmod(fd, mode) mpe_fchmod ((fd), (mode))
/* mkstemp: Windows _mktemp_s + _open based emulation. Template must end
 * in XXXXXX (same contract as POSIX). Returns open fd, template replaced
 * with actual path. */
static inline int mpe_mkstemp (char *tmpl) {
    if (!tmpl) {
        errno = EINVAL;
        return -1;
    }
    size_t n = strlen (tmpl);
    if (n < 6 || strcmp (tmpl + n - 6, "XXXXXX") != 0) {
        errno = EINVAL;
        return -1;
    }
    /* _mktemp_s replaces XXXXXX with a unique name. */
    errno_t e = _mktemp_s (tmpl, n + 1);
    if (e != 0) {
        errno = EEXIST;
        return -1;
    }
    int fd = -1;
    errno_t oe = _sopen_s (&fd, tmpl, _O_CREAT | _O_EXCL | _O_RDWR | _O_BINARY, _SH_DENYRW, _S_IREAD | _S_IWRITE);
    if (oe != 0 || fd < 0) {
        return -1;
    }
    return fd;
}
#ifdef mkstemp
#undef mkstemp
#endif
#define mkstemp(t) mpe_mkstemp (t)
/* realpath: GetFullPathNameA based. resolved==NULL => malloc'd (POSIX). */
static inline char *mpe_realpath (const char *path, char *resolved) {
    char tmp[PATH_MAX * 2];
    DWORD n;
    int need_free = 0;
    if (!path) {
        errno = EINVAL;
        return NULL;
    }
    if (!resolved) {
        resolved = (char *) malloc (PATH_MAX * 2);
        if (!resolved)
            return NULL;
        need_free = 1;
    }
    n = GetFullPathNameA (path, PATH_MAX * 2, resolved, NULL);
    if (n == 0 || n >= (DWORD) (PATH_MAX * 2)) {
        if (need_free)
            free (resolved);
        errno = ENOENT;
        return NULL;
    }
    /* Normalise separators to '/' for internal comparisons (loader jail). */
    for (char *p = resolved; *p; ++p)
        if (*p == '\\')
            *p = '/';
    (void) tmp;
    return resolved;
}
#ifdef realpath
#undef realpath
#endif
#define realpath(p, r) mpe_realpath ((p), (r))
/* localtime_r: localtime_s on Windows. */
static inline struct tm *mpe_localtime_r (const time_t *t, struct tm *out) {
    if (!t || !out) {
        errno = EINVAL;
        return NULL;
    }
    if (localtime_s (out, t) != 0)
        return NULL;
    return out;
}
#ifdef localtime_r
#undef localtime_r
#endif
#define localtime_r(t, o) mpe_localtime_r ((t), (o))
/* strcasecmp / strncasecmp -> _stricmp / _strnicmp */
static inline int mpe_strcasecmp (const char *a, const char *b) {
    return _stricmp (a, b);
}
static inline int mpe_strncasecmp (const char *a, const char *b, size_t n) {
    return _strnicmp (a, b, n);
}
#ifdef strcasecmp
#undef strcasecmp
#endif
#define strcasecmp(a, b) mpe_strcasecmp ((a), (b))
#ifdef strncasecmp
#undef strncasecmp
#endif
#define strncasecmp(a, b, n) mpe_strncasecmp ((a), (b), (n))
/* strdup always exists on MinGW/MSVC (_strdup); strndup may not. */
static inline char *mpe_strndup (const char *s, size_t n) {
    size_t len = 0;
    char *out;
    if (!s)
        return NULL;
    while (len < n && s[len])
        len++;
    out = (char *) malloc (len + 1);
    if (!out)
        return NULL;
    memcpy (out, s, len);
    out[len] = '\0';
    return out;
}
#ifdef strndup
#undef strndup
#endif
#define strndup(s, n) mpe_strndup ((s), (n))
/* clock_gettime fallback via QueryPerformanceCounter. */
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif
#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif
static inline int mpe_clock_gettime (int clk, struct timespec *ts) {
    static LARGE_INTEGER freq = {0};
    LARGE_INTEGER cnt;
    ULONGLONG ms;
    (void) clk;
    if (!ts) {
        errno = EINVAL;
        return -1;
    }
    if (freq.QuadPart == 0) {
        QueryPerformanceFrequency (&freq);
        if (freq.QuadPart == 0) {
            /* fallback: GetTickCount64 ms resolution */
            ms = GetTickCount64 ();
            ts->tv_sec = (time_t) (ms / 1000ULL);
            ts->tv_nsec = (long) ((ms % 1000ULL) * 1000000ULL);
            return 0;
        }
    }
    QueryPerformanceCounter (&cnt);
    /* QPC epoch is boot, not Unix epoch — fine for MONOTONIC. For REALTIME
     * callers (rare), add Unix-time offset via GetSystemTimeAsFileTime. */
    if (clk == CLOCK_REALTIME) {
        FILETIME ft;
        ULARGE_INTEGER u;
        GetSystemTimeAsFileTime (&ft);
        u.LowPart = ft.dwLowDateTime;
        u.HighPart = ft.dwHighDateTime;
        /* 100ns since 1601-01-01 -> seconds since 1970-01-01 */
        u.QuadPart -= 116444736000000000ULL;
        ts->tv_sec = (time_t) (u.QuadPart / 10000000ULL);
        ts->tv_nsec = (long) ((u.QuadPart % 10000000ULL) * 100LL);
        return 0;
    }
    ts->tv_sec = (time_t) (cnt.QuadPart / freq.QuadPart);
    ts->tv_nsec = (long) (((cnt.QuadPart % freq.QuadPart) * 1000000000LL) / freq.QuadPart);
    return 0;
}
#ifdef clock_gettime
#undef clock_gettime
#endif
#define clock_gettime(c, t) mpe_clock_gettime ((c), (t))
/* open/openat wrappers: O_* extras are 0 on Windows (no-op). openat and
 * renameat/unlinkat have no Win32 equivalent; provide path-join fallback
 * for the single status/ jail use-case. See term_admin.c Windows branch. */
#ifndef AT_FDCWD
#define AT_FDCWD (-100)
#endif
/* getenv HOME fallback: also consult USERPROFILE (native Windows). */
static inline const char *mpe_home_dir (void) {
    const char *h = getenv ("HOME");
    if (h && *h)
        return h;
    h = getenv ("USERPROFILE");
    if (h && *h)
        return h;
    return NULL;
}
#else /* POSIX: nothing to shim (headers come from system) */
#include <unistd.h>
static inline const char *mpe_home_dir (void) {
    const char *h = getenv ("HOME");
    return (h && *h) ? h : NULL;
}
#endif /* MPE_OS_WINDOWS */
/* ------------------------------------------------------------------ */
/* 7. Portable sleep / directory helpers                               */
/* ------------------------------------------------------------------ */
static inline int mpe_mkdir_p (const char *path) {
#ifdef MPE_OS_WINDOWS
    /* Minimal mkdir -p: create each ancestor with _mkdir. */
    char tmp[PATH_MAX * 2];
    size_t n;
    if (!path || !*path)
        return -1;
    snprintf (tmp, sizeof (tmp), "%s", path);
    n = strlen (tmp);
    /* strip trailing slashes */
    while (n > 1 && (tmp[n - 1] == '/' || tmp[n - 1] == '\\'))
        tmp[--n] = '\0';
    for (size_t i = 1; i <= n; i++) {
        if (tmp[i] == '/' || tmp[i] == '\\' || tmp[i] == '\0') {
            char c = tmp[i];
            tmp[i] = '\0';
            if (tmp[0] != '\0') {
                _mkdir (tmp);
            }
            tmp[i] = c;
        }
    }
    return 0;
#else
    (void) path;
    return 0; /* shell mkdir -p used by make/scripts on POSIX */
#endif
}
#endif /* mpe_platform_h */
