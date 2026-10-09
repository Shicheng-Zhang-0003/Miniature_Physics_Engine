/* mgb.h — 472-MGB public API: S2→MPE bridge core (standalone).
 *
 * What this is: the link/pipe/configure/transform half of the S2 display
 * bridge. It spawns an s2tui child, configures an S2 world through the
 * shell it already has, pulls machine state via `sync` (S2SAVE1 frames),
 * and maps frames into an abstract sink (bodies + bonds in display units).
 * The MPE sink (real bodies/joints) is implemented at copy time into 475;
 * see mgb_sink_mpe_stub.c for the contract.
 *
 * What this is NOT: physics coupling (display-only by construction — the
 * sink receives positions, never forces), the MPE module wrapper (MPI
 * attach/tick comes at copy time), or a second S2 (all chemistry stays in
 * the child).
 *
 * Units in:  Å (positions), e (charge), eV (LJ), fs (velocities, unused).
 * Units out: millimetres of display space (1 A -> MGB_MM_PER_A mm).
 * The scale is a display transform, not physics. It is labeled everywhere
 * the mapped data lands and inverts exactly (double arithmetic, one
 * rounding at the sink boundary if the sink is float).
 */
#ifndef MGB_H
#define MGB_H

#include <stddef.h>

/* Display scale: millimetres per Angstrom. Large on purpose (user call):
 * a 48 A petri dish becomes 48 mm across — graspable, readable. */
#ifndef MGB_MM_PER_A
#define MGB_MM_PER_A 1.0
#endif

/* Hard caps: frames bigger than this are refused, not truncated. */
#define MGB_MAX_ATOMS 100000
#define MGB_MAX_BONDS 200000
#define MGB_MAX_RESTRAINTS 64

/* ---- link: owned s2tui child session ---- */

typedef struct mgb_session mgb_session_t;

/* Spawn `s2tui_path` with piped stdin/stdout, handshake the banner.
 * Returns NULL on any failure (bad path, no banner, timeout). Stderr of
 * the child is captured to a temp file for forensics (never parsed). */
/* Why a spawn failed. Every one of these used to collapse into a bare NULL,
 * so the caller had to GUESS between "no such file", "not executable" and
 * "s2tui never answered" — three unrelated problems, one indistinguishable
 * result. `mgb_spawn_reason` reports the true cause and `*why_errno` the
 * OS errno when one applies (e.g. ENOENT, EACCES, ENOEXEC). */
typedef enum {
    mgb_spawn_ok = 0,
    mgb_spawn_e_no_path,   /* NULL/empty path */
    mgb_spawn_e_pipe,      /* pipe() failed */
    mgb_spawn_e_fork,      /* fork() failed */
    mgb_spawn_e_notfound,  /* no such file/dir at that path */
    mgb_spawn_e_noexec,    /* exec failed: see errno (ENOEXEC, ETXTBSY, ...) */
    mgb_spawn_e_noexecbit, /* file is not executable (no exec permission) */
    mgb_spawn_e_handshake, /* s2tui ran but never sent the expected banner/prompt */
    mgb_spawn_e_alloc      /* out of memory */
} mgb_spawn_status_t;
const char *mgb_spawn_strerror (mgb_spawn_status_t st);
/* Last spawn failure on this thread, for diagnostics after a NULL return.
 * Thread-local, so a concurrent spawn elsewhere cannot overwrite it before
 * the caller reads it. */
mgb_spawn_status_t mgb_spawn_reason (int *why_errno);
mgb_session_t *mgb_spawn (const char *s2tui_path, int timeout_sec);
/* Send one shell line, wait for the next `s2>` prompt. 0 ok, -1 dead. */
int mgb_cmd (mgb_session_t *s, const char *line, int timeout_sec);
/* Snapshot: `sync` to a fresh temp file, return its path (caller frees).
 * NULL on any failure. */
char *mgb_snapshot (mgb_session_t *s, int timeout_sec);
/* Tear down: SIGTERM, grace, SIGKILL, reap. Safe on NULL/dead. */
void mgb_close (mgb_session_t *s);
/* True once any link/parse failure has marked the session dead. */
int mgb_dead (const mgb_session_t *s);

/* ---- frame: strict S2SAVE1 parse ---- */

typedef struct {
    int Z;
    double x, y, z;             /* A */
    double q;                   /* e */
    double lj_eps, lj_sigma;    /* eV, A */
    double vx, vy, vz;          /* A/fs */
} mgb_atom_t;

typedef struct {
    int a, b, order;
    double r0, k;
} mgb_bond_t;

typedef struct {
    double seed, dt, cutoff, dielectric, temp, tau, nu;
    int thermostat;
    int natoms, nbonds, nrestraints;
    mgb_atom_t *atoms;
    mgb_bond_t *bonds;
    /* restraints parsed + counted, coordinates not retained in Phase 0 */
} mgb_frame_t;

/* Parse path strictly. 0 ok, -1 corrupt (nothing half-filled). */
int mgb_frame_parse (const char *path, mgb_frame_t *f);
void mgb_frame_free (mgb_frame_t *f);

/* ---- map: frame -> sink in display units ---- */

typedef struct {
    /* Return 0 ok, nonzero aborts the map (propagated). */
    int (*body)(void *ctx, int idx, int Z,
                double x_mm, double y_mm, double z_mm,
                double r_mm, const float rgb[3]);
    int (*bond)(void *ctx, int a, int b);
    int (*finish)(void *ctx);
} mgb_sink_t;

/* Element display data. 0 ok, -1 unknown Z (mapper uses fallback grey). */
int mgb_element_display (int Z, double *r_mm, float rgb[3]);
/* Map frame through scale into sink. 0 ok, -1 on sink abort / bad frame. */
int mgb_map (const mgb_frame_t *f, const mgb_sink_t *sink, void *ctx,
             double mm_per_a);
/* Exact inverse (for gates): display mm back to A. */
static inline double mgb_unmap (double v_mm, double mm_per_a) {
    return v_mm / mm_per_a;
}

#endif /* MGB_H */
