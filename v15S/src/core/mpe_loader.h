#ifndef mpe_loader_h
#define mpe_loader_h
/* Dynamic .so loader for MPI modules.
 *
 * Contracts (tick-boundary admin):
 *  - load/unload must run at a tick boundary (quiesced via physics_halt
 *    or between steps). Concurrent loader use from step threads is misuse:
 *    rollback on load failure truncates to a pre-dlopen snapshot, which
 *    assumes no other loader transaction is in flight.
 *  - Paths must resolve inside plugins/<name>.so RELATIVE TO THE PROCESS
 *    WORKING DIRECTORY (normally v15S/src). Launches from elsewhere fail
 *    closed with "path must resolve inside...". TOCTOU between realpath
 *    and dlopen is accepted for a local-debug affordance (not a sandbox).
 *  - Unload return codes: 0 ok, -1 unknown handle/bad path, -2 busy
 *    (a live world still references the module: attached tick module,
 *    active stage backend, or pair handler in range). Detach/reset the
 *    world slots first, then retry. */
int mpe_loader_load(const char *path, char *errbuf, int errlen);
int mpe_loader_unload(const char *path_or_name);
int mpe_loader_count(void);
const char *mpe_loader_path_at(int i);
/* Module display name of handle i (from its mpe_module_desc), or NULL. */
const char *mpe_loader_name_at(int i);
/* Resolve an exported symbol from a loaded handle (path, module name, or
 * ecosystem name). RTLD_NOLOAD: never loads, only resolves. NULL when
 * unknown — for terminal-driven plugin APIs (ftc/eco commands). */
void *mpe_loader_symbol(const char *path_or_name, const char *sym);
/* World attachment accounting prevents dlclose while a world can still call
 * a module callback. Static (non-dlopen) descriptors are harmless no-ops.
 * Matched by module NAME (registry copies vs .so originals differ). */
void mpe_loader_retain_module(const void *desc);
void mpe_loader_release_module(const void *desc);
#endif
