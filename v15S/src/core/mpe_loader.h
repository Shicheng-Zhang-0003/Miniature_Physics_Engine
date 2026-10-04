#ifndef mpe_loader_h
#define mpe_loader_h
/* Dynamic plugin loader for MPI modules (.so on Linux, .dll on Windows).
 *
 * Contracts (tick-boundary admin):
 *  - load/unload must run at a tick boundary (quiesced via physics_halt
 *    or between steps). Concurrent loader use from step threads is misuse:
 *    rollback on load failure truncates to a pre-dlopen snapshot, which
 *    assumes no other loader transaction is in flight.
 *  - Paths must resolve inside plugins/<name>.so (Linux) or
 *    plugins/<name>.dll (Windows) RELATIVE TO THE PROCESS WORKING
 *    DIRECTORY (normally v15S/src). Windows accepts '/' and '\\' and both
 *    extensions (MSYS2 .so + native .dll). Launches from elsewhere fail
 *    closed with "path must resolve inside...". TOCTOU between realpath
 *    and dlopen is accepted for a local-debug affordance (not a sandbox).
 *  - Load return codes: 0 ok (fresh load, or already loaded with an
 *    UNCHANGED file), -1 bad path/jail/dlopen/registry failure, -3 stale
 *    (path already loaded but the file changed on disk — the in-memory
 *    image is old code; unload then load again, or restart the engine).
 *    Never silently runs stale code.
 *  - Unload return codes: 0 ok, -1 unknown handle/bad path, -2 busy
 *    (a live world still references the module: attached tick module,
 *    active stage backend, or pair handler in range). Detach/reset the
 *    world slots first, then retry.
 *  - Thread-safety: the handle table and attachments counter are guarded
 *    by an internal (recursive) loader mutex. The registry is touched only
 *    through its locked public API (never by direct struct access), so the
 *    lock order loader -> registry always holds. */
int mpe_loader_load (const char *path, char *errbuf, int errlen);
/* Unload a loaded handle by path or name.
 *   0  unloaded
 *  -1  unknown handle
 *  -2  busy: a live world still references this image (as a tick module or
 *      as a broadphase/solver stage). Applies to ECOSYSTEM bundles too --
 *      they used to bypass this check and could be dlclose'd while a world
 *      held a descriptor pointer into the image, which segfaulted on the next
 *      step. Detach, then retry.
 *  -3  refused: the registered name does not fit the 128-byte teardown
 *      buffer and truncating it would half-tear-down the registry. */
int mpe_loader_unload (const char *path_or_name);
int mpe_loader_count (void);
const char *mpe_loader_path_at (int i);
/* Module display name of handle i (from its mpe_module_desc), or NULL. */
const char *mpe_loader_name_at (int i);
/* Resolve an exported symbol from a loaded handle (path, module name, or
 * ecosystem name). RTLD_NOLOAD: never loads, only resolves. NULL when
 * unknown — for terminal-driven plugin APIs (ftc/eco commands). */
void *mpe_loader_symbol (const char *path_or_name, const char *sym);
/* World attachment accounting prevents dlclose while a world can still call
 * a module callback. Static (non-dlopen) descriptors are harmless no-ops.
 * Matched by module NAME (registry copies vs .so originals differ). */
void mpe_loader_retain_module (const void *desc);
void mpe_loader_release_module (const void *desc);
/* Stage-detach dispatch (foreign-state leak backstop). Validates the
 * append-only stage_detach pointer (null-check + dladdr image check so a
 * stale .so built against the pre-hook header can never redirect control)
 * and invokes it with the owning world. No-op on NULL/foreign hooks. */
struct physics_world;
void mpe_loader_call_stage_detach (const void *desc, struct physics_world *world);
/* Code-address variant for stage slots that only retain the iface (not the
 * desc): finds the handle whose .so owns `fn` and runs its stage_detach. */
void mpe_loader_call_stage_detach_for_fn (const void *fn, struct physics_world *world);
#endif
