#ifndef mpe_loader_h
#define mpe_loader_h
/* Dynamic .so loader for MPI modules. Tick-boundary safe:
 * load registers immediately; unload/detach only at tick boundary
 * (caller must quiesce via physics_halt or between steps). */
int mpe_loader_load(const char *path, char *errbuf, int errlen);
int mpe_loader_unload(const char *path_or_name);
int mpe_loader_count(void);
const char *mpe_loader_path_at(int i);
/* World attachment accounting prevents dlclose while a world can still call
 * a module callback. Static (non-dlopen) descriptors are harmless no-ops. */
void mpe_loader_retain_module(const void *desc);
void mpe_loader_release_module(const void *desc);
#endif
