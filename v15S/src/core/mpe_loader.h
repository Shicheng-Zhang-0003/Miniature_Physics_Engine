#ifndef mpe_loader_h
#define mpe_loader_h
/* Dynamic .so loader for MPI modules. Tick-boundary safe:
 * load registers immediately; unload/detach only at tick boundary
 * (caller must quiesce via physics_halt or between steps). */
int mpe_loader_load(const char *path, char *errbuf, int errlen);
int mpe_loader_unload(const char *path_or_name);
int mpe_loader_count(void);
const char *mpe_loader_path_at(int i);
#endif
