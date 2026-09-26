#ifndef mfs_internal_h
#define mfs_internal_h
/* MFS Internal Module System
 *
 * Provides a way to register and manage internal modules within the ecosystem
 * without requiring dlopen. Modules are compiled directly into the ecosystem.
 *
 * THREAD SAFETY (MFS_REGISTRY_RACE)
 * Module callbacks (attach/detach/pre_step/post_step) are invoked with the
 * registry mutex RELEASED, so module code may call back into the registry and
 * must not be serialised behind a lock it might hold. That makes two
 * orderings dangerous, and both are now closed:
 *
 *   1. Use-after-free: pre_step snapshots {desc,state} under the lock, then
 *      runs outside it. A concurrent detach could free that state in the
 *      gap. Fixed with a per-slot `in_flight` reference count: a callback
 *      holds a reference for its whole duration, and detach waits for the
 *      count to reach zero before freeing.
 *
 *   2. Double-attach: two threads attaching the same (name,world) both saw
 *      `attached == false` and both ran the module's attach, creating two
 *      robots and orphaning one state. Fixed with a reservation: the first
 *      thread marks the slot in_flight before releasing the lock, the second
 *      blocks on the condvar and re-checks, finding the idempotent case.
 *
 * `detaching` is the third guard: while a detach owns the slot, new
 * pre_step/post_step snapshots skip it, so no new callback can start against
 * a state that is about to be freed.
 */

#include <stdint.h>
#include <stdbool.h>
#include "core/physics_world.h"
#include "core/mpe_module.h"

/* FIX-AUDIT-DESPOT: was 8. A bundle attach consumes 1 primary slot per
 * module plus 1 alias slot per additional world (8 worlds x 2 modules = 16
 * worst case), so 8 slot-exhausted at exactly the ecosystem's own world
 * cap. 16 = MFS_ECO_MAX_WORLDS(8) x 2 aliases. */
#define MFS_MAX_INTERNAL_MODULES 16

/* Register outcomes. Distinct codes let callers tell "already registered"
 * (harmless, skip) from "table full" (fatal) — the pre-audit version
 * returned -1 for both, which hid permanent slot exhaustion. */
#define MFS_REG_OK 0
#define MFS_REG_DUPLICATE (-1)
#define MFS_REG_FULL (-2)
#define MFS_REG_BAD_ARG (-3)
#define MFS_REG_BUSY (-4)
/* FIX-AUDIT-DESPOT: attach used to return MFS_REG_DUPLICATE ("already
 * registered") when the name was NOT registered at all. Distinct code so
 * callers can tell "attach before register" (bug) from "already there". */
#define MFS_REG_NOT_FOUND (-5)

/* Detach outcomes. */
#define MFS_DET_OK 0
#define MFS_DET_NOT_FOUND (-1)
#define MFS_DET_NO_CALLBACK (-2)

/* Unregister outcomes. */
#define MFS_UNREG_OK 0
#define MFS_UNREG_NOT_FOUND (-1)
#define MFS_UNREG_STILL_ATTACHED (-2)

/* Snapshot of a registered module, for callers that want to enumerate
 * registrations without touching state (registry-internal shape). */
typedef struct {
    const mpe_module_desc_t *desc;
    void *state;
    bool attached;
} mfs_internal_module_t;

/* Initialise the internal module registry. Refuses with MFS_REG_BUSY while
 * any module is attached or has a callback in flight (clearing slots
 * underneath live attachments would orphan their state and use-after-free
 * the next pre_step). Returns MFS_REG_OK on success.
 * FIX-AUDIT-DESPOT: was void and cleared unconditionally. */
int mfs_internal_registry_init(void);

/* Register an internal module (compiled directly into ecosystem).
 * Returns MFS_REG_* . A name may be registered only once per descriptor
 * identity; use mfs_internal_module_registered() to make registration
 * idempotent across repeated bundle attaches. */
int mfs_internal_module_register(const mpe_module_desc_t *desc);

/* 1 if name is registered, else 0 (idempotent ensure-register). */
int mfs_internal_module_registered(const char *name);

/* Release a registration (and any alias slots sharing the descriptor).
 * Refuses with MFS_UNREG_STILL_ATTACHED while the module is attached
 * anywhere or has a callback in flight, so slots can no longer leak
 * permanently across attach/detach cycles. */
int mfs_internal_module_unregister(const char *name);

/* Attach an internal module by name. Idempotent: attaching a module that is
 * already attached to the same world returns MFS_REG_OK without re-running
 * the module's attach. */
int mfs_internal_module_attach(const char *name, physics_world *world);

/* Detach an internal module by name (waits for in-flight callbacks). */
int mfs_internal_module_detach(const char *name, physics_world *world);

/* Get internal module state for a specific world. Returns NULL unless that
 * world has a live (attached, not-detaching) attachment — this is the
 * unambiguous accessor and the one callers should prefer. */
void *mfs_internal_module_state_for(const void *world, const char *name);

/* Get state for ANY live attachment of `name`. Returns NULL when no world
 * currently holds the module. The pre-audit version matched on name alone
 * and could return the NULL state of a detached alias slot while a later
 * slot was live; it now only reports live attachments and is therefore
 * ambiguous only in the (documented, unavoidable) multi-world case. */
void *mfs_internal_module_state(const char *name);

/* Run pre_step for all attached internal modules. */
void mfs_internal_modules_pre_step(physics_world *world, float dt);

/* Run post_step for all attached internal modules. */
void mfs_internal_modules_post_step(physics_world *world, float dt);

/* Detach all internal modules attached to `world`. */
void mfs_internal_modules_detach_all(physics_world *world);

/* Test/observability: number of callbacks currently executing outside the
 * lock. Non-zero after a pre_step/post_step/attach/detach returns means a
 * module is still running — used by the concurrency suite to assert that
 * detach really does drain. */
int mfs_internal_modules_inflight(void);

/* Test/observability: live registration slots (primary + aliases). */
int mfs_internal_modules_slot_count(void);

#endif
