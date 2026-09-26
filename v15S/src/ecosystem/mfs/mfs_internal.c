/* MFS internal module registry.
 *
 * Concurrency contract: see mfs_internal.h. Callbacks run OUTSIDE the
 * registry mutex (module code may re-enter the registry, and holding a lock
 * across a callback risks lock-order inversion with the host). Three
 * per-slot fields make that safe:
 *
 *   in_flight  reference count of callbacks currently executing. detach()
 *              blocks until it is 0 before freeing state, which closes the
 *              snapshot-then-invoke use-after-free.
 *   detaching  set while a detach owns the slot. pre_step/post_step skip
 *              detaching slots, so once a detach decides to wait, no new
 *              callback can appear and the wait always terminates.
 *   attached   published only after the module's attach() returns, so a
 *              concurrent attach either sees "not attached" and waits on the
 *              reservation, or sees the completed attachment and takes the
 *              idempotent path. Exactly one attach() ever runs per slot.
 */
#include "mfs_internal.h"
#include "core/mpe_module.h"
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

typedef struct {
    const mpe_module_desc_t *desc;
    void *state;
    bool attached;
    const void *world;
    unsigned in_flight; /* callbacks running outside the lock */
    bool detaching;     /* detach owns the slot; no new callbacks */
} mfs_slot;

static mfs_slot s_slots[MFS_MAX_INTERNAL_MODULES];
static int s_slot_count = 0;   /* live slots (primary + aliases) */
static int s_inflight = 0;     /* global sum of in_flight, for tests/telemetry */
static pthread_mutex_t s_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_cv = PTHREAD_COND_INITIALIZER;

static int name_matches(const mfs_slot *s, const char *name) {
    return s->desc && s->desc->name && strcmp(s->desc->name, name) == 0;
}

static void slot_clear(mfs_slot *s) {
    s->desc = NULL;
    s->state = NULL;
    s->attached = false;
    s->world = NULL;
    s->in_flight = 0;
    s->detaching = false;
}

static void wake_all(void) { pthread_cond_broadcast(&s_cv); }

int mfs_internal_registry_init(void) {
    pthread_mutex_lock(&s_lock);
    /* FIX-AUDIT-DESPOT: refuse to clear under live attachments instead of
     * orphaning them (old code was void + unconditional). */
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (s_slots[i].desc &&
            (s_slots[i].attached || s_slots[i].detaching || s_slots[i].in_flight > 0)) {
            pthread_mutex_unlock(&s_lock);
            return MFS_REG_BUSY;
        }
    }
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        slot_clear(&s_slots[i]);
    }
    s_slot_count = 0;
    s_inflight = 0;
    pthread_mutex_unlock(&s_lock);
    return MFS_REG_OK;
}

int mfs_internal_module_registered(const char *name) {
    if (!name) return 0;
    pthread_mutex_lock(&s_lock);
    int found = 0;
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (name_matches(&s_slots[i], name)) {
            found = 1;
            break;
        }
    }
    pthread_mutex_unlock(&s_lock);
    return found;
}

int mfs_internal_module_register(const mpe_module_desc_t *desc) {
    if (!desc || !desc->name || desc->abi != MPE_MODULE_ABI) return MFS_REG_BAD_ARG;
    pthread_mutex_lock(&s_lock);
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (name_matches(&s_slots[i], desc->name)) {
            pthread_mutex_unlock(&s_lock);
            return MFS_REG_DUPLICATE;
        }
    }
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (!s_slots[i].desc) {
            slot_clear(&s_slots[i]);
            s_slots[i].desc = desc;
            s_slot_count++;
            pthread_mutex_unlock(&s_lock);
            return MFS_REG_OK;
        }
    }
    pthread_mutex_unlock(&s_lock);
    return MFS_REG_FULL;
}

int mfs_internal_module_unregister(const char *name) {
    if (!name) return MFS_UNREG_NOT_FOUND;
    pthread_mutex_lock(&s_lock);
    int live = 0;
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (!name_matches(&s_slots[i], name)) continue;
        if (s_slots[i].attached || s_slots[i].detaching || s_slots[i].in_flight > 0) {
            live = 1;
            break;
        }
    }
    if (live) {
        pthread_mutex_unlock(&s_lock);
        return MFS_UNREG_STILL_ATTACHED;
    }
    int freed = 0;
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (name_matches(&s_slots[i], name)) {
            slot_clear(&s_slots[i]);
            s_slot_count--;
            freed++;
        }
    }
    wake_all();
    pthread_mutex_unlock(&s_lock);
    return freed > 0 ? MFS_UNREG_OK : MFS_UNREG_NOT_FOUND;
}

int mfs_internal_module_attach(const char *name, physics_world *world) {
    if (!name || !world) return MFS_REG_BAD_ARG;
    pthread_mutex_lock(&s_lock);
    for (;;) {
        /* Idempotent re-attach of the same world: never re-run attach(). */
        for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
            if (name_matches(&s_slots[i], name) && s_slots[i].attached &&
                s_slots[i].world == (const void *)world) {
                pthread_mutex_unlock(&s_lock);
                return MFS_REG_OK;
            }
        }

        /* Prefer an existing idle slot for this name (primary registration
         * or a previously used alias). */
        int reg = -1;
        for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
            if (name_matches(&s_slots[i], name) && !s_slots[i].attached &&
                !s_slots[i].detaching && s_slots[i].in_flight == 0) {
                reg = i;
                break;
            }
        }

        int made_alias = 0;
        if (reg < 0) {
            /* No idle slot: either every matching slot is busy (wait for it
             * to settle, then re-evaluate) or the table is full. */
            int busy_match = 0;
            const mpe_module_desc_t *proto = NULL;
            for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
                if (name_matches(&s_slots[i], name)) {
                    if (s_slots[i].detaching || s_slots[i].in_flight > 0) busy_match = 1;
                    if (!proto) proto = s_slots[i].desc;
                }
            }
            if (busy_match) {
                /* A peer is mid-attach/mid-detach on this name. Its own
                 * wait is bounded, so this wait cannot deadlock. */
                pthread_cond_wait(&s_cv, &s_lock);
                continue;
            }
            if (!proto) {
                pthread_mutex_unlock(&s_lock);
                /* FIX-AUDIT-DESPOT: was MFS_REG_DUPLICATE ("already
                 * registered") for the NOT-registered case. */
                return MFS_REG_NOT_FOUND; /* not registered */
            }
            int free_slot = -1;
            for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
                if (!s_slots[i].desc) {
                    free_slot = i;
                    break;
                }
            }
            if (free_slot < 0) {
                pthread_mutex_unlock(&s_lock);
                return MFS_REG_FULL;
            }
            /* Claim the alias slot immediately: publishing desc under the
             * lock is what reserves it against a racing attach. */
            slot_clear(&s_slots[free_slot]);
            s_slots[free_slot].desc = proto;
            s_slot_count++;
            reg = free_slot;
            made_alias = 1;
        }

        const mpe_module_desc_t *d = s_slots[reg].desc;
        if (!d || !d->attach) {
            if (made_alias) {
                slot_clear(&s_slots[reg]);
                s_slot_count--;
            }
            pthread_mutex_unlock(&s_lock);
            return MFS_REG_BAD_ARG;
        }

        /* Reserve: the in_flight bump both blocks a second attach on this
         * slot and counts as the reference held across the callback. */
        s_slots[reg].in_flight++;
        s_inflight++;
        pthread_mutex_unlock(&s_lock);

        void *state = NULL;
        int r = d->attach((physics_world *)world, &state);

        pthread_mutex_lock(&s_lock);
        s_slots[reg].in_flight--;
        s_inflight--;
        if (r != 0) {
            /* Alias slots roll back so a failed attach cannot leak a
             * registration; the primary registration persists. */
            if (made_alias) {
                slot_clear(&s_slots[reg]);
                s_slot_count--;
            }
            wake_all();
            pthread_mutex_unlock(&s_lock);
            return (r < 0) ? r : MFS_REG_BAD_ARG;
        }
        s_slots[reg].state = state;
        s_slots[reg].attached = true;
        s_slots[reg].world = (const void *)world;
        wake_all();
        pthread_mutex_unlock(&s_lock);
        return MFS_REG_OK;
    }
}

int mfs_internal_module_detach(const char *name, physics_world *world) {
    if (!name || !world) return MFS_DET_NOT_FOUND;
    pthread_mutex_lock(&s_lock);
    for (;;) {
        int reg = -1;
        for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
            if (name_matches(&s_slots[i], name) && s_slots[i].attached &&
                s_slots[i].world == (const void *)world) {
                reg = i;
                break;
            }
        }
        if (reg < 0) {
            /* Idempotent detach: no live (name,world) attachment, so there
             * is nothing to run. OK when the name is known at all (already
             * detached, or never attached to this world), NOT_FOUND only
             * when the name is not registered.
             * FIX-AUDIT-DESPOT: was a name-only `!attached` check, which
             * returned NOT_FOUND for a known-but-elsewhere-attached name
             * and OK for a name with any idle slot regardless of world. */
            for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
                if (name_matches(&s_slots[i], name)) {
                    pthread_mutex_unlock(&s_lock);
                    return MFS_DET_OK;
                }
            }
            pthread_mutex_unlock(&s_lock);
            return MFS_DET_NOT_FOUND;
        }
        /* Drain: block until no callback holds a reference to this state. */
        if (s_slots[reg].in_flight > 0) {
            pthread_cond_wait(&s_cv, &s_lock);
            continue;
        }
        const mpe_module_desc_t *d = s_slots[reg].desc;
        void *st = s_slots[reg].state;
        s_slots[reg].detaching = true; /* no new callbacks from here on */
        if (!d || !d->detach) {
            s_slots[reg].attached = false;
            s_slots[reg].state = NULL;
            s_slots[reg].world = NULL;
            s_slots[reg].detaching = false;
            wake_all();
            pthread_mutex_unlock(&s_lock);
            return MFS_DET_NO_CALLBACK;
        }
        s_slots[reg].in_flight++;
        s_inflight++;
        pthread_mutex_unlock(&s_lock);

        d->detach((physics_world *)world, st);

        pthread_mutex_lock(&s_lock);
        s_slots[reg].in_flight--;
        s_inflight--;
        s_slots[reg].state = NULL;
        s_slots[reg].attached = false;
        s_slots[reg].world = NULL;
        s_slots[reg].detaching = false;
        /* Registration (desc) SURVIVES detach: re-attach by name keeps
         * working. mfs_internal_module_unregister() reclaims slots. */
        wake_all();
        pthread_mutex_unlock(&s_lock);
        return MFS_DET_OK;
    }
}

void *mfs_internal_module_state_for(const void *world, const char *name) {
    if (!world || !name) return NULL;
    pthread_mutex_lock(&s_lock);
    void *out = NULL;
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (name_matches(&s_slots[i], name) && s_slots[i].attached && !s_slots[i].detaching &&
            s_slots[i].world == world) {
            out = s_slots[i].state;
            break;
        }
    }
    pthread_mutex_unlock(&s_lock);
    return out;
}

void *mfs_internal_module_state(const char *name) {
    if (!name) return NULL;
    pthread_mutex_lock(&s_lock);
    void *out = NULL;
    /* Only live attachments are reported. Matching on name alone (the
     * pre-audit behaviour) could return the NULL state of a detached alias
     * slot while a later slot for the same module was live. */
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (name_matches(&s_slots[i], name) && s_slots[i].attached && !s_slots[i].detaching) {
            out = s_slots[i].state;
            break;
        }
    }
    pthread_mutex_unlock(&s_lock);
    return out;
}

/* Shared dispatch for pre_step/post_step. The snapshot is taken under the
 * lock WITH a reference per slot, so a concurrent detach drains instead of
 * freeing state we are about to call. */
static void mfs_dispatch(physics_world *world, float dt, bool pre) {
    if (!world) return;
    const mpe_module_desc_t *ds[MFS_MAX_INTERNAL_MODULES];
    void *sts[MFS_MAX_INTERNAL_MODULES];
    int slot_of[MFS_MAX_INTERNAL_MODULES];
    int n = 0;
    pthread_mutex_lock(&s_lock);
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES && n < MFS_MAX_INTERNAL_MODULES; i++) {
        if (!s_slots[i].desc || !s_slots[i].attached || s_slots[i].detaching) continue;
        if (s_slots[i].world != (const void *)world) continue;
        if (!(pre ? s_slots[i].desc->pre_step : s_slots[i].desc->post_step)) continue;
        ds[n] = s_slots[i].desc;
        sts[n] = s_slots[i].state;
        slot_of[n] = i;
        n++;
        s_slots[i].in_flight++;
        s_inflight++;
    }
    pthread_mutex_unlock(&s_lock);

    for (int k = 0; k < n; k++) {
        if (pre) {
            ds[k]->pre_step((physics_world *)world, dt, sts[k]);
        } else {
            ds[k]->post_step((physics_world *)world, dt, sts[k]);
        }
        pthread_mutex_lock(&s_lock);
        int i = slot_of[k];
        /* The slot cannot have been detached or unregistered while we held
         * a reference, but match on state anyway so a logic error degrades
         * to a no-op instead of corrupting the counter. */
        if (s_slots[i].state == sts[k] && s_slots[i].in_flight > 0) {
            s_slots[i].in_flight--;
            s_inflight--;
        }
        wake_all();
        pthread_mutex_unlock(&s_lock);
    }
}

void mfs_internal_modules_pre_step(physics_world *world, float dt) {
    mfs_dispatch(world, dt, true);
}

void mfs_internal_modules_post_step(physics_world *world, float dt) {
    mfs_dispatch(world, dt, false);
}

void mfs_internal_modules_detach_all(physics_world *world) {
    if (!world) return;
    for (;;) {
        pthread_mutex_lock(&s_lock);
        int reg = -1;
        for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
            if (s_slots[i].desc && s_slots[i].attached && s_slots[i].world == (const void *)world) {
                reg = i;
                break;
            }
        }
        if (reg < 0) {
            pthread_mutex_unlock(&s_lock);
            return;
        }
        if (s_slots[reg].in_flight > 0) {
            pthread_cond_wait(&s_cv, &s_lock);
            continue;
        }
        const mpe_module_desc_t *d = s_slots[reg].desc;
        void *st = s_slots[reg].state;
        s_slots[reg].detaching = true;
        if (d && d->detach) {
            s_slots[reg].in_flight++;
            s_inflight++;
            pthread_mutex_unlock(&s_lock);
            d->detach((physics_world *)world, st);
            pthread_mutex_lock(&s_lock);
            s_slots[reg].in_flight--;
            s_inflight--;
        }
        s_slots[reg].state = NULL;
        s_slots[reg].attached = false;
        s_slots[reg].world = NULL;
        s_slots[reg].detaching = false;
        wake_all();
        pthread_mutex_unlock(&s_lock);
    }
}

int mfs_internal_modules_inflight(void) {
    pthread_mutex_lock(&s_lock);
    int n = s_inflight;
    pthread_mutex_unlock(&s_lock);
    return n;
}

int mfs_internal_modules_slot_count(void) {
    pthread_mutex_lock(&s_lock);
    int n = s_slot_count;
    pthread_mutex_unlock(&s_lock);
    return n;
}
