#include "mfs_internal.h"
#include "core/mpe_module.h"
#include <string.h>
#include <stdlib.h>
#include <pthread.h>

/* Per-(module, world) slots: the same internal module may attach to
 * several worlds with independent state (fleet body indices are
 * per-world). Detach clears state+attachment but KEEPS the registration
 * (detach must never destroy re-attachability). */
static struct {
    const mpe_module_desc_t *desc;
    void *state;
    bool attached;
    const void *world;
} s_internal_modules[8];
static int s_module_count = 0;
static pthread_mutex_t s_mfs_lock = PTHREAD_MUTEX_INITIALIZER;

void mfs_internal_registry_init(void) {
    pthread_mutex_lock(&s_mfs_lock);
    for (int i = 0; i < 8; i++) {
        s_internal_modules[i].desc = 0;
        s_internal_modules[i].state = 0;
        s_internal_modules[i].attached = false;
        s_internal_modules[i].world = 0;
    }
    s_module_count = 0;
    pthread_mutex_unlock(&s_mfs_lock);
}

/* 1 if name is registered (live slot), else 0. Lets bundles ensure
 * registration idempotently across repeated attaches. */
int mfs_internal_module_registered(const char *name) {
    if (!name) return 0;
    pthread_mutex_lock(&s_mfs_lock);
    int found = 0;
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, name) == 0) {
            found = 1;
            break;
        }
    }
    pthread_mutex_unlock(&s_mfs_lock);
    return found;
}

int mfs_internal_module_register(const mpe_module_desc_t *desc) {
    if (!desc || desc->abi != MPE_MODULE_ABI || !desc->name) return -1;
    pthread_mutex_lock(&s_mfs_lock);
    int rc = -1;
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, desc->name) == 0) {
            rc = -1;
            pthread_mutex_unlock(&s_mfs_lock);
            return rc;
        }
    }
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (!s_internal_modules[i].desc) {
            s_internal_modules[i].desc = desc;
            s_internal_modules[i].attached = false;
            s_internal_modules[i].state = 0;
            s_internal_modules[i].world = 0;
            s_module_count++;
            rc = i;
            break;
        }
    }
    pthread_mutex_unlock(&s_mfs_lock);
    return rc;
}

int mfs_internal_module_attach(const char *name, physics_world *world) {
    if (!name || !world) return -1;
    pthread_mutex_lock(&s_mfs_lock);
    int rc = -1;
    int reg = -1;
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, name) == 0) {
            if (s_internal_modules[i].attached &&
                s_internal_modules[i].world == (const void *)world) {
                rc = 0; /* idempotent re-attach, same world */
                break;
            }
            if (!s_internal_modules[i].attached && reg < 0) reg = i;
        }
    }
    /* Second world? The registration slot is taken; share is wrong
     * (per-world state). Reuse a free slot aliasing the same desc. */
    if (rc == 0) {
        pthread_mutex_unlock(&s_mfs_lock);
        return rc;
    }
    if (reg < 0) {
        for (int i = 0; i < 8; i++) {
            if (!s_internal_modules[i].desc) {
                /* alias slot: find the desc by name first */
                for (int k = 0; k < 8; k++) {
                    if (s_internal_modules[k].desc &&
                        strcmp(s_internal_modules[k].desc->name, name) == 0) {
                        s_internal_modules[i].desc = s_internal_modules[k].desc;
                        reg = i;
                        s_module_count++;
                        break;
                    }
                }
                break;
            }
        }
    }
    if (reg >= 0 && s_internal_modules[reg].desc && s_internal_modules[reg].desc->attach) {
        void *state = 0;
        const mpe_module_desc_t *d = s_internal_modules[reg].desc;
        pthread_mutex_unlock(&s_mfs_lock);
        int r = d->attach((physics_world *)world, &state);
        pthread_mutex_lock(&s_mfs_lock);
        if (r < 0) {
            /* alias slots roll back; original registrations persist */
            int is_alias = 0;
            for (int k = 0; k < 8; k++) {
                if (k != reg && s_internal_modules[k].desc == d) {
                    is_alias = 1;
                    break;
                }
            }
            if (is_alias) {
                s_internal_modules[reg].desc = 0;
                s_module_count--;
            }
            rc = -1;
        } else {
            s_internal_modules[reg].state = state;
            s_internal_modules[reg].attached = true;
            s_internal_modules[reg].world = (const void *)world;
            rc = 0;
        }
    }
    pthread_mutex_unlock(&s_mfs_lock);
    return rc;
}

int mfs_internal_module_detach(const char *name, physics_world *world) {
    if (!name || !world) return -1;
    pthread_mutex_lock(&s_mfs_lock);
    int rc = -1;
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, name) == 0 &&
            (!s_internal_modules[i].attached || s_internal_modules[i].world == (const void *)world)) {
            if (s_internal_modules[i].attached && s_internal_modules[i].desc->detach) {
                const mpe_module_desc_t *d = s_internal_modules[i].desc;
                void *st = s_internal_modules[i].state;
                pthread_mutex_unlock(&s_mfs_lock);
                d->detach((physics_world *)world, st);
                pthread_mutex_lock(&s_mfs_lock);
            }
            s_internal_modules[i].state = 0;
            s_internal_modules[i].attached = false;
            s_internal_modules[i].world = 0;
            /* Registration (desc) SURVIVES detach: re-attach by name
             * keeps working. Only an explicit unregister removes it
             * (no such API: registry lives as long as the process). */
            rc = 0;
            break;
        }
    }
    pthread_mutex_unlock(&s_mfs_lock);
    return rc;
}

void *mfs_internal_module_state(const char *name) {
    if (!name) return 0;
    pthread_mutex_lock(&s_mfs_lock);
    void *out = 0;
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, name) == 0) {
            out = s_internal_modules[i].state;
            break;
        }
    }
    pthread_mutex_unlock(&s_mfs_lock);
    return out;
}

void mfs_internal_modules_pre_step(physics_world *world, float dt) {
    const mpe_module_desc_t *ds[8];
    void *sts[8];
    int n = 0;
    pthread_mutex_lock(&s_mfs_lock);
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && s_internal_modules[i].attached &&
            s_internal_modules[i].world == (const void *)world &&
            s_internal_modules[i].desc->pre_step) {
            ds[n] = s_internal_modules[i].desc;
            sts[n] = s_internal_modules[i].state;
            n++;
        }
    }
    pthread_mutex_unlock(&s_mfs_lock);
    for (int i = 0; i < n; i++) ds[i]->pre_step((physics_world *)world, dt, sts[i]);
}

void mfs_internal_modules_post_step(physics_world *world, float dt) {
    const mpe_module_desc_t *ds[8];
    void *sts[8];
    int n = 0;
    pthread_mutex_lock(&s_mfs_lock);
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && s_internal_modules[i].attached &&
            s_internal_modules[i].world == (const void *)world &&
            s_internal_modules[i].desc->post_step) {
            ds[n] = s_internal_modules[i].desc;
            sts[n] = s_internal_modules[i].state;
            n++;
        }
    }
    pthread_mutex_unlock(&s_mfs_lock);
    for (int i = 0; i < n; i++) ds[i]->post_step((physics_world *)world, dt, sts[i]);
}

void mfs_internal_modules_detach_all(physics_world *world) {
    for (int i = 0; i < MFS_MAX_INTERNAL_MODULES; i++) {
        const mpe_module_desc_t *d = 0;
        void *st = 0;
        int do_it = 0;
        pthread_mutex_lock(&s_mfs_lock);
        if (s_internal_modules[i].desc && s_internal_modules[i].attached &&
            s_internal_modules[i].world == (const void *)world) {
            d = s_internal_modules[i].desc;
            st = s_internal_modules[i].state;
            s_internal_modules[i].state = 0;
            s_internal_modules[i].attached = false;
            s_internal_modules[i].world = 0;
            do_it = 1;
        }
        pthread_mutex_unlock(&s_mfs_lock);
        if (do_it && d && d->detach) d->detach((physics_world *)world, st);
    }
}
