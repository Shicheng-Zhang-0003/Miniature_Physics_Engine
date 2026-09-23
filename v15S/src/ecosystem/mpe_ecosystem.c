#include "mpe_ecosystem.h"
/* Canonical headers live in core/ (see mpe_ecosystem.h note). */
#include "../core/mpe_module.h"
#include "../core/mpe_registry.h"
#include "../core/mpe_loader.h"
#include <string.h>
#include <stdio.h>
#include <pthread.h>

#define MPE_MAX_ECOSYSTEMS 16
#define MPE_MAX_ATTACHED_ECOSYSTEMS 8

static mpe_ecosystem_desc_t s_ecosystems[16];
static int s_eco_live[16];
static pthread_mutex_t s_eco_lock = PTHREAD_MUTEX_INITIALIZER;

struct attached_eco {
    mpe_ecosystem_desc_t *desc;
    void *state;
    const void *world; /* per-world attachment (was process-global) */
};
static struct attached_eco s_attached[8];

int mpe_ecosystem_register(const mpe_ecosystem_desc_t *desc) {
    if (!desc || desc->abi != MPE_ECOSYSTEM_ABI || !desc->name) return -1;
    pthread_mutex_lock(&s_eco_lock);
    int rc = -1;
    for (int i = 0; i < 16; i++) {
        if (s_eco_live[i] && s_ecosystems[i].name && strcmp(s_ecosystems[i].name, desc->name) == 0) {
            s_ecosystems[i] = *desc;
            rc = i;
            pthread_mutex_unlock(&s_eco_lock);
            return rc;
        }
    }
    for (int i = 0; i < 16; i++) {
        if (!s_eco_live[i]) {
            s_ecosystems[i] = *desc;
            s_eco_live[i] = 1;
            rc = i;
            break;
        }
    }
    pthread_mutex_unlock(&s_eco_lock);
    return rc;
}

/* Unregister + detach everywhere first (interior pointers in s_attached
 * must not survive the removal). */
int mpe_ecosystem_unregister(const char *name) {
    if (!name) return -1;
    mpe_ecosystem_detach_everywhere(name);
    pthread_mutex_lock(&s_eco_lock);
    int rc = -1;
    for (int i = 0; i < 16; i++) {
        if (s_eco_live[i] && s_ecosystems[i].name && strcmp(s_ecosystems[i].name, name) == 0) {
            s_eco_live[i] = 0;
            s_ecosystems[i].name = 0;
            rc = 0;
            break;
        }
    }
    pthread_mutex_unlock(&s_eco_lock);
    return rc;
}

const mpe_ecosystem_desc_t *mpe_ecosystem_find(const char *name) {
    if (!name) return 0;
    pthread_mutex_lock(&s_eco_lock);
    const mpe_ecosystem_desc_t *out = 0;
    for (int i = 0; i < 16; i++)
        if (s_eco_live[i] && s_ecosystems[i].name && strcmp(s_ecosystems[i].name, name) == 0) {
            out = &s_ecosystems[i];
            break;
        }
    pthread_mutex_unlock(&s_eco_lock);
    return out;
}

int mpe_ecosystem_count(void) {
    pthread_mutex_lock(&s_eco_lock);
    int c = 0;
    for (int i = 0; i < 16; i++) if (s_eco_live[i]) c++;
    pthread_mutex_unlock(&s_eco_lock);
    return c;
}
const mpe_ecosystem_desc_t *mpe_ecosystem_at(int i) {
    pthread_mutex_lock(&s_eco_lock);
    const mpe_ecosystem_desc_t *out = 0;
    int seen = -1;
    for (int k = 0; k < 16; k++) {
        if (!s_eco_live[k]) continue;
        if (++seen == i) { out = &s_ecosystems[k]; break; }
    }
    pthread_mutex_unlock(&s_eco_lock);
    return out;
}

/* Attach ecosystem to a world (per-world: the same ecosystem may attach
 * to several worlds with independent state). */
int mpe_ecosystem_attach(mpe_world_t *world, const char *eco_name) {
    if (!world || !eco_name) return -1;
    pthread_mutex_lock(&s_eco_lock);
    int rc = -1;
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && s_attached[i].world == (const void *)world &&
            strcmp(s_attached[i].desc->name, eco_name) == 0) {
            rc = 0;
            pthread_mutex_unlock(&s_eco_lock);
            return rc;
        }
    }
    for (int i = 0; i < 8; i++) {
        if (!s_attached[i].desc) {
            const mpe_ecosystem_desc_t *found = NULL;
            for (int k = 0; k < 16; k++) {
                if (s_eco_live[k] && s_ecosystems[k].name &&
                    strcmp(s_ecosystems[k].name, eco_name) == 0) {
                    found = &s_ecosystems[k];
                    break;
                }
            }
            if (!found) {
                pthread_mutex_unlock(&s_eco_lock);
                return -1;
            }
            s_attached[i].desc = (mpe_ecosystem_desc_t *)found;
            s_attached[i].world = (const void *)world;
            s_attached[i].state = 0;
            mpe_ecosystem_desc_t *d = s_attached[i].desc;
            pthread_mutex_unlock(&s_eco_lock);
            if (d->attach) {
                void *st = 0;
                int r = d->attach((mpe_world_t *)world, &st);
                pthread_mutex_lock(&s_eco_lock);
                if (r < 0) {
                    s_attached[i].desc = 0;
                    s_attached[i].state = 0;
                    s_attached[i].world = 0;
                    pthread_mutex_unlock(&s_eco_lock);
                    return -1;
                }
                s_attached[i].state = st;
                pthread_mutex_unlock(&s_eco_lock);
            }
            return 0;
        }
    }
    pthread_mutex_unlock(&s_eco_lock);
    return -1;
}

int mpe_ecosystem_detach(mpe_world_t *world, const char *eco_name) {
    if (!world || !eco_name) return -1;
    pthread_mutex_lock(&s_eco_lock);
    int rc = -1;
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && s_attached[i].world == (const void *)world &&
            strcmp(s_attached[i].desc->name, eco_name) == 0) {
            mpe_ecosystem_desc_t *d = s_attached[i].desc;
            void *st = s_attached[i].state;
            s_attached[i].desc = 0;
            s_attached[i].state = 0;
            s_attached[i].world = 0;
            pthread_mutex_unlock(&s_eco_lock);
            if (d->detach) d->detach((mpe_world_t *)world, st);
            return 0;
        }
    }
    pthread_mutex_unlock(&s_eco_lock);
    return rc;
}

/* Per-world state lookup for terminal-driven commands (eco command/
 * config forward to these states on the primary world). */
void *mpe_ecosystem_state(mpe_world_t *world, const char *eco_name) {
    if (!world || !eco_name) return NULL;
    pthread_mutex_lock(&s_eco_lock);
    void *out = NULL;
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && s_attached[i].world == (const void *)world &&
            strcmp(s_attached[i].desc->name, eco_name) == 0) {
            out = s_attached[i].state;
            break;
        }
    }
    pthread_mutex_unlock(&s_eco_lock);
    return out;
}

/* Detach everywhere (unload path): hooks run while the .so is mapped. */
void mpe_ecosystem_detach_everywhere(const char *eco_name) {
    if (!eco_name) return;
    for (;;) {
        pthread_mutex_lock(&s_eco_lock);
        int idx = -1;
        for (int i = 0; i < 8; i++) {
            if (s_attached[i].desc && strcmp(s_attached[i].desc->name, eco_name) == 0) {
                idx = i;
                break;
            }
        }
        if (idx < 0) {
            pthread_mutex_unlock(&s_eco_lock);
            return;
        }
        mpe_ecosystem_desc_t *d = s_attached[idx].desc;
        void *st = s_attached[idx].state;
        void *w = (void *)s_attached[idx].world;
        s_attached[idx].desc = 0;
        s_attached[idx].state = 0;
        s_attached[idx].world = 0;
        pthread_mutex_unlock(&s_eco_lock);
        if (d->detach) d->detach((mpe_world_t *)w, st);
    }
}

void mpe_ecosystem_pre_step(mpe_world_t *world, float dt) {
    mpe_ecosystem_desc_t *ds[8];
    void *sts[8];
    int n = 0;
    pthread_mutex_lock(&s_eco_lock);
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && s_attached[i].world == (const void *)world &&
            s_attached[i].desc->pre_step) {
            ds[n] = s_attached[i].desc;
            sts[n] = s_attached[i].state;
            n++;
        }
    }
    pthread_mutex_unlock(&s_eco_lock);
    for (int i = 0; i < n; i++) ds[i]->pre_step((mpe_world_t *)world, dt, sts[i]);
}

void mpe_ecosystem_post_step(mpe_world_t *world, float dt) {
    mpe_ecosystem_desc_t *ds[8];
    void *sts[8];
    int n = 0;
    pthread_mutex_lock(&s_eco_lock);
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && s_attached[i].world == (const void *)world &&
            s_attached[i].desc->post_step) {
            ds[n] = s_attached[i].desc;
            sts[n] = s_attached[i].state;
            n++;
        }
    }
    pthread_mutex_unlock(&s_eco_lock);
    for (int i = 0; i < n; i++) ds[i]->post_step((mpe_world_t *)world, dt, sts[i]);
}

/* Register built-in ecosystems */
void mpe_register_ecosystems(void) {
    // Ecosystems loaded as .so files via mpe_loader
}