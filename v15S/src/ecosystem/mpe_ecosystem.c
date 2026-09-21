#include "mpe_ecosystem.h"
/* Canonical headers live in core/ (see mpe_ecosystem.h note). */
#include "../core/mpe_module.h"
#include "../core/mpe_registry.h"
#include "../core/mpe_loader.h"
#include <string.h>
#include <stdio.h>

#define MPE_MAX_ECOSYSTEMS 16
#define MPE_MAX_ATTACHED_ECOSYSTEMS 8

static mpe_ecosystem_desc_t s_ecosystems[16];
static int s_eco_count = 0;

struct attached_eco {
    mpe_ecosystem_desc_t *desc;
    void *state;
};
static struct attached_eco s_attached[8];

int mpe_ecosystem_register(const mpe_ecosystem_desc_t *desc) {
    if (!desc || desc->abi != 1 || !desc->name) return -1;
    for (int i = 0; i < 16; i++)
        if (s_ecosystems[i].name && strcmp(s_ecosystems[i].name, desc->name) == 0) {
            s_ecosystems[i] = *desc; return i;
        }
    for (int i = 0; i < 16; i++)
        if (!s_ecosystems[i].name) { s_ecosystems[i] = *desc; return i; }
    return -1;
}

const mpe_ecosystem_desc_t *mpe_ecosystem_find(const char *name) {
    if (!name) return 0;
    for (int i = 0; i < 16; i++)
        if (s_ecosystems[i].name && strcmp(s_ecosystems[i].name, name) == 0)
            return &s_ecosystems[i];
    return 0;
}

int mpe_ecosystem_count(void) { 
    int c = 0; for (int i = 0; i < 16; i++) if (s_ecosystems[i].name) c++; return c; 
}
const mpe_ecosystem_desc_t *mpe_ecosystem_at(int i) {
    if (i < 0 || i >= 16 || !s_ecosystems[i].name) return 0;
    return &s_ecosystems[i];
}

/* Attach ecosystem to a world */
int mpe_ecosystem_attach(mpe_world_t *world, const char *eco_name) {
    if (!world || !eco_name) return -1;
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && strcmp(s_attached[i].desc->name, eco_name) == 0) return 0;
    }
    for (int i = 0; i < 8; i++) {
        if (!s_attached[i].desc) {
            s_attached[i].desc = (mpe_ecosystem_desc_t*)mpe_ecosystem_find((char*)eco_name);
            if (!s_attached[i].desc) return -1;
            if (s_attached[i].desc->attach) {
                int r = s_attached[i].desc->attach((struct mpe_world*)world, &s_attached[i].state);
                if (r < 0) { s_attached[i].desc = 0; s_attached[i].state = 0; return -1; }
            }
            return 0;
        }
    }
    return -1;
}

int mpe_ecosystem_detach(mpe_world_t *world, const char *eco_name) {
    if (!world || !eco_name) return -1;
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && strcmp(s_attached[i].desc->name, eco_name) == 0) {
            if (s_attached[i].desc->detach)
                s_attached[i].desc->detach((struct mpe_world*)world, s_attached[i].state);
            s_attached[i].desc = 0;
            s_attached[i].state = 0;
            return 0;
        }
    }
    return -1;
}

void mpe_ecosystem_pre_step(mpe_world_t *world, float dt) {
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && s_attached[i].desc->pre_step)
            s_attached[i].desc->pre_step((struct mpe_world*)world, dt, s_attached[i].state);
    }
}

void mpe_ecosystem_post_step(mpe_world_t *world, float dt) {
    for (int i = 0; i < 8; i++) {
        if (s_attached[i].desc && s_attached[i].desc->post_step)
            s_attached[i].desc->post_step((struct mpe_world*)world, dt, s_attached[i].state);
    }
}

/* Register built-in ecosystems */
void mpe_register_ecosystems(void) {
    // Ecosystems loaded as .so files via mpe_loader
}