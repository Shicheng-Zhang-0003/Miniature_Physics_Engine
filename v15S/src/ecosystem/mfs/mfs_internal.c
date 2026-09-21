#include "mfs_internal.h"
#include <string.h>
#include <stdlib.h>

static struct {
    const mpe_module_desc_t *desc;
    void *state;
    bool attached;
} s_internal_modules[8];
static int s_module_count = 0;

void mfs_internal_registry_init(void) {
    for (int i = 0; i < 8; i++) {
        s_internal_modules[i].desc = 0;
        s_internal_modules[i].state = 0;
        s_internal_modules[i].attached = false;
    }
}

int mfs_internal_module_register(const mpe_module_desc_t *desc) {
    if (!desc || desc->abi != 1 || !desc->name) return -1;
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, desc->name) == 0) {
            return -1;
        }
    }
    for (int i = 0; i < 8; i++) {
        if (!s_internal_modules[i].desc) {
            s_internal_modules[i].desc = desc;
            return i;
        }
    }
    return -1;
}

int mfs_internal_module_attach(const char *name, physics_world *world) {
    if (!name || !world) return -1;
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, name) == 0) {
            if (s_internal_modules[i].attached) return 0;
            if (s_internal_modules[i].desc->attach) {
                void *state = 0;
                int r = s_internal_modules[i].desc->attach((physics_world*)world, &state);
                if (r < 0) return -1;
                s_internal_modules[i].state = state;
                s_internal_modules[i].attached = true;
                return 0;
            }
            return -1;
        }
    }
    return -1;
}

int mfs_internal_module_detach(const char *name, physics_world *world) {
    if (!name || !world) return -1;
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, name) == 0) {
            if (!s_internal_modules[i].attached) return 0;
            if (s_internal_modules[i].desc->detach) {
                s_internal_modules[i].desc->detach((physics_world*)world, s_internal_modules[i].state);
            }
            s_internal_modules[i].state = 0;
            s_internal_modules[i].attached = false;
            return 0;
        }
    }
    return -1;
}

void *mfs_internal_module_state(const char *name) {
    if (!name) return 0;
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && strcmp(s_internal_modules[i].desc->name, name) == 0) {
            return s_internal_modules[i].state;
        }
    }
    return 0;
}

void mfs_internal_modules_pre_step(physics_world *world, float dt) {
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && s_internal_modules[i].attached && s_internal_modules[i].desc->pre_step) {
            s_internal_modules[i].desc->pre_step((physics_world*)world, dt, s_internal_modules[i].state);
        }
    }
}

void mfs_internal_modules_post_step(physics_world *world, float dt) {
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && s_internal_modules[i].attached && s_internal_modules[i].desc->post_step) {
            s_internal_modules[i].desc->post_step((physics_world*)world, dt, s_internal_modules[i].state);
        }
    }
}

void mfs_internal_modules_detach_all(physics_world *world) {
    for (int i = 0; i < 8; i++) {
        if (s_internal_modules[i].desc && s_internal_modules[i].attached) {
            if (s_internal_modules[i].desc->detach) {
                s_internal_modules[i].desc->detach((physics_world*)world, s_internal_modules[i].state);
            }
            s_internal_modules[i].state = 0;
            s_internal_modules[i].attached = false;
        }
    }
}