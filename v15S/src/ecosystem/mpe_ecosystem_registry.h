#ifndef mpe_ecosystem_registry_h
#define mpe_ecosystem_registry_h
/* Ecosystem registry API (internal) */
#include "mpe_ecosystem.h"

int mpe_ecosystem_register(const mpe_ecosystem_desc_t *desc);
const mpe_ecosystem_desc_t *mpe_ecosystem_find(const char *name);
int mpe_ecosystem_count(void);
const mpe_ecosystem_desc_t *mpe_ecosystem_at(int index);

int mpe_ecosystem_attach(mpe_world_t *world, const char *eco_name);
int mpe_ecosystem_detach(mpe_world_t *world, const char *eco_name);
void mpe_ecosystem_pre_step(mpe_world_t *world, float dt);
void mpe_ecosystem_post_step(mpe_world_t *world, float dt);
void mpe_register_ecosystems(void);

#endif