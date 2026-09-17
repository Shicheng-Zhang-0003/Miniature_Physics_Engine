/* Solver islands (contact-graph connected components).
 *
 * Bodies joined by a broadphase pair or an active revolute joint belong to
 * the same island. The iteration loop skips islands whose every dynamic
 * member sleeps: their solves are exact no-ops (zero velocity, zeroed
 * inverses), so skipping changes nothing but time.
 *
 * Determinism: unions run in pair order, island ids are assigned in
 * first-seen body order, and skips never reorder solves. Twin runs agree
 * bit-for-bit (see determinism test).
 *
 * Memory: three static scratch arrays sized by mpe_max_bodies
 * (64 KB + 64 KB + 16 KB). No per-tick allocation, ever.
 * Fail-open: any query against a foreign base pointer (or before a build)
 * reports awake, i.e. exactly the old solve-everything behaviour.
 */
#ifndef islands_h
#define islands_h

#include "../core/rigidbody.h"
#include "broadphase.h"
#include <stdbool.h>

struct physics_world;
void islands_build(struct physics_world *world, broadphase_pair *pairs, int pair_count);
int islands_count(const struct physics_world *world);
/* Island id of a member body, or -1 for foreign pointers. */
int islands_body_island(struct physics_world *world, rigidbody *body);
/* False only when the body's island exists and every dynamic member sleeps.
 * Foreign pointers (floor proxy, stale) report true: never skip those. */
bool islands_body_awake(struct physics_world *world, rigidbody *body);

#endif /* islands_h */
