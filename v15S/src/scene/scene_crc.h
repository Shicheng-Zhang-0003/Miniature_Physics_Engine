/* Scene v2 codec: CRC32-IEEE integrity + explicit little-endian field
 * encoding. v1 files stay native-byte-order (legacy path untouched); v200
 * files decode identically on every platform. Zero heap use. */
#ifndef scene_crc_h
#define scene_crc_h

#include <stdint.h>
#include <stdio.h>

/* CRC32-IEEE (zip/ethernet polynomial, init 0xFFFFFFFF, xor-out). */
uint32_t scene_crc32_update(uint32_t crc, const void *data, unsigned long length);

/* Little-endian scalar/vector codec. Write fns return 0 on I/O error,
 * read fns return 0 on short read. */
int scene_w32(FILE *f, uint32_t *crc, uint32_t v);
int scene_wfloat(FILE *f, uint32_t *crc, float v);
int scene_r32(FILE *f, uint32_t *crc, uint32_t *v);
int scene_rfloat(FILE *f, uint32_t *crc, float *v);

/* FIX-AUDIT-DESPOT: per-tick deterministic state hash for lockstep
 * desync detection. CRC32-IEEE over body pos/vel/orientation/ids in
 * body-index order (float bits fed little-endian; +/-0 canonicalized to
 * +0 so numerically-equal states hash equal). NULL world hashes as 0.
 * Implemented here (not physics_world.c) so headless twins and the GUI
 * share one definition. Declared for worlds in core/physics_world.h. */
struct physics_world;
uint32_t physics_world_hash_state(const struct physics_world *world);

#endif /* scene_crc_h */
