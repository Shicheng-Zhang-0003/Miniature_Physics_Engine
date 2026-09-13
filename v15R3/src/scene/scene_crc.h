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

#endif /* scene_crc_h */
