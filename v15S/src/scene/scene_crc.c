/* Scene v2 codec implementation. See scene_crc.h. */
#include "scene_crc.h"
#include <string.h>

static uint32_t crc_table[256];
static int crc_table_ready = 0;

static void crc_make_table(void) {
    for (uint32_t n = 0; n < 256; n++) {
        uint32_t c = n;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        crc_table[n] = c;
    }
    crc_table_ready = 1;
}

uint32_t scene_crc32_update(uint32_t crc, const void *data, unsigned long length) {
    if (!crc_table_ready) {
        crc_make_table();
    }
    const unsigned char *bytes = (const unsigned char *) data;
    for (unsigned long i = 0; i < length; i++) {
        crc = crc_table[(crc ^ bytes[i]) & 0xFFu] ^ (crc >> 8);
    }
    return crc;
}

static void encode_le32(unsigned char out[4], uint32_t v) {
    out[0] = (unsigned char) (v & 0xFFu);
    out[1] = (unsigned char) ((v >> 8) & 0xFFu);
    out[2] = (unsigned char) ((v >> 16) & 0xFFu);
    out[3] = (unsigned char) ((v >> 24) & 0xFFu);
}

static uint32_t decode_le32(const unsigned char in[4]) {
    return ((uint32_t) in[0]) | (((uint32_t) in[1]) << 8) | (((uint32_t) in[2]) << 16) |
           (((uint32_t) in[3]) << 24);
}

int scene_w32(FILE *f, uint32_t *crc, uint32_t v) {
    unsigned char buf[4];
    encode_le32(buf, v);
    if (fwrite(buf, 1, 4, f) != 4) {
        return 0;
    }
    if (crc) {
        *crc = scene_crc32_update(*crc, buf, 4);
    }
    return 1;
}

int scene_wfloat(FILE *f, uint32_t *crc, float v) {
    union {
        float f;
        uint32_t u;
    } converter;
    converter.f = v;
    return scene_w32(f, crc, converter.u);
}

int scene_r32(FILE *f, uint32_t *crc, uint32_t *v) {
    unsigned char buf[4];
    if (fread(buf, 1, 4, f) != 4) {
        return 0;
    }
    if (crc) {
        *crc = scene_crc32_update(*crc, buf, 4);
    }
    *v = decode_le32(buf);
    return 1;
}

int scene_rfloat(FILE *f, uint32_t *crc, float *v) {
    uint32_t u = 0;
    union {
        float f;
        uint32_t u;
    } converter;
    if (!scene_r32(f, crc, &u)) {
        return 0;
    }
    converter.u = u;
    *v = converter.f;
    return 1;
}

/* FIX-AUDIT-DESPOT: lockstep state hash (see header). Deferred include of
 * physics_world.h keeps scene_crc.h light; the hash feeds CRC32 over LE
 * bytes so it is bit-identical on every LE host (v200's LE contract).
 * +0/-0 canonicalized: bitwise twins must not desync on sign-of-zero. */
#include "../core/physics_world.h"

static uint32_t hash_u32_le(uint32_t crc, uint32_t v) {
    unsigned char b[4];
    b[0] = (unsigned char)(v & 0xFFu);
    b[1] = (unsigned char)((v >> 8) & 0xFFu);
    b[2] = (unsigned char)((v >> 16) & 0xFFu);
    b[3] = (unsigned char)((v >> 24) & 0xFFu);
    return scene_crc32_update(crc, b, 4);
}

static uint32_t hash_float_le(uint32_t crc, float f) {
    uint32_t u = 0;
    memcpy(&u, &f, sizeof(u));
    if (f == 0.0f) {
        u = 0u; /* canonicalize +/-0 */
    }
    return hash_u32_le(crc, u);
}

uint32_t physics_world_hash_state(const struct physics_world *world) {
    if (!world || !world->bodies || world->body_count <= 0) {
        return 0u;
    }
    uint32_t crc = 0xFFFFFFFFu;
    crc = hash_u32_le(crc, (uint32_t)world->body_count);
    for (int i = 0; i < world->body_count; i++) {
        const rigidbody *rb = &world->bodies[i];
        crc = hash_u32_le(crc, rb->object_id);
        crc = hash_u32_le(crc, rb->object_generation);
        crc = hash_u32_le(crc, (uint32_t)rb->type);
        crc = hash_float_le(crc, rb->position.x);
        crc = hash_float_le(crc, rb->position.y);
        crc = hash_float_le(crc, rb->position.z);
        crc = hash_float_le(crc, rb->velocity.x);
        crc = hash_float_le(crc, rb->velocity.y);
        crc = hash_float_le(crc, rb->velocity.z);
        crc = hash_float_le(crc, rb->orientation.w);
        crc = hash_float_le(crc, rb->orientation.x);
        crc = hash_float_le(crc, rb->orientation.y);
        crc = hash_float_le(crc, rb->orientation.z);
    }
    return crc ^ 0xFFFFFFFFu;
}
