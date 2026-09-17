/* Scene v2 codec implementation. See scene_crc.h. */
#include "scene_crc.h"

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
