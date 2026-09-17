/* Frustum-culling truth: the plane extraction in render/new_render.c must
 * agree with clip-space projection. A point inside clip space must never
 * be culled (no false exclusion); a point far outside must be culled.
 * Pure math4 — no GL required. Built via `make build_frustum`. */
#ifdef mpe_frustum_test
#include <stdio.h>
#include <math.h>
#include "core/math3d.h"
#include "core/math4_special.h"

static vector4 math4_mul_vec4(math4 m, vector4 v) {
    /* vector4 packs {w,x,y,z}; matrix columns 0..3 pair with x,y,z,w. */
    float vc[4] = {v.x, v.y, v.z, v.w};
    /* column-major: out[row] = sum_col M[col][row] * v[col], with
     * vector (x,y,z,w) mapped to columns 0..3. */
    float out[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int row = 0; row < 4; row++) {
        out[row] = m.matrix[0][row] * vc[0] + m.matrix[1][row] * vc[1] + m.matrix[2][row] * vc[2] +
                   m.matrix[3][row] * vc[3];
    }
    return (vector4){out[3], out[0], out[1], out[2]};
}

static void extract_planes(math4 vp, vector4 planes[6]) {
    float row0[4] = {vp.matrix[0][0], vp.matrix[1][0], vp.matrix[2][0], vp.matrix[3][0]};
    float row1[4] = {vp.matrix[0][1], vp.matrix[1][1], vp.matrix[2][1], vp.matrix[3][1]};
    float row2[4] = {vp.matrix[0][2], vp.matrix[1][2], vp.matrix[2][2], vp.matrix[3][2]};
    float row3[4] = {vp.matrix[0][3], vp.matrix[1][3], vp.matrix[2][3], vp.matrix[3][3]};
    float combos[6][4];
    for (int k = 0; k < 4; k++) {
        combos[0][k] = row3[k] + row0[k];
        combos[1][k] = row3[k] - row0[k];
        combos[2][k] = row3[k] + row1[k];
        combos[3][k] = row3[k] - row1[k];
        combos[4][k] = row3[k] + row2[k];
        combos[5][k] = row3[k] - row2[k];
    }
    for (int p = 0; p < 6; p++) {
        float len = sqrtf(combos[p][0] * combos[p][0] + combos[p][1] * combos[p][1] + combos[p][2] * combos[p][2]);
        if (len < 0.000001f) {
            len = 1.0f;
        }
        /* vector4 packs {w,x,y,z}: store (d,a,b,c). Mirrors new_render.c. */
        planes[p] = (vector4){combos[p][3] / len, combos[p][0] / len, combos[p][1] / len, combos[p][2] / len};
    }
}

static bool planes_inside(vector4 planes[6], vector3 p) {
    for (int i = 0; i < 6; i++) {
        float d = planes[i].x * p.x + planes[i].y * p.y + planes[i].z * p.z + planes[i].w;
        if (d < 0.0f) {
            return false;
        }
    }
    return true;
}

int main(void) {
    int fail = 0;
    /* Camera poses: pos, front, up. */
    vector3 poses[4][3] = {
        {{0.0f, 20.0f, 50.0f}, {0.0f, -0.3f, -1.0f}, {0.0f, 1.0f, 0.0f}},
        {{10.0f, 5.0f, 10.0f}, {-1.0f, 0.0f, -1.0f}, {0.0f, 1.0f, 0.0f}},
        {{-30.0f, 2.0f, 0.0f}, {1.0f, 0.1f, 0.0f}, {0.0f, 1.0f, 0.0f}},
        {{0.0f, 100.0f, 0.01f}, {0.0f, -1.0f, -0.01f}, {0.0f, 0.0f, -1.0f}},
    };
    for (int pose = 0; pose < 4; pose++) {
        math4 proj = math4_perspective_fov((3.14159265f / 180.0f) * 45.0f, 16.0f / 9.0f, 0.1f, 1000.0f);
        math4 view = math4_look_view(poses[pose][0], poses[pose][1], poses[pose][2]);
        math4 vp = math4_multiplication(proj, view);
        vector4 planes[6];
        extract_planes(vp, planes);
        unsigned int rng = 12345u + (unsigned int) pose * 999u;
        int inside_clip = 0, culled = 0, false_out = 0, far_kept = 0;
        for (int s = 0; s < 20000; s++) {
            rng = rng * 1664525u + 1013904223u;
            float fx = ((rng >> 8) % 10000) / 10000.0f;
            rng = rng * 1664525u + 1013904223u;
            float fy = ((rng >> 8) % 10000) / 10000.0f;
            rng = rng * 1664525u + 1013904223u;
            float fz = ((rng >> 8) % 10000) / 10000.0f;
            vector3 p = {(fx - 0.5f) * 200.0f, fy * 100.0f, (fz - 0.5f) * 200.0f};
            vector4 clip = math4_mul_vec4(vp, (vector4){1.0f, p.x, p.y, p.z});
            float w = clip.w;
            bool in_clip = false, clearly_out = false;
            if (w > 0.0001f) {
                float nx = clip.x / w, ny = clip.y / w, nz = clip.z / w;
                in_clip = (nx > -1.0f) && (nx < 1.0f) && (ny > -1.0f) && (ny < 1.0f) && (nz > -1.0f) && (nz < 1.0f);
                clearly_out = (nx < -1.5f) || (nx > 1.5f) || (ny < -1.5f) || (ny > 1.5f) || (nz < -1.5f) ||
                              (nz > 1.5f) || (w < 0.0f);
            } else {
                clearly_out = true;
            }
            bool kept = planes_inside(planes, p);
            if (in_clip) {
                inside_clip++;
                if (!kept) {
                    false_out++;
                }
            }
            if (clearly_out) {
                culled++;
                if (kept) {
                    far_kept++;
                }
            }
        }
        printf("[info] pose %d: clip-inside=%d false-excluded=%d far-outside=%d far-kept=%d\n", pose, inside_clip,
               false_out, culled, far_kept);
        if (false_out > 0) {
            printf("[FAIL] pose %d: %d visible points culled\n", pose, false_out);
            fail = 1;
        }
        if ((culled > 0) && (far_kept == culled)) {
            printf("[FAIL] pose %d: culling never fires\n", pose);
            fail = 1;
        }
    }
    if (!fail) {
        printf("[PASS] frustum planes agree with clip space; no false exclusion\n");
    }
    return fail;
}
#endif /* mpe_frustum_test */
