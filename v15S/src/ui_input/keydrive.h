/* DESPOT-2026-10-14: keyboard robot drive (G V B N C M), as a
 * dependency-free pure function so it can be tested from a headless build
 * (same rationale as mouse_look.h: the piece that can be wrong in a
 * directional way must not require a compositor and a physical keyboard).
 *
 * CONVENTION (the thing under test), matching the gamepad + mixer frame:
 *   G / B         = forward +1 / back -1  (chassis +Z forward)
 *   V / N         = strafe left -1 / right +1 (robot's right is -X, so
 *                   +strafe moves -X, exactly like the gamepad left-X)
 *   C / M         = rotate left -1 / right +1 (right key = +1, exactly
 *                   like gamepad right-stick-right)
 * Opposing pairs cancel pair-wise (G+B, V+N, C+M); each axis is
 * independent, so diagonals combine (G+N = forward-right arc).
 * Full deflection = full command (no expo here — expo lives in the
 * gamepad path; keyboard keys are already binary).
 *
 * Physical QWERTY rationale for the choice: G sits above B (forward/back
 * column), V sits left of N (strafe pair), C/M are the outer pair
 * (rotate). All six are free globally (microvim's same-letter vim-keys
 * are modal, editor-only).
 *
 * No GTK, no engine headers: this compiles anywhere, including the
 * headless test suite, so a remap cannot pass CI by never being
 * exercised.
 */
#ifndef keydrive_h
#define keydrive_h
/* Six held keys in, three drive commands out (each -1, 0, or +1). */
static inline void mpe_keydrive_axes (int g, int v, int b, int n, int c, int m, float *out_fwd, float *out_strafe,
                                      float *out_rotate) {
    float fwd = 0.0f, strafe = 0.0f, rotate = 0.0f;
    if (g) { fwd += 1.0f; }
    if (b) { fwd -= 1.0f; }
    if (n) { strafe += 1.0f; }
    if (v) { strafe -= 1.0f; }
    if (m) { rotate += 1.0f; }
    if (c) { rotate -= 1.0f; }
    if (out_fwd) { *out_fwd = fwd; }
    if (out_strafe) { *out_strafe = strafe; }
    if (out_rotate) { *out_rotate = rotate; }
}
/* Idle predicate on the RESULTANT command (not on held keys): opposing
 * pairs (G+B, V+N, C+M) cancel to a zero drive, which must brake exactly
 * like a centered stick — holding both keys of a pair is a finger slip,
 * not a drive command. The tick brakes on this, never on raw key state. */
static inline int mpe_keydrive_idle (int g, int v, int b, int n, int c, int m) {
    float fwd = 0.0f, strafe = 0.0f, rotate = 0.0f;
    mpe_keydrive_axes (g, v, b, n, c, m, &fwd, &strafe, &rotate);
    return (fwd == 0.0f && strafe == 0.0f && rotate == 0.0f) ? 1 : 0;
}
#endif /* keydrive_h */
