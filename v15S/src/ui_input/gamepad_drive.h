/* F310 gamepad drive — the single robot controller.
 *
 * Streamlined 2026-10-04: the robot is controlled ONLY by the physical
 * controller (Logitech F310, mode switch X). Terminal `ftc drive`/`stop`
 * are removed; this TU owns the pad, polls it every GUI frame, and drives
 * ftc-fleet robot 0 mecanum:
 *   left Y  -> forward/backward (forward = -Y, up on stick)
 *   left X  -> strafe left/right
 *   right X -> rotate
 *   START   -> toggle control enable (works while disabled, else latch-dead)
 *   LB + RB -> e-stop (zero drive)
 * Deadzone 0.15. Intake/shooter/buttons are NOT mapped (game layer parked).
 *
 * Linking discipline (same as term_ftc.c): the engine never links FTC.
 * Every FTC/gamepad symbol resolves through the loaded bundle handle and
 * is cached here; with no bundle loaded this TU is a silent no-op. Struct
 * layouts come from headers only. Not linked into headless/TUI builds
 * (ui_input/ is GUI-only), so MPE_GAMEPAD_DEVICE=disabled test rigs never
 * touch a device.
 */
#ifndef gamepad_drive_h
#define gamepad_drive_h

/* Open the pad on first use (honours MPE_GAMEPAD_DEVICE; missing device =
 * disconnected, retried periodically for hot-plug). Idempotent. */
void gamepad_drive_init(void);

/* Poll + command once per GUI frame, before the physics tick. No-op without
 * bundle, pad, or robot 0. Zeroes the wheels once on disconnect/vanish. */
void gamepad_drive_tick(void);

/* True while the pad holds live commands (for status/telemetry honesty). */
int gamepad_drive_active(void);

/* Joint watchdog: watches fleet robot 0 wheel mounts every GUI frame and
 * reports discontinuities (mount jump = anchor integrity, tilt spikes,
 * NaN) with a build tag, so a live "wheel snapped" report arrives with
 * evidence instead of adjectives. Event-driven (rising edge, re-arms
 * below half threshold); silent otherwise. */
void ftc_watchdog_tick(void);

#endif /* gamepad_drive_h */
