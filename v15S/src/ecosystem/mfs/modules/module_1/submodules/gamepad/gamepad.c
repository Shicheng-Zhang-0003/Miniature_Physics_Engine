#include "gamepad.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#ifdef _WIN32
/* ---------------- Windows backend: XInput (+ stub fallback) -------- */
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
/* xinput.h exists on both MinGW-w64 and MSVC SDKs. If a minimal toolchain
 * lacks it, we fall back to a disconnected stub (same API, neutral). */
#if defined(__has_include)
#if __has_include(<xinput.h>)
#include <xinput.h>
#define MPE_HAVE_XINPUT 1
#endif
#else
#include <xinput.h>
#define MPE_HAVE_XINPUT 1
#endif
/* MFS_155_GAMEPAD_PRIMARY: singleton gamepad state */
static gamepad_state g_primary_gamepad;
gamepad_state *gamepad_get_primary (void) {
    return &g_primary_gamepad;
}
/* Parse controller index from device_path / env.
 * Accepted: NULL (use env/default), "disabled", "xinput:N", "N", anything
 * else -> 0. Returns -2 for disabled, else 0..3 (clamped). */
static int win_parse_index (const char *device_path) {
    const char *s = device_path;
    if (!s) {
        const char *env = getenv ("MPE_GAMEPAD_DEVICE");
        if (env && strcmp (env, "disabled") == 0)
            return -2;
        s = (env && *env) ? env : "xinput:0";
    } else if (strcmp (s, "disabled") == 0) {
        return -2;
    }
    if (strncmp (s, "xinput:", 7) == 0)
        s += 7;
    /* bare number? */
    if (s [0] >= '0' && s [0] <= '9' && s [1] == '\0')
        return s [0] - '0';
    /* /dev/input/jsN style passed through WSL docs -> map jsN to N */
    const char *js = strstr (s, "js");
    if (js && js [1] >= '0' && js [1] <= '9')
        return js [1] - '0';
    return 0;
}
bool gamepad_init (gamepad_state *pad, const char *device_path) {
    if (!pad) {
        return false;
    }
    /* Preserve double-init safety: stash liveness before memset. On
     * Windows fd doubles as controller index (0..3) or -1. */
    int old_fd = pad->fd;
    bool was_connected = pad->connected;
    (void) old_fd;
    (void) was_connected;
    memset (pad, 0, sizeof (gamepad_state));
    pad->fd = -1;
    pad->deadzone = 0.15f;
    if (!device_path) {
        const char *env = getenv ("MPE_GAMEPAD_DEVICE");
        if (env && strcmp (env, "disabled") == 0) {
            pad->connected = false;
            return false;
        }
        device_path = (env && *env) ? env : "xinput:0";
    } else if (strcmp (device_path, "disabled") == 0) {
        pad->connected = false;
        strncpy (pad->device_path, device_path, sizeof (pad->device_path) - 1);
        return false;
    }
    strncpy (pad->device_path, device_path, sizeof (pad->device_path) - 1);
    pad->device_path [sizeof (pad->device_path) - 1] = '\0';
    int idx = win_parse_index (device_path);
    if (idx == -2) {
        pad->connected = false;
        return false;
    }
    if (idx < 0)
        idx = 0;
    if (idx > 3)
        idx = 3;
#ifdef MPE_HAVE_XINPUT
    XINPUT_STATE st;
    memset (&st, 0, sizeof (st));
    if (XInputGetState ((DWORD) idx, &st) == ERROR_SUCCESS) {
        pad->fd = idx;
        pad->connected = true;
        printf ("[gamepad] opened xinput:%d\n", idx);
        return true;
    }
    /* No controller yet: remember index, mark disconnected, but do NOT
     * spam stderr every init (headless CI sets disabled; interactive
     * Windows without a pad should stay quiet and poll will retry). */
    pad->fd = idx;
    pad->connected = false;
    return false;
#else
    (void) idx;
    pad->connected = false;
    return false;
#endif
}
void gamepad_close (gamepad_state *pad) {
    if (!pad) {
        return;
    }
    pad->fd = -1;
    pad->connected = false;
}
void gamepad_poll (gamepad_state *pad) {
    if (!pad) {
        return;
    }
#ifdef MPE_HAVE_XINPUT
    if (pad->fd < 0 || pad->fd > 3) {
        return;
    }
    XINPUT_STATE st;
    memset (&st, 0, sizeof (st));
    DWORD rc = XInputGetState ((DWORD) pad->fd, &st);
    if (rc != ERROR_SUCCESS) {
        /* Controller removed: stay disconnected but keep index so a
         * re-plug is picked up next poll without re-init. */
        pad->connected = false;
        for (int i = 0; i < gamepad_axis_count; i++)
            pad->axes [i] = 0.0f;
        for (int i = 0; i < gamepad_button_count; i++)
            pad->buttons [i] = false;
        return;
    }
    pad->connected = true;
    /* Sticks: XInput SHORT range is asymmetric like Linux js. */
    float lx =
        (st.Gamepad.sThumbLX < 0) ? (float) st.Gamepad.sThumbLX / 32768.0f : (float) st.Gamepad.sThumbLX / 32767.0f;
    float ly =
        (st.Gamepad.sThumbLY < 0) ? (float) st.Gamepad.sThumbLY / 32768.0f : (float) st.Gamepad.sThumbLY / 32767.0f;
    float rx =
        (st.Gamepad.sThumbRX < 0) ? (float) st.Gamepad.sThumbRX / 32768.0f : (float) st.Gamepad.sThumbRX / 32767.0f;
    float ry =
        (st.Gamepad.sThumbRY < 0) ? (float) st.Gamepad.sThumbRY / 32768.0f : (float) st.Gamepad.sThumbRY / 32767.0f;
    pad->axes [gamepad_axis_left_x] = lx;
    pad->axes [gamepad_axis_left_y] = ly;
    pad->axes [gamepad_axis_right_x] = rx;
    pad->axes [gamepad_axis_right_y] = ry;
    /* Triggers: BYTE 0..255, rest 0 -> store [0,1] (0-rest convention;
     * gamepad_get_trigger handles it). */
    pad->axes [gamepad_axis_left_trigger] = (float) st.Gamepad.bLeftTrigger / 255.0f;
    pad->axes [gamepad_axis_right_trigger] = (float) st.Gamepad.bRightTrigger / 255.0f;
    WORD b = st.Gamepad.wButtons;
    pad->buttons [gamepad_button_a] = (b & XINPUT_GAMEPAD_A) != 0;
    pad->buttons [gamepad_button_b] = (b & XINPUT_GAMEPAD_B) != 0;
    pad->buttons [gamepad_button_x] = (b & XINPUT_GAMEPAD_X) != 0;
    pad->buttons [gamepad_button_y] = (b & XINPUT_GAMEPAD_Y) != 0;
    pad->buttons [gamepad_button_lb] = (b & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0;
    pad->buttons [gamepad_button_rb] = (b & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
    pad->buttons [gamepad_button_back] = (b & XINPUT_GAMEPAD_BACK) != 0;
    pad->buttons [gamepad_button_start] = (b & XINPUT_GAMEPAD_START) != 0;
    pad->buttons [gamepad_button_stick_l] = (b & XINPUT_GAMEPAD_LEFT_THUMB) != 0;
    pad->buttons [gamepad_button_stick_r] = (b & XINPUT_GAMEPAD_RIGHT_THUMB) != 0;
    /* D-pad -> extra buttons 11..14 when in range. */
    if (gamepad_button_count > 11)
        pad->buttons [11] = (b & XINPUT_GAMEPAD_DPAD_UP) != 0;
    if (gamepad_button_count > 12)
        pad->buttons [12] = (b & XINPUT_GAMEPAD_DPAD_DOWN) != 0;
    if (gamepad_button_count > 13)
        pad->buttons [13] = (b & XINPUT_GAMEPAD_DPAD_LEFT) != 0;
    if (gamepad_button_count > 14)
        pad->buttons [14] = (b & XINPUT_GAMEPAD_DPAD_RIGHT) != 0;
#else
    /* No XInput headers: always disconnected, neutral. */
    pad->connected = false;
#endif
}
#else
/* ---------------- Linux / POSIX backend (unchanged) ----------------- */
#include <fcntl.h>
#ifndef MPE_OS_WINDOWS
#include <unistd.h>
#endif
#include <errno.h>
#include <linux/joystick.h>
/* MFS_155_GAMEPAD_PRIMARY: singleton gamepad state */
static gamepad_state g_primary_gamepad;
gamepad_state *gamepad_get_primary (void) {
    return &g_primary_gamepad;
}
bool gamepad_init (gamepad_state *pad, const char *device_path) {
    if (!pad) {
        return false;
    }
    /* FIX-AUDIT-DESPOT: double-init leaked the old fd (open overwrote it).
     * Stash liveness BEFORE the memset wipes it, then close. Gated on
     * connected (not fd >= 0 alone): a calloc'd pad has fd == 0, which is
     * stdin, not a joystick. */
    int old_fd = pad->fd;
    bool was_connected = pad->connected;
    memset (pad, 0, sizeof (gamepad_state));
    if (was_connected && old_fd >= 0) {
        close (old_fd);
    }
    pad->fd = -1;
    pad->deadzone = 0.15f;
    /* DESPOT-FIX: MPE_GAMEPAD_DEVICE was set to "disabled" by every headless
     * script but never read here — gamepad_init(NULL) always opened
     * /dev/input/js0 and spammed stderr on headless boxes. Now: NULL path
     * consults the env; "disabled" skips device access silently (returns
     * false, not connected); any other value is the device path. */
    if (!device_path) {
        const char *env = getenv ("MPE_GAMEPAD_DEVICE");
        if (env && strcmp (env, "disabled") == 0) {
            pad->connected = false;
            return false;
        }
        device_path = (env && *env) ? env : "/dev/input/js0";
    }
    strncpy (pad->device_path, device_path, sizeof (pad->device_path) - 1);
    pad->device_path [sizeof (pad->device_path) - 1] = '\0';
    pad->fd = open (device_path, O_RDONLY | O_NONBLOCK);
    if (pad->fd < 0) {
        fprintf (stderr, "[gamepad] could not open %s: %s\n", device_path, strerror (errno));
        fprintf (stderr, "[gamepad] hint: try 'sudo usermod -aG input $USER' "
                         "then log out and back in\n");
        pad->connected = false;
        return false;
    }
    pad->connected = true;
    printf ("[gamepad] opened %s\n", device_path);
    return true;
}
void gamepad_close (gamepad_state *pad) {
    if (!pad) {
        return;
    }
    if (pad->fd >= 0) {
        close (pad->fd);
        pad->fd = -1;
    }
    pad->connected = false;
}
void gamepad_poll (gamepad_state *pad) {
    if (!pad || !pad->connected || pad->fd < 0) {
        return;
    }
    struct js_event ev;
    ssize_t bytes;
    /* drain all pending events */
    while ((bytes = read (pad->fd, &ev, sizeof (ev))) == sizeof (ev)) {
        __u8 type = ev.type & ~JS_EVENT_INIT;
        if (type == JS_EVENT_AXIS) {
            if (ev.number < gamepad_axis_count) {
                /* FIX-AUDIT-DESPOT: js values are asymmetric (-32768..32767).
                 * Dividing everything by 32767 maps full-down to -1.00003
                 * (out of the documented [-1,1] range). Scale each side by
                 * its own extreme so both ends land exactly on +-1. */
                pad->axes [ev.number] = (ev.value < 0) ? (float) ev.value / 32768.0f : (float) ev.value / 32767.0f;
            }
        } else if (type == JS_EVENT_BUTTON) {
            if (ev.number < gamepad_button_count) {
                pad->buttons [ev.number] = (ev.value != 0);
            }
        }
    }
    /* EAGAIN/EWOULDBLOCK means no more events (non-blocking) - that's fine */
    if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        fprintf (stderr, "[gamepad] read error on %s: %s\n", pad->device_path, strerror (errno));
        close (pad->fd);
        pad->fd = -1;
        pad->connected = false;
    }
}
#endif
/* ---------------- Shared (portable) helpers ------------------------ */
static float apply_deadzone (float value, float deadzone) {
    if (value > deadzone) {
        return (value - deadzone) / (1.0f - deadzone);
    }
    if (value < -deadzone) {
        return (value + deadzone) / (1.0f - deadzone);
    }
    return 0.0f;
}
float gamepad_get_axis (const gamepad_state *pad, int axis) {
    if (!pad || axis < 0 || axis >= gamepad_axis_count) {
        return 0.0f;
    }
    float value = pad->axes [axis];
    if (axis == gamepad_axis_left_y && pad->invert_left_y) {
        value = -value;
    }
    if (axis == gamepad_axis_left_x && pad->invert_left_x) {
        value = -value;
    }
    if (axis == gamepad_axis_right_x && pad->invert_right_x) {
        value = -value;
    }
    return apply_deadzone (value, pad->deadzone);
}
float gamepad_get_trigger (const gamepad_state *pad, int axis) {
    if (!pad || axis < 0 || axis >= gamepad_axis_count) {
        return 0.0f;
    }
    float v = pad->axes [axis];
    if (!isfinite (v))
        return 0.0f;
    if (v < -1.0f)
        v = -1.0f;
    if (v > 1.0f)
        v = 1.0f;
    /* DESPOT-2026-09-26: two driver conventions exist — 0-rest (rest 0,
     * press +1) and -1-rest (rest -1, press +1). Auto-detect per read:
     * v<=-0.95 is the -1-rest rest position -> 0; otherwise map [-1,1] to
     * [0,1] linearly for -1-rest ((v+1)/2) and [0,1] to [0,1] for 0-rest
     * (v clamped >=0). The old max(0,v) mapped a -1-rest mid-travel (v=0)
     * to 0 instead of 0.5 — nonlinear for proportional use. Heuristic: if
     * the axis has ever been seen below -0.9, treat as -1-rest. Stateless
     * fallback below assumes 0-rest only when no negative excursion is
     * possible; since we are stateless, use: v<=-0.95 -> 0 (rest), else if
     * v<0 -> (v+1)/2 *only* when the driver is -1-rest. We cannot know
     * statelessly, so blend: treat [-1,0) as (v+1)/2 scaled by 0.5 when
     * v>-0.95? No — that breaks 0-rest noise. Resolution: -1-rest drivers
     * idle at exactly -1; 0-rest drivers never go below ~-0.05 noise.
     * So v < -0.5 MUST be a -1-rest driver in transit -> (v+1)/2;
     * v in [-0.5,0] is ambiguous rest noise -> 0. */
    float t;
    if (v <= -0.95f) {
        t = 0.0f; /* -1-rest at rest */
    } else if (v < -0.5f) {
        t = (v + 1.0f) * 0.5f; /* -1-rest in transit, linear */
    } else if (v <= 0.0f) {
        t = 0.0f; /* rest noise on either convention */
    } else {
        t = v; /* pressed: both conventions agree on (0,1] */
    }
    if (t < 0.05f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;
    return t;
}
bool gamepad_get_button (const gamepad_state *pad, int button) {
    if (!pad || button < 0 || button >= gamepad_button_count) {
        return false;
    }
    return pad->buttons [button];
}
bool gamepad_is_connected (const gamepad_state *pad) {
    if (!pad) {
        return false;
    }
    return pad->connected;
}
void gamepad_set_deadzone (gamepad_state *pad, float deadzone) {
    if (!pad) {
        return;
    }
    if (deadzone < 0.0f) {
        deadzone = 0.0f;
    }
    if (deadzone > 0.9f) {
        deadzone = 0.9f;
    }
    pad->deadzone = deadzone;
}
