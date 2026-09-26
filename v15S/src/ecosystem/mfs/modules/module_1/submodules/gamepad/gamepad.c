#include "gamepad.h"

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <linux/joystick.h>

/* MFS_155_GAMEPAD_PRIMARY: singleton gamepad state */
static gamepad_state g_primary_gamepad;

gamepad_state *gamepad_get_primary(void) {
    return &g_primary_gamepad;
}

bool gamepad_init(gamepad_state *pad, const char *device_path) {
    if (!pad) { return false; }
    /* FIX-AUDIT-DESPOT: double-init leaked the old fd (open overwrote it).
     * Stash liveness BEFORE the memset wipes it, then close. Gated on
     * connected (not fd >= 0 alone): a calloc'd pad has fd == 0, which is
     * stdin, not a joystick. */
    int old_fd = pad->fd;
    bool was_connected = pad->connected;
    memset(pad, 0, sizeof(gamepad_state));
    if (was_connected && old_fd >= 0) {
        close(old_fd);
    }
    pad->fd = -1;
    pad->deadzone = 0.15f;
    /* DESPOT-FIX: MPE_GAMEPAD_DEVICE was set to "disabled" by every headless
     * script but never read here — gamepad_init(NULL) always opened
     * /dev/input/js0 and spammed stderr on headless boxes. Now: NULL path
     * consults the env; "disabled" skips device access silently (returns
     * false, not connected); any other value is the device path. */
    if (!device_path) {
        const char *env = getenv("MPE_GAMEPAD_DEVICE");
        if (env && strcmp(env, "disabled") == 0) {
            pad->connected = false;
            return false;
        }
        device_path = (env && *env) ? env : "/dev/input/js0";
    }
    strncpy(pad->device_path, device_path, sizeof(pad->device_path) - 1);
    pad->device_path[sizeof(pad->device_path) - 1] = '\0';
    pad->fd = open(device_path, O_RDONLY | O_NONBLOCK);
    if (pad->fd < 0) {
        fprintf(stderr, "[gamepad] could not open %s: %s\n",
                device_path, strerror(errno));
        fprintf(stderr, "[gamepad] hint: try 'sudo usermod -aG input $USER' "
                "then log out and back in\n");
        pad->connected = false;
        return false;
    }
    pad->connected = true;
    printf("[gamepad] opened %s\n", device_path);
    return true;
}

void gamepad_close(gamepad_state *pad) {
    if (!pad) { return; }
    if (pad->fd >= 0) {
        close(pad->fd);
        pad->fd = -1;
    }
    pad->connected = false;
}

void gamepad_poll(gamepad_state *pad) {
    if (!pad || !pad->connected || pad->fd < 0) { return; }
    struct js_event ev;
    ssize_t bytes;
    /* drain all pending events */
    while ((bytes = read(pad->fd, &ev, sizeof(ev))) == sizeof(ev)) {
        __u8 type = ev.type & ~JS_EVENT_INIT;
        if (type == JS_EVENT_AXIS) {
            if (ev.number < gamepad_axis_count) {
                /* FIX-AUDIT-DESPOT: js values are asymmetric (-32768..32767).
                 * Dividing everything by 32767 maps full-down to -1.00003
                 * (out of the documented [-1,1] range). Scale each side by
                 * its own extreme so both ends land exactly on +-1. */
                pad->axes[ev.number] = (ev.value < 0)
                    ? (float)ev.value / 32768.0f
                    : (float)ev.value / 32767.0f;
            }
        } else if (type == JS_EVENT_BUTTON) {
            if (ev.number < gamepad_button_count) {
                pad->buttons[ev.number] = (ev.value != 0);
            }
        }
    }
    /* EAGAIN/EWOULDBLOCK means no more events (non-blocking) - that's fine */
    if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        fprintf(stderr, "[gamepad] read error on %s: %s\n",
                pad->device_path, strerror(errno));
        close(pad->fd);
        pad->fd = -1;
        pad->connected = false;
    }
}

static float apply_deadzone(float value, float deadzone) {
    if (value > deadzone) {
        return (value - deadzone) / (1.0f - deadzone);
    }
    if (value < -deadzone) {
        return (value + deadzone) / (1.0f - deadzone);
    }
    return 0.0f;
}

float gamepad_get_axis(const gamepad_state *pad, int axis) {
    if (!pad || axis < 0 || axis >= gamepad_axis_count) { return 0.0f; }
    float value = pad->axes[axis];
    if (axis == gamepad_axis_left_y && pad->invert_left_y) { value = -value; }
    if (axis == gamepad_axis_left_x && pad->invert_left_x) { value = -value; }
    if (axis == gamepad_axis_right_x && pad->invert_right_x) { value = -value; }
    return apply_deadzone(value, pad->deadzone);
}

float gamepad_get_trigger(const gamepad_state *pad, int axis) {
    if (!pad || axis < 0 || axis >= gamepad_axis_count) { return 0.0f; }
    float v = pad->axes[axis];
    if (!isfinite(v)) return 0.0f;
    if (v < -1.0f) v = -1.0f;
    if (v > 1.0f) v = 1.0f;
    /* Collapse the ambiguous lower half: rest is 0 on every driver. */
    float t = (v > 0.0f) ? v : 0.0f;
    /* Small deadzone so resting noise never reads as a press. */
    if (t < 0.05f) t = 0.0f;
    return t;
}

bool gamepad_get_button(const gamepad_state *pad, int button) {
    if (!pad || button < 0 || button >= gamepad_button_count) { return false; }
    return pad->buttons[button];
}

bool gamepad_is_connected(const gamepad_state *pad) {
    if (!pad) { return false; }
    return pad->connected;
}

void gamepad_set_deadzone(gamepad_state *pad, float deadzone) {
    if (!pad) { return; }
    if (deadzone < 0.0f) { deadzone = 0.0f; }
    if (deadzone > 0.9f) { deadzone = 0.9f; }
    pad->deadzone = deadzone;
}
