#include "gamepad.h"

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <linux/joystick.h>

/* MFS_155_GAMEPAD_PRIMARY: singleton gamepad state */
static gamepad_state g_primary_gamepad;

gamepad_state *gamepad_get_primary(void) {
    return &g_primary_gamepad;
}

bool gamepad_init(gamepad_state *pad, const char *device_path) {
    if (!pad) { return false; }
    memset(pad, 0, sizeof(gamepad_state));
    pad->fd = -1;
    pad->deadzone = 0.15f;
    if (!device_path) { device_path = "/dev/input/js0"; }
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
                pad->axes[ev.number] = (float) ev.value / 32767.0f;
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
