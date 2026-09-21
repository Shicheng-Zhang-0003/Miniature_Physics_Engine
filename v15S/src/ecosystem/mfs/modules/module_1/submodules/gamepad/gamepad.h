#ifndef gamepad_h
#define gamepad_h

#include <stdbool.h>

/* axis indices (standard gamepad layout) */
/* MFS_159_F310_AXES: Logitech F310 XInput mode axis mapping.
 * Set the mode switch on the back of the controller to X.
 * Axis 2 = LT trigger, Axis 5 = RT trigger (not used for drive). */
#define gamepad_axis_left_x         0
#define gamepad_axis_left_y         1
#define gamepad_axis_left_trigger   2  /* F310 XInput: LT */
#define gamepad_axis_right_x        3  /* F310 XInput: right stick X */
#define gamepad_axis_right_y        4  /* F310 XInput: right stick Y */
#define gamepad_axis_right_trigger  5  /* F310 XInput: RT */
#define gamepad_axis_count          8

/* button indices (standard gamepad layout) */
#define gamepad_button_a       0
#define gamepad_button_b       1
#define gamepad_button_x       2
#define gamepad_button_y       3
#define gamepad_button_lb      4
#define gamepad_button_rb      5
#define gamepad_button_back    6
#define gamepad_button_start   7
#define gamepad_button_guide   8
#define gamepad_button_stick_l 9
#define gamepad_button_stick_r 10
#define gamepad_button_count   16

typedef struct {
    bool connected;
    float axes[gamepad_axis_count];
    bool buttons[gamepad_button_count];
    char device_path[256];
    int fd;
    float deadzone;
    bool invert_left_y;
    bool invert_left_x;
    bool invert_right_x;
} gamepad_state;

/* open the evdev joystick device. returns true on success.
 * if device_path is null, defaults to /dev/input/js0.
 * on failure, prints a hint about the 'input' group. */
bool gamepad_init(gamepad_state *pad, const char *device_path);

/* close the device and release resources. */
void gamepad_close(gamepad_state *pad);

/* drain all pending events from the device. call once per frame. */
void gamepad_poll(gamepad_state *pad);

/* read an axis value in [-1, 1] with deadzone and inversion applied. */
float gamepad_get_axis(const gamepad_state *pad, int axis);

/* read a button's pressed state. */
bool gamepad_get_button(const gamepad_state *pad, int button);

/* true if the device is open and reading events. */
bool gamepad_is_connected(const gamepad_state *pad);

/* set the deadzone threshold (clamped to [0, 0.9]). */
void gamepad_set_deadzone(gamepad_state *pad, float deadzone);

/* return a pointer to the engine's primary gamepad state (singleton).
 * call gamepad_init() on this pointer at startup. */
gamepad_state *gamepad_get_primary(void);
#endif /* gamepad_h */
