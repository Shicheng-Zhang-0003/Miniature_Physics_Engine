#ifndef input_state_h
#define input_state_h
/* GTK4-PREP: clean input state (no GUI headers). Extracted from
 * ui_input/input_control.h so core, physics and headless code can
 * read is_debug_mode/is_mouse_locked etc without pulling gtk/gtk.h.
 * UI-only event handlers remain in input_control.h (GTK-dependent). */
#include <stdbool.h>

typedef struct {
    bool w_key_pressed, a_key_pressed, s_key_pressed, d_key_pressed, space_key_pressed, shift_key_pressed, escape_key_pressed, f_key_pressed;
    bool enter_spawn_held;
    bool r_key_pressed;
    bool i_key_pressed, j_key_pressed, k_key_pressed, l_key_pressed;
    bool is_menu_open;
    bool menu_1_pressed, menu_2_pressed, menu_3_pressed;
    bool menu_4_pressed, menu_5_pressed, menu_6_pressed;
    int spawner_menu_level;
    int velocity_menu_level;
    int object_menu_level;
    int current_spawn_type;
    bool up_arrow_pressed, down_arrow_pressed, enter_key_pressed, e_key_pressed;
    bool stability_test_pressed;
    bool sleep_wake_test_pressed;
    bool editor_torture_pressed;
    bool spawn_stress_pressed;
    bool validation_report_pressed;
    bool debug_terminal_pressed;
    bool long_run_validation_pressed;
    bool config_torture_pressed;
    bool is_mouse_locked, is_debug_mode_active;
    bool right_mouse_button_clicked, middle_mouse_button_clicked;
    float mouse_delta_x, mouse_delta_y;
    bool suppress_mouse_delta;
    int marked_joint_object_index;
} input_status;

#endif /* input_state_h */
