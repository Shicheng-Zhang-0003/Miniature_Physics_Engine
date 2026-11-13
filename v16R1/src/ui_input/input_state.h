#ifndef input_state_h
#define input_state_h
/* GTK4-PREP: clean input state (no GUI headers). Extracted from
 * ui_input/input_control.h so core, physics and headless code can
 * read is_debug_mode/is_mouse_locked etc without pulling gtk/gtk.h.
 * UI-only event handlers remain in input_control.h (GTK-dependent). */
#include <stdbool.h>
typedef struct {
    bool w_key_pressed, a_key_pressed, s_key_pressed, d_key_pressed, space_key_pressed, shift_key_pressed,
        escape_key_pressed, f_key_pressed;
    /* Robot keyboard drive (G V B N C M — all free globally; microvim's
     * vim-keys are modal-only). G/B = forward/back, V/N = strafe
     * left/right, C/M = rotate left/right. See ui_input/keydrive.h. */
    bool g_key_pressed, v_key_pressed, b_key_pressed, n_key_pressed, c_key_pressed, m_key_pressed;
    /* Robot keyboard intake/shoot (comma/period — both free globally).
     * Comma (or shifted `<`) held = intake runs; period (or shifted `>`)
     * pressed = shoot once (edge). See ui_input/keydrive.h. */
    bool comma_key_pressed, period_key_pressed;
    /* SANDBOX S1: freeze/step. P toggles the physics halt, O steps one
     * tick while frozen. Free globally by the same modal-microvim
     * convention as the robot keys above. */
    bool p_key_pressed, o_key_pressed;
    /* SANDBOX S4a: T toggles the resize panel (the tiny terminal-style
     * editor window for the selected body). Free by the same convention. */
    bool t_key_pressed;
    /* SANDBOX S2: the Situation screen's cursor (object menu level 5).
     * 0 = static, 1 = kinematic, 2 = no-collide; Up/Down move it, Enter
     * toggles the row. Lives in the input state so editor.c mutates and
     * overlay.c renders without cross-file statics. */
    int situation_cursor;
    bool enter_spawn_held;
    bool r_key_pressed;
    bool i_key_pressed, j_key_pressed, k_key_pressed, l_key_pressed;
    bool is_menu_open;
    bool menu_1_pressed, menu_2_pressed, menu_3_pressed;
    bool menu_4_pressed, menu_5_pressed, menu_6_pressed;
    int spawner_menu_level;
    int velocity_menu_level;
    int object_menu_level;
    int current_spawn_type; /* spawn gun: 0 sphere, 1 cube, 2 cylinder, 3 prism, 4 pyr4, 5 pyr3, 6 icosa, 7 ell */
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
    /* SANDBOX S4b: LMB drag latches. pressed is armed on button 1 ONLY
     * while the mouse is already locked (the lock-acquiring first click
     * must not grab); released arms on button 1 or on the generic
     * release (btn==0) — the drag module no-ops it when not dragging. */
    bool lmb_pressed, lmb_released;
    float mouse_delta_x, mouse_delta_y;
    bool suppress_mouse_delta;
    int marked_joint_object_index;
} input_status;
#endif /* input_state_h */
